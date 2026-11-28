#include "astar_planner/astar_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>

namespace astar_planner
{

namespace
{

const int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
const int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
constexpr double kSqrt2 = 1.4142135623730951;
const double kCost[8] = {1.0, 1.0, 1.0, 1.0, kSqrt2, kSqrt2, kSqrt2, kSqrt2};

std::string trim(const std::string & text)
{
  const size_t first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return std::string();
  }
  const size_t last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

// Reads the next integer token from a PNM header, skipping '#' comment lines.
int readHeaderToken(std::ifstream & stream)
{
  std::string token;
  while (stream >> token) {
    if (!token.empty() && token[0] == '#') {
      std::string rest;
      std::getline(stream, rest);
      continue;
    }
    try {
      return std::stoi(token);
    } catch (const std::exception &) {
      return -1;
    }
  }
  return -1;
}

bool loadPgm(const std::string & pgm_path, GridMap & map, std::string & error)
{
  std::ifstream pgm(pgm_path, std::ios::binary);
  if (!pgm.good()) {
    error = "cannot open image: " + pgm_path;
    return false;
  }

  std::string magic;
  pgm >> magic;
  if (magic != "P5") {
    error = "only binary PGM (P5) is supported, got '" + magic + "'";
    return false;
  }

  const int width = readHeaderToken(pgm);
  const int height = readHeaderToken(pgm);
  const int max_value = readHeaderToken(pgm);
  if (width <= 0 || height <= 0 || max_value <= 0 || max_value > 255) {
    error = "bad PGM header";
    return false;
  }

  // Consume the single whitespace byte that terminates the header.
  pgm.get();

  const size_t count = static_cast<size_t>(width) * static_cast<size_t>(height);
  std::vector<unsigned char> pixels(count);
  pgm.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(count));
  if (!pgm) {
    error = "truncated PGM payload";
    return false;
  }

  map.width = width;
  map.height = height;
  map.occupancy.assign(count, 1);

  for (size_t i = 0; i < count; ++i) {
    const unsigned char value = pixels[i];
    // nav2 map_server convention (negate = 0):
    //   254 -> free, 0 -> occupied, 205 -> unknown.
    // Unknown cells are treated as obstacles so the planner never cuts through them.
    map.occupancy[i] = (value >= 250) ? 0 : 1;
  }

  return true;
}

}  // namespace

const char * heuristicName(HeuristicType type)
{
  switch (type) {
    case HeuristicType::DIJKSTRA:
      return "dijkstra (h=0)";
    case HeuristicType::MANHATTAN:
      return "manhattan";
    case HeuristicType::EUCLIDEAN:
      return "euclidean";
    case HeuristicType::OCTILE:
      return "octile";
    default:
      return "unknown";
  }
}

bool GridMap::worldToCell(double wx, double wy, int & cx, int & cy) const
{
  if (resolution <= 0.0) {
    return false;
  }
  cx = static_cast<int>(std::floor((wx - origin_x) / resolution));
  cy = static_cast<int>(std::floor((wy - origin_y) / resolution));
  return cx >= 0 && cy >= 0 && cx < width && cy < height;
}

void GridMap::cellToWorld(int cx, int cy, double & wx, double & wy) const
{
  wx = origin_x + (static_cast<double>(cx) + 0.5) * resolution;
  wy = origin_y + (static_cast<double>(cy) + 0.5) * resolution;
}

bool GridMap::isFree(int cx, int cy) const
{
  if (cx < 0 || cy < 0 || cx >= width || cy >= height) {
    return false;
  }
  const size_t index = static_cast<size_t>(cy) * static_cast<size_t>(width) +
    static_cast<size_t>(cx);
  return occupancy[index] == 0;
}

bool GridMap::isFreeWithClearance(int cx, int cy, int clearance) const
{
  for (int dy = -clearance; dy <= clearance; ++dy) {
    for (int dx = -clearance; dx <= clearance; ++dx) {
      if (!isFree(cx + dx, cy + dy)) {
        return false;
      }
    }
  }
  return true;
}

bool loadMapFromYaml(const std::string & yaml_path, GridMap & map, std::string & error)
{
  std::ifstream yaml(yaml_path);
  if (!yaml.good()) {
    error = "cannot open yaml: " + yaml_path;
    return false;
  }

  std::string image_name;
  double origin_x = 0.0;
  double origin_y = 0.0;

  std::string line;
  while (std::getline(yaml, line)) {
    const size_t hash = line.find('#');
    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }
    const std::string stripped = trim(line);
    if (stripped.empty()) {
      continue;
    }

    const size_t colon = stripped.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = trim(stripped.substr(0, colon));
    const std::string value = trim(stripped.substr(colon + 1));

    if (key == "image") {
      image_name = value;
    } else if (key == "resolution") {
      try {
        map.resolution = std::stod(value);
      } catch (const std::exception &) {
        error = "bad resolution field";
        return false;
      }
    } else if (key == "origin") {
      const size_t open = value.find('[');
      const size_t close = value.find(']');
      if (open == std::string::npos || close == std::string::npos || close <= open) {
        error = "bad origin field";
        return false;
      }
      std::stringstream stream(value.substr(open + 1, close - open - 1));
      std::string token;
      std::vector<double> numbers;
      while (std::getline(stream, token, ',')) {
        try {
          numbers.push_back(std::stod(trim(token)));
        } catch (const std::exception &) {
          break;
        }
      }
      if (numbers.size() < 2) {
        error = "origin needs at least two values";
        return false;
      }
      origin_x = numbers[0];
      origin_y = numbers[1];
      map.origin_x = origin_x;
      map.origin_y = origin_y;
    }
  }

  if (image_name.empty()) {
    error = "yaml has no 'image' field";
    return false;
  }

  const size_t slash = yaml_path.find_last_of('/');
  const std::string directory = (slash == std::string::npos) ? std::string() :
    yaml_path.substr(0, slash + 1);
  const std::string pgm_path = directory + image_name;

  return loadPgm(pgm_path, map, error);
}

void AStarPlanner::setMap(const GridMap & map)
{
  map_ = map;
}

void AStarPlanner::setHeuristic(HeuristicType type)
{
  type_ = type;
}

bool AStarPlanner::inBounds(int cx, int cy) const
{
  return cx >= 0 && cy >= 0 && cx < map_.width && cy < map_.height;
}

double AStarPlanner::heuristic(int cx, int cy, int gx, int gy) const
{
  if (type_ == HeuristicType::DIJKSTRA) {
    return 0.0;
  }
  const double dx = std::abs(static_cast<double>(gx - cx));
  const double dy = std::abs(static_cast<double>(gy - cy));
  if (type_ == HeuristicType::MANHATTAN) {
    return (dx + dy) * map_.resolution;
  }
  if (type_ == HeuristicType::OCTILE) {
    // exact cost on an 8-connected grid without obstacles: straight steps plus
    // diagonal steps, so h never overestimates and is also consistent.
    const double mn = std::min(dx, dy);
    return ((dx + dy) + (kSqrt2 - 2.0) * mn) * map_.resolution;
  }
  return std::sqrt(dx * dx + dy * dy) * map_.resolution;
}

PlanStats AStarPlanner::plan(int start_cx, int start_cy, int goal_cx, int goal_cy)
{
  PlanStats stats;
  path_.clear();

  if (map_.width <= 0 || map_.height <= 0) {
    return stats;
  }
  if (!inBounds(start_cx, start_cy) || !inBounds(goal_cx, goal_cy)) {
    return stats;
  }
  if (!map_.isFree(start_cx, start_cy) || !map_.isFree(goal_cx, goal_cy)) {
    return stats;
  }

  const int width = map_.width;
  const size_t total = static_cast<size_t>(width) * static_cast<size_t>(map_.height);
  const double infinity = std::numeric_limits<double>::infinity();

  std::vector<double> g_score(total, infinity);
  std::vector<int> parent(total, -1);
  std::vector<char> closed(total, 0);

  auto index_of = [width](int cx, int cy) {
      return static_cast<size_t>(cy) * static_cast<size_t>(width) +
             static_cast<size_t>(cx);
    };

  struct QueueNode
  {
    double f;
    size_t index;
  };

  struct Compare
  {
    bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
    {
      return lhs.f > rhs.f;
    }
  };

  std::priority_queue<QueueNode, std::vector<QueueNode>, Compare> open;

  const size_t start_index = index_of(start_cx, start_cy);
  const size_t goal_index = index_of(goal_cx, goal_cy);
  g_score[start_index] = 0.0;
  open.push(QueueNode{heuristic(start_cx, start_cy, goal_cx, goal_cy), start_index});

  const auto begin = std::chrono::steady_clock::now();
  bool reached = false;

  while (!open.empty()) {
    const QueueNode current = open.top();
    open.pop();

    if (closed[current.index] != 0) {
      continue;
    }
    closed[current.index] = 1;
    stats.expanded_nodes++;

    if (current.index == goal_index) {
      reached = true;
      break;
    }

    const int cx = static_cast<int>(current.index % static_cast<size_t>(width));
    const int cy = static_cast<int>(current.index / static_cast<size_t>(width));

    for (int direction = 0; direction < 8; ++direction) {
      const int nx = cx + kDx[direction];
      const int ny = cy + kDy[direction];

      if (!map_.isFree(nx, ny)) {
        continue;
      }

      // Diagonal moves must not squeeze between two occupied cells.
      if (direction >= 4) {
        if (!map_.isFree(cx + kDx[direction], cy) || !map_.isFree(cx, cy + kDy[direction])) {
          continue;
        }
      }

      const size_t next_index = index_of(nx, ny);
      if (closed[next_index] != 0) {
        continue;
      }

      const double tentative = g_score[current.index] + kCost[direction] * map_.resolution;
      if (tentative < g_score[next_index]) {
        g_score[next_index] = tentative;
        parent[next_index] = static_cast<int>(current.index);
        open.push(QueueNode{tentative + heuristic(nx, ny, goal_cx, goal_cy), next_index});
      }
    }
  }

  const auto finish = std::chrono::steady_clock::now();
  stats.elapsed_ms =
    std::chrono::duration<double, std::milli>(finish - begin).count();
  stats.found = reached;

  if (!reached) {
    return stats;
  }

  std::vector<std::pair<int, int>> reversed;
  size_t cursor = goal_index;
  while (true) {
    const int cx = static_cast<int>(cursor % static_cast<size_t>(width));
    const int cy = static_cast<int>(cursor / static_cast<size_t>(width));
    reversed.emplace_back(cx, cy);
    if (cursor == start_index) {
      break;
    }
    const int previous = parent[cursor];
    if (previous < 0) {
      stats.found = false;
      path_.clear();
      return stats;
    }
    cursor = static_cast<size_t>(previous);
  }

  std::reverse(reversed.begin(), reversed.end());
  path_ = reversed;
  stats.path_cells = path_.size();

  double length = 0.0;
  for (size_t i = 1; i < path_.size(); ++i) {
    const int dx = path_[i].first - path_[i - 1].first;
    const int dy = path_[i].second - path_[i - 1].second;
    length += std::sqrt(static_cast<double>(dx * dx + dy * dy)) * map_.resolution;
  }
  stats.path_length = length;

  return stats;
}

}  // namespace astar_planner
