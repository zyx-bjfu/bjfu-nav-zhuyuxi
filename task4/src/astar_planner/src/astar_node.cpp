#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "astar_planner/astar_planner.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

struct Options
{
  std::string map_path;
  std::string out_pgm;
  bool has_start = false;
  bool has_goal = false;
  double start_x = 0.0;
  double start_y = 0.0;
  double goal_x = 0.0;
  double goal_y = 0.0;
  bool publish = true;
};

void printUsage()
{
  std::cout <<
    "usage: astar_node --map <map.yaml> [options]\n"
    "  --map   <path>        nav2 style map yaml (image + resolution + origin)\n"
    "  --start <x> <y>       start pose in map frame (default: auto pick)\n"
    "  --goal  <x> <y>       goal pose in map frame (default: auto pick)\n"
    "  --out-pgm <path>      write the grid with all four paths burned in\n"
    "  --no-publish          skip rviz publishing, pure offline benchmark\n"
              << std::endl;
}

bool parseNumber(const char * text, double & value)
{
  try {
    value = std::stod(text);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

bool parseArguments(int argc, char ** argv, Options & options)
{
  const char * home = std::getenv("HOME");
  const std::string home_prefix = (home != nullptr) ? std::string(home) : std::string("~");
  options.map_path = home_prefix + "/ws_pb_rm/src/rm_nav_bringup/map/RMUL.yaml";

  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--help" || argument == "-h") {
      printUsage();
      return false;
    } else if (argument == "--map" && i + 1 < argc) {
      options.map_path = argv[++i];
    } else if (argument == "--out-pgm" && i + 1 < argc) {
      options.out_pgm = argv[++i];
    } else if (argument == "--no-publish") {
      options.publish = false;
    } else if (argument == "--start" && i + 2 < argc) {
      if (!parseNumber(argv[i + 1], options.start_x) ||
        !parseNumber(argv[i + 2], options.start_y))
      {
        std::cerr << "bad --start values" << std::endl;
        return false;
      }
      options.has_start = true;
      i += 2;
    } else if (argument == "--goal" && i + 2 < argc) {
      if (!parseNumber(argv[i + 1], options.goal_x) ||
        !parseNumber(argv[i + 2], options.goal_y))
      {
        std::cerr << "bad --goal values" << std::endl;
        return false;
      }
      options.has_goal = true;
      i += 2;
    } else {
      std::cerr << "unknown argument: " << argument << std::endl;
      printUsage();
      return false;
    }
  }
  return true;
}

bool pickAutoEndpoints(
  const astar_planner::GridMap & map, int clearance,
  int & start_cx, int & start_cy, int & goal_cx, int & goal_cy)
{
  int best_low_score = 1e9;
  int best_high_score = -1e9;
  bool low_found = false;
  bool high_found = false;

  for (int cy = clearance; cy < map.height - clearance; ++cy) {
    for (int cx = clearance; cx < map.width - clearance; ++cx) {
      if (!map.isFreeWithClearance(cx, cy, clearance)) {
        continue;
      }
      const int score = cx + cy;
      if (score < best_low_score) {
        best_low_score = score;
        start_cx = cx;
        start_cy = cy;
        low_found = true;
      }
      if (score > best_high_score) {
        best_high_score = score;
        goal_cx = cx;
        goal_cy = cy;
        high_found = true;
      }
    }
  }

  return low_found && high_found;
}

bool writePgmWithPaths(
  const astar_planner::GridMap & map,
  const std::vector<std::vector<std::pair<int, int>>> & paths,
  const std::string & output_path)
{
  std::vector<unsigned char> pixels(
    static_cast<size_t>(map.width) * static_cast<size_t>(map.height), 205);

  for (int cy = 0; cy < map.height; ++cy) {
    for (int cx = 0; cx < map.width; ++cx) {
      const size_t index = static_cast<size_t>(cy) * static_cast<size_t>(map.width) +
        static_cast<size_t>(cx);
      pixels[index] = map.isFree(cx, cy) ? 254 : 0;
    }
  }

  // Drawn in reverse order, so the first heuristic in the list ends up on top.
  for (size_t p = paths.size(); p > 0; --p) {
    const std::vector<std::pair<int, int>> & path = paths[p - 1];
    for (size_t i = 0; i < path.size(); ++i) {
      const int cx = path[i].first;
      const int cy = path[i].second;
      // PGM row 0 is the top of the image, map row 0 is the bottom.
      const int row = map.height - 1 - cy;
      if (row < 0 || row >= map.height || cx < 0 || cx >= map.width) {
        continue;
      }
      const size_t index = static_cast<size_t>(row) * static_cast<size_t>(map.width) +
        static_cast<size_t>(cx);
      pixels[index] = 100;
    }
  }

  std::ofstream out(output_path, std::ios::binary);
  if (!out.good()) {
    return false;
  }
  out << "P5\n# generated by astar_planner\n" << map.width << " " << map.height << "\n255\n";
  out.write(
    reinterpret_cast<const char *>(pixels.data()),
    static_cast<std::streamsize>(pixels.size()));
  return out.good();
}

nav_msgs::msg::Path buildPathMessage(
  const astar_planner::GridMap & map,
  const std::vector<std::pair<int, int>> & path,
  const std::string & frame_id)
{
  nav_msgs::msg::Path message;
  message.header.frame_id = frame_id;
  message.header.stamp = rclcpp::Clock().now();
  message.poses.reserve(path.size());

  for (size_t i = 0; i < path.size(); ++i) {
    double wx = 0.0;
    double wy = 0.0;
    map.cellToWorld(path[i].first, path[i].second, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = message.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.05;
    pose.pose.orientation.w = 1.0;
    message.poses.push_back(pose);
  }

  return message;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  if (!parseArguments(argc, argv, options)) {
    return 1;
  }

  astar_planner::GridMap map;
  std::string error;
  if (!astar_planner::loadMapFromYaml(options.map_path, map, error)) {
    std::cerr << "failed to load map: " << error << std::endl;
    return 1;
  }

  std::cout << "map   : " << options.map_path << std::endl;
  std::cout << "grid  : " << map.width << " x " << map.height
            << " @ " << map.resolution << " m/cell" << std::endl;
  std::cout << "origin: [" << map.origin_x << ", " << map.origin_y << "]" << std::endl;

  int start_cx = 0;
  int start_cy = 0;
  int goal_cx = 0;
  int goal_cy = 0;

  if (options.has_start && options.has_goal) {
    if (!map.worldToCell(options.start_x, options.start_y, start_cx, start_cy)) {
      std::cerr << "start pose is outside the map" << std::endl;
      return 1;
    }
    if (!map.worldToCell(options.goal_x, options.goal_y, goal_cx, goal_cy)) {
      std::cerr << "goal pose is outside the map" << std::endl;
      return 1;
    }
  } else {
    if (!pickAutoEndpoints(map, 3, start_cx, start_cy, goal_cx, goal_cy)) {
      std::cerr << "no pair of free cells found" << std::endl;
      return 1;
    }
  }

  double start_wx = 0.0;
  double start_wy = 0.0;
  double goal_wx = 0.0;
  double goal_wy = 0.0;
  map.cellToWorld(start_cx, start_cy, start_wx, start_wy);
  map.cellToWorld(goal_cx, goal_cy, goal_wx, goal_wy);

  std::cout << "start : [" << start_wx << ", " << start_wy << "]"
            << "  cell (" << start_cx << ", " << start_cy << ")" << std::endl;
  std::cout << "goal  : [" << goal_wx << ", " << goal_wy << "]"
            << "  cell (" << goal_cx << ", " << goal_cy << ")" << std::endl;

  constexpr size_t kHeuristicCount = 4;
  const astar_planner::HeuristicType types[kHeuristicCount] = {
    astar_planner::HeuristicType::DIJKSTRA,
    astar_planner::HeuristicType::MANHATTAN,
    astar_planner::HeuristicType::EUCLIDEAN,
    astar_planner::HeuristicType::OCTILE,
  };

  astar_planner::AStarPlanner planner;
  planner.setMap(map);

  std::vector<astar_planner::PlanStats> results;
  std::vector<std::vector<std::pair<int, int>>> paths;

  for (size_t i = 0; i < kHeuristicCount; ++i) {
    planner.setHeuristic(types[i]);
    const astar_planner::PlanStats stats =
      planner.plan(start_cx, start_cy, goal_cx, goal_cy);
    results.push_back(stats);
    paths.push_back(planner.path());
  }

  const astar_planner::PlanStats & baseline = results[0];

  std::cout << std::endl;
  std::cout << "heuristic          expanded  vs dijkstra  path pts  length[m]  time[ms]"
            << std::endl;
  std::cout << "---------------------------------------------------------------------"
            << std::endl;
  for (size_t i = 0; i < kHeuristicCount; ++i) {
    const astar_planner::PlanStats & stats = results[i];
    char reduction[32] = "-";
    if (i > 0 && baseline.expanded_nodes > 0 && stats.expanded_nodes > 0) {
      const double ratio = 100.0 * (1.0 - static_cast<double>(stats.expanded_nodes) /
        static_cast<double>(baseline.expanded_nodes));
      std::snprintf(reduction, sizeof(reduction), "%.1f%%", ratio);
    }
    std::printf(
      "%-17s %9zu %11s %9zu %10.3f %9.2f\n",
      astar_planner::heuristicName(types[i]),
      stats.expanded_nodes,
      reduction,
      stats.path_cells,
      stats.path_length,
      stats.elapsed_ms);
  }
  std::cout << std::endl;

  for (size_t i = 0; i < kHeuristicCount; ++i) {
    if (!results[i].found) {
      std::cerr << "search failed to reach the goal with "
                << astar_planner::heuristicName(types[i]) << std::endl;
      return 1;
    }
  }

  if (!options.out_pgm.empty()) {
    if (writePgmWithPaths(map, paths, options.out_pgm)) {
      std::cout << "wrote " << options.out_pgm << std::endl;
    } else {
      std::cerr << "failed to write " << options.out_pgm << std::endl;
      return 1;
    }
  }

  if (!options.publish) {
    return 0;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("astar_demo");
  auto path_publisher = node->create_publisher<nav_msgs::msg::Path>("astar/path", 10);
  auto marker_publisher =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("astar/markers", 10);

  const std::string frame_id = "map";
  const nav_msgs::msg::Path euclidean_path = buildPathMessage(map, paths[2], frame_id);

  visualization_msgs::msg::MarkerArray markers;
  const double colors[4][3] = {
    {0.1, 0.3, 1.0},   // dijkstra: blue
    {1.0, 0.2, 0.2},   // manhattan: red
    {0.1, 1.0, 0.2},   // euclidean: green
    {0.6, 0.2, 0.9},   // octile: purple
  };

  for (size_t i = 0; i < kHeuristicCount; ++i) {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = frame_id;
    line.header.stamp = node->now();
    line.ns = "astar";
    line.id = static_cast<int>(i);
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.03;
    line.pose.orientation.w = 1.0;
    line.color.r = static_cast<float>(colors[i][0]);
    line.color.g = static_cast<float>(colors[i][1]);
    line.color.b = static_cast<float>(colors[i][2]);
    line.color.a = 0.9f;

    line.points.reserve(paths[i].size());
    for (size_t k = 0; k < paths[i].size(); ++k) {
      double wx = 0.0;
      double wy = 0.0;
      map.cellToWorld(paths[i][k].first, paths[i][k].second, wx, wy);
      geometry_msgs::msg::Point point;
      point.x = wx;
      point.y = wy;
      point.z = 0.05;
      line.points.push_back(point);
    }
    markers.markers.push_back(line);
  }

  visualization_msgs::msg::Marker start_marker;
  start_marker.header.frame_id = frame_id;
  start_marker.header.stamp = node->now();
  start_marker.ns = "astar_points";
  start_marker.id = 10;
  start_marker.type = visualization_msgs::msg::Marker::SPHERE;
  start_marker.action = visualization_msgs::msg::Marker::ADD;
  start_marker.scale.x = 0.2;
  start_marker.scale.y = 0.2;
  start_marker.scale.z = 0.2;
  start_marker.pose.position.x = start_wx;
  start_marker.pose.position.y = start_wy;
  start_marker.pose.position.z = 0.05;
  start_marker.pose.orientation.w = 1.0;
  start_marker.color.r = 0.0f;
  start_marker.color.g = 1.0f;
  start_marker.color.b = 1.0f;
  start_marker.color.a = 1.0f;
  markers.markers.push_back(start_marker);

  visualization_msgs::msg::Marker goal_marker = start_marker;
  goal_marker.id = 11;
  goal_marker.pose.position.x = goal_wx;
  goal_marker.pose.position.y = goal_wy;
  goal_marker.color.r = 1.0f;
  goal_marker.color.g = 0.6f;
  goal_marker.color.b = 0.0f;
  markers.markers.push_back(goal_marker);

  std::cout << "publishing /astar/path and /astar/markers for 8 s "
            << "(open rviz with fixed frame = map to see them)" << std::endl;

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    path_publisher->publish(euclidean_path);
    marker_publisher->publish(markers);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  rclcpp::shutdown();
  return 0;
}
