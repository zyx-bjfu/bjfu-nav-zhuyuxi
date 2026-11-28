#ifndef ASTAR_PLANNER__ASTAR_PLANNER_HPP_
#define ASTAR_PLANNER__ASTAR_PLANNER_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace astar_planner
{

enum class HeuristicType
{
  DIJKSTRA = 0,
  MANHATTAN = 1,
  EUCLIDEAN = 2,
  OCTILE = 3,
};

const char * heuristicName(HeuristicType type);

struct GridMap
{
  int width = 0;
  int height = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;

  // row-major, index = cy * width + cx, cy = 0 is the bottom row of the map.
  std::vector<int8_t> occupancy;  // 0 = free, 1 = occupied / unknown

  bool worldToCell(double wx, double wy, int & cx, int & cy) const;
  void cellToWorld(int cx, int cy, double & wx, double & wy) const;
  bool isFree(int cx, int cy) const;
  bool isFreeWithClearance(int cx, int cy, int clearance) const;
};

bool loadMapFromYaml(const std::string & yaml_path, GridMap & map, std::string & error);

struct PlanStats
{
  size_t expanded_nodes = 0;
  size_t path_cells = 0;
  double path_length = 0.0;
  double elapsed_ms = 0.0;
  bool found = false;
};

class AStarPlanner
{
public:
  void setMap(const GridMap & map);
  void setHeuristic(HeuristicType type);
  PlanStats plan(int start_cx, int start_cy, int goal_cx, int goal_cy);
  const std::vector<std::pair<int, int>> & path() const { return path_; }

private:
  double heuristic(int cx, int cy, int gx, int gy) const;
  bool inBounds(int cx, int cy) const;

  GridMap map_;
  HeuristicType type_ = HeuristicType::EUCLIDEAN;
  std::vector<std::pair<int, int>> path_;
};

}  // namespace astar_planner

#endif  // ASTAR_PLANNER__ASTAR_PLANNER_HPP_
