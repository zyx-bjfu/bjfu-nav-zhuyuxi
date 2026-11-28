#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include "astar_planner/astar_planner.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2) {
    return 1;
  }

  astar_planner::GridMap map;
  std::string error;
  if (!astar_planner::loadMapFromYaml(argv[1], map, error)) {
    std::cerr << "load failed: " << error << std::endl;
    return 1;
  }

  astar_planner::AStarPlanner planner;
  planner.setMap(map);

  constexpr int kN = 4;
  const astar_planner::HeuristicType types[kN] = {
    astar_planner::HeuristicType::DIJKSTRA,
    astar_planner::HeuristicType::MANHATTAN,
    astar_planner::HeuristicType::EUCLIDEAN,
    astar_planner::HeuristicType::OCTILE,
  };

  int pairs = 0;
  int suboptimal[kN] = {0, 0, 0, 0};
  double worst_gap[kN] = {0.0, 0.0, 0.0, 0.0};
  long long sum_expanded[kN] = {0, 0, 0, 0};

  const int step = 24;
  for (int sy = 6; sy < map.height - 6; sy += step) {
    for (int sx = 6; sx < map.width - 6; sx += step) {
      if (!map.isFreeWithClearance(sx, sy, 3)) {
        continue;
      }
      for (int gy = 6; gy < map.height - 6; gy += step) {
        for (int gx = 6; gx < map.width - 6; gx += step) {
          if (!map.isFreeWithClearance(gx, gy, 3)) {
            continue;
          }
          if (sx == gx && sy == gy) {
            continue;
          }

          astar_planner::PlanStats results[kN];
          for (int i = 0; i < kN; ++i) {
            planner.setHeuristic(types[i]);
            results[i] = planner.plan(sx, sy, gx, gy);
          }
          if (!results[0].found) {
            continue;
          }
          pairs++;

          const double base = results[0].path_length;
          for (int i = 1; i < kN; ++i) {
            sum_expanded[i] += static_cast<long long>(results[i].expanded_nodes);
            const double gap = results[i].path_length - base;
            if (gap > 1e-6) {
              suboptimal[i]++;
              if (gap > worst_gap[i]) {
                worst_gap[i] = gap;
              }
            }
          }
          sum_expanded[0] += static_cast<long long>(results[0].expanded_nodes);

          if (std::fabs(results[1].path_length - base) > 1e-6) {
            std::printf(
              "%-5d %-5d %-5d %-5d %8zu %8zu %8zu %8zu %8.3f %8.3f %8.3f %8.3f\n",
              sx, sy, gx, gy,
              results[0].expanded_nodes, results[1].expanded_nodes,
              results[2].expanded_nodes, results[3].expanded_nodes,
              results[0].path_length, results[1].path_length,
              results[2].path_length, results[3].path_length);
          }
        }
      }
    }
  }

  std::printf("\npairs=%d\n", pairs);
  std::printf("%-17s %14s %13s %11s %10s\n",
    "heuristic", "sum_expanded", "avg_expanded", "suboptimal", "worst[m]");
  for (int i = 0; i < kN; ++i) {
    std::printf("%-17s %14lld %13.1f %11d %10.3f\n",
      astar_planner::heuristicName(types[i]),
      sum_expanded[i],
      pairs > 0 ? static_cast<double>(sum_expanded[i]) / pairs : 0.0,
      suboptimal[i], worst_gap[i]);
  }
  return 0;
}
