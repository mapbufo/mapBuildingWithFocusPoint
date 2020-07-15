#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <map.h>
#include <math.h>
namespace PathPlanning {
struct Point2DWithFloat {
  Point2D point;
  float value;
};

std::vector<Point2D> GetNeighbors(Point2D current, Map map);

float CalculateCostSoFar(Point2D start_pos, Point2D current);

float HeuristicFunction(Point2D next, Point2D end_pos);

void OptimizePath(std::vector<Point2D> &path, Point2D start_pos);

std::vector<Point2D> PathPlanning(Point2D start_pos, Point2D end_pos,
                                  const Map &map);
}  // namespace PathPlanning

#endif  // !PATH_PLANNING_H