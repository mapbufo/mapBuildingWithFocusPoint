#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <map.h>
#include <math.h>
namespace PathPlanning
{
  struct Point2DWithFloat
  {
    Point2D point;
    float value;
  };

  std::vector<Point2D> GetNeighbors(Point2D current, Map map, bool global);

  float CalculateCostSoFar(Point2D start_pos, Point2D current);

  float HeuristicFunction(Point2D next, Point2D end_pos);

  void OptimizePath(std::vector<Point2D> &path, Point2D start_pos);

  //TODO(YiLuo) : the path in the empty-area has higher priority than in the
  // grey-area
  std::vector<Point2D> PathPlanning(Point2D start_pos, Point2D end_pos,
                                    const Map &map, bool global);
} // namespace PathPlanning

#endif // !PATH_PLANNING_H