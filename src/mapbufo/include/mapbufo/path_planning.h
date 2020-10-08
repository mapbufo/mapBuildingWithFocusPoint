#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <map.h>
#include <math.h>
namespace PathPlanning
{
  // used in path-planning, saving the needed cost-value for each point
  struct Point2DWithFloat
  {
    Point2D point;
    float value;
  };

  /**
   * @brief calculate the 4 points(up, down, left, right) near the current point,
   * check if these points are available. According to the type of map(local or
   * global) there are different standards for checking
   * @param input current: position of the current point
   * @param input map: the content of the map
   * @param input global: flag for the type of the given map
   * @return vector<Point2D>: list of the available points
   */
  std::vector<Point2D> GetNeighbors(Point2D current, Map map, bool global);

  /**
   * @brief use a simple way to calculate the cost from start-point to current
   * -point, this way is suitable here because the robot goes only vertically
   * and horizontally
   * @param input start_pos: position of the start point
   * @param input current: position of the current point
   * @return float: the calculated cost between start-point and current-point
   */
  float CalculateCostSoFar(Point2D start_pos, Point2D current);

  /**
   * @brief use Euclidean distance as the estimated distance between the current
   * point and end point
   * @param input next: position of the next point
   * @param input end_pos: position of the end point
   * @return float: the estimated cost between next-point and end-point
   */
  float HeuristicFunction(Point2D next, Point2D end_pos);

  /**
   * @brief delete the unnecessary points in the straight line, only save the
   * points at the turning points
   * @param output path: list of the path-points
   */
  void OptimizePath(std::vector<Point2D> &path);

  //TODO(YiLuo) : the path in the empty-area has higher priority than in the
  // grey-area

  /**
   * @brief use A* algorithm to calculate the path
   * @param input start_pos: position of the start point
   * @param input end_pos: position of the current point
   * @param input map: the content of the map
   * @param input global: flag for the type of the given map
   * @return vector<Point2D>: list of the path-points
   */
  std::vector<Point2D> PathPlanning(Point2D start_pos, Point2D end_pos,
                                    const Map &map, bool global);
} // namespace PathPlanning

#endif // !PATH_PLANNING_H