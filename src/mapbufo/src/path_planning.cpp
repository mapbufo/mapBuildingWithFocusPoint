#include "path_planning.h"
namespace PathPlanning
{
  // 4 points in left, right, up and down
  std::vector<Point2D> GetNeighbors(Point2D current, Map map, bool global)
  {
    std::vector<Point2D> neighbors;
    //if (global)
    {
      bool none_occupied(true);
      // left
      for (int i = -3; i < 4; i++)
      {
        if (map.GetCell(Point2D(current.first - 4, current.second + i)) == CellOccupied::occupied)
        {
          none_occupied = false;
          break;
        }
      }
      if (none_occupied)
      {
        neighbors.push_back(Point2D(current.first - 1, current.second));
      }
      // right
      none_occupied = true;
      for (int i = -3; i < 4; i++)
      {
        if (map.GetCell(Point2D(current.first + 4, current.second + i)) == CellOccupied::occupied)
        {
          none_occupied = false;
          break;
        }
      }
      if (none_occupied)
      {
        neighbors.push_back(Point2D(current.first + 1, current.second));
      }
      // down
      none_occupied = true;
      for (int i = -3; i < 4; i++)
      {
        if (map.GetCell(Point2D(current.first + i, current.second - 4)) == CellOccupied::occupied)
        {
          none_occupied = false;
          break;
        }
      }
      if (none_occupied)
      {
        neighbors.push_back(Point2D(current.first, current.second - 1));
      }
      // up
      none_occupied = true;
      for (int i = -3; i < 4; i++)
      {
        if (map.GetCell(Point2D(current.first + i, current.second + 4)) == CellOccupied::occupied)
        {
          none_occupied = false;
          break;
        }
      }
      if (none_occupied)
      {
        neighbors.push_back(Point2D(current.first, current.second + 1));
      }
    } /*
    else
    {
      std::cerr << "path planning local" << std::endl;
      // left
      bool all_empty(true);
      for (int i = -3; i < 4; i++)
      {
        if (map.GetCell(Point2D(current.first - 4, current.second + i)) == CellOccupied::occupied)
        {
          all_empty = false;
          break;
        }
      }
      if (all_empty)
      {
        neighbors.push_back(Point2D(current.first - 1, current.second));
      }
      // right
      all_empty = true;
      for (int i = -3; i < 4; i++)
      {
        if (map.GetCell(Point2D(current.first + 4, current.second + i)) == CellOccupied::occupied)
        {
          all_empty = false;
          break;
        }
      }
      if (all_empty)
      {
        neighbors.push_back(Point2D(current.first + 1, current.second));
      }
      // down
      all_empty = true;
      for (int i = -3; i < 4; i++)
      {
        if (map.GetCell(Point2D(current.first + i, current.second - 4)) == CellOccupied::occupied)
        {
          all_empty = false;
          break;
        }
      }
      if (all_empty)
      {
        neighbors.push_back(Point2D(current.first, current.second - 1));
      }
      // up
      all_empty = true;
      for (int i = -3; i < 4; i++)
      {
        if (map.GetCell(Point2D(current.first + i, current.second + 4)) == CellOccupied::occupied)
        {
          all_empty = false;
          break;
        }
      }
      if (all_empty)
      {
        neighbors.push_back(Point2D(current.first, current.second + 1));
      }
      //std::cerr << "neighbours.size: " << neighbors.size() << std::endl;
    }*/
    return neighbors;
  }

  // distance in x-achs plus distance in y-achs
  float CalculateCostSoFar(Point2D start_pos, Point2D current)
  {
    return (abs(current.first - start_pos.first) +
            abs(current.second - start_pos.second));
  }

  // use Euclidean distance
  float HeuristicFunction(Point2D next, Point2D end_pos)
  {
    float x_2 = pow(next.first - end_pos.first, 2);
    float y_2 = pow(next.second - end_pos.second, 2);
    return pow(x_2 + y_2, 0.5);
  }

  // calculate the ratio of the first 2-points and the second 2-points,
  // if both ratio are the same, then it means the 3 points are in a same straight
  // line, then delete the middle point.
  // at the end only keeping the turning points in path
  void OptimizePath(std::vector<Point2D> &path, Point2D start_pos)
  {
    for (int i = 0; i < path.size() - 2; i++)
    {
      float ratio_first;
      if (path[i].first == path[i + 1].first)
      {
        ratio_first = 100000;
      }
      else
      {
        ratio_first = (path[i + 1].second - path[i].second) /
                      (path[i + 1].first - path[i].first);
      }
      float ratio_second;
      if (path[i + 1].first == path[i + 2].first)
      {
        ratio_second = 100000;
      }
      else
      {
        ratio_second = (path[i + 2].second - path[i + 1].second) /
                       (path[i + 2].first - path[i + 1].first);
      }
      if (ratio_first == ratio_second)
      {
        path.erase(begin(path) + i + 1);
        i--;
      }
    }
  }

  std::vector<Point2D> PathPlanning(Point2D start_pos, Point2D end_pos,
                                    const Map &map, bool global)
  {
    // result path
    std::vector<Point2D> path;
    // check if start position same as end position
    if (start_pos.first == end_pos.first && start_pos.second == end_pos.second)
    {
      path.push_back(end_pos);
      return path;
    }
    // find all frontier and sort them with priority
    std::vector<Point2DWithFloat> frontier;
    // set the priority of start_position to 0
    Point2DWithFloat start;
    start.point = start_pos;
    start.value = 0;
    frontier.push_back(start);
    // save all explored points with cost so far
    boost::unordered_map<Point2D, float> explored_points;
    explored_points[start_pos] = 0;
    // save the parent of the explored points
    boost::unordered_map<Point2D, Point2D> parents;
    parents[start_pos] = {-666, -666};

    Point2D current;
    // iterate through all points in frontier
    while (!frontier.empty())
    {
      // sort the frontier with priority descending
      std::sort(begin(frontier), end(frontier),
                [](Point2DWithFloat a, Point2DWithFloat b) {
                  return a.value > b.value;
                });
      // choose the point with best priority as current point
      current = frontier.back().point;
      // if this is the end position, then stop the iterating
      if (current == end_pos)
      {
        break;
      }
      // get the neighbor-points
      std::vector<Point2D> neighbors = GetNeighbors(current, map, global);
      // iterate through all the neighbors
      for (auto next : neighbors)
      {
        // caculate the cost so far for this point
        float next_cost =
            explored_points[current] + CalculateCostSoFar(start_pos, next);
        // if this point is already explored and has higher cost than before, then
        // skip this point
        if (explored_points.count(next) == 1)
        {
          if (explored_points[next] <= next_cost)
          {
            continue;
          }
        }
        // else if this point is not explored yet, or has lower cost than before,
        // then save this in explored_points
        explored_points[next] = next_cost;
        float priority = next_cost + HeuristicFunction(next, end_pos);
        // set current point as parent of this next point
        parents[next] = current;
        // also save this next point in frontier with it's priority
        Point2DWithFloat new_frontier;
        new_frontier.point = next;
        new_frontier.value = priority;
        frontier.insert(begin(frontier), new_frontier);
      }
      // delete this current point in frontier
      frontier.pop_back();
    }

    // get the path with the saved parents list
    Point2D path_current_point = end_pos;
    // if there is no path found, then return empty path
    if (!parents.count(path_current_point))
    {
      return path;
    }
    while (path_current_point != start_pos)
    {
      path.insert(begin(path), path_current_point);
      path_current_point = parents[path_current_point];
    }

    // optimize the path
    OptimizePath(path, start_pos);
    return path;
  }
} // namespace PathPlanning