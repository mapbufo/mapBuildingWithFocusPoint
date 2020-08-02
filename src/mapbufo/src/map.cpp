#include "map.h"

Map::Map(ros::NodeHandle &nh) : nh_(nh)
{
  nav_msgs::OccupancyGrid map_tmp;
  // set the frame as default "map"
  map_tmp.header.frame_id = "/odom";
  // 10m x 10m, with resolution 10cm
  map_tmp.info.resolution = 0.1;
  map_tmp.info.height = 100;
  map_tmp.info.width = 100;
  map_tmp.info.origin.position.x = 0.0;
  map_tmp.info.origin.position.y = 0.0;
  map_tmp.info.origin.position.z = 0.0;
  map_tmp.info.origin.orientation.x = 0.0;
  map_tmp.info.origin.orientation.y = 0.0;
  map_tmp.info.origin.orientation.z = 0.0;
  map_tmp.info.origin.orientation.w = 1.0;
  // initialize map_data to -1
  // use row-major to save the data, index = y*width + x
  map_tmp.data.insert(begin(map_tmp.data),
                      map_tmp.info.width * map_tmp.info.height, -1);

  // 1. quadrant
  maps_.push_back(map_tmp);

  tf2::Quaternion q;
  // 2. quadrant
  q.setRPY(0, 0, M_PI / 2);
  map_tmp.info.origin.orientation.x = q.getX();
  map_tmp.info.origin.orientation.y = q.getY();
  map_tmp.info.origin.orientation.z = q.getZ();
  map_tmp.info.origin.orientation.w = q.getW();
  maps_.push_back(map_tmp);

  // 3. quadrant
  q.setRPY(0, 0, M_PI);
  map_tmp.info.origin.orientation.x = q.getX();
  map_tmp.info.origin.orientation.y = q.getY();
  map_tmp.info.origin.orientation.z = q.getZ();
  map_tmp.info.origin.orientation.w = q.getW();
  maps_.push_back(map_tmp);

  // 4. quadrant
  q.setRPY(0, 0, -M_PI / 2);
  map_tmp.info.origin.orientation.x = q.getX();
  map_tmp.info.origin.orientation.y = q.getY();
  map_tmp.info.origin.orientation.z = q.getZ();
  map_tmp.info.origin.orientation.w = q.getW();
  maps_.push_back(map_tmp);

  // prepare the parameter
  GetParam();
}

Map::Map(ros::NodeHandle &nh, float resolution, int width, int height) : nh_(nh)
{
  nav_msgs::OccupancyGrid map_tmp;
  // set the frame as default "map"
  map_tmp.header.frame_id = "/base_footprint";
  // 10m x 10m, with resolution 10cm
  map_tmp.info.resolution = resolution;
  map_tmp.info.height = width;
  map_tmp.info.width = height;
  map_tmp.info.origin.position.x = 0.0;
  map_tmp.info.origin.position.y = 0.0;
  map_tmp.info.origin.position.z = 0.0;
  map_tmp.info.origin.orientation.x = 0.0;
  map_tmp.info.origin.orientation.y = 0.0;
  map_tmp.info.origin.orientation.z = 0.0;
  map_tmp.info.origin.orientation.w = 1.0;
  // initialize map_data to -1
  // use row-major to save the data, index = y*width + x
  map_tmp.data.insert(begin(map_tmp.data),
                      map_tmp.info.width * map_tmp.info.height, -1);

  // 1. quadrant
  maps_.push_back(map_tmp);

  tf2::Quaternion q;
  // 2. quadrant
  q.setRPY(0, 0, M_PI / 2);
  map_tmp.info.origin.orientation.x = q.getX();
  map_tmp.info.origin.orientation.y = q.getY();
  map_tmp.info.origin.orientation.z = q.getZ();
  map_tmp.info.origin.orientation.w = q.getW();
  maps_.push_back(map_tmp);

  // 3. quadrant
  q.setRPY(0, 0, M_PI);
  map_tmp.info.origin.orientation.x = q.getX();
  map_tmp.info.origin.orientation.y = q.getY();
  map_tmp.info.origin.orientation.z = q.getZ();
  map_tmp.info.origin.orientation.w = q.getW();
  maps_.push_back(map_tmp);

  // 4. quadrant
  q.setRPY(0, 0, -M_PI / 2);
  map_tmp.info.origin.orientation.x = q.getX();
  map_tmp.info.origin.orientation.y = q.getY();
  map_tmp.info.origin.orientation.z = q.getZ();
  map_tmp.info.origin.orientation.w = q.getW();
  maps_.push_back(map_tmp);

  // prepare the parameter
  GetParam();
}

void Map::ResetMapData()
{
  for (auto &map : maps_)
  {
    map.data.assign(maps_.front().info.height * maps_.front().info.width, -1);
  }
}

CellOccupied Map::GetCell(int x, int y)
{
  int x_qua;
  int y_qua;
  int qua;
  TransformPositionIntoQuadrant(x, y, x_qua, y_qua, qua);
  return GetCellInSingleQuadrant(x_qua, y_qua, qua);
}

CellOccupied Map::GetCell(Point2D pos)
{
  return GetCell(pos.first, pos.second);
}

status::status Map::Update(int x, int y, int update_value, bool global)
{
  int x_qua, y_qua, qua;
  TransformPositionIntoQuadrant(x, y, x_qua, y_qua, qua);
  return UpdateCellInSingleQuadrant(x_qua, y_qua, qua, update_value, global);
}

status::status Map::Update(Point2D pos, int update_value, bool global)
{
  return Update(pos.first, pos.second, update_value, global);
}

status::status Map::UpdateWithScanPoint(float x0, float y0, float x1, float y1,
                                        int update_value, bool global)
{
  Point2D robot_point = TransformIndex(x0, y0, maps_.front().info.resolution);
  Point2D scan_point = TransformIndex(x1, y1, maps_.front().info.resolution);
  Update(scan_point, update_value, global);
  if (x0 == x1 && y0 == y1)
  {
    return status::Ok;
  }
  std::vector<Point2D> scan_line =
      GetLine(robot_point.first, robot_point.second, scan_point.first,
              scan_point.second);
  for (auto point : scan_line)
  {
    Update(point, -abs(update_value), global);
  }
  return status::Ok;
}

status::status Map::UpdateWithScanPoints(Point2DWithFloat robot_pos, ScanPointsFloatWithUpdateValue curr_scan)
{
  for (auto point : curr_scan)
  {
    UpdateWithScanPoint(robot_pos.first, robot_pos.second,
                        point.first.first, point.first.second, point.second, true);
  }
  return status::Ok;
}

status::status Map::UpdateLocalMapWithScanPoints(Point2DWithFloat robot_pos,
                                                 ScanPointsFloatWithUpdateValue curr_scan)
{
  ResetMapData();

  for (auto point : curr_scan)
  {
    UpdateWithScanPoint(robot_pos.first, robot_pos.second,
                        point.first.first, point.first.second, point.second, false);
  }
  return status::Ok;
}

std::vector<Point2D> Map::GetLineLow(int x0, int y0, int x1, int y1)
{
  int d_x = x1 - x0;
  int d_y = y1 - y0;
  int y_step = 1;
  if (d_y < 0)
  {
    y_step = -1;
    d_y = -d_y;
  }
  int D = 2 * d_y - d_x;
  int x = x0;
  int y = y0;
  std::vector<Point2D> res;
  Point2D curr({x, y});
  while (x < x1)
  {
    if (D > 0)
    {
      y += y_step;
      D -= 2 * d_x;
    }
    D += 2 * d_y;
    x += 1;
    curr.first = x;
    curr.second = y;
    res.insert(begin(res), curr);
  }
  res.erase(begin(res));
  return res;
}

std::vector<Point2D> Map::GetLineHigh(int x0, int y0, int x1, int y1)
{
  int d_x = x1 - x0;
  int d_y = y1 - y0;
  int x_step = 1;
  if (d_x < 0)
  {
    x_step = -1;
    d_x = -d_x;
  }
  int D = 2 * d_x - d_y;
  int x = x0;
  int y = y0;
  std::vector<Point2D> res;
  Point2D curr({x, y});
  while (y < y1)
  {
    if (D > 0)
    {
      x += x_step;
      D -= 2 * d_y;
    }
    D += 2 * d_x;
    y += 1;
    curr.first = x;
    curr.second = y;
    res.insert(begin(res), curr);
  }
  res.erase(begin(res));
  return res;
}

std::vector<Point2D> Map::GetLine(int x0, int y0, int x1, int y1)
{
  if (abs(y1 - y0) < abs(x1 - x0))
  {
    if (x0 > x1)
    {
      return GetLineLow(x1, y1, x0, y0);
    }
    return GetLineLow(x0, y0, x1, y1);
  }
  if (y0 > y1)
  {
    return GetLineHigh(x1, y1, x0, y0);
  }
  return GetLineHigh(x0, y0, x1, y1);
}

void Map::TransformPositionIntoQuadrant(int x, int y, int &x_qua, int &y_qua,
                                        int &qua)
{

  // find out in which quadrant is this point, then transform the position
  if (y >= 0)
  {
    // 1. quadrant
    if (x >= 0)
    {
      x_qua = abs(x);
      y_qua = abs(y);
      qua = 0;
    }
    else
    {
      // 2. quadrant
      x_qua = abs(y);
      y_qua = abs(x) - 1;
      qua = 1;
    }
  }
  else
  {
    // 3. quadrant
    if (x < 0)
    {
      x_qua = abs(x) - 1;
      y_qua = abs(y) - 1;
      qua = 2;
    }
    else
    {
      // 4. quadrant
      x_qua = abs(y) - 1;
      y_qua = abs(x);
      qua = 3;
    }
  }
}

CellOccupied Map::GetCellInSingleQuadrant(int x, int y, int qua)
{
  int index = y * maps_[qua].info.width + x;
  // if this point is not in the map
  if (index >= maps_[qua].data.size())
  {
    return CellOccupied::unknown;
  }
  int value = maps_[qua].data[index];
  if (value == -1)
  {
    return CellOccupied::unknown;
  }
  if (value >= occupied_bound_)
  {
    return CellOccupied::occupied;
  }
  if (value <= empty_bound_)
  {
    return CellOccupied::empty;
  }
  return CellOccupied::grey;
}

status::status Map::UpdateCellInSingleQuadrant(int x, int y, int qua,
                                               int value, bool global)
{
  // if this is local map, then skip all the points outside the map
  // check if new point pos is not in the old map
  // if not, then extend the map
  if (x >= static_cast<int>(maps_[qua].info.width))
  {
    if (!global)
    {
      return status::Ok;
    }
    for (int i = maps_[qua].info.height; i >= 1; i--)
    {
      maps_[qua].data.insert(begin(maps_[qua].data) + i * maps_[qua].info.width,
                             x - maps_[qua].info.width + 1, -1);
    }
    maps_[qua].info.width = x + 1;
  }
  if (y >= static_cast<int>(maps_[qua].info.height))
  {
    if (!global)
    {
      return status::Ok;
    }
    maps_[qua].data.insert(
        end(maps_[qua].data),
        (y - maps_[qua].info.height + 1) * maps_[qua].info.width, -1);
    maps_[qua].info.height = y + 1;
  }
  // update the probability
  auto data = &maps_[qua].data[y * maps_[qua].info.width + x];
  if (*data == -1)
  {
    *data = 50;
  }
  if (value > 0)
  {
    *data = std::min(*data + value, 100);
  }
  if (value < 0)
  {
    *data = std::max(*data + value, 0);
  }
  return status::Ok;
}

void Map::GetParam()
{
  nh_.getParam("/mapbufo_node/lidar_range", lidar_range_);
  nh_.getParam("/mapbufo_node/empty_bound", empty_bound_);
  nh_.getParam("/mapbufo_node/occupied_bound", occupied_bound_);
}
