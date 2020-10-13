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
  getParam();
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
  getParam();
}

void Map::resetMapData()
{
  for (auto &map : maps_)
  {
    map.data.assign(maps_.front().info.height * maps_.front().info.width, -1);
  }
}

CellOccupied Map::getCell(int x, int y)
{
  int x_qua;
  int y_qua;
  int qua;
  transformPositionIntoQuadrant(x, y, x_qua, y_qua, qua);
  return getCellInSingleQuadrant(x_qua, y_qua, qua);
}

CellOccupied Map::getCell(Point2D pos)
{
  return getCell(pos.first, pos.second);
}

status::status Map::update(int x, int y, int update_value, bool global)
{
  int x_qua, y_qua, qua;
  transformPositionIntoQuadrant(x, y, x_qua, y_qua, qua);
  return updateCellInSingleQuadrant(x_qua, y_qua, qua, update_value, global);
}

status::status Map::update(Point2D pos, int update_value, bool global)
{
  return update(pos.first, pos.second, update_value, global);
}

status::status Map::updateWithScanPoint(float x0, float y0, float x1, float y1,
                                        int update_value, bool global)
{
  Point2D robot_point = transformIndex(x0, y0, maps_.front().info.resolution);
  Point2D scan_point = transformIndex(x1, y1, maps_.front().info.resolution);
  update(scan_point, update_value, global);
  if (x0 == x1 && y0 == y1)
  {
    return status::Ok;
  }
  std::vector<Point2D> scan_line =
      getLine(robot_point.first, robot_point.second, scan_point.first,
              scan_point.second);
  for (auto point : scan_line)
  {
    update(point, -abs(update_value), global);
  }
  return status::Ok;
}

status::status Map::updateWithScanPoints(Point2DWithFloat robot_pos, ScanPointsFloatWithUpdateValue curr_scan)
{
  for (auto point : curr_scan)
  {
    updateWithScanPoint(robot_pos.first, robot_pos.second,
                        point.first.first, point.first.second, point.second, true);
  }
  return status::Ok;
}

status::status Map::updateLocalMapWithScanPoints(Point2DWithFloat robot_pos,
                                                 ScanPointsFloatWithUpdateValue curr_scan)
{
  resetMapData();

  for (auto point : curr_scan)
  {
    updateWithScanPoint(robot_pos.first, robot_pos.second,
                        point.first.first, point.first.second, point.second, false);
  }
  return status::Ok;
}

void Map::transformPositionIntoQuadrant(int x, int y, int &x_qua, int &y_qua,
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

CellOccupied Map::getCellInSingleQuadrant(int x, int y, int qua)
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

status::status Map::updateCellInSingleQuadrant(int x, int y, int qua,
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

void Map::getParam()
{
  nh_.getParam("/mapbufo_node/lidar_range", lidar_range_);
  nh_.getParam("/mapbufo_node/empty_bound", empty_bound_);
  nh_.getParam("/mapbufo_node/occupied_bound", occupied_bound_);
}
