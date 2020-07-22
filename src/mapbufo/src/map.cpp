#include "map.h"

nav_msgs::OccupancyGrid &Map::GetMap() { return map_; }

CellOccupied Map::GetCell(int x, int y) {
  int index = y * map_.info.width + x;
  int value = map_.data[index];
  if (value == -1) {
    return CellOccupied::unknown;
  }
  if (value >= occupied_bound) {
    return CellOccupied::occupied;
  }
  if (value <= empty_bound) {
    return CellOccupied::empty;
  }
  return CellOccupied::grey;
}

CellOccupied Map::GetCell(Point2D pos) {
  return GetCell(pos.first, pos.second);
}

status::status Map::Update(int x, int y, int update_value) {
  // check if new point pos is not in the old map
  if (x >= static_cast<int>(map_.info.width)) {
    for (int i = map_.info.height; i >= 1; i--) {
      map_.data.insert(begin(map_.data) + i * map_.info.width,
                       x - map_.info.width + 1, -1);
    }
    map_.info.width = x + 1;
  }
  if (y >= static_cast<int>(map_.info.height)) {
    map_.data.insert(end(map_.data),
                     (y - map_.info.height + 1) * map_.info.width, -1);
    map_.info.height = y + 1;
  }
  if (x < 0) {
    for (int i = map_.info.height - 1; i >= 0; i--) {
      map_.data.insert(begin(map_.data) + i * map_.info.width, abs(x), -1);
    }
    map_.info.width += abs(x);
    // TODO(YiLuo) : Check if this is right!
    map_.info.origin.position.x -= abs(x) * map_.info.resolution;
    x = 0;
  }
  if (y < 0) {
    map_.data.insert(begin(map_.data), abs(y) * map_.info.width, -1);
    map_.info.height += abs(y);
    map_.info.origin.position.y -= abs(y) * map_.info.resolution;
    y = 0;
  }

  auto data = &map_.data[y * map_.info.width + x];
  if (*data == -1) {
    *data = 50;
  }
  if (update_value > 0) {
    *data = std::min(*data + update_value, 100);
  }
  if (update_value < 0) {
    *data = std::max(*data + update_value, 0);
  }
  return status::Ok;
}

status::status Map::Update(Point2D pos, int update_value) {
  return Update(pos.first, pos.second, update_value);
}

status::status Map::UpdateWithScanPoint(float x0, float y0, float x1, float y1,
                                        int update_value) {
  Point2D robot_point = TransformIndex(x0, y0);
  Point2D scan_point = TransformIndex(x1, y1);
  Update(scan_point, update_value);
  if (x1 < 0) {
    robot_point.first -= scan_point.first;
    scan_point.first = 0;
  }
  if (y1 < 0) {
    robot_point.second -= scan_point.second;
    scan_point.second = 0;
  }
  Update(scan_point, update_value);
  std::vector<Point2D> scan_line =
      GetLine(robot_point.first, robot_point.second, scan_point.first,
              scan_point.second);
  for (auto point : scan_line) {
    Update(point, -abs(update_value));
  }
  return status::Ok;
}

std::vector<Point2D> Map::GetLineLow(int x0, int y0, int x1, int y1) {
  int d_x = x1 - x0;
  int d_y = y1 - y0;
  int y_step = 1;
  if (d_y < 0) {
    y_step = -1;
    d_y = -d_y;
  }
  int D = 2 * d_y - d_x;
  int x = x0;
  int y = y0;
  std::vector<Point2D> res;
  Point2D curr({x, y});
  while (x < x1) {
    if (D > 0) {
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

std::vector<Point2D> Map::GetLineHigh(int x0, int y0, int x1, int y1) {
  int d_x = x1 - x0;
  int d_y = y1 - y0;
  int x_step = 1;
  if (d_x < 0) {
    x_step = -1;
    d_x = -d_x;
  }
  int D = 2 * d_x - d_y;
  int x = x0;
  int y = y0;
  std::vector<Point2D> res;
  Point2D curr({x, y});
  while (y < y1) {
    if (D > 0) {
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

std::vector<Point2D> Map::GetLine(int x0, int y0, int x1, int y1) {
  if (abs(y1 - y0) < abs(x1 - x0)) {
    if (x0 > x1) {
      return GetLineLow(x1, y1, x0, y0);
    }
    return GetLineLow(x0, y0, x1, y1);
  }
  if (y0 > y1) {
    return GetLineHigh(x1, y1, x0, y0);
  }
  return GetLineHigh(x0, y0, x1, y1);
}

status::status Map::Load(std::string path_to_map) {
  // open the map file
  std::ifstream infile;
  infile.open(path_to_map.c_str());

  // fill the map data into the map_
  int tmp;
  while (!infile.eof()) {
    infile >> tmp;
    map_.data.push_back(tmp);
  }
  infile.close();
  return status::Ok;
}

Point2D Map::TransformIndex(float x, float y) {
  Point2D pos;
  if (x >= 0) {
    pos.first = std::ceil(x * 10);
  }
  pos.first = std::floor(x * 10);
  if (y >= 0) {
    pos.second = std::ceil(y * 10);
  }
  pos.second = std::floor(y * 10);
  return pos;
}
