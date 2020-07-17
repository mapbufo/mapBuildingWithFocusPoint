#ifndef POINT_H
#define POINT_H

#include <math.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>

class Point2D {
 public:
  typedef std::shared_ptr<Point2D> Ptr;

  int x_;
  int y_;
  int value_;

 public:
  Point2D() {
    x_ = 0;
    y_ = 0;
    value_ = 0;
  }
  Point2D(int x, int y) {
    x_ = x;
    y_ = y;
    value_ = 0;
  }

  ~Point2D() {}

  void setPositionX(int x) { x_ = x; }
  void setPositionY(int y) { y_ = y; }
  void setValue(int val) { value_ = val; }
  int getPositionX() { return x_; }
  int getPositionY() { return y_; }
  int getValue() { return value_; }
};

#endif  // POINT_H
