#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "transform_support_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  return 0;
}