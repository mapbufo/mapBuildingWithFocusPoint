#ifndef COMMUNICATION_INTERFACE_H
#define COMMUNICATION_INTERFACE_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <boost/bind.hpp>
#include "common.h"
#include "map.h"
#include "pid_controller.h"

class CommunicationInterface
{
private:
  ros::Subscriber scan_subscriber_;
  ros::Subscriber pos_subscriber_;
  ros::Publisher pos_publisher_;
  ros::Publisher control_publisher_;
  ros::Publisher scan_publisher_;
  // map publishers
  ros::Publisher pub_map_quadrant_1_;
  ros::Publisher pub_map_quadrant_2_;
  ros::Publisher pub_map_quadrant_3_;
  ros::Publisher pub_map_quadrant_4_;
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
  message_filters::TimeSynchronizer<sensor_msgs::LaserScan, nav_msgs::Odometry> sync;

  boost::unordered_map<std::pair<float, float>, int> curr_scan_;
  bool reached_pos_ = true;
  std::pair<float, float> curr_robot_pos_;
  std::pair<float, float> estimated_pos_;
  std::pair<float, float> goal_;
  bool angle_setted = false;
  nav_msgs::Odometry input_odom_;
  nav_msgs::Odometry input_odom_at_scan_;
  sensor_msgs::LaserScan input_scan_;
  sensor_msgs::PointCloud filtered_point_cloud_;
  geometry_msgs::Twist output_twist_;

public:
  CommunicationInterface(ros::NodeHandle &nh);
  void scanOdomCallback(const sensor_msgs::LaserScan::ConstPtr &scan, const nav_msgs::Odometry::ConstPtr &msg);
  void processScan();
  void publishPointCloud();

  void robotPositionCallback(const nav_msgs::Odometry::Ptr &odom);
  void processOdom();
  void publishTwist();

  void cycle(Map &map);
};
#endif // COMMUNICATION_INTERFACE_H
