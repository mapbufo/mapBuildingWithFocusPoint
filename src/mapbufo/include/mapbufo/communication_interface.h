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
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <boost/bind.hpp>
#include <fstream>
#include <iostream>
#include "common.h"
#include "map.h"
#include "path_planning.h"
#include "pid_controller.h"
class CommunicationInterface
{
private:
  // ros::Subscriber depth_image_subscriber_;
  ros::Subscriber scan_subscriber_;
  ros::Subscriber goal_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Publisher pos_publisher_;
  ros::Publisher control_publisher_;
  ros::Publisher scan_publisher_;
  // global map publishers
  ros::Publisher pub_global_map_quadrant_1_;
  ros::Publisher pub_global_map_quadrant_2_;
  ros::Publisher pub_global_map_quadrant_3_;
  ros::Publisher pub_global_map_quadrant_4_;
  int counter;
  // local map, publishers
  Map map_local_;
  ros::Publisher pub_local_map_quadrant_1_;
  ros::Publisher pub_local_map_quadrant_2_;
  ros::Publisher pub_local_map_quadrant_3_;
  ros::Publisher pub_local_map_quadrant_4_;

  // planned path
  ros::Publisher pub_path_;
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
  message_filters::TimeSynchronizer<sensor_msgs::LaserScan, nav_msgs::Odometry> sync;
  ScanPointsFloatWithUpdateValue curr_scan_;
  ScanPointsFloatWithUpdateValue curr_local_scan_;

  bool reached_pos_ = true;
  Point2DWithFloat curr_robot_pos_;
  Point2DWithFloat estimated_pos_;
  Point2DWithFloat goal_;
  bool angle_setted = false;
  nav_msgs::Odometry input_odom_;
  nav_msgs::Odometry input_odom_at_scan_;
  sensor_msgs::LaserScan input_scan_;
  geometry_msgs::Twist output_twist_;

  Point2DWithFloat final_goal_;
  std::vector<Point2DWithFloat> planned_path_vec_;
  bool new_goal_updated_;

  visualization_msgs::Marker path_line_;

  // depth camera info
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]

  float cam_intr_[9] = {554.254691191187, 0.0, 320.5, 0.0, 554.254691191187, 240.5, 0.0, 0.0, 1.0};

public:
  CommunicationInterface(ros::NodeHandle &nh);

  void setGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal);
  void setPath(const Map map);

  /**
   * publish the planned path
   * @param input std::vector<Point2DWithFloat> path: planned path
   */
  void publishPlannedPath(std::vector<Point2DWithFloat> path);

  /**
   * save the received messages(laserscan and odometry)
   * @param input sensor_msgs::LaserScan scan: sensor information from lidar
   * @param input nav_msgs::Odometry odom: state information from robot
   */
  void scanOdomCallback(const sensor_msgs::LaserScan::ConstPtr &scan, const nav_msgs::Odometry::ConstPtr &odom);

  void depthImageCallback(const sensor_msgs::Image::ConstPtr &depth_img);
  /**
   * filter the sensor data, transform them into global-coordinate
   */
  void processScan();

  /**
   * save the received messages
   * @param input nav_msgs::Odometry odom: state information of robot
   */
  void robotPositionCallback(const nav_msgs::Odometry::ConstPtr &odom);

  /**
   * ???
   */
  void processOdom();

  /**
   * publish the control information for robot
   */
  void publishTwist();

  /**
   * publish the global map message
   * @param input Map map
   */
  void publishGlobalMap(Map &map);

  /**
   * publish the local map message
   */
  void publishLocalMap();

  /**
   * run the main algorithm
   * @param input Map map: the current global map
   */
  void cycle(Map &map);
};
#endif // COMMUNICATION_INTERFACE_H
