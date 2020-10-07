#pragma once
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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/bind.hpp>
#include "common.h"
#include "map.h"
#include "path_planning.h"
#include "pid_controller.h"

class CommunicationInterface
{
private:
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

  // local map, publishers
  Map map_local_;
  ros::Publisher pub_local_map_quadrant_1_;
  ros::Publisher pub_local_map_quadrant_2_;
  ros::Publisher pub_local_map_quadrant_3_;
  ros::Publisher pub_local_map_quadrant_4_;

  // planned path
  ros::Publisher pub_path_;

  ros::Publisher pub_local_path_;

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

  std::vector<Point2DWithFloat> local_path_vec_;

  visualization_msgs::Marker path_line_;
  visualization_msgs::Marker local_path_line_;

  // check ros time
  ros::Time begin_;
  ros::Time end_;

public:
  CommunicationInterface(ros::NodeHandle &nh);

  /**
   * @brief get goal position from rviz
   * @param input goal: manually set goal position
   */
  void setGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal);

  /**
   * @brief plan a new path with input global map data
   * @param input map: global grid map data
   */
  void setPath(const Map &map);

  /**
   * @brief plan a new path with input local map data
   * @param input map_local: local grid map data
   */
  void setLocalPath(const Map &map_local);

  /**
   * @brief check if the planned path is blocked in the given map
   * @param input map: global map that contains information about obstacles
   * @return bool: the planned path is blocked or not
   */
  bool checkIfPathBlocked(Map &map);

  /**
   * @brief check if a straight line is blocked in the given map
   * @param input map: global map that contains information about obstacles
   * @param input first_path_pt: the start point of the given line
   * @param input second_path_pt: the end point of the given line
   * @param input resolution: resolution of the map
   * @return bool: the given line is blocked or not
   */
  bool checkIfLineBlocked(Map &map, Point2DWithFloat first_path_pt, Point2DWithFloat second_path_pt, float resolution);

  /**
   * @brief publish the planned path as a rostopic
   * @param input path: planned path to be published
   */
  void publishPlannedPath(std::vector<Point2DWithFloat> path);

  /**
   * @brief publish the planned local path as a rostopic
   * @param input path: planned local path to be published
   */
  void publishPlannedLocalPath(std::vector<Point2DWithFloat> path);

  /**
   * @brief save the synchronized input laserscan data and odometry data
   * @param input scan: sensor information from lidar
   * @param input odom: state information from robot
   */
  void scanOdomCallback(const sensor_msgs::LaserScan::ConstPtr &scan, const nav_msgs::Odometry::ConstPtr &odom);

  /**
   * @brief filter the sensor data based on angle, transform the laser point from polar to cartesian & from local to
   * global coordinate system
   */
  void processScan();

  /**
   * @brief save the robot odometry information
   * @param input odom: state information of robot
   */
  void robotPositionCallback(const nav_msgs::Odometry::ConstPtr &odom);

  /**
   * @brief process the robot odometry and adjust the heading and speed using pid controllers so that the robot goes
   * along the planned path
   */
  void processOdom();

  /**
   * @brief publish the control information for robot
   */
  void publishTwist();

  /**
   * @brief publish the global map message
   * @param input map: global map data
   */
  void publishGlobalMap(Map &map);

  /**
   * @brief publish the local map message
   */
  void publishLocalMap();

  /**
   * @brief main algorithm
   * @param input map: the current global map
   */
  void cycle(Map &map);
};
