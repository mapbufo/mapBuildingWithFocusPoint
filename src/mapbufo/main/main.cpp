#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <message_filters/subscriber.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <ros/package.h>

#include <ros/rate.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <boost/bind.hpp>
#include <iostream>
#include <iterator>
#include <vector>
#include "common.h"
#include "lidar_sim.h"
#include "map.h"
#include "map_simulator.h"

#include "path_planning.h"
#include "pid_controller.h"

#include "robot.h"
static std::vector<std::pair<float, float>> curr_scan_;
static bool reached_pos_ = true;
static std::pair<float, float> curr_robot_pos_;
static std::pair<float, float> estimated_pos_;
static std::pair<float, float> goal_;
static bool angle_setted = false;
static geometry_msgs::Pose robot_pose_;
class CommunicationInterface
{
public:
  CommunicationInterface()
  {
    pos_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    control_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    scan_publisher_ = nh_.advertise<sensor_msgs::PointCloud>("/processed_scan", 1);

    scan_subscriber_ = nh_.subscribe("/scan", 1, &CommunicationInterface::laserScanCallback, this);
    pos_subscriber_ = nh_.subscribe("/odom", 1, &CommunicationInterface::robotPositionCallback, this);

    // message_filters::Subscriber<sensor_msgs::LaserScan> scan_input(nh_, "/scan", 3);

    // message_filters::Subscriber<nav_msgs::Odometry> odom_input(nh_, "/odom", 3);

    // message_filters::TimeSynchronizer<sensor_msgs::LaserScan, nav_msgs::Odometry> sync(scan_input, odom_input, 3);
    // sync.registerCallback(boost::bind(&this->scanOdomCallBack, _1, _2));
  }

  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
  {
    float angle_min = scan->angle_min;
    float angle_max = scan->angle_max;
    float angle_increment = scan->angle_increment;
    float range_min = scan->range_min;
    float range_max = scan->range_max;
    int number_of_laser = scan->ranges.size();
    // get laser scan id of the mid +- 10 deg
    float priority_angle_range_deg = 10;
    float priority_angle_range_rad = priority_angle_range_deg * M_PI / 180.0;
    // laser scan: 0-----1*****2*****3-----4
    // 0: angle min
    // 1: priority start
    // 2: center
    // 3: priority end
    // 4: angle max

    // process scan
    sensor_msgs::PointCloud point_cloud;
    static int frame_id_ = 0;
    point_cloud.header.frame_id = "/base_footprint";
    point_cloud.header.stamp = ros::Time::now();
    point_cloud.header.seq = frame_id_;
    frame_id_++;

    std::pair<float, float> next_pos(0, 0);
    float max_dist = 0;

    curr_scan_.clear();
    int counter = 0;
    int resolution = 10;  // pick only every ten points outside the interested area
    for (int i = 0; i < number_of_laser; i++)
    {
      float laser_angle = angle_min + i * angle_increment;
      // robot heading: x axis, robot left: y axis
      float x = scan->ranges[i] * std::cos(laser_angle);
      float y = scan->ranges[i] * std::sin(laser_angle);
      // insert
      double pos_x = robot_pose_.position.x;
      double pos_y = robot_pose_.position.y;
      tf::Quaternion q(robot_pose_.orientation.x, robot_pose_.orientation.y, robot_pose_.orientation.z,
                       robot_pose_.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      double robot_heading = yaw;
      if (laser_angle > -priority_angle_range_rad && laser_angle < priority_angle_range_rad)
      {
        if (std::isnan(scan->ranges[i]))  // if a point is nan, then the 10 points before and after must be nan so that
                                          // it can be considered empty
        {
          // check the +- 10 points
          if ((priority_angle_range_rad - abs(laser_angle)) / angle_increment < 10)
          {
            continue;
          }
          else
          {
            bool valid = true;
            for (int j = 0; j < 10; j++)
            {
              if (!std::isnan(scan->ranges[i + j]) || !std::isnan(scan->ranges[i - j]))
              {
                valid = false;
                break;
              }
            }
            if (!valid)
            {
              continue;
            }
          }
          if (range_max > max_dist)
          {
            next_pos.first = range_max * std::cos(laser_angle);
            next_pos.second = range_max * std::sin(laser_angle);
            max_dist = scan->range_max;
          }
          continue;
        }

        // transform estimated_pos_ into absolute pos

        float global_x = x * std::cos(yaw) - y * std::sin(yaw) + pos_x;
        float global_y = x * std::sin(yaw) + y * std::cos(yaw) + pos_y;
        if (!std::isnan(global_x) && !std::isnan(global_y))
        {
          curr_scan_.push_back({ global_x, global_y });
        }
        geometry_msgs::Point32 pt;
        pt.x = x;
        pt.y = y;
        pt.z = 0;

        point_cloud.points.push_back(pt);
        if (scan->ranges[i] > max_dist)
        {
          next_pos.first = x;
          next_pos.second = y;
          max_dist = scan->ranges[i];
        }
      }
      else
      {
        counter++;
        if (counter < resolution)
        {
          continue;
        }
        counter = 0;
        // insert
        if (std::isnan(scan->ranges[i]))
          continue;

        float global_x = x * std::cos(yaw) - y * std::sin(yaw) + pos_x;
        float global_y = x * std::sin(yaw) + y * std::cos(yaw) + pos_y;
        if (!std::isnan(global_x) && !std::isnan(global_y))
        {
          curr_scan_.push_back({ global_x, global_y });
        }
        geometry_msgs::Point32 pt;
        pt.x = x;
        pt.y = y;
        pt.z = 0;

        point_cloud.points.push_back(pt);
      }
    }
    if (reached_pos_)
    {
      estimated_pos_ = next_pos;
      std::cerr << "scan received" << std::endl;
      std::cerr << curr_robot_pos_.first << " " << curr_robot_pos_.second << " " << estimated_pos_.first << " "
                << estimated_pos_.second << std::endl;
    }
    scan_publisher_.publish(point_cloud);
  }

  void robotPositionCallback(const nav_msgs::Odometry msg)
  {
    // ROS_INFO("pos received");
    PIDController pid_angle(1, 0.2, 2.5);
    PIDController pid_speed(0.1, 0.00, 0.0);

    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
    pose = msg.pose.pose;
    robot_pose_ = pose;
    twist = msg.twist.twist;
    double pos_x = pose.position.x;
    double pos_y = pose.position.y;
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double robot_heading = yaw;
    // transform estimated_pos_ into absolute pos
    if (reached_pos_)
    {
      goal_.first = estimated_pos_.first * std::cos(yaw) - estimated_pos_.second * std::sin(yaw) + pos_x;
      goal_.second = estimated_pos_.first * std::sin(yaw) + estimated_pos_.second * std::cos(yaw) + pos_y;
    }
    // goal_.first = 10;
    // goal_.second = 10;
    curr_robot_pos_.first = pos_x;
    curr_robot_pos_.second = pos_y;

    double target_angle = atan2(goal_.second - pos_y, goal_.first - pos_x);

    double diff_angle = target_angle - robot_heading;
    double update_angle = pid_angle.Control(diff_angle);
    if (fabs(diff_angle) < 1 * M_PI / 180.0)
    {
      if (!angle_setted)
      {
        angle_setted = true;
      }
    }
    if (fabs(diff_angle) < 5 * M_PI / 180.0)
    {
      if (angle_setted)
      {
        update_angle = 0;
      }
    }
    else
    {
      angle_setted = false;
    }

    double diff_x = goal_.first - pos_x;
    double diff_y = goal_.second - pos_y;
    double diff_dist = sqrt(pow(diff_x, 2) + pow(diff_y, 2));

    double speed = pid_speed.Control(diff_dist);

    double set_angle;

    if (set_angle > 0)
    {
      set_angle = std::min(update_angle, 0.1);
    }
    else
    {
      set_angle = std::max(update_angle, -0.1);
    }

    double set_speed;
    if (set_speed > 0)
    {
      set_speed = std::min(speed, 0.1);
    }
    else
    {
      set_speed = std::max(speed, -0.1);
    }
    std::cerr << set_angle << std::endl;

    geometry_msgs::Twist new_twist;
    if (angle_setted)
    {
      new_twist.linear.x = set_speed;
    }
    else
    {
      new_twist.linear.x = 0;
    }
    new_twist.linear.y = 0.0;
    new_twist.linear.z = 0.0;
    new_twist.angular.x = 0.0;
    new_twist.angular.y = 0.0;
    new_twist.angular.z = set_angle;

    if (fabs(curr_robot_pos_.first - goal_.first) > 1e-1 || fabs(curr_robot_pos_.first - goal_.first) > 1e-1)
    {
      std::cerr << "not reached yet" << std::endl;
      std::cerr << "curr pos: " << curr_robot_pos_.first << " " << curr_robot_pos_.second << " "
                << "next pos: " << goal_.first << " " << goal_.second << std::endl;
      reached_pos_ = false;
      control_publisher_.publish(new_twist);
    }
    else
    {
      std::cerr << "arrived" << std::endl;
      std::cerr << "curr pos: " << curr_robot_pos_.first << " " << curr_robot_pos_.second << " "
                << "next pos: " << goal_.first << " " << goal_.second << std::endl;
      reached_pos_ = true;

      geometry_msgs::Twist stand_still;

      stand_still.linear.x = 0;

      stand_still.linear.y = 0.0;
      stand_still.linear.z = 0.0;
      stand_still.angular.x = 0.0;
      stand_still.angular.y = 0.0;
      stand_still.angular.z = 0;
      control_publisher_.publish(stand_still);
    }

    // std::cerr << goal_.first << " " << goal_.second << std::endl;
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pos_publisher_;
  ros::Publisher control_publisher_;
  ros::Publisher scan_publisher_;
  ros::Subscriber scan_subscriber_;
  ros::Subscriber pos_subscriber_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main");
  ros::NodeHandle nh;
  // subscriber for scan and publisher for next pos
  CommunicationInterface CIObject;

  // ros::Rate loop_rate(1);
  Map test_map;

  ros::Publisher pub_map_quadrant_1 = nh.advertise<nav_msgs::OccupancyGrid>("occu_map_quadrant_1", 100);
  ros::Publisher pub_map_quadrant_2 = nh.advertise<nav_msgs::OccupancyGrid>("occu_map_quadrant_2", 100);
  ros::Publisher pub_map_quadrant_3 = nh.advertise<nav_msgs::OccupancyGrid>("occu_map_quadrant_3", 100);
  ros::Publisher pub_map_quadrant_4 = nh.advertise<nav_msgs::OccupancyGrid>("occu_map_quadrant_4", 100);

  while (ros::ok())
  {
    for (auto point : curr_scan_)
    {
      test_map.UpdateWithScanPoint(curr_robot_pos_.first, curr_robot_pos_.second, point.first, point.second, 20);
      // std::cerr << curr_robot_pos_.first << " " << curr_robot_pos_.second << " " << point.first << " " <<
      // point.second
      //           << std::endl;
      // std::cerr << point.first << " " << point.second << std::endl;
    }
    pub_map_quadrant_1.publish(test_map.GetMap()[0]);
    pub_map_quadrant_2.publish(test_map.GetMap()[1]);
    pub_map_quadrant_3.publish(test_map.GetMap()[2]);
    pub_map_quadrant_4.publish(test_map.GetMap()[3]);
    ros::spinOnce();
  }
  // ros::spin();
  return 0;
}
