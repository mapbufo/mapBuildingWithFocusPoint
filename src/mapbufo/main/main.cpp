#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <iterator>
#include "common.h"
#include "lidar_sim.h"
#include "map.h"
#include "map_simulator.h"
#include "path_planning.h"
#include "pid_controller.h"
#include "robot.h"

static std::vector<Point2D> curr_scan_;
static Point2D curr_robot_pos_;
class CommunicationInterface
{
public:
  CommunicationInterface()
  {
    pos_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    control_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

    scan_subscriber_ = nh_.subscribe("/scan", 1000, &CommunicationInterface::laserScanCallback, this);

    pos_subscriber_ = nh_.subscribe("/odom", 1000, &CommunicationInterface::robotPositionCallback, this);
  }

  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
  {
  // ROS_INFO("scan received");

  laserscanner:
    1. filter(20) > 20 var : resolution 1 : 10
  }

  void robotPositionCallback(const nav_msgs::Odometry msg)
  {
    // ROS_INFO("pos received");
    PIDController pid_angle(0.5, 0.01, 2);
    PIDController pid_speed(0.2, 0.01, 0.0);
    double goal_x = 10.0;
    double goal_y = 10.0;

    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
    pose = msg.pose.pose;
    twist = msg.twist.twist;
    double pos_x = pose.position.x;
    double pos_y = pose.position.y;

    curr_robot_pos_.first = pos_x;
    curr_robot_pos_.second = pos_y;

    // std::cerr << speed << std::endl;
    double target_angle = atan2(goal_y - pos_y, goal_x - pos_x);
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double robot_heading = yaw;

    double diff_angle = target_angle - robot_heading;

    double update_angle = pid_angle.Control(diff_angle);

    double diff_x = goal_x - pos_x;
    double diff_y = goal_y - pos_y;
    double diff_dist = sqrt(pow(diff_x, 2) + pow(diff_y, 2));

    double speed = pid_speed.Control(diff_dist);

    double set_angle;
    if (set_angle > 0)
    {
      set_angle = std::min(update_angle, 1.0);
    }
    else
    {
      set_angle = std::max(update_angle, -1.0);
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
    std::cout << target_angle * 180.0 / M_PI << "    " << robot_heading * 180.0 / M_PI << "    "
              << diff_angle * 180.0 / M_PI << "    " << set_angle * 180 / M_PI << std::endl;
    geometry_msgs::Twist new_twist;
    if (fabs(set_angle) < 1 * M_PI / 180)
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
    // new_twist.angular.z = 0.0;
    new_twist.angular.z = set_angle;

    control_publisher_.publish(new_twist);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pos_publisher_;
  ros::Publisher control_publisher_;
  ros::Subscriber scan_subscriber_;
  ros::Subscriber pos_subscriber_;
};

int main(int argc, char **argv)
{
  Map global_map;
  ros::init(argc, argv, "main");

  // subscriber for scan and publisher for next pos
  CommunicationInterface CIObject;

  ros::rate loop_rate(10);

  while (ros::ok())
  {
    for (auto point : curr_scan_)
    {
      global_map.UpdateWithScanpoint(curr_ro, float y0, float x1, float y1, int value);
    }
    ros::spinOnce();
  }

  return 0;
}
