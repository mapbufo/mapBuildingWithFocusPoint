#include "communication_interface.h"

CommunicationInterface::CommunicationInterface(ros::NodeHandle &nh) : sync(scan_sub, odom_sub, 10)
{
  // input: laserscan, robot_position
  scan_sub.subscribe(nh, "/scan", 10);
  odom_sub.subscribe(nh, "/odom", 10);
  sync.registerCallback(boost::bind(&CommunicationInterface::scanOdomCallback, this, _1, _2));
  // sync.registerCallback(&CommunicationInterface::scanOdomCallback, this);
  pos_subscriber_ = nh.subscribe("/odom", 1, &CommunicationInterface::robotPositionCallback, this);
  // output: robot_next_pos, robot_control
  // pos_publisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
  control_publisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

  pub_map_quadrant_1_ = nh.advertise<nav_msgs::OccupancyGrid>("occu_map_quadrant_1", 100);
  pub_map_quadrant_2_ = nh.advertise<nav_msgs::OccupancyGrid>("occu_map_quadrant_2", 100);
  pub_map_quadrant_3_ = nh.advertise<nav_msgs::OccupancyGrid>("occu_map_quadrant_3", 100);
  pub_map_quadrant_4_ = nh.advertise<nav_msgs::OccupancyGrid>("occu_map_quadrant_4", 100);

  // debug output: processed scan_points
  scan_publisher_ = nh.advertise<sensor_msgs::PointCloud>("/processed_scan", 1);
}
void CommunicationInterface::scanOdomCallback(const sensor_msgs::LaserScan::ConstPtr &scan,
                                              const nav_msgs::Odometry::ConstPtr &msg)
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
      if (std::isnan(scan->ranges[i]))  // if a point is nan, then the 10
                                        // points before and after must be nan
                                        // so that it can be considered empty
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
        float x = range_max * std::cos(laser_angle);
        float y = range_max * std::sin(laser_angle);
        float global_x = x * std::cos(yaw) - y * std::sin(yaw) + pos_x;
        float global_y = x * std::sin(yaw) + y * std::cos(yaw) + pos_y;

        curr_scan_[{ global_x, global_y }] = -20;
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
        curr_scan_[{ global_x, global_y }] = 20;
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
      {
        float x = range_max * std::cos(laser_angle);
        float y = range_max * std::sin(laser_angle);
        float global_x = x * std::cos(yaw) - y * std::sin(yaw) + pos_x;
        float global_y = x * std::sin(yaw) + y * std::cos(yaw) + pos_y;

        curr_scan_[{ global_x, global_y }] = 6;

        continue;
      }
      float global_x = x * std::cos(yaw) - y * std::sin(yaw) + pos_x;
      float global_y = x * std::sin(yaw) + y * std::cos(yaw) + pos_y;
      if (!std::isnan(global_x) && !std::isnan(global_y))
      {
        curr_scan_[{ global_x, global_y }] = 6;
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
    // std::cerr << "scan received" << std::endl;
    // std::cerr << curr_robot_pos_.first << " " << curr_robot_pos_.second << " " << estimated_pos_.first << " "
    //           << estimated_pos_.second << std::endl;
  }
  scan_publisher_.publish(point_cloud);
}

void CommunicationInterface::robotPositionCallback(const nav_msgs::Odometry msg)
{
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
    // std::cerr << "not reached yet" << std::endl;
    // std::cerr << "curr pos: " << curr_robot_pos_.first << " " << curr_robot_pos_.second << " "
    //           << "next pos: " << goal_.first << " " << goal_.second << std::endl;
    reached_pos_ = false;
    control_publisher_.publish(new_twist);
  }
  else
  {
    // std::cerr << "arrived" << std::endl;
    // std::cerr << "curr pos: " << curr_robot_pos_.first << " " << curr_robot_pos_.second << " "
    //           << "next pos: " << goal_.first << " " << goal_.second << std::endl;
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
}

void CommunicationInterface::cycle(Map &map)
{
  for (auto point : curr_scan_)
  {
    map.UpdateWithScanPoint(curr_robot_pos_.first, curr_robot_pos_.second, point.first.first, point.first.second,
                            point.second);
  }
  // publish the updated map
  pub_map_quadrant_1_.publish(map.GetMap()[0]);
  pub_map_quadrant_2_.publish(map.GetMap()[1]);
  pub_map_quadrant_3_.publish(map.GetMap()[2]);
  pub_map_quadrant_4_.publish(map.GetMap()[3]);
}
