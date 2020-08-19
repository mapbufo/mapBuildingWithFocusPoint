#include "communication_interface.h"

CommunicationInterface::CommunicationInterface(ros::NodeHandle &nh)
    : sync(scan_sub, odom_sub, 10), map_local_(nh, 0.05, 20, 20)
{
  // input: laserscan, robot_position
  scan_sub.subscribe(nh, "/scan", 10);
  odom_sub.subscribe(nh, "/odom", 10);
  sync.registerCallback(boost::bind(&CommunicationInterface::scanOdomCallback, this, _1, _2));
  odom_subscriber_ = nh.subscribe("/odom", 1, &CommunicationInterface::robotPositionCallback, this);

  goal_subscriber_ = nh.subscribe("/move_base_simple/goal", 1, &CommunicationInterface::setGoalCallback, this);
  // initialize the current goal
  new_goal_updated_ = false;
  reached_pos_ = false;
  final_goal_.first = 0;
  final_goal_.second = 0;
  curr_robot_pos_.first = 0;
  curr_robot_pos_.second = 0;
  goal_.first = 0;
  goal_.second = 0;
  goal_.first = 0;
  goal_.second = 0;
  planned_path_vec_.clear();
  // output: robot_control
  control_publisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

  // global map publishers
  pub_global_map_quadrant_1_ = nh.advertise<nav_msgs::OccupancyGrid>("occu_global_map_quadrant_1", 100);
  pub_global_map_quadrant_2_ = nh.advertise<nav_msgs::OccupancyGrid>("occu_global_map_quadrant_2", 100);
  pub_global_map_quadrant_3_ = nh.advertise<nav_msgs::OccupancyGrid>("occu_global_map_quadrant_3", 100);
  pub_global_map_quadrant_4_ = nh.advertise<nav_msgs::OccupancyGrid>("occu_global_map_quadrant_4", 100);

  // local map publishers
  pub_local_map_quadrant_1_ = nh.advertise<nav_msgs::OccupancyGrid>("occu_local_map_quadrant_1", 100);
  pub_local_map_quadrant_2_ = nh.advertise<nav_msgs::OccupancyGrid>("occu_local_map_quadrant_2", 100);
  pub_local_map_quadrant_3_ = nh.advertise<nav_msgs::OccupancyGrid>("occu_local_map_quadrant_3", 100);
  pub_local_map_quadrant_4_ = nh.advertise<nav_msgs::OccupancyGrid>("occu_local_map_quadrant_4", 100);

  // debug output: processed scan_points
  scan_publisher_ = nh.advertise<sensor_msgs::PointCloud>("/processed_scan", 1);

  // publish planned path
  pub_path_ = nh.advertise<visualization_msgs::Marker>("planned_path", 100);

  pub_local_path_ = nh.advertise<visualization_msgs::Marker>("planned_local_path", 100);

  path_line_.header.frame_id = "odom";
  path_line_.ns = "path";
  path_line_.id = 0;
  path_line_.type = visualization_msgs::Marker::LINE_STRIP;
  path_line_.action = visualization_msgs::Marker::ADD;
  path_line_.pose.orientation.w = 1.0;
  path_line_.scale.x = 0.05;
  path_line_.color.g = 1.0;
  path_line_.color.a = 0.6;

  local_path_line_.header.frame_id = "odom";
  local_path_line_.ns = "path";
  local_path_line_.id = 1;
  local_path_line_.type = visualization_msgs::Marker::LINE_STRIP;
  local_path_line_.action = visualization_msgs::Marker::ADD;
  local_path_line_.pose.orientation.w = 1.0;
  local_path_line_.scale.x = 0.05;
  local_path_line_.color.r = 1.0;
  local_path_line_.color.a = 0.6;
}

void CommunicationInterface::scanOdomCallback(const sensor_msgs::LaserScan::ConstPtr &scan,
                                              const nav_msgs::Odometry::ConstPtr &odom)
{
  // update input scan
  input_scan_.header = scan->header;
  input_scan_.angle_min = scan->angle_min;
  input_scan_.angle_max = scan->angle_max;
  input_scan_.angle_increment = scan->angle_increment;
  input_scan_.time_increment = scan->time_increment;
  input_scan_.scan_time = scan->scan_time;
  input_scan_.range_min = scan->range_min;
  input_scan_.range_max = scan->range_max;
  input_scan_.ranges.clear();
  for (auto r : scan->ranges)
  {
    input_scan_.ranges.push_back(r);
  }
  input_scan_.intensities.clear();
  for (auto i : scan->intensities)
  {
    input_scan_.intensities.push_back(i);
  }

  // update input odom at scan
  input_odom_at_scan_.header = odom->header;
  input_odom_at_scan_.child_frame_id = odom->child_frame_id;
  input_odom_at_scan_.pose = odom->pose;
  input_odom_at_scan_.twist = odom->twist;
}

void CommunicationInterface::processScan()
{
  float angle_min = input_scan_.angle_min;
  float angle_max = input_scan_.angle_max;
  float angle_increment = input_scan_.angle_increment;
  float range_min = input_scan_.range_min;
  float range_max = input_scan_.range_max;
  int number_of_laser = input_scan_.ranges.size();
  // get laser scan id of the mid +- 10 deg
  float priority_angle_range_deg = 10;
  float priority_angle_range_rad = priority_angle_range_deg * M_PI / 180.0;
  // laser scan: 0-----1*****2*****3-----4
  // 0: angle min
  // 1: priority start
  // 2: center
  // 3: priority end
  // 4: angle max

  Point2DWithFloat next_pos(0, 0);
  float max_dist = 0;

  curr_scan_.clear();
  curr_local_scan_.clear();
  int counter = 0;
  int resolution = 10; // pick only every ten points outside the interested area
  for (int i = 0; i < number_of_laser; i++)
  {
    float laser_angle = angle_min + i * angle_increment;
    // robot heading: x axis, robot left: y axis
    float x = input_scan_.ranges[i] * std::cos(laser_angle);
    float y = input_scan_.ranges[i] * std::sin(laser_angle);
    // insert
    double pos_x = input_odom_at_scan_.pose.pose.position.x;
    double pos_y = input_odom_at_scan_.pose.pose.position.y;
    tf::Quaternion q(input_odom_at_scan_.pose.pose.orientation.x, input_odom_at_scan_.pose.pose.orientation.y,
                     input_odom_at_scan_.pose.pose.orientation.z, input_odom_at_scan_.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double robot_heading = yaw;
    if (laser_angle > -priority_angle_range_rad && laser_angle < priority_angle_range_rad)
    {
      if (std::isnan(input_scan_.ranges[i])) // if a point is nan, then the 10
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
            if (!std::isnan(input_scan_.ranges[i + j]) || !std::isnan(input_scan_.ranges[i - j]))
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
        curr_local_scan_[{x, y}] = -20;
        float global_x = x * std::cos(yaw) - y * std::sin(yaw) + pos_x;
        float global_y = x * std::sin(yaw) + y * std::cos(yaw) + pos_y;
        curr_scan_[{global_x, global_y}] = -20;
        if (range_max > max_dist)
        {
          next_pos.first = range_max * std::cos(laser_angle);
          next_pos.second = range_max * std::sin(laser_angle);
          max_dist = input_scan_.range_max;
        }
        continue;
      }

      float global_x = x * std::cos(yaw) - y * std::sin(yaw) + pos_x;
      float global_y = x * std::sin(yaw) + y * std::cos(yaw) + pos_y;
      if (!std::isnan(global_x) && !std::isnan(global_y))
      {
        curr_local_scan_[{x, y}] = 20;
        curr_scan_[{global_x, global_y}] = 20;
      }
      geometry_msgs::Point32 pt;
      pt.x = x;
      pt.y = y;
      pt.z = 0;

      if (input_scan_.ranges[i] > max_dist)
      {
        next_pos.first = x;
        next_pos.second = y;
        max_dist = input_scan_.ranges[i];
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
      if (std::isnan(input_scan_.ranges[i]))
      {
        float x = range_max * std::cos(laser_angle);
        float y = range_max * std::sin(laser_angle);
        float global_x = x * std::cos(yaw) - y * std::sin(yaw) + pos_x;
        float global_y = x * std::sin(yaw) + y * std::cos(yaw) + pos_y;
        curr_local_scan_[{x, y}] = -6;
        curr_scan_[{global_x, global_y}] = -6;

        continue;
      }
      float global_x = x * std::cos(yaw) - y * std::sin(yaw) + pos_x;
      float global_y = x * std::sin(yaw) + y * std::cos(yaw) + pos_y;
      if (!std::isnan(global_x) && !std::isnan(global_y))
      {
        curr_local_scan_[{x, y}] = 6;
        curr_scan_[{global_x, global_y}] = 6;
      }
      geometry_msgs::Point32 pt;
      pt.x = x;
      pt.y = y;
      pt.z = 0;
    }
  }
}

void CommunicationInterface::robotPositionCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
  input_odom_.header = odom->header;
  input_odom_.child_frame_id = odom->child_frame_id;
  input_odom_.pose = odom->pose;
  input_odom_.twist = odom->twist;
}

void CommunicationInterface::processOdom()
{
  // map_local_.SetPos(input_odom_.pose.pose);
  PIDController pid_angle(1, 0.2, 2.5);
  PIDController pid_speed(0.1, 0.00, 0.0);

  geometry_msgs::Pose pose;
  geometry_msgs::Twist twist;
  pose = input_odom_.pose.pose;
  twist = input_odom_.twist.twist;
  double pos_x = pose.position.x;
  double pos_y = pose.position.y;

  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  double robot_heading = yaw;
  // transform goal_ into absolute pos

  curr_robot_pos_.first = pos_x;
  curr_robot_pos_.second = pos_y;

  double target_angle = atan2(goal_.second - pos_y, goal_.first - pos_x);

  double diff_angle = target_angle - robot_heading;
  if (diff_angle > M_PI)
  {
    diff_angle = 2 * M_PI - diff_angle;
  }
  else if (diff_angle < -M_PI)
  {
    diff_angle = 2 * M_PI + diff_angle;
  }

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

  if (angle_setted)
  {
    output_twist_.linear.x = set_speed;
  }
  else
  {
    output_twist_.linear.x = 0;
  }
  output_twist_.linear.y = 0.0;
  output_twist_.linear.z = 0.0;
  output_twist_.angular.x = 0.0;
  output_twist_.angular.y = 0.0;
  output_twist_.angular.z = set_angle;

  if (fabs(curr_robot_pos_.first - goal_.first) > 1e-1 || fabs(curr_robot_pos_.second - goal_.second) > 1e-1)
  {
    std::cerr << "not reached yet" << std::endl;
  }
  else
  {
    std::cerr << "reached" << std::endl;

    output_twist_.linear.x = 0;

    output_twist_.linear.y = 0.0;
    output_twist_.linear.z = 0.0;
    output_twist_.angular.x = 0.0;
    output_twist_.angular.y = 0.0;
    output_twist_.angular.z = 0;
  }
}

void CommunicationInterface::publishTwist()
{
  control_publisher_.publish(output_twist_);
}

void CommunicationInterface::publishGlobalMap(Map &map)
{
  pub_global_map_quadrant_1_.publish(map.GetMap()[0]);
  pub_global_map_quadrant_2_.publish(map.GetMap()[1]);
  pub_global_map_quadrant_3_.publish(map.GetMap()[2]);
  pub_global_map_quadrant_4_.publish(map.GetMap()[3]);
}

void CommunicationInterface::publishLocalMap()
{
  pub_local_map_quadrant_1_.publish(map_local_.GetMap()[0]);
  pub_local_map_quadrant_2_.publish(map_local_.GetMap()[1]);
  pub_local_map_quadrant_3_.publish(map_local_.GetMap()[2]);
  pub_local_map_quadrant_4_.publish(map_local_.GetMap()[3]);
}

void CommunicationInterface::publishPlannedPath(std::vector<Point2DWithFloat> path)
{
  path_line_.points.clear();

  // add the robot position at first
  geometry_msgs::Point p_point;

  if (!local_path_vec_.empty())
  {
    p_point.x = local_path_vec_.back().first;
    p_point.y = local_path_vec_.back().second;
    p_point.z = 0.0;
  }
  else
  {
    p_point.x = curr_robot_pos_.first;
    p_point.y = curr_robot_pos_.second;
    p_point.z = 0.0;
  }
  path_line_.points.push_back(p_point);
  for (auto point : path)
  {
    geometry_msgs::Point p;
    p.x = point.first;
    p.y = point.second;
    p.z = 0.0;
    path_line_.points.push_back(p);
  }

  // publish
  pub_path_.publish(path_line_);
}

void CommunicationInterface::publishPlannedLocalPath(std::vector<Point2DWithFloat> path)
{
  local_path_line_.points.clear();

  // add the robot position at first
  geometry_msgs::Point p_robot;
  p_robot.x = curr_robot_pos_.first;
  p_robot.y = curr_robot_pos_.second;
  p_robot.z = 0.0;
  local_path_line_.points.push_back(p_robot);

  for (auto point : path)
  {
    geometry_msgs::Point p;
    p.x = point.first;
    p.y = point.second;
    p.z = 0.0;
    local_path_line_.points.push_back(p);
  }

  // publish
  pub_local_path_.publish(local_path_line_);
}

void CommunicationInterface::cycle(Map &map)
{
  ros::Time t1 = ros::Time::now();
  // process the received scan
  processScan();
  ros::Time t2 = ros::Time::now();
  std::cerr << "t2 - t1: " << t2 - t1 << std::endl;

  map.UpdateWithScanPoints(curr_robot_pos_, curr_scan_);
  ros::Time t3 = ros::Time::now();
  std::cerr << "t3 - t2: " << t3 - t2 << std::endl;

  map_local_.UpdateLocalMapWithScanPoints({0, 0}, curr_local_scan_);
  ros::Time t4 = ros::Time::now();
  std::cerr << "t4 - t3: " << t4 - t3 << std::endl;

  // check if a new goal is received; if so, update the planned path
  if (!new_goal_updated_)
  {
    setPath(map);
  }
  ros::Time t5 = ros::Time::now();
  std::cerr << "t5 - t4: " << t5 - t4 << std::endl;

  // need a condition to choose whether local path should be updated or not.
  if (planned_path_vec_.size() > 1)
  {
    setLocalPath(map_local_);
  }
  ros::Time t6 = ros::Time::now();
  std::cerr << "t6 - t5: " << t6 - t5 << std::endl;

  if (!local_path_vec_.empty())
  {
    // set the first element of local path as goal_
    goal_.first = local_path_vec_[0].first;
    goal_.second = local_path_vec_[0].second;
  }
  else if (!planned_path_vec_.empty())
  {
    // set the first element of planned path as goal_
    goal_.first = planned_path_vec_[0].first;
    goal_.second = planned_path_vec_[0].second;
  }
  ros::Time t7 = ros::Time::now();
  std::cerr << "t7 - t6: " << t7 - t6 << std::endl;

  std::cerr << "goal: " << goal_.first << " " << goal_.second << std::endl;
  if (fabs(curr_robot_pos_.first - goal_.first) < 1e-1 && fabs(curr_robot_pos_.second - goal_.second) < 1e-1)
  {
    reached_pos_ = true;
  }
  else
  {
    reached_pos_ = false;
  }

  if (reached_pos_)
  {
    if (!local_path_vec_.empty())
    {
      local_path_vec_.erase(begin(local_path_vec_)); // remove the reached pos from path
      reached_pos_ = false;
      if (!local_path_vec_.empty())
      {
        // set the first element of local path as goal_
        goal_.first = local_path_vec_[0].first;
        goal_.second = local_path_vec_[0].second;
      }
      else if (!planned_path_vec_.empty())
      {
        // set the first element of planned path as goal_
        goal_.first = planned_path_vec_[0].first;
        goal_.second = planned_path_vec_[0].second;
      }
    }
    else if (!planned_path_vec_.empty())
    {
      planned_path_vec_.erase(begin(planned_path_vec_)); // remove the reached pos from path
      reached_pos_ = false;
      if (!planned_path_vec_.empty())
      {
        goal_.first = planned_path_vec_[0].first;
        goal_.second = planned_path_vec_[0].second;
      }
    }
  }
  ros::Time t8 = ros::Time::now();
  std::cerr << "t8 - t7: " << t8 - t7 << std::endl;

  if (!planned_path_vec_.empty())
  {
    if (checkIfPathBlocked(map))
    {
      std::cerr << "blocked. replan path." << std::endl;
      setPath(map);
    }
  }

  ros::Time t9 = ros::Time::now();
  std::cerr << "t9 - t8: " << t9 - t8 << std::endl;

  processOdom();
  ros::Time t10 = ros::Time::now();
  std::cerr << "t10 - t9: " << t10 - t9 << std::endl;

  publishTwist();

  // publish the updated global map
  publishGlobalMap(map);
  // publish the updated local map
  publishLocalMap();

  publishPlannedPath(planned_path_vec_);
  publishPlannedLocalPath(local_path_vec_);
  ros::Time t11 = ros::Time::now();
  std::cerr << "t11 - t10: " << t11 - t10 << std::endl;
}

void CommunicationInterface::setGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal)
{
  // compare current goal with new goal
  // if different, then call global path-planning
  // clear the old path, save the new path
  // if same, then skip
  if (final_goal_.first != goal->pose.position.x || final_goal_.second != goal->pose.position.y)
  {
    // update the current goal
    final_goal_.first = goal->pose.position.x;
    final_goal_.second = goal->pose.position.y;
    new_goal_updated_ = false;
  }
}

void CommunicationInterface::setPath(const Map &map)
{
  // delete the old planned path
  planned_path_vec_.clear();
  // get a new path
  Point2D start_pos = TransformIndex(curr_robot_pos_.first, curr_robot_pos_.second, 0.1);
  Point2D end_pos = TransformIndex(final_goal_.first, final_goal_.second, 0.1);
  std::vector<Point2D> planned_path;
  planned_path = PathPlanning::PathPlanning(start_pos, end_pos, map, true);

  for (int i = 0; i < planned_path.size(); i++)
  {
    Point2DWithFloat pt_float;
    pt_float = ReverseIndex(planned_path[i].first, planned_path[i].second, 0.1);
    planned_path_vec_.push_back(pt_float);
  }
  new_goal_updated_ = true;
  reached_pos_ = false;
}

void CommunicationInterface::setLocalPath(const Map &map_local)
{
  if (planned_path_vec_.empty())
  {
    return;
  }
  // find all elements from planned_path_vec_ in FoV
  std::vector<Point2DWithFloat>::iterator fa_target;
  for (fa_target = begin(planned_path_vec_); fa_target != end(planned_path_vec_); fa_target++)
  {
    // estimate if this target is in FoV
    float distance_2 =
        powf(fa_target->first - curr_robot_pos_.first, 2) + powf(fa_target->second - curr_robot_pos_.second, 2);
    // if this target is not in FoV, then break.(The rest targets are further)
    if (distance_2 >= 1 * 1)
    {
      break;
    }
  }
  // if there is no target in FoV, then return
  if (fa_target == begin(planned_path_vec_))
  {
    return;
  }

  // iterate through all possible targets, find if there is a local path
  for (; fa_target != begin(planned_path_vec_); fa_target--)
  {
    // try to find a local path, start from the fartherst target
    // transform the points into local-coordinate and local-index
    // start_pos is the robot_Pos, which is always at the center of local map
    Point2D start_pos(2, 0);
    tf::Quaternion q(input_odom_.pose.pose.orientation.x, input_odom_.pose.pose.orientation.y,
                     input_odom_.pose.pose.orientation.z, input_odom_.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    Point2DWithFloat tmp_end_pos = TransformFromGlobalToLocal(curr_robot_pos_.first, curr_robot_pos_.second,
                                                              (fa_target - 1)->first, (fa_target - 1)->second, yaw);

    // if the angle is bigger than 45 degree, then skip
    float angle = tanf(tmp_end_pos.second / tmp_end_pos.first);
    if (tmp_end_pos.first < 0 || angle > 1 || angle < -1)
    {
      continue;
    }

    Point2D end_pos =
        TransformIndex(tmp_end_pos.first, tmp_end_pos.second, map_local_.GetMap().front().info.resolution);

    std::vector<Point2D> planned_path;
    planned_path = PathPlanning::PathPlanning(start_pos, end_pos, map_local_, false);
    // if there is a path found, then remove the first element of planned_path_vec_
    // and add the local path into planned_path_vec_
    // then return
    if (!planned_path.empty())
    {
      planned_path_vec_.erase(begin(planned_path_vec_), fa_target);
      local_path_vec_.clear();
      // transform the points back into float
      for (int i = planned_path.size() - 1; i >= 0; i--)
      {
        Point2DWithFloat pt_float;

        pt_float =
            ReverseIndex(planned_path[i].first, planned_path[i].second, map_local_.GetMap().front().info.resolution);
        Point2DWithFloat pt_float_global = TransformFromLocalToGlobal(curr_robot_pos_.first, curr_robot_pos_.second,
                                                                      pt_float.first, pt_float.second, yaw);
        local_path_vec_.insert(begin(local_path_vec_), pt_float_global);

        // if it is not blocked between robot and the planned path-point, then skip all the points before it
        if (!checkIfLineBlocked(map_local_, Point2DWithFloat({0, 0}), pt_float, map_local_.GetMap().front().info.resolution))
        {
          return;
        }
      }
      return;
    }
  }
}

bool CommunicationInterface::checkIfPathBlocked(Map &map)
{
  // loop over all planned path points
  for (int idx = 0; idx < planned_path_vec_.size(); idx++)
  {
    // firstly, check if this point is out of laser range - only those points that are within FoV are considered
    Point2DWithFloat path_pt = planned_path_vec_[idx];
    float dist =
        powf(powf(curr_robot_pos_.first - path_pt.first, 2) + powf(curr_robot_pos_.second - path_pt.second, 2), 0.5);
    // if out of sight, return
    if (dist > 8)
    {
      return false; // not blocked
    }

    // secondly, get points between two planned path points and check if they are blocked
    Point2DWithFloat first_path_pt = curr_robot_pos_; // initialization with the current robot pos
    Point2DWithFloat second_path_pt = curr_robot_pos_;

    if (idx == 0) // The first line is from the current robot position to the first planned path point
    {
      first_path_pt = curr_robot_pos_;
      second_path_pt = planned_path_vec_[idx];
    }
    else
    {
      first_path_pt = planned_path_vec_[idx - 1];
      second_path_pt = planned_path_vec_[idx];
    }

    bool res = checkIfLineBlocked(map, first_path_pt, second_path_pt, 0.1f);
    if (res)
      return true;
    // // transform float, "exact" positions into integer, "map coord." points for the getLine() function
    // Point2D firstPosInMap = TransformIndex(first_path_pt.first, first_path_pt.second, 0.1f);
    // Point2D secondPosInMap = TransformIndex(second_path_pt.first, second_path_pt.second, 0.1f);
    // if (firstPosInMap == secondPosInMap)  // function getLine needs two different positions
    // {
    //   continue;
    // }

    // // get points between two path points
    // std::vector<Point2D> pointsToBeChecked =
    //     GetLine(firstPosInMap.first, firstPosInMap.second, secondPosInMap.first, secondPosInMap.second);

    // for (auto pt : pointsToBeChecked)
    // {
    //   // check if the surroundings are occupied

    //   // left
    //   for (int i = -3; i < 4; i++)
    //   {
    //     if (map.GetCell(Point2D(pt.first - 4, pt.second + i)) == CellOccupied::occupied)
    //     {
    //       return true;  // blocked!
    //     }
    //   }

    //   // right

    //   for (int i = -3; i < 4; i++)
    //   {
    //     if (map.GetCell(Point2D(pt.first + 4, pt.second + i)) == CellOccupied::occupied)
    //     {
    //       return true;  // blocked!
    //     }
    //   }

    //   // down
    //   for (int i = -3; i < 4; i++)
    //   {
    //     if (map.GetCell(Point2D(pt.first + i, pt.second - 4)) == CellOccupied::occupied)
    //     {
    //       return true;  // blocked!
    //     }
    //   }

    //   // up
    //   for (int i = -3; i < 4; i++)
    //   {
    //     if (map.GetCell(Point2D(pt.first + i, pt.second + 4)) == CellOccupied::occupied)
    //     {
    //       return true;  // blocked!
    //     }
    //   }
    // }
  }
  return false;
}

bool CommunicationInterface::checkIfLineBlocked(Map &map, Point2DWithFloat first_path_pt,
                                                Point2DWithFloat second_path_pt, float resolution)
{
  // transform float, "exact" positions into integer, "map coord." points for the getLine() function
  Point2D firstPosInMap = TransformIndex(first_path_pt.first, first_path_pt.second, resolution);
  Point2D secondPosInMap = TransformIndex(second_path_pt.first, second_path_pt.second, resolution);
  if (firstPosInMap == secondPosInMap) // function getLine needs two different positions
  {
    return false;
  }

  // get points between two path points
  std::vector<Point2D> pointsToBeChecked =
      GetLine(firstPosInMap.first, firstPosInMap.second, secondPosInMap.first, secondPosInMap.second);

  for (auto pt : pointsToBeChecked)
  {
    // check if the surroundings are occupied

    // left
    for (int i = -3; i < 4; i++)
    {
      if (map.GetCell(Point2D(pt.first - 4, pt.second + i)) == CellOccupied::occupied)
      {
        return true; // blocked!
      }
    }

    // right

    for (int i = -3; i < 4; i++)
    {
      if (map.GetCell(Point2D(pt.first + 4, pt.second + i)) == CellOccupied::occupied)
      {
        return true; // blocked!
      }
    }

    // down
    for (int i = -3; i < 4; i++)
    {
      if (map.GetCell(Point2D(pt.first + i, pt.second - 4)) == CellOccupied::occupied)
      {
        return true; // blocked!
      }
    }

    // up
    for (int i = -3; i < 4; i++)
    {
      if (map.GetCell(Point2D(pt.first + i, pt.second + 4)) == CellOccupied::occupied)
      {
        return true; // blocked!
      }
    }
  }
  return false;
}