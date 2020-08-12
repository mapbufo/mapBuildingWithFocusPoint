#include "communication_interface.h"

CommunicationInterface::CommunicationInterface(ros::NodeHandle &nh)
    : sync(scan_sub, odom_sub, 10), map_local_(nh, 0.01, 100, 100)
{
  counter = 0;
  // input: laserscan, robot_position
  scan_sub.subscribe(nh, "/scan", 10);
  odom_sub.subscribe(nh, "/odom", 10);
  sync.registerCallback(boost::bind(&CommunicationInterface::scanOdomCallback, this, _1, _2));

  depth_image_subscriber_ =
      nh.subscribe("/camera/depth/image_raw", 1, &CommunicationInterface::depthImageCallback, this);

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
  scan_publisher_ = nh.advertise<sensor_msgs::LaserScan>("/processed_scan", 1);

  // publish planned path
  pub_path_ = nh.advertise<visualization_msgs::Marker>("planned_path", 100);

  path_line_.header.frame_id = "odom";
  path_line_.ns = "path";
  path_line_.id = 0;
  path_line_.type = visualization_msgs::Marker::LINE_STRIP;
  path_line_.action = visualization_msgs::Marker::ADD;
  path_line_.pose.orientation.w = 1.0;
  path_line_.scale.x = 0.05;
  path_line_.color.g = 1.0;
  path_line_.color.a = 0.6;
}

void CommunicationInterface::depthImageCallback(const sensor_msgs::Image::ConstPtr &depth_img)
{
  // std_msgs/Header header
  // uint32 height
  // uint32 width
  // string encoding
  // uint8 is_bigendian
  // uint32 step
  // uint8[] data
  int img_height = depth_img->height;
  int img_width = depth_img->width;

  std::string img_encoding = depth_img->encoding;
  int img_step = depth_img->step;

  std::vector<float> x_vec;
  std::vector<float> y_vec;
  std::vector<float> z_vec;

  // ///////////////////////////////////////////////////////////

  const float center_x = cam_intr_[2];
  const float center_y = cam_intr_[5];
  const float focal_x = cam_intr_[0];
  const float focal_y = cam_intr_[4];
  const float *depth_row = reinterpret_cast<const float *>(&depth_img->data[0]);
  const int row_step = img_step / sizeof(float);

  for (int v = 0; v < img_height; ++v, depth_row += row_step)
  {
    for (int u = 0; u < img_width; ++u) // Loop over each pixel in row
    {
      const float depth = depth_row[u];

      if (std::isfinite(depth))
      { // Not NaN or Inf
        double z = depth;
        float x = (u - center_x) * z / focal_x;
        float y = (v - center_y) * z / focal_y;

        x_vec.push_back(x);
        y_vec.push_back(y);
        z_vec.push_back(z);
      }
    }
  }

  // save the point cloud into a ply-file so that it can be displayed with meshlab

  // std::ofstream output_file;

  // output_file.open("/home/yu/mapBuFo/mapBuildingWithFocusPoint/depth_image.ply");
  // output_file << "ply" << std::endl;
  // output_file << "format ascii 1.0" << std::endl;
  // output_file << "element vertex " << x_vec.size() << std::endl;
  // output_file << "property float x" << std::endl;
  // output_file << "property float y" << std::endl;
  // output_file << "property float z" << std::endl;
  // output_file << "property uchar red" << std::endl;
  // output_file << "property uchar green" << std::endl;
  // output_file << "property uchar blue" << std::endl;
  // output_file << "end_header" << std::endl;

  // // set poses
  // for (int j = 0; j < x_vec.size(); j++)
  // {
  //   output_file << x_vec[j] << " " << y_vec[j] << " " << z_vec[j] << " "
  //               << " " << 0 << " " << 255 << " " << 0 << std::endl;
  // }

  ///////////////////////////////////////////////////////////

  // get laser scan from the point cloud:
  // left:   (0, cy)
  // center: (cx, cy)
  // right:  (2*cx, cy)

  std::vector<std::pair<float, float>> scan_points;
  const float *new_depth_row = reinterpret_cast<const float *>(&depth_img->data[0]);
  new_depth_row += row_step * (int)(center_y - 1);
  for (int u = 0; u < img_width; ++u) // Loop over each pixel in row
  {
    const float depth = new_depth_row[u];
    const double th = -atan2((double)(u - center_x) / focal_x, 1.0); // Atan2(x, z), but depth divides out

    if (std::isfinite(depth))
    { // Not NaN or Inf

      double x = (u - center_x) * depth / focal_x;
      double z = depth;
      // std::cerr << x << " " << z << std::endl;

      // double r = std::sqrt(std::pow(x, 2) + std::pow(z, 2));
      double r = hypot(x, z);
      scan_points.push_back({th, r});
    }
  }

  // std::ofstream output_file;

  // output_file.open("/home/yu/mapBuFo/mapBuildingWithFocusPoint/scan_points.txt");

  // // set poses
  // for (int j = 0; j < scan_points.size(); j++)
  // {
  //   output_file << scan_points[j].first << " " << scan_points[j].second << std::endl;
  // }

  // create laser scan data
  sensor_msgs::LaserScan ls;
  ls.header.seq = counter++;
  ls.header.stamp = ros::Time::now();
  ls.header.frame_id = "camera_depth_frame";
  int num_points = scan_points.size();
  ls.angle_min = scan_points[0].first;
  ls.angle_max = scan_points[num_points - 1].first;
  ls.angle_increment = (ls.angle_max - ls.angle_min) / num_points;
  ls.range_min = 0.02;
  ls.range_max = 10;
  ls.scan_time = 0.02;
  for (int j = 0; j < scan_points.size(); j++)
  {
    ls.ranges.push_back(scan_points[j].second);
  }

  scan_publisher_.publish(ls);
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
    // std::cerr << "not reached yet" << std::endl;
  }
  else
  {
    // std::cerr << "reached" << std::endl;

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
  geometry_msgs::Point p_robot;
  p_robot.x = curr_robot_pos_.first;
  p_robot.y = curr_robot_pos_.second;
  p_robot.z = 0.0;
  path_line_.points.push_back(p_robot);

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

void CommunicationInterface::cycle(Map &map)
{
  // process the received scan
  processScan();

  // update maps
  map.UpdateWithScanPoints(curr_robot_pos_, curr_scan_);

  map_local_.UpdateLocalMapWithScanPoints({0, 0}, curr_local_scan_);

  // // // check if the planned path is blocked
  // // // if so, re-plan path (re plan full path with setPath() for now)
  // transformIndex
  // get points on line
  // for(every point check four sides for obstacles)
  // // if (check... == false)
  // // {
  // //   setPath(map);
  // // }

  // check if a new goal is received; if so, update the planned path
  if (!new_goal_updated_)
  {
    setPath(map);
  }
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
    if (!planned_path_vec_.empty())
    {
      // std::cerr << "reached a planned pos, remaining poses are:" << std::endl;
      planned_path_vec_.erase(begin(planned_path_vec_)); // remove the reached pos from path
      for (auto pt : planned_path_vec_)
      {
        // std::cerr << pt.first << " " << pt.second << std::endl;
      }
      reached_pos_ = false;
      if (!planned_path_vec_.empty())
      {
        goal_.first = planned_path_vec_[0].first;
        goal_.second = planned_path_vec_[0].second;
      }
    }
  }
  processOdom();

  publishTwist();

  // publish the updated global map
  publishGlobalMap(map);
  // publish the updated local map
  publishLocalMap();

  publishPlannedPath(planned_path_vec_);
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

void CommunicationInterface::setPath(const Map map)
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

  goal_.first = planned_path_vec_[0].first;
  goal_.second = planned_path_vec_[0].second;
}
