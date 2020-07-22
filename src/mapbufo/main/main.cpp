#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <iterator>
#include "common.h"
#include "lidar_sim.h"
#include "map.h"
#include "map_simulator.h"
#include "path_planning.h"
#include "pid_controller.h"
#include "robot.h"
class CommunicationInterface
{
public:
  CommunicationInterface()
  {
    pos_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    control_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

    scan_subscriber_ = nh_.subscribe("/scan", 1000, &CommunicationInterface::scanCallback, this);

    pos_subscriber_ = nh_.subscribe("/odom", 1000, &CommunicationInterface::posCcontroller, this);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
  {
    // ROS_INFO("scan received");
  }

  void posCcontroller(const nav_msgs::Odometry msg)
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
    // std::cerr << speed << std::endl;
    double target_angle = atan2(goal_y - pos_y, goal_x - pos_x);
    double diff_angle = target_angle - 2 * acos(pose.orientation.w);
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
      set_speed = std::min(speed, 0.01);
    }
    else
    {
      set_speed = std::max(speed, -0.01);
    }
    std::cout << target_angle * 180.0 / M_PI << " " << 2 * acos(pose.orientation.w) * 180 / M_PI << " "
              << set_angle * 180 / M_PI << std::endl;
    geometry_msgs::Twist new_twist;
    if (fabs(set_angle) < 1e-2)
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
  ros::init(argc, argv, "main");

  // subscriber for scan and publisher for next pos
  CommunicationInterface CIObject;

  ros::spin();
  return 0;
  // ************************ initialization ************************ //

  // global map: the map used to generate scan points and as the reference for
  // comparison
  Map global_map({20, 20});

  if (status::Error == global_map.LoadGlobalMap(ros::package::getPath("mapbufo") + "/maps/scenario_03.txt"))
  {
    std::cerr << "Invalid map data!" << std::endl;
  }
  // empty map: the map created/filled by scans. Used for path planning
  Map empty_map({10, 10});

  if (status::Error == empty_map.Load(ros::package::getPath("mapbufo") + "/maps/scenario_04.txt"))
  {
    std::cerr << "Invalid map data!" << std::endl;
  }

  // set start and end position
  Point2D start_pos(8, 0);
  Point2D end_pos(6, 8);

  // initialize robot and lidar
  // since the lidar is mounted on the robot, they share the same position and
  // heading
  Robot my_robot(start_pos.first, start_pos.second, M_PI / 4.0);
  simulation::LidarSim lidar_sim(my_robot.getPosX(), my_robot.getPosY(), 5.0, M_PI / 4.0, my_robot.getHeading());

  // list for robot movement
  std::vector<std::pair<int, int>> estimated_next_pos_list;
  estimated_next_pos_list.push_back(start_pos);

  // get initial target position list from path planning
  std::vector<Point2D> target_pos_list = PathPlanning::PathPlanning(start_pos, end_pos, empty_map);

  // ************************ robot movement ************************ //

  // prevent endless loop
  int num_max_loop = 100;
  int num_loop = 0;

  // robot movement:
  /// 1. get the next target point from the top of the tarrget pos list
  /// 2. generate scan points with position and cell status (e.g. empty,
  /// occupied, etc.) from the global map
  /// 3. update the empty map with scan points
  /// 4.
  while (!target_pos_list.empty())
  {
    // safety: exit loop
    if (num_loop > num_max_loop)
      break;

    // get the next target pos where the robot is supposed to go
    Point2D next_target_pos = *target_pos_list.begin();

    // pre-check if the robot is heading towards the target pos
    // if not, first turn to the target pos
    float dist_x = float(next_target_pos.first - my_robot.getPosX());
    float dist_y = float(next_target_pos.second - my_robot.getPosY());
    float suggested_heading = std::atan2(dist_y, dist_x);
    if (std::abs(suggested_heading - my_robot.getHeading()) > 10e-2)
    {
      // need to adjust heading first
      my_robot.setHeading(suggested_heading);
      lidar_sim.setHeading(my_robot.getHeading());
      num_loop++;
      continue;
    }

    // print maps
    std::cerr << "loop : " << num_loop << std::endl;

    std::cerr << "====== current map =======" << std::endl;
    Map curr_map(empty_map);
    curr_map.Update(my_robot.getPosX(), my_robot.getPosY(), CellOccupied::robot_pos);
    curr_map.Update(next_target_pos.first, next_target_pos.second, CellOccupied::path);
    curr_map.PrintMap();

    std::cerr << "====== map with planned path =======" << std::endl;
    Map temp_map(empty_map);
    for (auto target_pos : target_pos_list)
    {
      temp_map.Update(target_pos.first, target_pos.second, CellOccupied::path);
    }
    temp_map.PrintMap();

    // create scan points from the global map
    ScanData scan_point_list;
    scan_point_list = lidar_sim.createInputScan(global_map);

    // update the empty map with scan points:
    /// here, points registered as empty and occupied are used for map update;
    /// unknown points have no influence on map update because they are unknown
    for (auto cell : scan_point_list)
    {
      if (cell.second == CellOccupied::empty || cell.second == CellOccupied::occupied)
      {
        empty_map.Update(cell.first.first, cell.first.second, cell.second);
      }
    }

    // now the robot estimates the next step based on the current scan points
    /// if the robot can't reach the target pos due to some reason (e.g.
    /// obstacles),
    /// a new path planning is required, so the robot calls help.
    bool call_help = false;
    Point2D estimated_next_pos =
        my_robot.estimateNextStep(scan_point_list, next_target_pos, lidar_sim.getMaxDist(), call_help);
    if (call_help)
    {
      std::cerr << "Robot calls help; target path re-planned." << std::endl;
      target_pos_list = PathPlanning::PathPlanning(estimated_next_pos, end_pos, empty_map);

      std::cerr << "====== map with new planned path =======" << std::endl;
      Map temp_map(empty_map);
      for (auto target_pos : target_pos_list)
      {
        temp_map.Update(target_pos.first, target_pos.second, CellOccupied::path);
      }
      temp_map.Update(my_robot.getPosX(), my_robot.getPosY(), CellOccupied::robot_pos);

      temp_map.PrintMap();
      num_loop++;
      continue;
    }

    // the robot will moves to the target pos
    my_robot.move(estimated_next_pos);
    lidar_sim.updatePose(my_robot.getPosX(), my_robot.getPosY(), my_robot.getHeading());
    estimated_next_pos_list.push_back(estimated_next_pos);

    num_loop++;

    // update target pos list
    target_pos_list.erase(target_pos_list.begin());
  }

  // check if robot reached end pos
  if (my_robot.getPosX() == end_pos.first && my_robot.getPosY() == end_pos.second)
    std::cerr << "succeeded!" << std::endl;
  else
    std::cerr << "failed!" << std::endl;

  // plotting
  std::cerr << "==================== Global ====================" << std::endl;
  for (auto next_pos : estimated_next_pos_list)
  {
    global_map.Update(next_pos.first, next_pos.second, CellOccupied::robot_pos);
    global_map.Update(end_pos.first, end_pos.second, CellOccupied::target_pos);
  }
  global_map.PrintMap();
  std::cerr << "==================== Scan ====================" << std::endl;
  //  for (auto next_pos : estimated_next_pos_list) {
  //    empty_map.Update(next_pos.first, next_pos.second,
  //    CellOccupied::robot_pos);
  //  }

  empty_map.PrintMap();
  return 0;
}
