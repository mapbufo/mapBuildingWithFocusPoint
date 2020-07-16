#include <iostream>
#include <iterator>

#include "lidar_sim.h"
#include "map.h"
#include "map_simulator.h"
#include "path_planning.h"
#include "robot.h"
int main() {
  // ************************ initialization ************************ //

  // global map: the map used to generate scan points and as the reference for
  // comparison
  Map global_map({20, 20});

  if (status::Error == global_map.LoadGlobalMap("../../maps/scenario_03.txt")) {
    std::cerr << "Invalid map data!" << std::endl;
  }
  // empty map: the map created/filled by scans. Used for path planning
  Map empty_map({10, 10});

  if (status::Error == empty_map.Load("../../maps/scenario_04.txt")) {
    std::cerr << "Invalid map data!" << std::endl;
  }

  // set start and end position
  Point2D start_pos(5, 2);
  Point2D end_pos(0, 9);

  // initialize robot and lidar
  // since the lidar is mounted on the robot, they share the same position and
  // heading
  Robot my_robot(start_pos.first, start_pos.second, M_PI / 4.0);
  simulation::LidarSim lidar_sim(my_robot.getPosX(), my_robot.getPosY(), 5.0,
                                 M_PI / 4.0, my_robot.getHeading());

  // list for robot movement
  std::vector<std::pair<int, int>> estimated_next_pos_list;
  estimated_next_pos_list.push_back(start_pos);

  // get initial target position list from path planning
  std::vector<Point2D> target_pos_list =
      PathPlanning::PathPlanning(start_pos, end_pos, empty_map);

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
  while (!target_pos_list.empty()) {
    // safety: exit loop
    if (num_loop > num_max_loop) break;

    // get the next target pos where the robot is supposed to go
    Point2D next_target_pos = *target_pos_list.begin();

    // pre-check if the robot is heading towards the target pos
    // if not, first turn to the target pos
    float dist_x = float(next_target_pos.first - my_robot.getPosX());
    float dist_y = float(next_target_pos.second - my_robot.getPosY());
    float suggested_heading = std::atan2(dist_y, dist_x);
    if (std::abs(suggested_heading - my_robot.getHeading()) > 10e-2) {
      // need to adjust heading first
      my_robot.setHeading(suggested_heading);
      lidar_sim.setHeading(my_robot.getHeading());
      continue;
    }

    // print maps
    std::cerr << "loop : " << num_loop << std::endl;

    std::cerr << "====== current map =======" << std::endl;
    Map curr_map(empty_map);
    curr_map.Update(my_robot.getPosX(), my_robot.getPosY(),
                    CellOccupied::robot_pos);
    curr_map.Update(next_target_pos.first, next_target_pos.second,
                    CellOccupied::path);
    curr_map.PrintMap();

    std::cerr << "====== map with planned path =======" << std::endl;
    Map temp_map(empty_map);
    for (auto target_pos : target_pos_list) {
      temp_map.Update(target_pos.first, target_pos.second, CellOccupied::path);
    }
    temp_map.PrintMap();

    // create scan points from the global map
    ScanData scan_point_list;
    scan_point_list = lidar_sim.createInputScan(global_map);

    // update the empty map with scan points:
    /// here, points registered as empty and occupied are used for map update;
    /// unknown points have no influence on map update because they are unknown
    for (auto cell : scan_point_list) {
      if (cell.second == CellOccupied::empty ||
          cell.second == CellOccupied::occupied) {
        empty_map.Update(cell.first.first, cell.first.second, cell.second);
      }
    }

    // now the robot estimates the next step based on the current scan points
    /// if the robot can't reach the target pos due to some reason (e.g.
    /// obstacles),
    /// a new path planning is required, so the robot calls help.
    bool call_help = false;
    Point2D estimated_next_pos = my_robot.estimateNextStep(
        scan_point_list, next_target_pos, lidar_sim.getMaxDist(), call_help);
    if (call_help) {
      std::cerr << "Robot calls help; target path re-planned." << std::endl;
      target_pos_list =
          PathPlanning::PathPlanning(estimated_next_pos, end_pos, empty_map);

      std::cerr << "====== map with new planned path =======" << std::endl;
      Map temp_map(empty_map);
      for (auto target_pos : target_pos_list) {
        temp_map.Update(target_pos.first, target_pos.second,
                        CellOccupied::path);
      }
      temp_map.Update(my_robot.getPosX(), my_robot.getPosY(),
                      CellOccupied::robot_pos);

      temp_map.PrintMap();

      continue;
    }

    // the robot will moves to the target pos
    my_robot.move(estimated_next_pos);
    lidar_sim.updatePose(my_robot.getPosX(), my_robot.getPosY(),
                         my_robot.getHeading());
    estimated_next_pos_list.push_back(estimated_next_pos);

    num_loop++;

    // update target pos list
    target_pos_list.erase(target_pos_list.begin());
  }

  // check if robot reached end pos
  if (my_robot.getPosX() == end_pos.first &&
      my_robot.getPosY() == end_pos.second)
    std::cerr << "succeeded!" << std::endl;
  else
    std::cerr << "failed!" << std::endl;

  // plotting
  std::cerr << "==================== Global ====================" << std::endl;
  for (auto next_pos : estimated_next_pos_list) {
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
