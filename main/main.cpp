#include <iostream>

#include "lidar_sim.h"
#include "map.h"
#include "map_simulator.h"
#include "path_planning.h"
#include "robot.h"
#include <iostream>
#include <iterator>
int main() {
  Map global_map(10);
  // load map

  if (status::Error == global_map.Load("../../maps/scenario_03.txt")) {
    std::cerr << "Invalid map data!" << std::endl;
  }

  Map empty_map(10);
  // load map

  if (status::Error == empty_map.Load("../../maps/scenario_04.txt")) {
    std::cerr << "Invalid map data!" << std::endl;
  }

  // set start and end position
  Point2D start_pos(3, 2);
  Point2D end_pos(5, 9);
  // get initial target position list from path planning
  std::vector<Point2D> target_pos_list =
      PathPlanning::PathPlanning(start_pos, end_pos, empty_map);

  // initialize robot and lidar
  Robot my_robot(start_pos.first, start_pos.second, M_PI / 4.0);
  simulation::LidarSim lidar_sim(my_robot.getPosX(), my_robot.getPosY(), 5.0,
                                 M_PI / 4.0, my_robot.getHeading());

  // list for robot movement
  std::vector<std::pair<int, int>> estimated_next_pos_list;
  estimated_next_pos_list.push_back(start_pos);

  // move!
  int step_counter = 0;
  while (!target_pos_list.empty()) {
    Point2D next_target_pos = *target_pos_list.begin();

    // safety: exit loop
    if (step_counter > 100)
      break;

    float dist_x = float(next_target_pos.first - my_robot.getPosX());
    float dist_y = float(next_target_pos.second - my_robot.getPosY());

    float suggested_heading = std::atan2(dist_y, dist_x);

    if (std::abs(suggested_heading - my_robot.getHeading()) > 10e-2) {
      // need to adjust heading first
      my_robot.setHeading(suggested_heading);
      lidar_sim.setHeading(my_robot.getHeading());
      continue;
    }

    ScanData scan_point_list;

    scan_point_list = lidar_sim.createInputScan(global_map);
    for (auto cell : scan_point_list) {
      if (cell.second == CellOccupied::empty ||
          cell.second == CellOccupied::occupied) {
        empty_map.Update(cell.first.first, cell.first.second, cell.second);
      }
    }

    bool call_help = false;
    Point2D estimated_next_pos = my_robot.estimateNextStep(
        scan_point_list, next_target_pos, lidar_sim.getMaxDist(), call_help);
    //    std::cout << estimated_next_pos.first << " " <<
    //    estimated_next_pos.second
    //              << std::endl;
    //    std::cout << next_target_pos.first << " " << next_target_pos.second
    //              << std::endl;

    my_robot.move(estimated_next_pos);
    lidar_sim.updatePose(my_robot.getPosX(), my_robot.getPosY(),
                         my_robot.getHeading());
    estimated_next_pos_list.push_back(estimated_next_pos);

    if (call_help) {
      std::cerr << "call help!" << std::endl;
      target_pos_list =
          PathPlanning::PathPlanning(estimated_next_pos, end_pos, empty_map);

      //      for (auto pos : target_pos_list) {
      //        std::cout << pos.first << " " << pos.second << std::endl;
      //        empty_map.Update(pos.first, pos.second, CellOccupied::path);
      //      }
      continue;
    }
    std::cerr << "====== current robot pos =======" << std::endl;
    for (auto next_pos : estimated_next_pos_list) {
      empty_map.Update(next_pos.first, next_pos.second,
                       CellOccupied::robot_pos);
    }
    empty_map.PrintMap();
    step_counter++;
    target_pos_list.erase(target_pos_list.begin());

    // update map
  }

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
