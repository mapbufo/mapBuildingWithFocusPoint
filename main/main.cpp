#include <iostream>

#include "lidar_sim.h"
#include "map_simulator.h"
#include "robot.h"
#include <iostream>

int main() {
  Map map;
  // load map

  //  if (status::Error == simulation::LoadMap("../../maps/scenario_01.txt",
  //  map)) {
  //    std::cerr << "Invalid map data!" << std::endl;
  //  }

  std::pair<int, int> start_pos(4, 2);
  std::pair<int, int> end_pos(4, 7);

  Robot my_robot(start_pos.first, start_pos.second, 2.0, M_PI / 4.0,
                 M_PI / 4.0);
  std::vector<std::pair<int, int>> estimated_next_pos_list;
  estimated_next_pos_list.push_back(start_pos);
  int step_counter = 0;
  while (my_robot.getPosX() != end_pos.first ||
         my_robot.getPosY() != end_pos.second) {

    // exit loop
    if (step_counter > 100)
      break;

    float dist_x = float(end_pos.first - my_robot.getPosX());
    float dist_y = float(end_pos.second - my_robot.getPosY());

    float suggested_heading = std::atan2(dist_y, dist_x);

    std::cout << "suggested heading: " << suggested_heading << " "
              << my_robot.getHeading() * 180 / M_PI << std::endl;
    if (std::abs(suggested_heading - my_robot.getHeading()) > 10e-2) {
      // need to adjust heading first
      my_robot.setHeading(suggested_heading);
      continue;
    }

    Map observed_point_list;
    observed_point_list = my_robot.createInputScan(map);

    std::pair<int, int> estimated_next_pos =
        my_robot.estimateNextStep(observed_point_list);

    my_robot.move(estimated_next_pos);
    estimated_next_pos_list.push_back(estimated_next_pos);
    step_counter++;
  }

  // plotting
  boost::unordered_map<std::pair<int, int>, CellOccupied> my_map;
  for (auto next_pos : estimated_next_pos_list) {
    map.Update(next_pos.first, next_pos.second, CellOccupied::robot_pos);
  }

  // print map

  return 0;
}
