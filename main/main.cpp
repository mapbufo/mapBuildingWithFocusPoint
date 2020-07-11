#include <iostream>

#include <iostream>
#include "lidar_sim.h"
#include "map.h"
#include "map_simulator.h"
#include "robot.h"

int main() {
  Map map(10);
  // load map

  if (status::Error == map.Load("../../maps/scenario_01.txt")) {
    std::cerr << "Invalid map data!" << std::endl;
  }

  std::pair<int, int> start_pos(4, 4);
  std::pair<int, int> end_pos(7, 2);

  Robot my_robot(start_pos.first, start_pos.second, 2.0, M_PI / 4.0,
                 M_PI / 4.0);
  std::vector<std::pair<int, int>> estimated_next_pos_list;
  estimated_next_pos_list.push_back(start_pos);
  int step_counter = 0;
  while (my_robot.getPosX() != end_pos.first ||
         my_robot.getPosY() != end_pos.second) {
    // safety: exit loop
    if (step_counter > 10) break;

    float dist_x = float(end_pos.first - my_robot.getPosX());
    float dist_y = float(end_pos.second - my_robot.getPosY());

    float suggested_heading = std::atan2(dist_y, dist_x);

    if (std::abs(suggested_heading - my_robot.getHeading()) > 10e-2) {
      // need to adjust heading first
      my_robot.setHeading(suggested_heading);

      continue;
    }

    Map observed_point_list;
    observed_point_list = my_robot.createInputScan(map);

    std::pair<int, int> estimated_next_pos =
        my_robot.estimateNextStep(observed_point_list, end_pos);

    my_robot.move(estimated_next_pos);
    estimated_next_pos_list.push_back(estimated_next_pos);
    step_counter++;
  }

  // plotting
  for (auto next_pos : estimated_next_pos_list) {
    map.Update(next_pos.first, next_pos.second, CellOccupied::robot_pos);
  }

  // print map
  map.PrintMap();
  return 0;
}
