#include "lidar_sim.h"
#include "map_simulator.h"
#include "point.h"
#include "robot.h"
#include <iostream>

bool ptObserved(std::vector<Point2D> pt_list, int i, int j) {
  for (int k = 0; k < pt_list.size(); k++) {
    Point2D pt = pt_list[k];
    if (i == pt.getPositionX() && j == pt.getPositionY() &&
        pt.getValue() != 0) {
      return true;
    }
  }
  return false;
}

int main() {
  std::vector<std::vector<int>> map;
  if (status::Error == simulation::LoadMap("../../maps/scenario_03.txt", map)) {
    std::cerr << "Invalid map data!" << std::endl;
  }

  /*
   * start: (4, 2), end: (4, 7)
   *
   * suggested heading | expected next pos
   *        45 deg     |     (5, 3)
   *        0 deg      |     (7, 3)
   *        90 deg     |     (7, 5)
   *        135 deg    |     (6, 6)
   *        135 deg    |     (5, 7)
   *        180 deg    |     (4, 7)
   */

  Point2D start_pos(4, 2);
  Point2D end_pos(4, 7);

  Robot my_robot(start_pos.getPositionX(), start_pos.getPositionY(), 2.0,
                 M_PI / 4.0, M_PI / 4.0);
  std::vector<float> suggested_heading_list = {45, 0, 90, 135, 135, 180};
  std::vector<Point2D> expected_next_pos_list = {Point2D(5, 3), Point2D(7, 3),
                                                 Point2D(7, 5), Point2D(6, 6),
                                                 Point2D(5, 7), Point2D(4, 7)};
  std::vector<Point2D> estimated_next_pos_list;
  estimated_next_pos_list.push_back(start_pos);
  int step_counter = 0;

  while (my_robot.getPosX() != end_pos.getPositionX() ||
         my_robot.getPosY() != end_pos.getPositionY()) {

    // exit loop
    if (step_counter > 7)
      break;
    float suggested_heading =
        suggested_heading_list[step_counter] * M_PI / 180.0;
    std::cout << "suggested heading: " << suggested_heading_list[step_counter]
              << " " << my_robot.getHeading() * 180 / M_PI << std::endl;
    if (std::abs(suggested_heading - my_robot.getHeading()) > 10e-2) {
      // need to adjust heading first
      my_robot.setHeading(suggested_heading);
      continue;
    }

    std::vector<Point2D> observed_point_list;
    observed_point_list = my_robot.createInputScan(map);

    Point2D estimated_next_pos = my_robot.estimateNextStep(observed_point_list);

    //    return 0;
    Point2D expected_next_pos = expected_next_pos_list[step_counter];

    if (estimated_next_pos.getPositionX() != expected_next_pos.getPositionX() ||
        estimated_next_pos.getPositionY() != expected_next_pos.getPositionY()) {
      std::cerr << "estimated next pos: " << estimated_next_pos.getPositionX()
                << ", " << estimated_next_pos.getPositionY() << std::endl;
      std::cerr << "expected next pos: " << expected_next_pos.getPositionX()
                << ", " << expected_next_pos.getPositionY() << std::endl;
      break;
    }

    my_robot.move(estimated_next_pos);
    estimated_next_pos_list.push_back(estimated_next_pos);
    step_counter++;
  }

  // plotting

  int map_length = map.size();
  int map_width = map[0].size();
  for (auto next_pos : estimated_next_pos_list) {
    map[next_pos.getPositionX()][next_pos.getPositionY()] = 5;
  }

  for (int i = 0; i < map_length; i++) {
    for (int j = 0; j < map_width; j++) {
      std::cout << map[i][j] << " ";
    }
    std::cout << std::endl;
  }

  return 0;
}
