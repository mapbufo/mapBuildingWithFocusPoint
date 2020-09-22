#include "PathFunction.h"
// #include "GridAStar.h"
#include "VehicleCollisionCheck.h"
#include "HybridAStar.h"
void TestGridAStar();

void testCollisionCheck();

int main(int argc, char **argv)
{ // 1. add obstacle list
  std::vector<Point2D> ObstList;
  for (int i = -25; i <= 25; ++i)
  {
    ObstList.push_back(Point2D(i, 30));
  }
  for (int i = -10; i <= 10; ++i)
  {
    ObstList.push_back(Point2D(i, 0));
  }
  for (int i = -25; i <= -10; ++i)
  {
    ObstList.push_back(Point2D(i, 5));
  }
  for (int i = 10; i <= 25; ++i)
  {
    ObstList.push_back(Point2D(i, 5));
  }
  for (int i = 0; i <= 5; ++i)
  {
    ObstList.push_back(Point2D(10, i));
  }
  for (int i = 0; i <= 5; ++i)
  {
    ObstList.push_back(Point2D(-10, i));
  }

  // 2. add obstacle lines
  std::vector<std::pair<Point2D, Point2D>> ObstLine;
  ObstLine.push_back({{-25, 30}, {25, 30}});
  ObstLine.push_back({{-25, 5}, {-10, 5}});
  ObstLine.push_back({{-10, 5}, {-10, 0}});
  ObstLine.push_back({{-10, 0}, {10, 0}});
  ObstLine.push_back({{10, 0}, {10, 5}});
  ObstLine.push_back({{10, 5}, {25, 5}});
  ObstLine.push_back({{-25, 5}, {-25, 30}});
  ObstLine.push_back({{25, 5}, {25, 30}});

  // 3. create vehicle object
  Vehicle veh(3.7, 2.6, 4.5, 1.0, 0.6);

  // 4. config
  Configuration config;
  config.MOTION_RESOLUTION = 0.1;
  config.N_STEER = 20.0;
  config.EXTEND_AREA = 0;
  config.XY_GRID_RESOLUTION = 2.0;
  config.YAW_GRID_RESOLUTION = 15.0 * M_PI / 180.0;

  double minx = std::numeric_limits<double>::max();
  double maxx = 0;
  double miny = std::numeric_limits<double>::max();
  double maxy = 0;

  for (auto elem : ObstList)
  {
    minx = std::min(minx, elem.first);
    maxx = std::max(maxx, elem.first);
    miny = std::min(miny, elem.second);
    maxy = std::max(maxy, elem.second);
  }
  config.MINX = minx - config.EXTEND_AREA;
  config.MAXX = maxx + config.EXTEND_AREA;
  config.MINY = miny - config.EXTEND_AREA;
  config.MAXY = maxy - config.EXTEND_AREA;

  config.MINYAW = -M_PI;
  config.MAXYAW = M_PI;

  config.SB_COST = 0;
  config.BACK_COST = 1.5;
  config.STEER_CHANGE_COST = 1.5;
  config.STEER_COST = 1.5;
  config.H_COST = 10.0;

  // 5. set start and end point
  Eigen::Vector3d start_point(22, 12, M_PI);
  Eigen::Vector3d end_point(7, 13, -M_PI / 2.0);

  // 6. find path
  // std::vector<std::vector<int>> ObstMap = GridAStar(config.ObstList, {end_point[0], end_point[1]}, config.XY_GRID_RESOLUTION);
  std::vector<std::vector<double>> ObstMap;
  config.ObstMap = ObstMap;

  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> th_vec;
  std::vector<double> D_vec;
  std::vector<double> delta_vec;

  HybridAStar(start_point, end_point, veh, config, x_vec, y_vec, th_vec, D_vec, delta_vec);
  if (x_vec.empty())
  {
    std::cerr << "Failed to find path!" << std::endl;
  }
  else
  {
    }
  return 0;
}

void testCollisionCheck()
{

  // // find rs path
  // RSPath path = FindRSPath(2.5, -1.7, M_PI / 3.0);

  // // plot path
  // exportPath(path);

  Eigen::Vector3d pVec(-10, 15, -M_PI / 1.5);
  std::vector<std::pair<Point2D, Point2D>> ObstLine;
  ObstLine.push_back({{-25, 30}, {25, 30}});
  ObstLine.push_back({{-25, 5}, {-10, 5}});
  ObstLine.push_back({{-10, 5}, {-10, 0}});
  ObstLine.push_back({{-10, 0}, {10, 0}});
  ObstLine.push_back({{10, 0}, {10, 5}});
  ObstLine.push_back({{10, 5}, {25, 5}});
  ObstLine.push_back({{-25, 5}, {-25, 30}});
  ObstLine.push_back({{25, 5}, {25, 30}});

  // ObstLine.push_back({{-10, -10}, {10, 10}});
  // ObstLine.push_back({{10, 10}, {-8, 8}});
  // ObstLine.push_back({{-8, 8}, {-10, -10}});
  Vehicle veh;
  if (VehicleCollisionCheck(pVec, ObstLine, veh))
  {
    std::cerr << "blocked!" << std::endl;
  }
  else
  {
    std::cerr << "NOT blocked!" << std::endl;
  }

  // export to file

  std::ofstream output_file;

  output_file.open("obstacleLine.txt");

  // set points
  for (int i = 0; i < ObstLine.size(); i++)
  {

    output_file << ObstLine[i].first.first << " " << ObstLine[i].first.second
                << " " << ObstLine[i].second.first << " " << ObstLine[i].second.second << std::endl;
  }
  output_file.close();

  // car
  output_file.open("carPos.txt");

  double W = veh.W;
  double LF = veh.LF;
  double LB = veh.LB;

  Point2D Cornerfl(LF, W / 2.0);
  Point2D Cornerfr(LF, -W / 2.0);
  Point2D Cornerrr(-LB, -W / 2.0);
  Point2D Cornerrl(-LB, W / 2.0);

  Point2D Pos(pVec[0], pVec[1]);
  double theta = pVec[2];

  Eigen::Matrix3d dcm; // rotation matrix of theta
  dcm << cos(-theta), -sin(-theta), 0,
      sin(-theta), cos(-theta), 0,
      0, 0, 1;

  Eigen::Vector3d tvecFL(Cornerfl.first, Cornerfl.second, 0);
  tvecFL = dcm * tvecFL;
  Cornerfl.first = tvecFL[0] + Pos.first;
  Cornerfl.second = tvecFL[1] + Pos.second;

  Eigen::Vector3d tvecFR(Cornerfr.first, Cornerfr.second, 0);
  tvecFR = dcm * tvecFR;
  Cornerfr.first = tvecFR[0] + Pos.first;
  Cornerfr.second = tvecFR[1] + Pos.second;

  Eigen::Vector3d tvecRL(Cornerrl.first, Cornerrl.second, 0);
  tvecRL = dcm * tvecRL;
  Cornerrl.first = tvecRL[0] + Pos.first;
  Cornerrl.second = tvecRL[1] + Pos.second;

  Eigen::Vector3d tvecRR(Cornerrr.first, Cornerrr.second, 0);
  tvecRR = dcm * tvecRR;
  Cornerrr.first = tvecRR[0] + Pos.first;
  Cornerrr.second = tvecRR[1] + Pos.second;

  // set points

  output_file << Cornerfl.first << " " << Cornerfl.second << std::endl;
  output_file << Cornerfr.first << " " << Cornerfr.second << std::endl;

  output_file << Cornerrr.first << " " << Cornerrr.second << std::endl;
  output_file << Cornerrl.first << " " << Cornerrl.second << std::endl;
  output_file << Cornerfl.first << " " << Cornerfl.second << std::endl;
  output_file.close();
}

// void TestGridAStar()
// {
//   // create an obstlist
//   vector<vector<int>> obstlist;
//   vector<int> tmp;
//   tmp.push_back(30);
//   for (int i = -25; i < 26; i++)
//   {
//     tmp.push_back(i);
//   }
//   obstlist.push_back(tmp);
//   tmp.clear();
//   tmp.push_back(5);
//   for (int i = -25; i < -9; i++)
//   {
//     tmp.push_back(i);
//   }
//   for (int i = 10; i < 26; i++)
//   {
//     tmp.push_back(i);
//   }
//   obstlist.push_back(tmp);
//   for (int i = 4; i > 0; i--)
//   {
//     tmp.clear();
//     tmp.push_back(i);
//     tmp.push_back(-10);
//     tmp.push_back(10);
//     obstlist.push_back(tmp);
//   }
//   tmp.clear();
//   tmp.push_back(0);
//   for (int i = -10; i < 11; i++)
//   {
//     tmp.push_back(i);
//   }
//   obstlist.push_back(tmp);
//   /*
//   std::cerr << "the created obstmap: " << std::endl;
//   int gres = 2;
//   int minx = INT32_MAX;
//   int miny = INT32_MAX;
//   vector<vector<int>> obmap = CalcObstMap(obstlist, gres, minx, miny);
//   for (auto a : obmap)
//   {
//     for (auto b : a)
//     {
//       std::cerr << b << " ";
//     }
//     std::cerr << std::endl;
//   }
//   */

//   //std::cerr << "cost from (5, 5) to (3, 8) is: " << AStarSearch({5, 5}, {3, 8}, obmap) << std::endl;

//   vector<vector<float>> costmap = GridAStar(obstlist, {10, 5}, 2);
//   std::cerr << "the calculated costmap: " << std::endl;
//   for (auto a : costmap)
//   {
//     for (auto b : a)
//     {
//       std::cerr << b << "\t";
//     }
//     std::cerr << std::endl;
//   }
// }
