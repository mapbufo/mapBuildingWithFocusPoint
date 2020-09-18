#include "PathFunction.h"
#include "GridAStar.h"

void TestGridAStar();

int main(int argc, char **argv)
{ /*
  // find rs path
  RSPath path = FindRSPath(2.5, -1.7, M_PI / 3.0);

  // plot path
  exportPath(path);
*/
  TestGridAStar();
  return 0;
}

void TestGridAStar()
{
  // create an obstlist
  vector<vector<int>> obstlist;
  vector<int> tmp;
  tmp.push_back(30);
  for (int i = -25; i < 26; i++)
  {
    tmp.push_back(i);
  }
  obstlist.push_back(tmp);
  tmp.clear();
  tmp.push_back(5);
  for (int i = -25; i < -9; i++)
  {
    tmp.push_back(i);
  }
  for (int i = 10; i < 26; i++)
  {
    tmp.push_back(i);
  }
  obstlist.push_back(tmp);
  for (int i = 4; i > 0; i--)
  {
    tmp.clear();
    tmp.push_back(i);
    tmp.push_back(-10);
    tmp.push_back(10);
    obstlist.push_back(tmp);
  }
  tmp.clear();
  tmp.push_back(0);
  for (int i = -10; i < 11; i++)
  {
    tmp.push_back(i);
  }
  obstlist.push_back(tmp);
  /*
  std::cerr << "the created obstmap: " << std::endl;
  int gres = 2;
  int minx = INT32_MAX;
  int miny = INT32_MAX;
  vector<vector<int>> obmap = CalcObstMap(obstlist, gres, minx, miny);
  for (auto a : obmap)
  {
    for (auto b : a)
    {
      std::cerr << b << " ";
    }
    std::cerr << std::endl;
  }
  */

  //std::cerr << "cost from (5, 5) to (3, 8) is: " << AStarSearch({5, 5}, {3, 8}, obmap) << std::endl;

  vector<vector<float>> costmap = GridAStar(obstlist, {10, 5}, 2);
  std::cerr << "the calculated costmap: " << std::endl;
  for (auto a : costmap)
  {
    for (auto b : a)
    {
      std::cerr << b << "\t";
    }
    std::cerr << std::endl;
  }
}
