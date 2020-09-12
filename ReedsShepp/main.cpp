#include "PathFunction.h"

int main(int argc, char **argv)
{

  // find rs path
  RSPath path = FindRSPath(2.5, -1.7, M_PI / 3.0);

  // plot path
  exportPath(path);
  return 0;
}
