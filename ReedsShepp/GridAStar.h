#include <cmath>
#include <cstdint>
#include <vector>
#include <iostream>

using namespace std;

// the obstlist is formed by 2-dim vector, the fir.dim vector represents the
// row, but the first element of the sec.dem vector represents the number of
// row, which means, the row with none element will not be saved. In the sec.dim
// the numbers from second element represent the number of column.
vector<vector<int>> CalcObstMap(vector<vector<int>> obstlist, int gres,
                                int minx, int miny)
{
    int maxx = INT32_MIN;
    int maxy = INT32_MIN;
    for (auto row : obstlist)
    {
        if (row[0] > maxy)
            maxy = row[0];
        if (row[0] < miny)
            miny = row[0];
        for (int i = 1; i < row.size(); i++)
        {
            if (row[i] < minx)
                minx = row[i];
            if (row[i] > maxx)
                maxx = row[i];
        }
    }
    int xwidth = maxx - minx;
    xwidth = ceil(xwidth / gres);
    int ywidth = maxy - miny;
    ywidth = ceil(ywidth / gres) + 1;
    vector<vector<int>> obmap(ywidth, vector<int>(xwidth, 0));

    //need some tricks to connect the obstacles instead of using knnsearch
    for (int i = 0; i < obstlist.size(); i++)
    {
        int row = ceil((maxy - (obstlist[i][0])) / gres);
        for (int j = 1; j < obstlist[i].size(); j++)
        {
            int column = ceil((obstlist[i][j] - minx) / gres);
            obmap[row][column] = 1;
        }
    }
    return obmap;
}
// vector<vector<int>>
void GridAStar(vector<vector<int>> obstlist, pair<int, int> goal,
               int gres)
{
    int minx = INT32_MAX;
    int miny = INT32_MAX;
    vector<vector<int>> obmap = CalcObstMap(obstlist, gres, minx, miny);

    return;
}