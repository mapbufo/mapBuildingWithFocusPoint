#include <cmath>
#include <cstdint>
#include <vector>
#include <iostream>
#include <boost/unordered_map.hpp>
#include <algorithm>

using namespace std;

struct PointWithValue
{
    pair<int, int> pos;
    float value;
};

// the obstlist is formed by 2-dim vector, the fir.dim vector represents the
// row, but the first element of the sec.dem vector represents the number of
// row, which means, the row with none element will not be saved. In the sec.dim
// the numbers from second element represent the number of column.
vector<vector<int>> CalcObstMap(vector<vector<int>> &obstlist, int gres,
                                int &minx, int &miny);

float AStarSearch(pair<int, int> start, pair<int, int> goal,
                  vector<vector<int>> &obmap);

vector<vector<float>> GridAStar(vector<vector<int>> &obstlist,
                                pair<int, int> goal, int gres)
{
    int minx = INT32_MAX;
    int miny = INT32_MAX;
    vector<vector<int>> obmap = CalcObstMap(obstlist, gres, minx, miny);

    int goal_col = goal.first;
    int goal_row = goal.second;

    goal_col = ceil((goal_col - minx) / gres);
    goal_row = ceil((goal_row - miny) / gres);

    int dim_x = obmap.front().size();
    int dim_y = obmap.size();
    vector<vector<float>> costmap(dim_y, vector<float>(dim_x, 0));

    for (int i = 0; i < dim_y; i++)
    {
        for (int j = 0; j < dim_x; j++)
        {
            if (obmap[i][j] == 1)
            {
                costmap[i][j] = -1;
                continue;
            }
            else if (i == goal_row && j == goal_col)
            {
                continue;
            }
            pair<int, int> start = {j, i};
            float cost = AStarSearch(start, {goal_col, goal_row}, obmap);
            costmap[i][j] = cost;
        }
    }

    return costmap;
}

vector<vector<int>> CalcObstMap(vector<vector<int>> &obstlist, int gres,
                                int &minx, int &miny)
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

float AStarSearch(pair<int, int> start, pair<int, int> goal,
                  vector<vector<int>> &obmap)
{
    int dim_x = obmap.front().size();
    int dim_y = obmap.size();
    vector<PointWithValue> open;
    boost::unordered_map<pair<int, int>, pair<int, int>> parents;
    boost::unordered_map<pair<int, int>, float> close;
    open.push_back({start, 0});
    close[start] = 0;
    while (!open.empty())
    {
        sort(begin(open), end(open),
             [](PointWithValue a, PointWithValue b) { return a.value < b.value; });
        PointWithValue curr = open.front();
        for (int i : {-1, 0, 1})
        {
            for (int j : {-1, 0, 1})
            {
                pair<int, int> new_point = {curr.pos.first + i, curr.pos.second + j};
                if (i == 0 && j == 0)
                    continue;
                else if (new_point.first < 0 || new_point.first > dim_x ||
                         new_point.second < 0 || new_point.second > dim_y)
                    continue;
                else if (obmap[new_point.second][new_point.first] == 1)
                    continue;
                float new_cost = close[curr.pos] + powf(powf(i, 2) + powf(j, 2), 0.5);
                if (close.count(new_point) == 1)
                {
                    if (close[new_point] <= new_cost)
                        continue;
                }
                parents[new_point] = curr.pos;
                close[new_point] = new_cost;
                float priority = new_cost +
                                 powf(powf(goal.first - new_point.first, 2) +
                                          powf(goal.second - new_point.second, 2),
                                      0.5);
                open.push_back(PointWithValue({new_point, priority}));
            }
        }
        open.erase(begin(open));
        if (curr.pos.first == goal.first && curr.pos.second == goal.second)
            break;
    }
    float cost = close[goal];
    return cost;
}