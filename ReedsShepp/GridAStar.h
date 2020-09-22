#pragma once
#include <cmath>
#include <cstdint>
#include <vector>
#include <iostream>
#include <boost/unordered_map.hpp>
#include <algorithm>
#include "common.h"

using namespace std;

struct PointWithValue
{
    Point2D pos;
    double value;
};

// the obstlist is formed by 2-dim vector, the fir.dim vector represents the
// row, but the first element of the sec.dem vector represents the number of
// row, which means, the row with none element will not be saved. In the sec.dim
// the numbers from second element represent the number of column.
vector<vector<int>> CalcObstMap(vector<Point2D> &obstlist, double gres,
                                double minx, double miny, double maxx, double maxy);

double AStarSearch(Point2D start, Point2D goal,
                   vector<vector<int>> &obmap);

vector<vector<double>> GridAStar(vector<Point2D> &obstlist,
                                 Point2D goal, double gres, double minx, double miny, double maxx, double maxy)
{
    vector<vector<int>> obmap = CalcObstMap(obstlist, gres, minx, miny, maxx, maxy);

    double goal_col = goal.first;
    double goal_row = goal.second;

    goal_col = ceil((goal_col - minx) / gres);
    goal_row = ceil((goal_row - miny) / gres);

    int dim_x = obmap.front().size();
    int dim_y = obmap.size();
    vector<vector<double>> costmap(dim_y, vector<double>(dim_x, 0));

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
            Point2D start = {j, i};
            double cost = AStarSearch(start, {goal_col, goal_row}, obmap);
            costmap[i][j] = cost;
        }
    }

    return costmap;
}

vector<vector<int>> CalcObstMap(vector<Point2D> &obstlist, double gres,
                                double minx, double miny, double maxx, double maxy)
{
    int xwidth = maxx - minx;
    xwidth = ceil(xwidth / gres);
    int ywidth = maxy - miny;
    ywidth = ceil(ywidth / gres) + 1;
    vector<vector<int>> obmap(ywidth, vector<int>(xwidth, 0));

    //need some tricks to connect the obstacles instead of using knnsearch
    for (auto point : obstlist)
    {
        int row = ceil((maxy - point.second) / gres);
        int column = floor((point.first - minx) / gres);
        obmap[row][column] = 1;
    }
    return obmap;
}

double AStarSearch(Point2D start, Point2D goal,
                   vector<vector<int>> &obmap)
{
    int dim_x = obmap.front().size();
    int dim_y = obmap.size();
    vector<PointWithValue> open;
    boost::unordered_map<Point2D, Point2D> parents;
    boost::unordered_map<Point2D, double> close;
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
                Point2D new_point = {curr.pos.first + i, curr.pos.second + j};
                if (i == 0 && j == 0)
                    continue;
                else if (new_point.first < 0 || new_point.first >= dim_x ||
                         new_point.second < 0 || new_point.second >= dim_y)
                    continue;
                else if (obmap[new_point.second][new_point.first] == 1)
                    continue;
                double new_cost = close[curr.pos] + powf(powf(i, 2) + powf(j, 2), 0.5);
                if (close.count(new_point) == 1)
                {
                    if (close[new_point] <= new_cost)
                        continue;
                }
                parents[new_point] = curr.pos;
                close[new_point] = new_cost;
                double priority = new_cost +
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
    double cost = close[goal];
    return cost;
}