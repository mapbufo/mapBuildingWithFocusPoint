#pragma once
#include <cmath>
#include <string>
#include <vector>
#include <common.h>

class Configuration
{
public:
    std::vector<Point2D> ObstList;
    std::vector<std::pair<Point2D, Point2D>> ObstLine;
    vector<vector<double>> ObstMap;
    double MOTION_RESOLUTION;
    double N_STEER;
    double EXTEND_AREA;
    double XY_GRID_RESOLUTION;
    double YAW_GRID_RESOLUTION;

    double MINX;
    double MAXX;
    double MINY;
    double MAXY;
    double MINYAW;
    double MAXYAW;

    double SB_COST;
    double BACK_COST;
    double STEER_CHANGE_COST;
    double STEER_COST;
    double H_COST;

    Configuration(std::vector<Point2D> obstList, std::vector<std::pair<Point2D, Point2D>> obstLine)
    {
        ObstList = obstList;
        ObstLine = obstLine;

        MOTION_RESOLUTION = 0.1;
        N_STEER = 20.0;
        EXTEND_AREA = 0;
        XY_GRID_RESOLUTION = 2.0;
        YAW_GRID_RESOLUTION = 15.0 * M_PI / 180.0;

        MINX = ObstList[0].first;
        MAXX = ObstList[0].first;
        MINY = ObstList[0].second;
        MAXY = ObstList[0].second;
        for (auto obst : ObstList)
        {
            MINX = std::min(MINX, obst.first);
            MAXX = std::max(MAXX, obst.first);
            MINY = std::min(MINY, obst.second);
            MAXY = std::max(MAXY, obst.second);
        }
        MINX -= EXTEND_AREA;
        MAXX += EXTEND_AREA;
        MINY -= EXTEND_AREA;
        MAXY += EXTEND_AREA;

        MINYAW = -M_PI;
        MAXYAW = M_PI;

        SB_COST = 0;
        BACK_COST = 1.5;
        STEER_CHANGE_COST = 1.5;
        STEER_COST = 1.5;
        H_COST = 10;
    }
};