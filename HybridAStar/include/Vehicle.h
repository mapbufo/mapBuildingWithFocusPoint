#pragma once
#include <cmath>
#include <string>
#include <vector>

class Vehicle
{
public:
    double WB;
    double W;
    double LF;
    double LB;
    double MAX_STEER;
    double MIN_CIRCLE;

    Vehicle()
    {
        WB = 3.7;
        W = 2.6;
        LF = 4.5;
        LB = 1.0;
        MAX_STEER = 0.6;
        MIN_CIRCLE = WB / std::tan(MAX_STEER);
    }
    Vehicle(double wb, double w, double lf, double lb, double max_steer)
    {
        WB = wb;
        W = w;
        LF = lf;
        LB = lb;
        MAX_STEER = max_steer;
        MIN_CIRCLE = WB / std::tan(MAX_STEER);
    }
};