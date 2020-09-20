#pragma once
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <Vehicle.h>

typedef std::pair<double, double> Point2D;

// https://blog.csdn.net/HelloZEX/article/details/80880385
bool collisionCheck(std::vector<std::pair<Point2D, Point2D>> Object, std::pair<Point2D, Point2D> Line)
{
    for (size_t idx = 0; idx < Object.size(); ++idx)
    {
        // A
        Point2D A = Object[idx].first;
        // B
        Point2D B = Object[idx].second;
        // C
        Point2D C = Line.first;
        // D
        Point2D D = Line.second;

        if (std::fmax(C.first, D.first) < std::fmin(A.first, B.first) ||
            std::fmax(C.second, D.second) < std::fmin(A.second, B.second) ||
            std::fmax(A.first, B.first) < std::fmin(C.first, D.first) ||
            std::fmax(A.second, B.second) < std::fmin(C.second, D.second))
        {
        }
        else
        {
            // std::cerr << "here" << std::endl;
            // std::cerr << A.first << " " << A.second << std::endl;
            // std::cerr << B.first << " " << B.second << std::endl;
            // std::cerr << C.first << " " << C.second << std::endl;
            // std::cerr << D.first << " " << D.second << std::endl;

            // std::cerr << "condition 1.1: " << ((A.first - C.first) * (D.second - C.second) - (A.second - C.second) * (D.first - C.first)) << std::endl;
            // std::cerr << "condition 1.2: " << ((B.first - C.first) * (D.second - C.second) - (B.second - C.second) * (D.first - C.first)) << std::endl;
            // std::cerr << "condition 2.1: " << ((C.first - A.first) * (B.second - A.second) - (C.second - A.second) * (B.second - A.first)) << std::endl;
            // std::cerr << "condition 2.2: " << ((D.first - A.first) * (B.second - A.second) - (D.second - A.second) * (B.second - A.first)) << std::endl;

            if ((((A.first - C.first) * (D.second - C.second) - (A.second - C.second) * (D.first - C.first)) *
                 ((B.first - C.first) * (D.second - C.second) - (B.second - C.second) * (D.first - C.first))) > 0 ||
                (((C.first - A.first) * (B.second - A.second) - (C.second - A.second) * (B.second - A.first)) *
                 ((D.first - A.first) * (B.second - A.second) - (D.second - A.second) * (B.second - A.first))) > 0)
            {
            }
            else
            {
                return true;
            }
        }
    }
    return false;
}

bool VehicleCollisionCheck(Eigen::Vector3d pVec, std::vector<std::pair<Point2D, Point2D>> ObstLine, Vehicle veh)
{
    double W = veh.W;
    double LF = veh.LF;
    double LB = veh.LB;

    Point2D Cornerfl(LF, W / 2.0);
    Point2D Cornerfr(LF, -W / 2.0);
    Point2D Cornerrl(-LB, W / 2.0);
    Point2D Cornerrr(-LB, -W / 2.0);

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

    std::vector<std::pair<Point2D, Point2D>>
        Rect;
    Rect.push_back({Cornerfl, Cornerfr});
    Rect.push_back({Cornerfr, Cornerrr});
    Rect.push_back({Cornerrr, Cornerrl});
    Rect.push_back({Cornerrl, Cornerfl});

    // std::cerr << Cornerfl.first << " " << Cornerfl.second << std::endl;
    // std::cerr << Cornerfr.first << " " << Cornerfr.second << std::endl;
    // std::cerr << Cornerrr.first << " " << Cornerrr.second << std::endl;
    // std::cerr << Cornerrl.first << " " << Cornerrl.second << std::endl;

    bool isCollision = false;
    for (size_t i; i < ObstLine.size(); ++i)
    {

        isCollision = collisionCheck(Rect, ObstLine[i]);
        if (isCollision)
            return true;
    }
    return false;
}
