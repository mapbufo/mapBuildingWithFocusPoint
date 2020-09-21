#pragma once
#include <cmath>
#include <string>
#include <vector>
#include <common.h>
class Node
{
public:
    int xIdx;
    int yIdx;
    int yawIdx;
    double D;
    double Delta;
    double X;
    double Y;
    double Theta;
    Eigen::Vector3d Parent;
    double Cost;

    Node()
    {
        xIdx = 0;
        yIdx = 0;
        yawIdx = 0;
        D = 0;
        Delta = 0;
        X = 0;
        Y = 0;
        Theta = 0;
        Parent = Eigen::Vector3d(0, 0, 0);
        Cost = std::numeric_limits<double>::max();
    }

    Node(int xidx,
         int yidx,
         int yawidx,
         double d,
         double delta,
         double x,
         double y,
         double theta,
         Eigen::Vector3d parent,
         double cost)
    {
        xIdx = xidx;
        yIdx = yidx;
        yawIdx = yawidx;
        D = d;
        Delta = delta;
        X = x;
        Y = y;
        Theta = theta;
        Parent = parent;
        Cost = cost;
    }

    void update(Node node)
    {
        xIdx = node.xIdx;
        yIdx = node.yIdx;
        yawIdx = node.yawIdx;
        D = node.D;
        Delta = node.Delta;
        X = node.X;
        Y = node.Y;
        Theta = node.Theta;
        Parent = node.Parent;
        Cost = node.Cost;
    }
};