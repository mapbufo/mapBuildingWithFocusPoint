#pragma once
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <common.h>
#include <Vehicle.h>
#include <GridAStar.h>
#include <VehicleCollisionCheck.h>
#include <Config.h>
#include <Node.h>
#include <PathFunction.h>

double mod2pi(double x)
{
    double v = x;
    while (std::fabs(v) >= 2 * M_PI)
    {
        if (v > 0)
            v -= 2 * M_PI;
        else
            v += 2 * M_PI;
    }
    if (v < -M_PI)
        v += 2 * M_PI;
    else
        v -= 2 * M_PI;

    return v;
}

bool CalcIdx(double x, double y, double theta, Configuration cfg, int &xidx, int &yidx, int &thidx)
{
    double gres = cfg.XY_GRID_RESOLUTION;
    double yawres = cfg.YAW_GRID_RESOLUTION;
    xidx = std::ceil((x - cfg.MINX) / gres);
    yidx = std::ceil((y - cfg.MINY) / gres);
    theta = mod2pi(theta);
    thidx = std::ceil((theta - cfg.MINYAW) / yawres);
    bool isok = true;
    if (xidx <= 0 || xidx > std::ceil((cfg.MAXX - cfg.MINX) / gres))
    {
        isok = false;
    }
    else if (yidx <= 0 || yidx > ceil((cfg.MAXY - cfg.MINY) / gres))
    {
        isok = false;
    }
    vector<vector<double>> costmap = cfg.ObstMap;
    if (costmap[yidx][xidx] > 10e5)
    {
        isok = false;
    }
    return isok;
}

void VehicleDynamic(double &x, double &y, double &theta, double D, double delta, double L)
{
    x += D * std::cos(theta);
    y += D * std::sin(theta);
    theta += D * 1.0 / L * std::tan(delta);
    theta = mod2pi(theta);
}

bool AnalysticExpansion(Eigen::Vector3d Start, Eigen::Vector3d End, Vehicle veh, Configuration cfg, std::vector<Point2D> &path)
{
    bool isok = true;
    bool isCollision = false;

    Eigen::Vector3d pvec(End - Start);
    double pos_x = pvec[0];
    double pos_y = pvec[1];
    double pos_phi = Start[2];
    pos_phi = mod2pi(pos_phi);
    Eigen::Matrix3d dcm;
    dcm << cos(pos_phi), -sin(pos_phi), 0,
        sin(pos_phi), cos(pos_phi), 0,
        0, 0, 1;

    Eigen::Vector3d tvec = dcm * Eigen::Vector3d(pos_x, pos_y, 0);
    pos_x = tvec[0];
    pos_y = tvec[1];

    double rmin = veh.MIN_CIRCLE;
    double smax = veh.MAX_STEER;
    double mres = cfg.MOTION_RESOLUTION;
    std::vector<std::pair<Point2D, Point2D>> obstline = cfg.ObstLine;

    RSPath rspath = FindRSPath(pos_x, pos_y, pvec[2], veh);

    std::vector<int> types = rspath.type_;
    double t = rmin * rspath.t_;
    double u = rmin * rspath.u_;
    double v = rmin * rspath.v_;
    double w = rmin * rspath.w_;
    double x = rmin * rspath.x_;
    std::vector<double> segs = {t, u, v, w, x};
    pvec[0]  =Start[0];
    pvec[1]  =Start[1];
    pvec[2]  =Start[2];
    
    for (int i = 0; i < 5; ++i)
    {
        if (segs[i] == 0)
            continue;
        double px = pvec[0];
        double py = pvec[1];
        double pth = pvec[2];

        int direction = 0; // 1: forward, -1: backward
        if (segs[i] > 0)
        {
            direction = 1;
        }
        else if (segs[i] < 0)
        {
            direction = -1;
        }
        double D = direction * mres;
        double delta = 0;
        if (types[i] == RS_STRAIGHT) // straight
        {
            delta = 0;
        }
        else if (types[i] == RS_LEFT)
        {
            delta = smax;
        }
        else if (types[i] == RS_RIGHT)
        {
            delta = -smax;
        }
        else
        {
        }

        for (int idx = 0; idx < round(fabs(segs[i]) / mres); ++idx)
        {
            VehicleDynamic(px, py, pth, D, delta, veh.WB);
            if (idx % 5 == 0)
            {
                Eigen::Vector3d tvec(px, py, pth);
                isCollision = VehicleCollisionCheck(tvec, obstline, veh);
                if (isCollision)
                {
                    isok = false;
                    break;
                }
            }
        }
        pvec[0] = px;
        pvec[1] = py;
        pvec[2] = pth;
    }
    if (mod2pi(pvec[2]) - End[2] > 5.0 * M_PI / 180.0)
    {
        isok = false;
    }
    return isok;
}

double TotalCost(Node wknode, Configuration cfg)
{
    double gres = cfg.XY_GRID_RESOLUTION;
    std::vector<std::vector<double>> costmap = cfg.ObstMap;
    double cost = cfg.H_COST * costmap[wknode.yIdx][wknode.xIdx];
    double xshift = wknode.X - (gres * (wknode.xIdx - 0.5) + cfg.MINX);
    double yshift = wknode.Y - (gres * (wknode.yIdx - 0.5) + cfg.MINY);

    cost += cfg.H_COST * sqrt(xshift * xshift + yshift * yshift);
    cost += wknode.Cost;
    return cost;
}

Node PopNode(std::vector<Node> nodes, Configuration cfg)
{
    double mincost = std::numeric_limits<double>::max();
    int minidx = 0;
    double gres = cfg.XY_GRID_RESOLUTION;
    for (int idx = 0; idx < nodes.size(); ++idx)
    {
        double tcost = TotalCost(nodes[idx], cfg);
        if (tcost < mincost)
        {
            mincost = tcost;
            minidx = idx;
        }
    }
    Node wknode;
    wknode.update(nodes[minidx]);
    nodes.erase(nodes.begin() + minidx);
    return wknode;
}

bool CalcNextNode(Node wknode, double D, double delta, double veh, Configuration cfg, Node &tnode)
{
    return true;
}

bool inNodes(Node node, std::vector<Node> &nodes, int &idx)
{

    for (int i = 0; i < nodes.size(); ++i)
    {
        if (node.xIdx == nodes[i].xIdx &&
            node.yIdx == nodes[i].yIdx &&
            node.yawIdx == nodes[i].yawIdx)
        {
            idx = i;
            return true;
        }
    }

    return false;
}
// update
void Update(Node wknode, std::vector<Node> &Open, std::vector<Node> &Close, Vehicle veh, Configuration cfg)
{
    double mres = cfg.MOTION_RESOLUTION;
    double smax = veh.MAX_STEER;
    double sres = smax * 1.0 / cfg.N_STEER;
}
//getFinalPath
void getFinalPath(std::vector<Point2D> path, std::vector<Node> Close, Vehicle veh, Configuration cfg, std::vector<double> &x, std::vector<double> &y, std::vector<double> &theta, std::vector<double> &D, std::vector<double> &delta)
{
}
void HybridAStar(Eigen::Vector3d Start, Eigen::Vector3d End, Vehicle veh, Configuration cfg, std::vector<Eigen::Vector3d> &posVec, std::vector<double> &D, std::vector<double> &delta)
{
    float mres = cfg.MOTION_RESOLUTION;

    int xidx, yidx, thidx;
    xidx = yidx = thidx = 0;

    bool isok = CalcIdx(Start[0], Start[1], Start[2], cfg, xidx, yidx, thidx);
    Node tnode;
    if (isok)
    {
        tnode = Node(xidx, yidx, thidx, mres, 0, Start[0], Start[1], Start[2], Eigen::Vector3d(xidx, yidx, thidx), 0);
    }

    std::vector<Node> Open;
    Open.push_back(tnode);
    std::vector<Node> Close;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> th;

    while (!Open.empty())
    {
        Node wknode;
        wknode.update(PopNode(Open, cfg));
        int idx1 = -1;
        bool isok = inNodes(wknode, Close, idx1);
        if (isok)
        {
            Close[idx1].update(wknode);
        }
        else
        {
            Close.push_back(wknode);
        }
        std::vector<Point2D> rspath;
        bool isok2 = AnalysticExpansion(Eigen::Vector3d(wknode.X, wknode.Y, wknode.Theta), End, veh, cfg, rspath);
        if (isok2)
        {
            int idx2 = -1;
            inNodes(wknode, Close, idx2);
            Close.push_back(wknode);
            Close.erase(Close.begin() + idx2);

            getFinalPath(rspath, Close, veh, cfg, x, y, th, D, delta);
            break;
        }
        Update(wknode, Open, Close, veh, cfg);
    }
}