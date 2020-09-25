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
    else if (v > M_PI)
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
    std::cerr << xidx << " " << yidx << std::endl;
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

bool AnalysticExpansion(Eigen::Vector3d Start, Eigen::Vector3d End, Vehicle veh, Configuration cfg, RSPath &rspath)
{
    bool isok = true;
    bool isCollision = false;
    // 将起点转换到原点计算轨迹，变换坐标系了
    Eigen::Vector3d pvec(End - Start);
    double pos_x = pvec[0];
    double pos_y = pvec[1];
    double pos_phi = Start[2];
    pos_phi = mod2pi(pos_phi);

    /* 
    replaced angle2dcm in matlab: (pos_phi, 0, 0)

        dcm(1,1,:) = cang(:,2).*cang(:,1); -> cos(r1)
        dcm(1,2,:) = cang(:,2).*sang(:,1); -> sin(r1)
        dcm(1,3,:) = -sang(:,2); -> 0
        dcm(2,1,:) = sang(:,3).*sang(:,2).*cang(:,1) - cang(:,3).*sang(:,1); -> -sin(r1)
        dcm(2,2,:) = sang(:,3).*sang(:,2).*sang(:,1) + cang(:,3).*cang(:,1); -> cos(r1)
        dcm(2,3,:) = sang(:,3).*cang(:,2); -> 0
        dcm(3,1,:) = cang(:,3).*sang(:,2).*cang(:,1) + sang(:,3).*sang(:,1); -> 0
        dcm(3,2,:) = cang(:,3).*sang(:,2).*sang(:,1) - sang(:,3).*cang(:,1); -> 0
        dcm(3,3,:) = cang(:,3).*cang(:,2); -> 1
    
    * note: this is different from the normal roration matrix -> [[cos, -sin, 0], [sin, cos, 0], [0, 0, 1]]
    */

    Eigen::Matrix3d dcm; // 起点start坐标系在基坐标系下的方向余弦矩阵
    dcm << cos(pos_phi), sin(pos_phi), 0,
        -sin(pos_phi), cos(pos_phi), 0,
        0, 0, 1;
    // dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
    Eigen::Vector3d tvec = dcm * Eigen::Vector3d(pos_x, pos_y, 0); // 计算坐标旋转后各向量在起点start坐标中的表示
    pos_x = tvec[0];
    pos_y = tvec[1];

    double rmin = veh.MIN_CIRCLE;
    double smax = veh.MAX_STEER;
    double mres = cfg.MOTION_RESOLUTION;
    std::vector<std::pair<Point2D, Point2D>> obstline = cfg.ObstLine;

    rspath = FindRSPath(pos_x, pos_y, pvec[2], veh);

    std::vector<int>
        types = rspath.type_;
    double t = rmin * rspath.t_;
    double u = rmin * rspath.u_;
    double v = rmin * rspath.v_;
    double w = rmin * rspath.w_;
    double x = rmin * rspath.x_;
    std::vector<double> segs = {t, u, v, w, x};
    pvec[0] = Start[0];
    pvec[1] = Start[1];
    pvec[2] = Start[2];

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
                std::cerr << "in check" << std::endl;
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

bool CalcNextNode(Node wknode, double D, double delta, Vehicle veh, Configuration cfg, Node &tnode)
{
    bool isok = true;
    bool isCollision = false;
    double px = wknode.X;
    double py = wknode.Y;
    double pth = wknode.Theta;
    double gres = cfg.XY_GRID_RESOLUTION;
    std::vector<std::pair<Point2D, Point2D>> obstline = cfg.ObstLine;

    int nlist = std::floor(gres * 1.5 / cfg.MOTION_RESOLUTION) + 1;
    std::vector<double> pos_x(nlist + 1, 0);
    std::vector<double> pos_y(nlist + 1, 0);
    std::vector<double> pos_th(nlist + 1, 0);
    pos_x[0] = px;
    pos_y[0] = py;
    pos_th[0] = pth;

    for (int idx = 0; idx < nlist; ++idx)
    {
        VehicleDynamic(px, py, pth, D, delta, veh.WB);
        pos_x[idx + 1] = px;
        pos_y[idx + 1] = py;
        pos_th[idx + 1] = pth;
        if (idx % 5 == 0)
        {
            Eigen::Vector3d tvec(px, py, pth);
            isCollision = VehicleCollisionCheck(tvec, obstline, veh);
            if (isCollision)
                break;
        }
    }

    tnode.update(wknode);
    if (isCollision)
    {
        isok = false;
        return isok;
    }
    else
    {
        int xidx, yidx, thidx;
        isok = CalcIdx(px, py, pth, cfg, xidx, yidx, thidx);
        if (!isok)
        {
            return isok;
        }
        else
        {
            double cost = wknode.Cost;
            if (D > 0)
            {
                cost += gres * 1.5;
            }
            else
            {
                cost += cfg.BACK_COST * gres * 1.5;
            }
            if (D != wknode.D)
            {
                cost += cfg.SB_COST;
            }
            cost += cfg.STEER_COST * std::fabs(delta);
            cost += cfg.STEER_CHANGE_COST * std::fabs(delta - wknode.Delta);
            Node tempNode(xidx, yidx, thidx, D, delta, px, py, pth, Eigen::Vector3d(wknode.xIdx, wknode.yIdx, wknode.yawIdx), cost);
            tnode.update(tempNode);
        }
    }

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

    std::vector<double> mres_vec = {-mres, mres};
    std::vector<double> sres_vec;
    double cur_sres = -smax;
    do
    {
        sres_vec.push_back(cur_sres);
        cur_sres += sres;
    } while (cur_sres <= smax);

    for (auto D : mres_vec)
    {
        for (auto delta : sres_vec)
        {
            Node tnode;
            bool isok1 = CalcNextNode(wknode, D, delta, veh, cfg, tnode);
            if (!isok1)
            {
                continue;
            }
            int idx2 = -1;
            bool isok2 = inNodes(tnode, Close, idx2);
            if (isok2)
            {
                continue;
            }

            int idx3 = -1;
            bool isok3 = inNodes(tnode, Open, idx3);
            if (isok3)
            {
                double tcost = TotalCost(tnode, cfg);
                Node ttnode;
                ttnode.update(Open[idx3]);
                double ttcost = TotalCost(ttnode, cfg);
                if (tcost < ttcost)
                {
                    Open[idx3].update(tnode);
                }
            }
            else
            {
                Open.push_back(tnode);
            }
        }
    }
}
//getFinalPath
void getFinalPath(RSPath rspath, std::vector<Node> Close, Vehicle veh, Configuration cfg, std::vector<double> &x_vec, std::vector<double> &y_vec, std::vector<double> &th_vec, std::vector<double> &D_vec, std::vector<double> &delta_vec)
{
    Node wknode;
    wknode.update(Close[Close.size() - 1]);
    Close.erase(Close.end() - 1);
    std::vector<Node> nodes = {wknode};

    while (!Close.empty())
    {
        size_t n = Close.size();
        Eigen::Vector3d parent = wknode.Parent;

        for (size_t i = n - 1; i >= 0; --i)
        {
            Node tnode;
            tnode.update(Close[i]);
            if (tnode.xIdx == parent[0] || tnode.yIdx == parent[1] || tnode.yawIdx == parent[2])
            {
                nodes.push_back(tnode);
                wknode.update(tnode);
                Close.erase(Close.begin() + i);
                break;
            }
        }
    }
    double rmin = veh.MIN_CIRCLE;
    double smax = veh.MAX_STEER;
    double mres = cfg.MOTION_RESOLUTION;
    double gres = cfg.XY_GRID_RESOLUTION;

    double nlist = std::floor(gres * 1.5 / cfg.MOTION_RESOLUTION) + 1;

    int flag = 0;

    if (nodes.size() >= 2)
    {
        for (int i = nodes.size() - 1; i >= 1; --i)
        {
            Node tnode;
            tnode.update(nodes[i]);
            Node ttnode;
            ttnode.update(nodes[i - 1]);

            double px = tnode.X;
            double py = tnode.Y;
            double pth = tnode.Theta;

            x_vec.push_back(px);
            y_vec.push_back(py);
            th_vec.push_back(pth);
            D_vec.push_back(ttnode.D);
            delta_vec.push_back(ttnode.Delta);

            for (int idx = 0; idx < nlist; ++idx)
            {
                VehicleDynamic(px, py, pth, ttnode.D, ttnode.Delta, veh.WB);
                x_vec.push_back(px);
                y_vec.push_back(py);
                th_vec.push_back(pth);
                D_vec.push_back(ttnode.D);
                delta_vec.push_back(ttnode.Delta);
            }

            if (i != 1)
            {
                x_vec.erase(x_vec.end() - 1);
                y_vec.erase(y_vec.end() - 1);
                th_vec.erase(th_vec.end() - 1);
                D_vec.erase(D_vec.end() - 1);
                delta_vec.erase(delta_vec.end() - 1);
            }
        }
    }
    else
    {
        flag = 1;
        Node tnode;
        tnode.update(nodes[0]);
        double px = tnode.X;
        double py = tnode.Y;
        double pth = tnode.Theta;
        x_vec.push_back(px);
        y_vec.push_back(py);
        th_vec.push_back(pth);
    }

    std::vector<int> types = rspath.type_;
    double t = rmin * rspath.t_;
    double u = rmin * rspath.u_;
    double v = rmin * rspath.v_;
    double w = rmin * rspath.w_;
    double x = rmin * rspath.x_;

    std::vector<double> segs = {t, u, v, w, rmin * rspath.x_};

    for (int i = 0; i < 5; ++i)
    {
        if (segs[i] == 0)
            continue;

        double direction = 0;
        if (segs[i] > 0)
        {
            direction = 1;
        }
        else if (segs[i] < 0)
        {
            direction = -1;
        }
        double tdelta;
        if (types[i] == RS_STRAIGHT)
        {
            tdelta = 0;
        }
        else if (types[i] == RS_LEFT)
        {
            tdelta = smax;
        }
        else if (types[i] == RS_RIGHT)
        {
            tdelta = -smax;
        }
        else
        {
            // do nothing
        }
        if (flag == 1)
        {
            D_vec.push_back(direction * mres);
            delta_vec.push_back(tdelta);
            flag = 0;
        }
        for (int idx = 0; idx < std::round(std::fabs(segs[i]) / mres); ++idx)
        {
            double px, py, pth;
            px = x_vec[x_vec.size() - 1];
            py = y_vec[y_vec.size() - 1];
            pth = th_vec[th_vec.size() - 1];

            VehicleDynamic(px, py, pth, direction * mres, tdelta, veh.WB);
            x_vec.push_back(px);
            y_vec.push_back(py);
            th_vec.push_back(pth);
            D_vec.push_back(direction * mres);
            delta_vec.push_back(tdelta);
        }
    }
}
void HybridAStar(Eigen::Vector3d &Start, Eigen::Vector3d &End, Vehicle &veh, Configuration &cfg, std::vector<double> &x_vec, std::vector<double> &y_vec, std::vector<double> &th_vec, std::vector<double> &D_vec, std::vector<double> &delta_vec)
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

    while (!Open.empty())
    {
        std::cerr << Close.size() << std::endl;
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
        RSPath rspath;
        bool isok2 = AnalysticExpansion(Eigen::Vector3d(wknode.X, wknode.Y, wknode.Theta), End, veh, cfg, rspath);
        if (isok2)
        {
            std::cerr << "rs path " << std::endl;
            int idx2 = -1;
            inNodes(wknode, Close, idx2);
            Close.push_back(wknode);
            Close.erase(Close.begin() + idx2);

            getFinalPath(rspath, Close, veh, cfg, x_vec, y_vec, th_vec, D_vec, delta_vec);
            break;
        }
        std::cerr << "not rs path " << std::endl;
        Update(wknode, Open, Close, veh, cfg);
    }
}