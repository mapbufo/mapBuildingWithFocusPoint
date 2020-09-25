#pragma once
#include <cmath>
#include <string>
#include "CalcShortDist.h"
#include "RSPath.h"
#include "RSPathElem.h"
#include <iostream>
#include <fstream>
#include <Vehicle.h>
#include <common.h>
RSPath FindRSPath(double x, double y, double phi, Vehicle veh)
{
    // std::cerr << x << " " << y
    double rmin = veh.MIN_CIRCLE; // minimum turning radius
    x = x / rmin;
    y = y / rmin;
    std::vector<int> type = {RS_NOP, RS_NOP, RS_NOP, RS_NOP, RS_NOP};

    int candidateId = -1;

    RSPath path1(type, 0.0, 0.0, 0.0, 0.0, 0.0);
    bool isOk1 = CSC(x, y, phi, path1);
    RSPath path2(type, 0.0, 0.0, 0.0, 0.0, 0.0);
    bool isOk2 = CCC(x, y, phi, path2);
    RSPath path3(type, 0.0, 0.0, 0.0, 0.0, 0.0);
    bool isOk3 = CCCC(x, y, phi, path3);
    RSPath path4(type, 0.0, 0.0, 0.0, 0.0, 0.0);
    bool isOk4 = CCSC(x, y, phi, path4);
    RSPath path5(type, 0.0, 0.0, 0.0, 0.0, 0.0);
    bool isOk5 = CCSCC(x, y, phi, path5);

    std::vector<bool> isOkVec = {isOk1, isOk2, isOk3, isOk4, isOk5};
    std::vector<RSPath> RSPathVec = {path1, path2, path3, path4, path5};
    // std::cerr << isOk1 << " " << isOk2 << " " << isOk3 << " " << isOk4 << " " << isOk5 << " " << std::endl;
    // std::cerr << path1.totalLength_ << " " << path2.totalLength_ << " " << path3.totalLength_ << " " << path4.totalLength_ << " " << path5.totalLength_ << " " << std::endl;
    double Lmin = std::numeric_limits<float>::max();

    for (int i = 0; i < 5; i++)
    {
        if (isOkVec[i])
        {
            if (Lmin > RSPathVec[i].totalLength_)
            {
                Lmin = RSPathVec[i].totalLength_;
                candidateId = i;
            }
        }
    }
    if (candidateId >= 0)
    {
        // std::cerr << candidateId << std::endl;
        return RSPathVec[candidateId];
    }
}

enum RS
{
    STRAIGHT = 2,
    LEFT = 1,
    RIGHT = 3
};
std::vector<float> linspace(double start, double end, int size = 100)
{
    // std::cerr << start << " " << end << std::endl;
    std::vector<float> res;
    double diff = end - start;
    double step = diff / (1.0 * size);

    for (int i = 0; i < size; i++)
    {
        res.push_back(start + i * step);
    }
    return res;
}
void exportPath(RSPath path, Vehicle veh, Eigen::Vector3d start_point)
{

    std::vector<int> type = path.type_;
    std::vector<float> seg = {path.t_, path.u_, path.v_, path.w_, path.x_};
    for (auto s : seg)
    {
        // std::cerr << s << std::endl;
    }
    std::vector<double> pvec = {start_point[0], start_point[1], start_point[2]};
    double rmin = veh.MIN_CIRCLE; // minimum turning radius

    const int const_straight = RS_STRAIGHT;
    const int const_left = RS_LEFT;
    const int const_right = RS_RIGHT;
    std::vector<std::pair<std::vector<float>, std::vector<float>>> full_path;
    std::vector<int> full_path_direction = {0, 0, 0, 0, 0}; // +1: forwards; -1: backwards
    for (int i = 0; i < 5; i++)
    {
        switch (type[i])
        {
        case RS::STRAIGHT:
        {

            double theta = pvec[2];
            double dl = rmin * seg[i];
            std::vector<float> dvec_x = linspace(pvec[0], pvec[0] + dl * cos(theta));
            std::vector<float> dvec_y = linspace(pvec[1], pvec[1] + dl * sin(theta));

            full_path.push_back({dvec_x, dvec_y});
            pvec[0] += dl * cos(theta);
            pvec[1] += dl * sin(theta);
            pvec[2] += 0;

            if (dl > 0)
            {
                full_path_direction[i] = 1;
            }
            else
            {

                full_path_direction[i] = -1;
            }

            break;
        }

        case RS::LEFT:
        {

            double theta = pvec[2];
            double dtheta = seg[i];

            double cenx = pvec[0] - rmin * sin(theta);
            double ceny = pvec[1] + rmin * cos(theta);

            std::vector<float> tvec = linspace(theta - M_PI / 2.0, theta - M_PI / 2.0 + dtheta);

            std::vector<float> dvec_x;
            std::vector<float> dvec_y;
            for (auto t : tvec)
            {
                dvec_x.push_back(cenx + rmin * cos(t));
                dvec_y.push_back(ceny + rmin * sin(t));
            }
            full_path.push_back({dvec_x, dvec_y});

            pvec[0] = *(dvec_x.end() - 1);
            pvec[1] = *(dvec_y.end() - 1);
            pvec[2] += dtheta;

            double dl = dtheta;
            if (dl > 0)
            {
                full_path_direction[i] = 1;
            }
            else
            {

                full_path_direction[i] = -1;
            }
            break;
        }

        case RS::RIGHT:
        {
            double theta = pvec[2];
            double dtheta = -seg[i];

            double cenx = pvec[0] + rmin * sin(theta);
            double ceny = pvec[1] - rmin * cos(theta);

            std::vector<float> tvec = linspace(theta + M_PI / 2.0, theta + M_PI / 2.0 + dtheta);

            std::vector<float> dvec_x;
            std::vector<float> dvec_y;
            for (auto t : tvec)
            {
                dvec_x.push_back(cenx + rmin * cos(t));
                dvec_y.push_back(ceny + rmin * sin(t));
            }
            full_path.push_back({dvec_x, dvec_y});

            pvec[0] = *(dvec_x.end() - 1);
            pvec[1] = *(dvec_y.end() - 1);
            pvec[2] += dtheta;

            double dl = -dtheta;

            if (dl > 0)
            {
                full_path_direction[i] = 1;
            }
            else
            {

                full_path_direction[i] = -1;
            }
            break;
        }

        default:
        {

            break;
        }
        }
    }

    // export to file
    int num_pts = 0;
    for (int i = 0; i < full_path.size(); i++)
    {
        num_pts += full_path[i].first.size();
    }

    std::ofstream output_file;

    output_file.open("path.txt");

    // set points
    for (int i = 0; i < full_path.size(); i++)
    {

        for (int j = 0; j < full_path[i].first.size(); j++)
        {
            output_file << full_path[i].first[j] << " " << full_path[i].second[j] << " " << 0 << " "
                        << " " << full_path_direction[i] << std::endl;
        }
    }
    output_file.close();
}
