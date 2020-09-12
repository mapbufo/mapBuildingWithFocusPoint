#pragma once
#include "RSPath.h"
#include "FormularList.h"
#include <iostream>
#include <limits>
#include <string>
#include "RSPathElem.h"

bool CCC(float x, float y, float phi, RSPath &path)
{
    float Lmin = std::numeric_limits<float>::max();
    std::vector<int> type = {RS_NOP, RS_NOP, RS_NOP, RS_NOP, RS_NOP};
    path.update(type, 0.0, 0.0, 0.0, 0.0, 0.0);

    float t, u, v = 0.0;
    bool isOk = false;
    // normal
    isOk = LpRmL(x, y, phi, t, u, v);
    // std::cerr << t << " " << u << " " << v << " " << 0 << " " << 0 << " " << std::endl;

    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type01"], t, u, v, 0, 0);
        }
    }

    // timeflip
    isOk = LpRmL(-x, y, -phi, t, u, v);
    // std::cerr << t << " " << u << " " << v << " " << 0 << " " << 0 << " " << std::endl;

    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type01"], -t, -u, -v, 0, 0);
        }
    }

    // reflect
    isOk = LpRmL(x, -y, -phi, t, u, v);
    // std::cerr << t << " " << u << " " << v << " " << 0 << " " << 0 << " " << std::endl;

    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type02"], t, u, v, 0, 0);
        }
    }

    // timeflip + reflect
    isOk = LpRmL(-x, -y, phi, t, u, v);
    // std::cerr << t << " " << u << " " << v << " " << 0 << " " << 0 << " " << std::endl;

    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type02"], -t, -u, -v, 0, 0);
        }
    }

    // backwards

    // normal
    float xb = x * cos(phi) + y * sin(phi);
    float yb = x * sin(phi) - y * cos(phi);

    isOk = LpRmL(xb, yb, phi, t, u, v);
    // std::cerr << t << " " << u << " " << v << " " << 0 << " " << 0 << " " << std::endl;

    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type01"], v, u, t, 0, 0);
        }
    }
    // timeflip
    isOk = LpRmL(-xb, yb, -phi, t, u, v);
    // std::cerr << t << " " << u << " " << v << " " << 0 << " " << 0 << " " << std::endl;

    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type01"], -v, -u, -t, 0, 0);
        }
    }
    // reflect
    isOk = LpRmL(xb, -yb, -phi, t, u, v);
    // std::cerr << t << " " << u << " " << v << " " << 0 << " " << 0 << " " << std::endl;

    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type02"], v, u, t, 0, 0);
        }
    }

    // timeflip + reflect
    isOk = LpRmL(-xb, -yb, phi, t, u, v);
    // std::cerr << t << " " << u << " " << v << " " << 0 << " " << 0 << " " << std::endl;

    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type02"], -v, -u, -t, 0, 0);
        }
    }

    if (Lmin == std::numeric_limits<float>::max())
    {
        isOk = false;
    }
    else
    {
        isOk = true;
    }

    return isOk;
}
