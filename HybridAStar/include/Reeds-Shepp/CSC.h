#pragma once
#include "RSPath.h"
#include "FormularList.h"
#include <iostream>
#include <limits>
#include <string>
#include "RSPathElem.h"

bool CSC(float x, float y, float phi, RSPath &path)
{
    float Lmin = std::numeric_limits<float>::max();
    std::vector<int> type = {RS_NOP, RS_NOP, RS_NOP, RS_NOP, RS_NOP};
    path.update(type, 0.0, 0.0, 0.0, 0.0, 0.0);

    float t, u, v = 0.0;
    bool isOk = false;
    // normal
    isOk = LpSpLp(x, y, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type15"], t, u, v, 0, 0);
        }
    }

    // timeflip
    isOk = LpSpLp(-x, y, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type15"], -t, -u, -v, 0, 0);
        }
    }

    // reflect
    isOk = LpSpLp(x, -y, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type16"], t, u, v, 0, 0);
        }
    }

    // timeflip + reflect
    isOk = LpSpLp(-x, -y, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type16"], -t, -u, -v, 0, 0);
        }
    }

    // normal
    isOk = LpSpRp(x, y, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type13"], t, u, v, 0, 0);
        }
    }

    // timeflip
    isOk = LpSpRp(-x, y, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type13"], -t, -u, -v, 0, 0);
        }
    }

    // reflect
    isOk = LpSpRp(x, -y, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type14"], t, u, v, 0, 0);
        }
    }

    // timeflip + reflect
    isOk = LpSpRp(-x, -y, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type14"], -t, -u, -v, 0, 0);
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
