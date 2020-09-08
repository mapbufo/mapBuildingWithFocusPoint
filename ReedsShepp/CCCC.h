#pragma once
#include "RSPath.h"
#include "FormularList.h"
#include <iostream>
#include <limits>
#include <string>
#include "RSPathElem.h"

bool CCCC(float x, float y, float phi, RSPath &path)
{
    float Lmin = std::numeric_limits<float>::max();
    std::vector<int> type = {RS_NOP, RS_NOP, RS_NOP, RS_NOP, RS_NOP};
    path.update(type, 0.0, 0.0, 0.0, 0.0, 0.0);

    float t, u, v = 0.0;
    bool isOk = false;
    // normal
    isOk = LpRupLumRm(x, y, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type03"], t, u, -u, v, 0);
        }
    }

    // timeflip
    isOk = LpRupLumRm(-x, y, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type03"], -t, -u, u, -v, 0);
        }
    }

    // reflect
    isOk = LpRupLumRm(x, -y, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type04"], t, u, -u, v, 0);
        }
    }

    // timeflip + reflect
    isOk = LpRupLumRm(-x, -y, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type04"], -t, -u, u, -v, 0);
        }
    }

    // normal
    isOk = LpRumLumRp(x, y, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type03"], t, u, u, v, 0);
        }
    }

    // timeflip
    isOk = LpRumLumRp(-x, y, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type03"], -t, -u, -u, -v, 0);
        }
    }

    // reflect
    isOk = LpRumLumRp(x, -y, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type04"], t, u, u, v, 0);
        }
    }

    // timeflip + reflect
    isOk = LpRumLumRp(-x, -y, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type04"], -t, -u, -u, -v, 0);
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
