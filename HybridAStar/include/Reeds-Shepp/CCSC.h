#pragma once
#include "RSPath.h"
#include "FormularList.h"
#include <iostream>
#include <limits>
#include <string>
#include "RSPathElem.h"

bool CCSC(float x, float y, float phi, RSPath &path)
{
    float Lmin = std::numeric_limits<float>::max();
    std::vector<int> type = {RS_NOP, RS_NOP, RS_NOP, RS_NOP, RS_NOP};
    path.update(type, 0.0, 0.0, 0.0, 0.0, 0.0);

    float t, u, v = 0.0;
    bool isOk = false;
    // normal
    isOk = LpRmSmLm(x, y, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type05"], t, -M_PI / 2.0, u, v, 0);
        }
    }

    // timeflip
    isOk = LpRmSmLm(-x, y, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type05"], -t, M_PI / 2.0, -u, -v, 0);
        }
    }

    // reflect
    isOk = LpRmSmLm(x, -y, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type06"], t, -M_PI / 2.0, u, v, 0);
        }
    }

    // timeflip + reflect
    isOk = LpRmSmLm(-x, -y, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type06"], -t, M_PI / 2.0, -u, -v, 0);
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
            path.update(RSPathType["Type09"], t, -M_PI / 2.0, u, v, 0);
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
            path.update(RSPathType["Type09"], -t, M_PI / 2.0, -u, -v, 0);
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
            path.update(RSPathType["Type10"], t, -M_PI / 2.0, u, v, 0);
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
            path.update(RSPathType["Type10"], -t, M_PI / 2.0, -u, -v, 0);
        }
    }

    // backwards

    // normal
    float xb = x * cos(phi) + y * sin(phi);
    float yb = x * sin(phi) - y * cos(phi);

    isOk = LpRmSmLm(xb, yb, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type07"], v, u, -M_PI / 2.0, t, 0);
        }
    }
    // timeflip
    isOk = LpRmSmLm(-xb, yb, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type07"], -v, -u, M_PI / 2.0, -t, 0);
        }
    }
    // reflect
    isOk = LpRmSmLm(xb, -yb, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type08"], v, u, -M_PI / 2.0, t, 0);
        }
    }

    // timeflip + reflect
    isOk = LpRmSmLm(-xb, -yb, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type08"], -v, -u, M_PI / 2.0, -t, 0);
        }
    }

    // normal

    isOk = LpRmSmLm(xb, yb, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type11"], v, u, -M_PI / 2.0, t, 0);
        }
    }
    // timeflip
    isOk = LpRmSmLm(-xb, yb, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type11"], -v, -u, M_PI / 2.0, -t, 0);
        }
    }
    // reflect
    isOk = LpRmSmLm(xb, -yb, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type12"], v, u, -M_PI / 2.0, t, 0);
        }
    }

    // timeflip + reflect
    isOk = LpRmSmLm(-xb, -yb, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type12"], -v, -u, M_PI / 2.0, -t, 0);
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
