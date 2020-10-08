#pragma once
#include "RSPath.h"
#include "FormularList.h"
#include <iostream>
#include <limits>
#include <string>
#include "RSPathElem.h"

bool CCSCC(float x, float y, float phi, RSPath &path)
{
    float Lmin = std::numeric_limits<float>::max();
    std::vector<int> type = {RS_NOP, RS_NOP, RS_NOP, RS_NOP, RS_NOP};
    path.update(type, 0.0, 0.0, 0.0, 0.0, 0.0);

    float t, u, v = 0.0;
    bool isOk = false;
    // normal
    isOk = LpRmSLmRp(x, y, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type17"], t, -M_PI / 2.0, u, -M_PI / 2.0, v);
        }
    }

    // timeflip
    isOk = LpRmSLmRp(-x, y, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type17"], -t, M_PI / 2.0, -u, M_PI / 2.0, -v);
        }
    }

    // reflect
    isOk = LpRmSLmRp(x, -y, -phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type18"], t, -M_PI / 2.0, u, -M_PI / 2.0, v);
        }
    }

    // timeflip + reflect
    isOk = LpRmSLmRp(-x, -y, phi, t, u, v);
    if (isOk)
    {
        float L = std::fabs(t) + std::fabs(u) + std::fabs(v);
        if (Lmin > L)
        {
            Lmin = L;
            path.update(RSPathType["Type18"], -t, M_PI / 2.0, -u, M_PI / 2.0, -v);
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
