#pragma once
#include <cmath>
#include <string>
#include "CalcShortDist.h"
#include "RSPath.h"
#include "RSPathElem.h"

RSPath FindRSPath(float x, float y, float phi)
{
    float rmin = 5.0; // minimum turning radius
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

    float Lmin = std::numeric_limits<float>::max();

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
        return RSPathVec[candidateId];
    }
}