#pragma once
#include <cmath>
#include <string>
#include <vector>
class RSPath
{
public:
    float t_;
    float u_;
    float v_;
    float w_;
    float x_;
    float totalLength_;
    std::vector<int> type_;
    RSPath()
    {

        t_ = 0;
        u_ = 0;
        v_ = 0;
        w_ = 0;
        x_ = 0;
        totalLength_ = std::fabs(t_) + std::fabs(u_) + std::fabs(v_) + std::fabs(w_) + std::fabs(x_);
    }
    RSPath(std::vector<int> type, float t, float u, float v, float w, float x)
    {
        type_ = type;
        t_ = t;
        u_ = u;
        v_ = v;
        w_ = w;
        x_ = x;
        totalLength_ = std::fabs(t_) + std::fabs(u_) + std::fabs(v_) + std::fabs(w_) + std::fabs(x_);
    }
    void update(std::vector<int> type, float t, float u, float v, float w, float x)
    {
        type_ = type;
        t_ = t;
        u_ = u;
        v_ = v;
        w_ = w;
        x_ = x;
        totalLength_ = std::fabs(t_) + std::fabs(u_) + std::fabs(v_) + std::fabs(w_) + std::fabs(x_);
    }
};