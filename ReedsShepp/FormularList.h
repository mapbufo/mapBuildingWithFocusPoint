#pragma once
#include <cmath>
#include <math.h>
#include <utility>
#include <vector>
float mod2pi(float x)
{
    float v = x;
    while (std::fabs(v) > M_PI)
    {
        if (v < -M_PI)
        {
            v += 2 * M_PI;
        }
        else
        {
            v -= 2 * M_PI;
        }
    }
    return v;
}

void tauOmega(float u, float v, float xi, float eta, float phi, float &tau, float &omega)
{
    float delta = mod2pi(u - v);
    float A = sin(u) - sin(delta);
    float B = cos(u) - cos(delta) - 1;
    float t1 = atan2(eta * A - xi * B, xi * A + eta * B);
    float t2 = 2 * (cos(delta) - cos(v) - cos(u)) + 3;
    if (t2 < 0)
    {
        tau = mod2pi(t1 + M_PI);
    }
    else
    {
        tau = mod2pi(t1);
    }
    omega = mod2pi(tau - u + v - phi);
}

void cart2pol(float x, float y, float &angle, float &dist)
{
    dist = std::sqrt(x * x + y * y);
    angle = std::atan2(y, x);
}

// formular 8.1
bool LpSpLp(float x, float y, float phi, float &t, float &u, float &v)
{
    cart2pol(x - sin(phi), y - 1 + cos(phi), t, u);
    if (t >= 0)
    {
        v = mod2pi(phi - t);
        if (v >= 0)
        {
            return true;
        }
    }
    t = 0;
    u = 0;
    v = 0;
    return false;
}

// formular 8.2
bool LpSpRp(float x, float y, float phi, float &t, float &u, float &v)
{
    float t1, u1;
    cart2pol(x + sin(phi), y - 1 - cos(phi), t1, u1);
    if (u1 * u1 >= 4)
    {
        u = std::sqrt(u1 * u1 - 4);
        float theta = std::atan2(2, u);
        t = mod2pi(t1 + theta);
        v = mod2pi(t - phi);
        if (t >= 0 && v >= 0)
        {
            return true;
        }
    }
    t = 0;
    u = 0;
    v = 0;
    return false;
}

// formular 8.3 / 8.4
bool LpRmL(float x, float y, float phi, float &t, float &u, float &v)
{
    float xi = x - sin(phi);
    float eta = y - 1 + cos(phi);
    float theta, u1;
    cart2pol(xi, theta, theta, u1);

    if (u1 <= 4)
    {
        u = -2 * std::asin(u1 / 4.0);
        t = mod2pi(theta + u / 2.0 + M_PI);
        v = mod2pi(phi - t + u);
        if (t >= 0 && u <= 0)
        {
            return true;
        }
    }
    t = 0;
    u = 0;
    v = 0;
    return false;
}

// formular 8.7
bool LpRupLumRm(float x, float y, float phi, float &t, float &u, float &v)
{
    float xi = x + sin(phi);
    float eta = y - 1 - cos(phi);
    float rho = (2 + std::sqrt(xi * xi + eta * eta)) / 4.0;
    if (rho <= 1)
    {
        u = std::acos(rho);
        tauOmega(u, -u, xi, eta, phi, t, v);
        if (t >= 0 && v <= 0)
        {
            return true;
        }
    }

    t = 0;
    u = 0;
    v = 0;
    return false;
}

// formular 8.8
bool LpRumLumRp(float x, float y, float phi, float &t, float &u, float &v)
{
    float xi = x + sin(phi);
    float eta = y - 1 - cos(phi);
    float rho = (20 - xi * xi - eta * eta) / 16.0;
    if (rho <= 1 && rho >= 0)
    {
        u = -std::acos(rho);

        if (u >= -M_PI / 2.0)
        {
            tauOmega(u, u, xi, eta, phi, t, v);
            if (t >= 0 && v >= 0)
            {
                return true;
            }
        }
    }

    t = 0;
    u = 0;
    v = 0;
    return false;
}

// formular 8.9
bool LpRmSmLm(float x, float y, float phi, float &t, float &u, float &v)
{
    float xi = x - sin(phi);
    float eta = y - 1 + cos(phi);

    float theta, rho;
    cart2pol(xi, eta, theta, rho);
    if (rho >= 2)
    {
        float r = std::sqrt(rho * rho - 4);
        u = 2 - r;
        t = mod2pi(theta + std::atan2(r, -2));
        v = mod2pi(phi - M_PI / 2.0 - t);
        if (t >= 0 && u <= 0 && v <= 0)
        {
            return true;
        }
    }

    t = 0;
    u = 0;
    v = 0;
    return false;
}

// formular 8.10
bool LpRmLmRm(float x, float y, float phi, float &t, float &u, float &v)
{
    float xi = x + sin(phi);
    float eta = y - 1 - cos(phi);

    float theta, rho;
    cart2pol(-eta, xi, theta, rho);
    if (rho >= 2)
    {
        t = theta;
        u = 2 - rho;
        v = mod2pi(t + M_PI / 2.0 - phi);

        if (t >= 0 && u <= 0 && v <= 0)
        {
            return true;
        }
    }

    t = 0;
    u = 0;
    v = 0;
    return false;
}

// formular 8.11
bool LpRmSLmRp(float x, float y, float phi, float &t, float &u, float &v)
{
    float xi = x + sin(phi);
    float eta = y - 1 - cos(phi);

    float theta, rho;
    cart2pol(xi, eta, theta, rho);
    if (rho >= 2)
    {
        u = 4 - std::sqrt(rho * rho - 4);
        if (u <= 0)
        {
            t = mod2pi(std::atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
            v = mod2pi(t - phi);
            if (t >= 0 && v >= 0)
            {
                return true;
            }
        }
    }

    t = 0;
    u = 0;
    v = 0;
    return false;
}
