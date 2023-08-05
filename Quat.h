#pragma once
#include "Vec3.h"

class Quat
{
public:
    union
    {
        struct { float a, b, c, d; };
        struct { float n[4]; };
        struct { float w, x, y, z; };
    };

    const float deg2rad = 3.1415926535 / 180;

    Quat();
    Quat(float a1, float b1, float c1, float d1);
    Quat(Vec3 v, float deg);
    Quat(Vec3 eulers);
    Quat operator*(Quat q);
    void operator=(const Quat q);
    Quat inverse();
    Vec3 operator*(Vec3 v);
    void setByVecDeg(Vec3 vin, float angle);
    void normalize();
    Quat slerp(float t);
    Vec3 eulerAngles();
    Vec3 getAxis();
    float getAngle();
};