#include "Quat.h"

Quat::Quat()
{
    a = 1;
    b = c = d = 0;
}

Quat::Quat(float a1, float b1, float c1, float d1)
{
    a = a1;
    b = b1;
    c = c1;
    d = d1;
}

Quat::Quat(Vec3 v, float deg)
{
    setByVecDeg(v, deg);
}

Quat::Quat(Vec3 eulers)
{
    Quat heading_rot(Vec3::Z, eulers.z);
    Vec3 roll_vec = heading_rot * Vec3::X;
    Vec3 pitch_vec = roll_vec.cross(Vec3::Z);

    *this = Quat(pitch_vec, -eulers.y) * Quat(roll_vec, eulers.x) * heading_rot;
}

Quat Quat::operator*(Quat q)
{
    Quat r;
    r.a = a * q.a - b * q.b - c * q.c - d * q.d;
    r.b = a * q.b + b * q.a + c * q.d - d * q.c;
    r.c = a * q.c - b * q.d + c * q.a + d * q.b;
    r.d = a * q.d + b * q.c - c * q.b + d * q.a;
    return r;
}

void Quat::operator=(const Quat q)
{
    for (int i = 0; i < 4; i++)
        n[i] = q.n[i];
}

Quat Quat::inverse()
{
    Quat q(a, -b, -c, -d);
    return q;
}

Vec3 Quat::operator*(Vec3 v)
{
    Quat qin(a, -b, -c, -d);
    Quat qVec(0, v.x, v.y, v.z);
    Quat temp = (*this * qVec) * qin;
    Vec3 r = { temp.b, temp.c, temp.d };
    return r;
}

void Quat::setByVecDeg(Vec3 v, float angle)
{
    float halfrad = 0.5 * angle * deg2rad;
    v.normalize();
    a = cos(halfrad);
    b = sin(halfrad) * v.x;
    c = sin(halfrad) * v.y;
    d = sin(halfrad) * v.z;
}

Vec3 Quat::getAxis()
{
    if (w > 1)
        normalize();
    Vec3 axis;
    float factor = sin(acos(a));
    if (factor == 0) return Vec3::zero;
    axis.x = b / factor;
    axis.y = c / factor;
    axis.z = d / factor;
    return axis.unit();
}

float Quat::getAngle()
{
    if (w > 1)
        normalize();
    float halfrad = acos(a);
    return 2 * halfrad / deg2rad;
}

void Quat::normalize()
{
    float length = sqrt(a * a + b * b + c * c + d * d);
    a /= length;
    b /= length;
    c /= length;
    d /= length;
}

Quat Quat::slerp(float t)
{
    float angle = getAngle();
    return Quat(getAxis(), angle * t);
}

Vec3 Quat::eulerAngles()
{
    Vec3 angles;
    angles.x = atan(2 * (w * x + y * z) / (1 - 2 * (x * x + y * y))) / deg2rad;
    angles.y = (2 * atan(sqrt(1 + 2 * (w * y - x * z)) / sqrt(1 - 2 * (w * y - x * z)))) / deg2rad - 90;
    angles.z = atan2(2 * (w * z + x * y), (1 - 2 * (y * y + z * z))) / deg2rad;
    for (int i = 0; i < 3; i++)
    {
        if (isnan(angles.n[i]))
            angles.n[i] = 0;
    }
    return angles;
}