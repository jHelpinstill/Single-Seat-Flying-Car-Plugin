#include "Vec3.h"

Vec3::Vec3()
{
    x = 0; y = 0; z = 0;
}

Vec3::Vec3(float X, float Y, float Z)
{
    x = X; y = Y; z = Z;
}

Vec3 Vec3::operator+(Vec3 v)
{
    return Vec3(x + v.x, y + v.y, z + v.z);
}

Vec3 Vec3::operator-(Vec3 v)
{
    return Vec3(x - v.x, y - v.y, z - v.z);
}

Vec3 Vec3::operator-()
{
    return Vec3(-x, -y, -z);
}

Vec3 Vec3::operator*(float a)
{
    return Vec3(x * a, y * a, z * a);
}

Vec3 Vec3::operator/(float a)
{
    if (a != 0) {
        Vec3 u(x / a, y / a, z / a);
        return u;
    }
    return Vec3(0, 0, 0);
}

void Vec3::operator+=(Vec3 v)
{
    x += v.x;
    y += v.y;
    z += v.z;
}

void Vec3::operator-=(Vec3 v)
{
    x -= v.x;
    y -= v.y;
    z -= v.z;
}

void Vec3::operator*=(float a)
{
    x *= a;
    y *= a;
    z *= a;
}

void Vec3::operator/=(float a)
{
    if (a != 0) {
        x /= a;
        y /= a;
        z /= a;
    }
    else {
        x = 0;
        y = 0;
        z = 0;
    }
}

bool Vec3::operator==(Vec3 v)
{
    if (this->x == v.x
        && this->y == v.y
        && this->z == v.z) return true;

    return false;
}

float Vec3::dot(Vec3 v)
{
    return (x * v.x) + (y * v.y) + (z * v.z);
}

Vec3 Vec3::cross(Vec3 v)
{
    return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
}


float Vec3::mag()
{
    return sqrt(x * x + y * y + z * z);
}

float Vec3::sqMag()
{
    return x * x + y * y + z * z;
}

Vec3 Vec3::unit() {
    float mag = this->mag();
    if (mag != 0) {
        return *this / this->mag();
    }
    return Vec3(0, 0, 0);
}

void Vec3::normalize()
{
    Vec3 u = this->unit();
    x = u.x;
    y = u.y;
    z = u.z;
}

bool Vec3::isZero()
{
    if (x == 0 && y == 0 && z == 0)
        return true;
    return false;
}

float Vec3::angleBetween(Vec3 v, Vec3 u)
{
    return acos(v.unit().dot(u.unit())) / deg2rad;
}

const float Vec3::deg2rad = 3.1415926535 / 180;
Vec3 Vec3::X(1, 0, 0);
Vec3 Vec3::Y(0, 1, 0);
Vec3 Vec3::Z(0, 0, 1);
Vec3 Vec3::zero(0, 0, 0);