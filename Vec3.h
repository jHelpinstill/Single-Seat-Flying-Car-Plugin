#pragma once
#ifndef VEC3
#define VEC3

#include <cmath>

class Vec3
{
public:

	static Vec3 X;
	static Vec3 Y;
	static Vec3 Z;
	static Vec3 zero;

	const static float deg2rad; //= 3.1415926535 / 180;

	static float angleBetween(Vec3 v, Vec3 u);

	union
	{
		struct { float x, y, z; };
		float n[3];
	};

	Vec3();
	Vec3(float x_in, float y_in, float z_in);
	Vec3 operator+(Vec3 v);
	Vec3 operator-(Vec3 v);
	Vec3 operator-();
	Vec3 operator*(float a);
	Vec3 operator/(float a);
	void operator+=(Vec3 v);
	void operator-=(Vec3 v);
	void operator*=(float a);
	void operator/=(float a);
	bool operator==(Vec3 v);
	float dot(Vec3 v);
	Vec3 cross(Vec3 v);
	float mag();
	float sqMag();
	Vec3 unit();
	void normalize();
	bool isZero();
};


#endif