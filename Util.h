#pragma once

#include "Vec3.h"
#include "Matrix.h"
#include "Quat.h"
#include <vector>
#include "GlobalVars.h"

float rBound(float num, float lower, float upper);
void bound(float& num, float lower, float upper);
void bound(Vec3& v, float lower, float upper);
template <class T>
inline T lerp(T a, T b, float t, bool bound_t)
{
	if (bound_t) bound(t, 0, 1);
	return a * (1 - t) + b * t;
}

float applyDeadzone(float data, float dead_zone, float power = 1, float data_max = 1);
float mag(float a);
float headingCorrection(float heading);

class RollingAvg
{
private:
	std::vector<float> prev_values;
	int num_values = 2;
	int values_index = 0;
	float accumulator = 0;

public:
	RollingAvg() {}
	RollingAvg(int num_values_);

	void apply(float& input);
};