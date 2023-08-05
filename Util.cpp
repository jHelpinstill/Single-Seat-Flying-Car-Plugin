#include "Util.h"


float rBound(float num, float lower, float upper)
{
	if (num > upper)
		num = upper;
	else if (num < lower)
		num = lower;
	return num;
}

void bound(float& num, float lower, float upper)
{
	if (num > upper)
		num = upper;
	else if (num < lower)
		num = lower;
}
void bound(Vec3& v, float lower, float upper)
{
	bound(v.x, lower, upper);
	bound(v.y, lower, upper);
	bound(v.z, lower, upper);
}

float applyDeadzone(float data, float dead_zone, float power, float data_max)
{
	data /= data_max;
	dead_zone /= data_max;

	if (data > dead_zone) data = (data - dead_zone) / (1 - dead_zone);
	else if (data < -dead_zone) data = (data + dead_zone) / (1 - dead_zone);
	else data = 0;

	bool negative = (data < 0);
	if (negative) data *= -1;
	data = pow(data, power);
	if (negative) data *= -1;

	return data * data_max;
}

float mag(float a)
{
	if (a > 0)
		return a;
	return -a;
}

float headingCorrection(float heading)
{
	heading = (heading < 0) ? -heading : 360 - heading;
	heading -= 15;
	if (heading > 360) heading -= 360;
	else if (heading < 0) heading += 360;
	return heading;
}

RollingAvg::RollingAvg(int num_values_)
{
	num_values = num_values_;
	for (int i = 0; i < num_values; i++)
		prev_values.push_back(0);
}

void RollingAvg::apply(float& input)
{
	accumulator += input;
	accumulator -= prev_values[values_index];
	prev_values[values_index] = input;
	values_index = (values_index + 1) % num_values;
	input = accumulator / (float)num_values;
	GlobalVars::debug.println("rolling avg accumulator: ", accumulator);
}