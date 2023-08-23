#include "Util.h"
#include "BinaryScroller.h"


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

void adjustPID(PID* pid)
{
	if (pid == nullptr) return;

	static BinaryScroller adjustP(&Global::joy_7, &Global::joy_8, 0.3, 0.5, false);
	static BinaryScroller adjustD(&Global::joy_9, &Global::joy_10, 0.3, 0.5, false);
	static BinaryScroller adjustI(&Global::joy_11, &Global::joy_12, 0.3, 0.5, false);
	
	adjustP.apply(pid->P, 0.005);
	adjustD.apply(pid->D, 0.005);
	adjustI.apply(pid->I, 0.005);
	
	Global::debug.println("P adjusted: ", pid->P);
	Global::debug.println("D adjusted: ", pid->D);
	Global::debug.println("I adjusted: ", pid->I);
}