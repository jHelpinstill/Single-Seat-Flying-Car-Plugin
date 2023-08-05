#include "PID.h"

PID::PID(float p, float d, float i)
{
	P = p; D = d; I = i;
}

PID::PID(float p, float d, float i, float accumulator_radius)
{
	P = p; D = d; I = i;
	accumulator_upper_bound = accumulator_radius;
	accumulator_lower_bound = -accumulator_radius;
}

PID::PID(float p, float d, float i, float lower_bound, float upper_bound)
{
	P = p; D = d; I = i;
	accumulator_upper_bound = upper_bound;
	accumulator_lower_bound = lower_bound;
}

float PID::update(float setPoint, float state, float dt)
{
	error = setPoint - state;
	errorRate = (error - prevError) / dt;
	prevError = error;
	accumulator += error * dt;

	if(accumulator_upper_bound != 0 || accumulator_lower_bound != 0)
	{
		if (accumulator < accumulator_lower_bound) accumulator = accumulator_lower_bound;
		if (accumulator > accumulator_upper_bound) accumulator = accumulator_upper_bound;
	}

	return P * error + D * errorRate + I * accumulator;
};

void PID::reset()
{
	prevError = 0;
	accumulator = 0;
};