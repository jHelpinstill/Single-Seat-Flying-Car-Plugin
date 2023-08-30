#include "PID.h"
#include <cmath>

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

PID::PID(float p, float d, float i, float accumulator_radius, float reduced_p, float reduction_radius)
{
	P = p;
	D = d;
	I = i;
	accumulator_upper_bound = accumulator_radius;
	accumulator_lower_bound = -accumulator_radius;
	this->reduced_p = reduced_p;
	this->reduction_radius = reduction_radius;
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
		else if (accumulator > accumulator_upper_bound) accumulator = accumulator_upper_bound;
	}

	float p = P;

	if (reduction_radius > 0)
	{
		if (abs(error) < reduction_radius)
		{
			float t = abs(error) / reduction_radius;
			p = t * P + (1 - t) * reduced_p;
		}
	}

	return p * error + D * errorRate + I * accumulator;
};

void PID::reset()
{
	prevError = 0;
	accumulator = 0;
};