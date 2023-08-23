#pragma once
#ifndef PID_
#define PID_

class PID
{
public:
	float P{};
	float I{};
	float D{};

	float error{};
	float prevError{};
	float errorRate{};
	float accumulator{};

	float accumulator_lower_bound{};
	float accumulator_upper_bound{};

	float reduced_p{};
	float reduction_radius{};

	PID() {}
	PID(float p, float d, float i);
	PID(float p, float d, float i, float accumulator_radius);
	PID(float p, float d, float i, float lower_bound, float upper_bound);
	PID(float p, float d, float i, float accumulator_radius, float reduced_p, float reduction_radius);

	float update(float setPoint, float state, float dt); // setPoint: desired output, state: actual output
	void reset();
};

#endif