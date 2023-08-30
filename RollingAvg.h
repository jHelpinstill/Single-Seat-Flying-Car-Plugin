#pragma once
#ifndef ROLLING_AVG
#define ROLLING_AVG

#include <vector>

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

#endif