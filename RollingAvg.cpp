#include "RollingAvg.h"
#include "GlobalVars.h"

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