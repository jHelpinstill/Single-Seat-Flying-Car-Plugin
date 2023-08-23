#include "BinaryScroller.h"
#include "GlobalVars.h"

BinaryScroller::BinaryScroller(Button* up, Button* down, float repeat_delay_, float scroll_spd_, float smooth_output_)
{
	up_but = up;
	down_but = down;
	repeat_delay = repeat_delay_;
	scroll_spd = scroll_spd_;
	smooth_output = smooth_output_;
}

void BinaryScroller::smooth(float& input)
{
	static float prev_input = 0;
	if (input != prev_input)
	{
		float rate = (input - prev_input) / GlobalVars::dt;
		if (rate > scroll_spd)
			input = prev_input + scroll_spd * GlobalVars::dt;
		else if (rate < -scroll_spd)
			input = prev_input - scroll_spd * GlobalVars::dt;
	}
}

void BinaryScroller::apply(float& input, float step)
{
	if (up_but->pressed)
		input += step;
	else if (down_but->pressed)
		input -= step;

	if (up_but->held || down_but->held)
	{
		if (repeat_timer < repeat_delay)
			repeat_timer += GlobalVars::dt;
		else
			held_charge += scroll_spd * GlobalVars::dt;
		if (held_charge >= 1)
		{
			if (up_but->held)
				input += step;
			else
				input -= step;
			held_charge -= 1;
		}
	}
	else
	{
		repeat_timer = 0;
		held_charge = 0;
	}
}