#pragma once
#ifndef BINARY_SCROLLER
#define BINARY_SCROLLER
#include "Joystick.h"

class BinaryScroller
{
private:
	float repeat_delay = 0;
	float repeat_timer = 0;
	float held_charge = 0;
	float scroll_spd = 0;

	Button* up_but = nullptr;
	Button* down_but = nullptr;

	bool smooth_output = true;
	void smooth(float& input);

public:
	BinaryScroller() {}
	BinaryScroller(Button* up, Button* down, float repeat_delay_, float scroll_spd_ = 0, float smooth_output_ = true);

	void apply(float& input, float step);
};

#endif