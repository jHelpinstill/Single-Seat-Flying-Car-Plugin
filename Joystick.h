#pragma once

#include "Vec3.h"
#include <vector>

struct Button
{
	bool prev_state = false;

	bool held = false;
	bool pressed = false;
	bool released = false;
};

class Joystick
{
private:
	int but_ids[16] = { 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 160, 172, 176, 178, 174, 161 };
	int axis_ids[3] = { 26, 25, 27 };

	Vec3 axes;
	float axis_deadzones[3]{};
	float axis_powers[3] = { 1, 1, 1 };

	float throttle{};
	float throttle_deadzone = 0;
	float throttle_power = 1;

	bool retrieveButtonState(int but_id);
	void retrieveAxes();
	void retrieveThrottle();

public:
	union
	{
		struct
		{
			Button
				button_3,
				button_4,
				button_5,
				button_6,
				button_7,
				button_8,
				button_9,
				button_10,
				button_11,
				button_12,
				button_trigger,
				button_up,
				button_down,
				button_left,
				button_right,
				button_thumb;
		};
		Button buttons[16];
	};

	Joystick();
	
	void update();

	Vec3 getRawAxes();
	Vec3 getFilteredAxes();

	void setAxisFilter(int axis, float deadzone, float power);
	void setThrottleFilter(float deadzone, float power);
	
	float getRawThrottle();
	float getSignedThrottle();
	float getUnsignedThrottle(bool flip = false);

	void printButtonNumbers();
	void printActiveAxes(float deadzone = 0.1);
	
	void adjustWithButton(float& input, float step, Button& button_up, Button& button_down, float repeat_delay);
};

extern Joystick joystick;