#pragma once

#include "Vec3.h"
#include <vector>

struct Button
{
	int id;
	bool prev_state;

	bool held;
	bool pressed;
	bool released;
};

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

void showButtonNumbers();
void showJoystickAxes(float dead_zone = 0.1);
bool getButton(int button);
void buttonSetup();
void updateButtons();
void adjustWithButtonf(float& input, float step, Button& button_up, Button& button_down, float repeat_delay);
float getJoystickAxis(int axis);
Vec3 getJoystickRotValues(float power = 1);
float getSignedJoystickThrottle(float power = 1);
float getUnsignedJoystickThrottle(bool flip = false, float power = 1);