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