#include "Joystick.h"
#include "GlobalVars.h"
#include "Util.h"

Joystick joystick;

Joystick::Joystick()
{
	for (int i = 0; i < 16; i++)
	{
		buttons[i] = { false, false, false, false };
	}
}

void Joystick::printButtonNumbers()
{
	TextBox button_search(Global::l + 450, Global::t);
	int all_buttons[3200];
	XPLMGetDatavi(Global::joy_buttons, all_buttons, 0, 3200);
	for (int i = 0; i < 3200; i++)
	{
		if (all_buttons[i])
			button_search.println(i);
	}
}

void Joystick::printActiveAxes(float deadzone)
{
	TextBox axes_search(Global::l + 450, Global::t - 50);
	float axes[500];
	XPLMGetDatavf(Global::joystick_axes, axes, 0, 500);
	int button_counter = 0;
	for (int i = 0; i < 500; i++)
	{
		if (axes[i] >= deadzone || axes[i] <= -deadzone)
		{
			axes_search.print(i);
			axes_search.println(": ", axes[i]);
			button_counter++;
		}
		if (button_counter >= 10)
			break;
	}
}

void Joystick::update()
{
	for (int i = 0; i < 16; i++)
	{
		buttons[i].held = retrieveButtonState(but_ids[i]);

		buttons[i].pressed = buttons[i].released = false;
		if (buttons[i].held)
			buttons[i].pressed = !buttons[i].prev_state;
		else
			buttons[i].released = buttons[i].prev_state;

		buttons[i].prev_state = buttons[i].held;
	}

	retrieveAxes();
	retrieveThrottle();
}

void Joystick::adjustWithButton(float& input, float step, Button& button_up, Button& button_down, float repeat_delay)
{
	static float repeat_timer;
	static float held_charge = 0;
	const float held_rate = 10;
	
	if (button_up.pressed)
		input += step;
	else if (button_down.pressed)
		input -= step;

	if (button_up.held || button_down.held)
	{
		if (repeat_timer < repeat_delay)
			repeat_timer += Global::dt;
		else
			held_charge += held_rate * Global::dt;
		if (held_charge >= 1)
		{
			if (button_up.held)
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

bool Joystick::retrieveButtonState(int but_id)
{
	int value;
	XPLMGetDatavi(Global::joy_buttons, &value, but_id, 1);
	return (value == true);
}

void Joystick::retrieveAxes()
{
	float value;
	
	for (int i = 0; i < 3; i++)
	{
		XPLMGetDatavf(Global::joystick_axes, &value, axis_ids[i], 1);
		axes.n[i] = value;
	}
}

Vec3 Joystick::getRawAxes()
{
	return axes;
}

Vec3 Joystick::getFilteredAxes()
{
	Vec3 values;
	for (int i = 0; i < 3; i++)
	{
		values.n[i] = applyDeadzone(axes.n[i], axis_deadzones[i], axis_powers[i]);
	}
	return values;
}

void Joystick::retrieveThrottle()
{
	XPLMGetDatavf(Global::joystickThrottleAxis, &throttle, 28, 1);
}

float Joystick::getRawThrottle()
{
	return throttle;
}

float Joystick::getSignedThrottle()
{
	float value = 1 - (throttle * 2);
	return applyDeadzone(value, throttle_deadzone, throttle_power);
}

float Joystick::getUnsignedThrottle(bool flip)
{
	float value = flip ? throttle : 1 - throttle;
	return applyDeadzone(value, throttle_deadzone, throttle_power);
}

void Joystick::setAxisFilter(int axis, float deadzone, float power)
{
	axis_deadzones[axis] = deadzone;
	axis_powers[axis] = power;
}

void Joystick::setThrottleFilter(float deadzone, float power)
{
	throttle_deadzone = deadzone;
	throttle_power = power;
}