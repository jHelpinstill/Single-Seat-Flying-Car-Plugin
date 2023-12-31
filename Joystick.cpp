#include "Joystick.h"
#include "GlobalVars.h"
#include "Util.h"


void showButtonNumbers()
{
	TextBox button_search(Global::l + 450, Global::t);
	int buttons[3200];
	XPLMGetDatavi(Global::joy_buttons, buttons, 0, 3200);
	for (int i = 0; i < 3200; i++)
	{
		if (buttons[i])
			button_search.println(i);
	}
}
void showJoystickAxes(float dead_zone)
{
	TextBox axes_search(Global::l + 450, Global::t - 50);
	float axes[500];
	XPLMGetDatavf(Global::joystick_axes, axes, 0, 500);
	int button_counter = 0;
	for (int i = 0; i < 500; i++)
	{
		if (axes[i] >= dead_zone || axes[i] <= -dead_zone)
		{
			axes_search.print(i);
			axes_search.println(": ", axes[i]);
			button_counter++;
		}
		if (button_counter >= 10)
			break;
	}
}

std::vector<Button*> buttons;
void buttonSetup()
{
	Global::joy_trigger.id = 160; buttons.push_back(&Global::joy_trigger);

	Global::joy_3.id = 162; buttons.push_back(&Global::joy_3);
	Global::joy_4.id = 163; buttons.push_back(&Global::joy_4);
	Global::joy_5.id = 164; buttons.push_back(&Global::joy_5);
	Global::joy_6.id = 165; buttons.push_back(&Global::joy_6);
	Global::joy_7.id = 166; buttons.push_back(&Global::joy_7);
	Global::joy_8.id = 167; buttons.push_back(&Global::joy_8);
	Global::joy_9.id = 168; buttons.push_back(&Global::joy_9);
	Global::joy_10.id = 169; buttons.push_back(&Global::joy_10);
	Global::joy_11.id = 170; buttons.push_back(&Global::joy_11);
	Global::joy_12.id = 171; buttons.push_back(&Global::joy_12);

	Global::joy_up.id = 172; buttons.push_back(&Global::joy_up);
	Global::joy_down.id = 176; buttons.push_back(&Global::joy_down);
	Global::joy_left.id = 178; buttons.push_back(&Global::joy_left);
	Global::joy_right.id = 174; buttons.push_back(&Global::joy_right);
	Global::joy_thumb.id = 161; buttons.push_back(&Global::joy_thumb);


	for (int i = 0; i < buttons.size(); i++)
		buttons[i]->prev_state = false;
}

void updateButtons()
{
	for (int i = 0; i < buttons.size(); i++)
	{
		buttons[i]->held = getButton(buttons[i]->id);

		buttons[i]->pressed = buttons[i]->released = false;
		if (buttons[i]->held)
			buttons[i]->pressed = !buttons[i]->prev_state;
		else
			buttons[i]->released = buttons[i]->prev_state;

		buttons[i]->prev_state = buttons[i]->held;
	}
}

void adjustWithButtonf(float& input, float step, Button& button_up, Button& button_down, float repeat_delay)
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

bool getButton(int button)
{
	int value;
	XPLMGetDatavi(Global::joy_buttons, &value, button, 1);
	return (value == true);
}

float getJoystickAxis(int axis)
{
	float* value = new float;
	XPLMGetDatavf(Global::joystick_axes, value, axis, 1);
	return *value;
}


Vec3 getJoystickRotValues(float power)
{
	Vec3 data_v;

	//data_v.x = XPLMGetDataf(yoke[1]);
	//data_v.y = -XPLMGetDataf(yoke[0]);
	//data_v.z = -XPLMGetDataf(yoke[2]);

	data_v.x = (getJoystickAxis(26) * 2 - 1);
	data_v.y = -(getJoystickAxis(25) * 2 - 1);
	data_v.z = -(getJoystickAxis(27) * 2 - 1);

	data_v.x = applyDeadzone(data_v.x, 0.05, power);
	data_v.y = applyDeadzone(data_v.y, 0.05, power);
	data_v.z = applyDeadzone(data_v.z, 0.3, power);

	return data_v;
}

float getSignedJoystickThrottle(float power)
{
	float* data = new float;
	XPLMGetDatavf(Global::joystickThrottleAxis, data, 28, 1);
	float throttle = 1 - (*data * 2);

	return applyDeadzone(throttle, 0, power);
}

float getUnsignedJoystickThrottle(bool flip, float power)
{
	float* data = new float;
	XPLMGetDatavf(Global::joystickThrottleAxis, data, 28, 1);
	data[0] = flip ? *data : 1 - *data;
	return applyDeadzone(*data, 0, power);
}