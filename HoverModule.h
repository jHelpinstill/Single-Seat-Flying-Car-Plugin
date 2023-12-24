#pragma once
#include "PID.h"

class HoverModule
{
public:
	PID vert_PID = PID(15, 8, 0);
	PID side_PID = PID(15, 8, 0);

	float vert = 0;
	float side = 0;

	float vert_rate = 0;
	float side_rate = 0;

	float max_angle = 60;
	float max_accel = 2000;

	int XPLM_ID = -1;

	HoverModule() {}
	HoverModule(float max_angle, float max_accel, int ID);

	void update(float vert_target, float side_target, float dt);
};