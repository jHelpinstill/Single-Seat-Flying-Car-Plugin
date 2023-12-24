#include "HoverModule.h"
#include "GlobalVars.h"
#include "Util.h"

HoverModule::HoverModule(float max_angle, float max_accel, int ID)
{
	this->max_angle = max_angle;
	this->max_accel = max_accel;
	this->XPLM_ID = ID;
}

void HoverModule::update(float vert_target, float side_target, float dt)
{
	bound(vert_target, -max_angle, max_angle);
	bound(side_target, -max_angle, max_angle);

	float vert_accel = vert_PID.update(vert_target, vert, dt); bound(vert_accel, -max_accel, max_accel);
	float side_accel = side_PID.update(side_target, side, dt); bound(side_accel, -max_accel, max_accel);

	vert_rate += vert_accel * dt;
	side_rate += side_accel * dt;

	vert += vert_rate * dt; bound(vert, -max_angle, max_angle);
	side += side_rate * dt; bound(side, -max_angle, max_angle);

	XPLMSetDatavf(Global::acf_vertcant, &vert, XPLM_ID, 1);
	XPLMSetDatavf(Global::acf_sidecant, &side, XPLM_ID, 1);

	Global::debug.println("vert_target		: ", vert_target);
	Global::debug.println("vert_accel		: ", vert_accel);
	Global::debug.println("vert				: ", vert);
}