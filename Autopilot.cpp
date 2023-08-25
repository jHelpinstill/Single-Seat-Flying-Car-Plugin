#include "Avionics.h"

float holdHeading(float target_heading, float max_bank_angle, float max_roll_rate)
{
	const float default_p = 5;
	const float reduced_ratio = 0.1;

	static PID heading_hold(default_p, 0.0, 0.0);
	float heading = Global::vehicle.attitude.eulerAngles().z;
	target_heading -= heading;
	if (target_heading < -180) target_heading += 360;
	else if (target_heading > 180) target_heading -= 360;

	//if (abs(target_heading) < 1)
	//	heading_hold.P = reduced_ratio* default_p + (1 - reduced_ratio) * default_p * abs(target_heading);
	//else
	//	heading_hold.P = default_p;
	float target_bank_angle = -rBound(heading_hold.update(target_heading, 0, Global::dt), -max_bank_angle, max_bank_angle);

	//GlobalVars::debug.println("HH PID INFO: ", heading_hold.error);
	//GlobalVars::debug.println("target: ", target_heading);
	//GlobalVars::debug.println("current: ", heading);

	return holdBankAngle(target_bank_angle, max_roll_rate);
}

float holdBankAngle(float target_bank_angle, float max_roll_rate)
{
	const float default_p = 0.05;
	const float reduced_ratio = 0.1;
	const float reduced_width = 2;

	max_roll_rate /= 180;	// ratio of 180 deg per sec
	static PID roll_hold(default_p, 0.002, 0.01, 1);
	float roll = Global::vehicle.attitude_roll_pitch.eulerAngles().x;
	float roll_error = target_bank_angle - roll;
	//if (abs(roll_error) < reduced_width)
	//	roll_hold.P = reduced_ratio * default_p + (1 - reduced_ratio) * default_p * abs(roll_error) / reduced_width;
	//else
	//	roll_hold.P = default_p;
	return rBound(roll_hold.update(roll_error, 0, Global::dt), -max_roll_rate, max_roll_rate);
}

float holdMSL(float target_alt, float max_vert_vel, float max_norm_gs)
{
	//const float max_vert_vel = 80;
	const float meters2feet = 3.28084;
	float alt = XPLMGetDataf(Global::MSL_elevation) * meters2feet;
	static PID alt_hold(0.5, 0.1, 0.0);

	float target_vert_v = rBound(alt_hold.update(target_alt, alt, Global::dt), -max_vert_vel, max_vert_vel);
	return holdVertVel(target_vert_v, max_norm_gs);
}

float holdVertVel(float target_vert_v, float max_norm_gs)
{
	//const float max_norm_gs = 0.2;
	const float meters2feet = 3.28084;
	static float prev_alt;
	float alt = XPLMGetDataf(Global::MSL_elevation) * meters2feet;
	float vert_vel = (alt - prev_alt) / Global::dt;
	prev_alt = alt;
	static PID vert_v_hold(0.025, 0.005, 0.01, 1);
	return -rBound(vert_v_hold.update(target_vert_v, vert_vel, Global::dt) * 0.5, -max_norm_gs, max_norm_gs);
}

void holdAirSpd(float final_spd)
{
	float dt = Global::dt;
	static PID auto_throttle(500, 40, 100, 20);
	const float max_accel = 5;	// ms^-2
	const float min_accel = -3;

	float prograde_spd = Global::vehicle.airflow_rel.mag();

	float thrust = rBound(auto_throttle.update(final_spd, prograde_spd, dt), min_accel * Global::vehicle.mass, max_accel * Global::vehicle.mass);
	setFwdThrust(thrust);
}