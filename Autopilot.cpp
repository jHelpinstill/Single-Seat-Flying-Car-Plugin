#include "Avionics.h"

float holdHeading(float target_heading, float max_bank_angle, float max_roll_rate)
{
	//const float max_bank_angle = 30;
	static PID heading_hold(5, 0.0, 0.0);
	float heading = GlobalVars::vehicle_rot.eulerAngles().z;
	//float heading = XPLMGetDataf(GlobalVars::psi);
	float target_bank_angle = -rBound(heading_hold.update(target_heading, heading, GlobalVars::dt), -max_bank_angle, max_bank_angle);

	//GlobalVars::debug.println("HH PID INFO: ", heading_hold.error);
	//GlobalVars::debug.println("target: ", target_heading);
	//GlobalVars::debug.println("current: ", heading);

	return holdBankAngle(target_bank_angle, max_roll_rate);
}

float holdBankAngle(float target_bank_angle, float max_roll_rate)
{
	max_roll_rate /= 180;	// ratio of 180 deg per sec
	static PID roll_hold(0.05, 0.005, 0.01, 1);
	float roll = GlobalVars::vehicle_roll_pitch.eulerAngles().x;
	return rBound(roll_hold.update(target_bank_angle, roll, GlobalVars::dt), -max_roll_rate, max_roll_rate);
}

float holdMSL(float target_alt, float max_vert_vel, float max_norm_gs)
{
	//const float max_vert_vel = 80;
	const float meters2feet = 3.28084;
	float alt = XPLMGetDataf(GlobalVars::MSL_elevation) * meters2feet;
	static PID alt_hold(0.5, 0.1, 0.0);

	float target_vert_v = rBound(alt_hold.update(target_alt, alt, GlobalVars::dt), -max_vert_vel, max_vert_vel);
	return holdVertVel(target_vert_v, max_norm_gs);
}

float holdVertVel(float target_vert_v, float max_norm_gs)
{
	//const float max_norm_gs = 0.2;
	const float meters2feet = 3.28084;
	static float prev_alt;
	float alt = XPLMGetDataf(GlobalVars::MSL_elevation) * meters2feet;
	float vert_vel = (alt - prev_alt) / GlobalVars::dt;
	prev_alt = alt;
	static PID vert_v_hold(0.025, 0.005, 0.01, 1);
	return -rBound(vert_v_hold.update(target_vert_v, vert_vel, GlobalVars::dt), -max_norm_gs, max_norm_gs);
}

void holdAirSpd(float final_spd)
{
	float dt = GlobalVars::dt;
	static PID auto_throttle(500, 40, 100, 20);
	const float max_accel = 5;	// ms^-2
	const float min_accel = -3;

	float prograde_spd = GlobalVars::airflow_rel.mag();

	float thrust = rBound(auto_throttle.update(final_spd, prograde_spd, dt), min_accel * GlobalVars::vehicle_mass, max_accel * GlobalVars::vehicle_mass);
	setFwdThrust(thrust);
}