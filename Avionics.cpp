#include "Avionics.h"

float thrustForVVHoldHover(float commanded_VV, PID& vv_hold)
{
	float dt = Global::dt;
	//static PID vv_hold(2, 0.5, 0.02, 0.1);

	float weight = aircraft.mass * Global::g0;
	if (aircraft.alt_agl < 5)
		bound(commanded_VV, -(aircraft.alt_agl + 1), 10);

	float thrust = weight + rBound(vv_hold.update(commanded_VV, aircraft.vert_vel, dt) * 1000, -weight * 0.75, weight * 2);

	//Global::debug.println("desired vv: ", commanded_VV);
	//Global::debug.println("actual vv: ", vv);

	return thrust;
}

Vec3 torqueForAttitudeRateHoldHover(float target_rate, int axis, PID& rot_PID)	// returns torque require to hold input rotation rate
{
	//static PID rotPIDs[3] =
	//{
	//	PID(0.1, 0.002, 0.01, 10, 0.02, 0.15),
	//	PID(0.1, 0.002, 0.01, 10, 0.05, 0.15),
	//	PID(0.2, 0.002, 0.01, 10)
	//};

	Vec3 angular_accel;
	angular_accel.n[axis] = rot_PID.update(target_rate, aircraft.rot_rate.n[axis], Global::dt);

	return aircraft.inertia_tensor * angular_accel;
}


///// DELETE WHEN ABLE! /////
float rotHoldHoverRate(float target_angle, int axis)
{
	static PID rotPIDs[3] =
	{
		PID(4, 0.05, 0),
		PID(4, 0.05, 0)
	};
	return rotPIDs[axis].update(target_angle, aircraft.attitude.eulerAngles().n[axis], Global::dt);
}

Vec3 torqueForAttitudeHoldHover(float target_angle, int axis, PID& att_PID, PID& rate_PID)// returns torque require to hold given angle about axis
{
	//static PID rotPIDs[3] =
	//{
	//	PID(4, 0.05, 0, 0, 0.02, 0.15),
	//	PID(4, 0.05, 0, 0, 0.05, 0.15)
	//};
	float rate = att_PID.update(target_angle, aircraft.attitude.eulerAngles().n[axis], Global::dt);
	return torqueForAttitudeRateHoldHover(rate, axis, rate_PID);
}

float yawRateForSSHoldHover(float target_slip_angle, PID& ss_PID)
{
	//static PID side_slip_PID(5, 0.05, 0);

	float slip_angle = rBound(asin(aircraft.airflow_rel.unit().y) / Global::deg2rad, -10, 10);
	float yaw_rate = ss_PID.update(target_slip_angle, slip_angle, Global::dt);
	return yaw_rate;
}

float pitchRateForNormalGsHold(float Gs)
{
	float kinematic_Gs = ((Vec3::Z * Gs) - (aircraft.attitude.inverse() * Vec3::Z)).z;
	float rate_estimate = (-((kinematic_Gs) * Global::g0) / aircraft.airflow_rel.mag()) / Global::deg2rad;

	return rate_estimate;
}

float rollRateForBankAngleHold(float target_bank_angle, float max_roll_rate, PID& rate_PID)
{
	max_roll_rate /= 180;	// ratio of 180 deg per sec
	//static PID roll_hold(0.05, 0.002, 0.01, 1);
	float roll = aircraft.attitude_roll_pitch.eulerAngles().x;
	float roll_error = target_bank_angle - roll;

	return rBound(rate_PID.update(roll_error, 0, Global::dt), -max_roll_rate, max_roll_rate);
}

float inputForMSLHold(float target_alt, float max_vert_vel, float max_input, PID& msl_PID, PID& vv_PID)
{
	//const float max_vert_vel = 80;
	const float meters2feet = 3.28084;
	float alt = XPLMGetDataf(Global::MSL_elevation) * meters2feet;
	//static PID alt_hold(0.5, 0.1, 0.0);

	float target_vert_v = rBound(msl_PID.update(target_alt, alt, Global::dt), -max_vert_vel, max_vert_vel);
	return inputForVVHold(target_vert_v, max_input, vv_PID);
}

float inputForVVHold(float target_vert_v, float max_input, PID& vv_PID)
{
	//const float max_norm_gs = 0.2;
	const float meters2feet = 3.28084;
	static float prev_alt;
	float alt = XPLMGetDataf(Global::MSL_elevation) * meters2feet;
	float vert_vel = (alt - prev_alt) / Global::dt;
	prev_alt = alt;
	//static PID vert_v_hold(0.025, 0.005, 0.01, 1);
	return -rBound(vv_PID.update(target_vert_v, vert_vel, Global::dt) * 0.5, -max_input, max_input);
}

float rollRateForHeadingHold(float target_heading, float max_bank_angle, float max_roll_rate, PID& heading_PID, PID& rate_PID)
{
	//static PID heading_hold(5, 0.0, 0.0);
	float heading = aircraft.attitude.eulerAngles().z;
	target_heading -= heading;
	if (target_heading < -180) target_heading += 360;
	else if (target_heading > 180) target_heading -= 360;

	float target_bank_angle = -rBound(heading_PID.update(target_heading, 0, Global::dt), -max_bank_angle, max_bank_angle);

	return rollRateForBankAngleHold(target_bank_angle, max_roll_rate, rate_PID);
}



