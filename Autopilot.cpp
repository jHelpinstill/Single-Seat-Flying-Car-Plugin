#include "Avionics.h"

void holdAirSpd(float final_spd)
{
	float dt = Global::dt;
	static PID auto_throttle(500, 40, 100, 20);
	const float max_accel = 5;	// ms^-2
	const float min_accel = -3;

	float prograde_spd = aircraft.airflow_rel.mag();

	float thrust = rBound(auto_throttle.update(final_spd, prograde_spd, dt), min_accel * aircraft.mass, max_accel * aircraft.mass);
	aircraft.setForwardThrust(thrust);
}

void holdPositionVaryAttitude(Vec3 input, float throttle_handle, float max_VV, Vec3 attitude_params)
// max_VV in m/s, attitude_params in deg roll, deg pitch, deg/s yaw rate
{
	Vec3 force;
	static PID vv_hold(2, 0.5, 0.02, 0.1);
	force.z = thrustForVVHoldHover(throttle_handle * max_VV, vv_hold);
	force = aircraft.attitude_roll_pitch.inverse() * force;

	aircraft.addHoverForce(force);

	input.x *= attitude_params.x;
	input.y *= attitude_params.y;
	input.z *= attitude_params.z;

	static PID att_PIDs[2] =
	{
		PID(4, 0.05, 0, 0, 0.02, 0.15),
		PID(4, 0.05, 0, 0, 0.05, 0.15)
	};
	static PID rate_PIDs[3] =
	{
		PID(0.1, 0.002, 0.01, 10, 0.02, 0.15),
		PID(0.1, 0.002, 0.01, 10, 0.05, 0.15),
		PID(0.2, 0.002, 0.01, 10)
	};

	Vec3 torque;
	torque += torqueForAttitudeHoldHover(input.x, 0, att_PIDs[0], rate_PIDs[0]);
	torque += torqueForAttitudeHoldHover(input.y, 1, att_PIDs[1], rate_PIDs[1]);
	torque += torqueForAttitudeRateHoldHover(input.z, 2, rate_PIDs[2]);

	aircraft.addHoverTorque(torque);
}

void holdSideSlip(float angle)
{
	static PID slip_PID(0.8, 0.15, 0.15, 10, 0.3, 1);

	float current_angle = asin(aircraft.airflow_rel.unit().y) / Global::deg2rad;
	float yaw_ratio = slip_PID.update(angle, current_angle, Global::dt);

	//float yaw_ratio = slip_PID.update(target_gs * GlobalVars::g0, GlobalVars::vehicle_accel.y, GlobalVars::dt);
	float air_spd = aircraft.airflow_rel.mag();
	if (air_spd < 1)
		yaw_ratio *= (75 / 1) * (75 / 1);
	else
		yaw_ratio *= (75 / air_spd) * (75 / air_spd);

	aircraft.setControlSurface(yaw_ratio, 2);

	//Global::debug.setPrecision(5);
	//Global::debug.println("target slip angle: ", angle);
	//Global::debug.println("actual slip angle: ", current_angle);
	//Global::debug.setPrecision(3);
}

void fwdStabilityControl(Vec3 command_input)
{
	float dt = Global::dt;
	static PID
		roll(0.35, 0.00, 0.7, 80),
		pitch(0.5, 0.02, 0.5);// ,
	//yaw(50, 0.5, 0.0);

	const float max_roll_rate = 300;
	const float max_gs = 6;
	const float min_gs = -2;
	const float max_slip_angle = 5;
	float input = command_input.y;
	bound(command_input, -1, 1);
	//command_input.x *= max_roll_rate;
	command_input.y = rBound(-command_input.y * max_gs + 1, min_gs, max_gs);// *GlobalVars::g0;
	command_input.z *= max_slip_angle;

	Vec3 surface_ratios;
	surface_ratios.x = roll.update(command_input.x * max_roll_rate, aircraft.rot_rate.x, dt);
	surface_ratios.y = 0;

	if (aircraft.airflow_rel.sqMag() != 0) surface_ratios /= aircraft.airflow_rel.sqMag() / 625;

	bound(surface_ratios, -1, 1);

	Global::debug.println("desired roll rate: ", command_input.x);
	Global::debug.println("desired normal gs: ", command_input.y);
	Global::debug.println("desired side slip: ", command_input.z);

	aircraft.setControlSurface(surface_ratios.x, 0);

	//holdAoA(holdNormalGs(command_input.y));

	//float rate_for_gs =  (-(command_input.y * Global::g0) / Global::vehicle.airflow_rel.mag()) / Global::deg2rad;
	holdPitchRateFwd(pitchRateForNormalGsHold(command_input.y));
	holdSideSlip(command_input.z);
}

void holdPitchRateFwd(float rate)
{
	static PID rate_PID(0.02, 0.00, 0.2, 5);
	const float max_rate = 60;

	bound(rate, -max_rate, max_rate);
	Vec3 airflow = aircraft.airflow_rel;
	airflow.y = 0;
	float lift_c = XPLMGetDataf(Global::atm_density_ratio) * airflow.sqMag() / 5625;
	if (lift_c < 0.5) lift_c = 0.5;

	aircraft.setControlSurface(rate_PID.update(rate, aircraft.rot_rate.y, Global::dt) / lift_c, 1);

	Global::debug.println("desired pitch rate: ", rate);
}

void holdAoA(float angle)
{
	static PID aoa_PID(0.5, 0.05, 0.0);
	float AoA = (asin(aircraft.airflow_rel.unit().z) / Global::deg2rad);
	holdPitchRateFwd(-aoa_PID.update(angle, AoA, Global::dt));
}

void printAutopilotData(float target_vel, float target_alt, float target_heading)
{
	float heading = aircraft.attitude.eulerAngles().z;
	Global::debug.println("");
	Global::debug.println("AUTO PILOT ON");
	Global::debug.println("Target speed mph: ", (float)(target_vel * 2.237));
	Global::debug.println("Target MSL: ", target_alt);
	Global::debug.println("Target Heading: ", headingCorrection(target_heading));
	Global::debug.println("Heading: ", headingCorrection(heading));
	Global::debug.println("Target Heading RAW: ", target_heading);
	Global::debug.println("Heading RAW: ", heading);
	Global::debug.println("psi: ", XPLMGetDataf(Global::psi));
	Global::debug.println();
}