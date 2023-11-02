#include "Avionics.h"

void updateVehicleInfo()
{
	Global::dt = XPLMGetDataf(Global::frame_time);
	findVehicleRotInfo();
	findRelativeAirflow();
	findVehicleAccel();
	findGroundState();
}

float VVIHold(float commanded_VVI)
{
	float dt = Global::dt;
	static PID vvi_hold(2, 0.5, 0.02, 0.1);
	//static PID accel_hold(2000, 200, 20);

	float weight = XPLMGetDataf(Global::total_mass) * Global::g0;
	static float prev_alt = 0;
	float alt = XPLMGetDataf(Global::MSL_elevation);
	float vvi = (alt - prev_alt) / dt;
	prev_alt = alt;

	//static float prev_vvi = 0;
	//float vert_accel = (vvi - prev_vvi) / dt;
	//prev_vvi = vvi;
	//
	//float desired_accel = vvi_hold.update(commanded_VVI, vvi, dt);
	//float thrust = weight + rBound(accel_hold.update(desired_accel, vert_accel, dt), -weight * 0.75, weight * 2);

	float agl = XPLMGetDataf(Global::alt_agl_handle);
	if (agl < 5)
		bound(commanded_VVI, -(agl + 1), 10);

	float thrust = weight + rBound(vvi_hold.update(commanded_VVI, vvi, dt) * 1000, -weight * 0.75, weight * 2);;

	Global::debug.println("desired vvi: ", commanded_VVI);
	Global::debug.println("actual vvi: ", vvi);
	//debug.println("desired accel: ", desired_accel);
	//debug.println("actual accel: ", vert_accel);
	//debug.println("desired differential thrust: ", thrust - weight);

	return thrust;
}

void findVehicleRotInfo()
{
	float dt = Global::dt;
	float roll_ = XPLMGetDataf(Global::roll);
	float pitch_ = -XPLMGetDataf(Global::pitch);
	float psi_ = 360 - XPLMGetDataf(Global::psi);
	Global::debug.println("psi: ", psi_);
	Global::vehicle.attitude = Quat(Vec3(roll_, pitch_, psi_));
	Global::vehicle.attitude_roll_pitch = Quat(Vec3(roll_, pitch_, 0));

	static Quat prev_attitude;
	Global::vehicle.rot_rate = (prev_attitude.inverse() * Global::vehicle.attitude).eulerAngles() / dt;

	prev_attitude = Global::vehicle.attitude;

	static Vec3 prev_rate;
	Global::vehicle.rot_accel = (Global::vehicle.rot_rate - prev_rate) / dt;
	prev_rate = Global::vehicle.rot_rate;
}
void findRelativeAirflow()
{
	Global::vehicle.airflow_rel.x = -XPLMGetDataf(Global::air_relative_velocity[2]);
	Global::vehicle.airflow_rel.y = -XPLMGetDataf(Global::air_relative_velocity[0]);
	Global::vehicle.airflow_rel.z = XPLMGetDataf(Global::air_relative_velocity[1]);
}
void findVehicleAccel()
{
	Global::vehicle.accel.x = -XPLMGetDataf(Global::g_forces[0]);
	Global::vehicle.accel.y = -XPLMGetDataf(Global::g_forces[1]);
	Global::vehicle.accel.z = XPLMGetDataf(Global::g_forces[2]);
	Global::vehicle.accel *= Global::g0;
}
void findGroundState()
{
	float gear_forces = XPLMGetDataf(Global::gear_nml_forces);
	Global::vehicle.on_ground = (gear_forces > 1);
	Global::debug.println("gear forces:", gear_forces);
}
Vec3 torqueForRateHold(
	Vec3 target_rate,
	Vec3& prev_error,
	Vec3& error_accumulator,
	float p,
	float d,
	float i,
	float dt)
{
	Vec3 error = target_rate - Global::vehicle.rot_rate;
	Vec3 error_rate = (error - prev_error) / dt;
	prev_error = error;
	error_accumulator += error * dt;
	const float acc_max = 10;
	for (int i = 0; i < 3; i++)
		bound(error_accumulator.n[i], -acc_max, acc_max);

	Vec3 correction = (error * p) + (error_rate * d) + (error_accumulator * i);
	return Global::vehicle.inertia_tensor * correction;
}

Vec3 rateHoldHover(float target_rate, int axis)	// returns torque require to hold input rotation rate
{
	static PID rotPIDs[3] =
	{
		PID(0.1, 0.002, 0.01, 10, 0.02, 0.15),
		PID(0.1, 0.002, 0.01, 10, 0.05, 0.15),
		PID(0.2, 0.002, 0.01, 10)
	};

	Vec3 angular_accel;
	angular_accel.n[axis] = rotPIDs[axis].update(target_rate, Global::vehicle.rot_rate.n[axis], Global::dt);

	return Global::vehicle.inertia_tensor * angular_accel;
}


///// DELETE WHEN ABLE! /////
float rotHoldHoverRate(float target_angle, int axis)
{
	static PID rotPIDs[3] =
	{
		PID(4, 0.05, 0),
		PID(4, 0.05, 0)
	};
	return rotPIDs[axis].update(target_angle, Global::vehicle.attitude.eulerAngles().n[axis], Global::dt);
}

Vec3 attitudeHoldHover(float target_angle, int axis)// returns torque require to hold given angle about axis
{
	static PID rotPIDs[3] =
	{
		PID(4, 0.05, 0, 0, 0.02, 0.15),
		PID(4, 0.05, 0, 0, 0.05, 0.15)
	};
	float rate = rotPIDs[axis].update(target_angle, Global::vehicle.attitude.eulerAngles().n[axis], Global::dt);
	return rateHoldHover(rate, axis);
}

float sideSlipHoldHover(float target_slip_angle)
{
	static PID side_slip_PID(5, 0.05, 0);

	float slip_angle = rBound(asin(Global::vehicle.airflow_rel.unit().y) / Global::deg2rad, -10, 10);
	float yaw_rate = side_slip_PID.update(target_slip_angle, slip_angle, Global::dt);
	return yaw_rate;
}

Vec3 attitudeControlTorque(Vec3 control_in)
{

	Vec3 torque;
	torque += attitudeHoldHover(control_in.x, 0);
	torque += attitudeHoldHover(control_in.y, 1);
	torque += rateHoldHover(control_in.z, 2);

	return torque;
}

void holdAoA(float angle)
{
	//static PID AoA_PID(0.5, 0.05, 0.2, 5);
	//
	//float AoA = (asin(Global::vehicle.airflow_rel.unit().z) / Global::deg2rad);
	//float pitch_ratio = -AoA_PID.update(-angle, AoA, Global::dt);
	//float air_spd = Global::vehicle.airflow_rel.mag();
	//if (air_spd == 0) return;
	//pitch_ratio *= (75 / air_spd) * (75 / air_spd);
	//
	//if (pitch_ratio > 1 || pitch_ratio < -1)
	//	AoA_PID.accumulator -= AoA_PID.error * Global::dt;
	//
	//Global::vehicle.setControlSurface(pitch_ratio, 1);
	//
	//Global::debug.println("AoA accumulator: ", AoA_PID.accumulator);
	////GlobalVars::debug.println("actual AoA: ", AoA);

	static PID aoa_PID(0.5, 0.05, 0.0);
	float AoA = (asin(Global::vehicle.airflow_rel.unit().z) / Global::deg2rad);
	holdPitchRateFwd(-aoa_PID.update(angle, AoA, Global::dt));
}

void holdPitchRateFwd(float rate)
{
	static PID rate_PID(0.02, 0.00, 0.2, 5);
	const float max_rate = 60;

	bound(rate, -max_rate, max_rate);
	Vec3 airflow = Global::vehicle.airflow_rel;
	airflow.y = 0;
	float lift_c = XPLMGetDataf(Global::atm_density_ratio) * airflow.sqMag() / 5625;
	if (lift_c < 0.5) lift_c = 0.5;

	Global::vehicle.setControlSurface(rate_PID.update(rate, Global::vehicle.rot_rate.y, Global::dt) / lift_c, 1);

	Global::debug.println("desired pitch rate: ", rate);
}

PID* holdSideSlip(float angle, bool return_PID_ptr)
{
	//const float default_p = 0.8;
	static PID slip_PID(0.8, 0.15, 0.15, 10, 0.3, 1);

	if (return_PID_ptr) return &slip_PID;

	float current_angle = asin(Global::vehicle.airflow_rel.unit().y) / Global::deg2rad;
	float yaw_ratio = slip_PID.update(angle, current_angle, Global::dt);

	//float yaw_ratio = slip_PID.update(target_gs * GlobalVars::g0, GlobalVars::vehicle_accel.y, GlobalVars::dt);
	float air_spd = Global::vehicle.airflow_rel.mag();
	if (air_spd == 0) return nullptr;
	yaw_ratio *= (75 / air_spd) * (75 / air_spd);

	Global::vehicle.setControlSurface(yaw_ratio, 2);
	Global::debug.setPrecision(5);
	Global::debug.println("target slip angle: ", angle);
	Global::debug.println("actual slip angle: ", current_angle);
	Global::debug.setPrecision(3);

	return nullptr;
}

float holdNormalGs(float Gs)
{
	//static PID g_force_PID(0.0, 0.000, 0.75, 75);
	//
	//float air_spd = Global::vehicle.airflow_rel.mag();
	//if (air_spd == 0) return 0;
	//float speed_correction_factor = (75 / air_spd) * (75 / air_spd);
	//
	//g_force_PID.I = 0.75 / speed_correction_factor;
	//
	//return -g_force_PID.update(Gs * Global::g0, Global::vehicle.accel.z, Global::dt) * speed_correction_factor;
	
	static PID g_force_PID(0.0, 0.000, 0.2, 2);

	float kinematic_Gs = ((Vec3::Z * Gs) - (Global::vehicle.attitude.inverse() * Vec3::Z)).z;//(Global::vehicle.attitude.inverse() * ((Global::vehicle.attitude * (Vec3::Z * Gs)) - Vec3::Z)).z;
	float rate_estimate = (-((kinematic_Gs) * Global::g0) / Global::vehicle.airflow_rel.mag()) / Global::deg2rad;

	return rate_estimate;
	//float actual_accel = Global::vehicle.accel.z;

	//return rate_estimate + g_force_PID.update(Gs * Global::g0, actual_accel, Global::dt);
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
	surface_ratios.x = roll.update(command_input.x * max_roll_rate, Global::vehicle.rot_rate.x, dt);
	surface_ratios.y = 0;

	if (Global::vehicle.airflow_rel.sqMag() != 0) surface_ratios /= Global::vehicle.airflow_rel.sqMag() / 625;

	bound(surface_ratios, -1, 1);

	Global::debug.println("desired roll rate: ", command_input.x);
	Global::debug.println("desired normal gs: ", command_input.y);
	Global::debug.println("desired side slip: ", command_input.z);

	Global::vehicle.setControlSurface(surface_ratios.x, 0);

	//holdAoA(holdNormalGs(command_input.y));

	//float rate_for_gs =  (-(command_input.y * Global::g0) / Global::vehicle.airflow_rel.mag()) / Global::deg2rad;
	holdPitchRateFwd(holdNormalGs(command_input.y));
	holdSideSlip(command_input.z);
}



