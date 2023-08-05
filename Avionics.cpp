#include "Avionics.h"

void updateVehicleInfo()
{
	GlobalVars::dt = XPLMGetDataf(GlobalVars::frame_time);
	findVehicleRotInfo();
	findRelativeAirflow();
	findVehicleAccel();
	findGroundState();
}

float VVIHold(float commanded_VVI)
{
	float dt = GlobalVars::dt;
	static PID vvi_hold(2, 0.5, 0.02, 0.1);
	//static PID accel_hold(2000, 200, 20);

	float weight = XPLMGetDataf(GlobalVars::total_mass) * GlobalVars::g0;
	static float prev_alt = 0;
	float alt = XPLMGetDataf(GlobalVars::MSL_elevation);
	float vvi = (alt - prev_alt) / dt;
	prev_alt = alt;

	//static float prev_vvi = 0;
	//float vert_accel = (vvi - prev_vvi) / dt;
	//prev_vvi = vvi;
	//
	//float desired_accel = vvi_hold.update(commanded_VVI, vvi, dt);
	//float thrust = weight + rBound(accel_hold.update(desired_accel, vert_accel, dt), -weight * 0.75, weight * 2);

	float agl = XPLMGetDataf(GlobalVars::alt_agl_handle);
	if (agl < 5)
		bound(commanded_VVI, -(agl + 1), 10);

	float thrust = weight + rBound(vvi_hold.update(commanded_VVI, vvi, dt) * 1000, -weight * 0.75, weight * 2);;

	GlobalVars::debug.println("desired vvi: ", commanded_VVI);
	GlobalVars::debug.println("actual vvi: ", vvi);
	//debug.println("desired accel: ", desired_accel);
	//debug.println("actual accel: ", vert_accel);
	//debug.println("desired differential thrust: ", thrust - weight);

	return thrust;
}

void findVehicleRotInfo()
{
	float dt = GlobalVars::dt;
	float roll_ = XPLMGetDataf(GlobalVars::roll);
	float pitch_ = -XPLMGetDataf(GlobalVars::pitch);
	float psi_ = 360 - XPLMGetDataf(GlobalVars::psi);
	GlobalVars::debug.println("psi: ", psi_);
	GlobalVars::vehicle_rot = Quat(Vec3(roll_, pitch_, psi_));
	GlobalVars::vehicle_roll_pitch = Quat(Vec3(roll_, pitch_, 0));

	static Quat prev_rot;
	GlobalVars::vehicle_rot_rate = (prev_rot.inverse() * GlobalVars::vehicle_rot).eulerAngles() / dt;

	prev_rot = GlobalVars::vehicle_rot;

	static Vec3 prev_rate;
	GlobalVars::vehicle_rot_accel = (GlobalVars::vehicle_rot_rate - prev_rate) / dt;
	prev_rate = GlobalVars::vehicle_rot_rate;
}
void findRelativeAirflow()
{
	GlobalVars::airflow_rel.y = -XPLMGetDataf(GlobalVars::incoming_air_flow[0]);
	GlobalVars::airflow_rel.x = -XPLMGetDataf(GlobalVars::incoming_air_flow[1]);
	GlobalVars::airflow_rel.z = XPLMGetDataf(GlobalVars::incoming_air_flow[2]);
}
void findVehicleAccel()
{
	GlobalVars::vehicle_accel.x = -XPLMGetDataf(GlobalVars::g_forces[0]);
	GlobalVars::vehicle_accel.y = -XPLMGetDataf(GlobalVars::g_forces[1]);
	GlobalVars::vehicle_accel.z = XPLMGetDataf(GlobalVars::g_forces[2]);
	GlobalVars::vehicle_accel *= GlobalVars::g0;
}
void findGroundState()
{
	float gear_forces = XPLMGetDataf(GlobalVars::gear_nml_forces);
	GlobalVars::on_ground_flag = (gear_forces > 1);
	GlobalVars::debug.println("gear forces:", gear_forces);
}
Vec3 torqueForRotVelHold(
	Vec3 target_rate,
	Vec3& prev_error,
	Vec3& error_accumulator,
	float p,
	float d,
	float i,
	float dt)
{
	//const float p = 0.1;
	//const float d = 0.002;
	//const float i = 0.01;

	Vec3 error = target_rate - GlobalVars::vehicle_rot_rate;
	Vec3 error_rate = (error - prev_error) / dt;
	prev_error = error;
	error_accumulator += error * dt;
	const float acc_max = 10;
	for (int i = 0; i < 3; i++)
		bound(error_accumulator.n[i], -acc_max, acc_max);

	Vec3 correction = (error * p) + (error_rate * d) + (error_accumulator * i);
	return GlobalVars::inertia_tensor * correction;
}

Vec3 rotRateHoldHover(Vec3 target_rates)	// returns torque require to hold input rotation rates
{
	Vec3 torque;
	for (int i = 0; i < 3; i++)
	{
		torque += rotRateHoldHover(target_rates.n[i], i);
	}
	return torque;
}
Vec3 rotRateHoldHover(float target_rate, int axis)	// returns torque require to hold input rotation rate
{
	static PID rotPIDs[3] =
	{
		PID(0.1, 0.002, 0.01, 10),
		PID(0.1, 0.002, 0.01, 10),
		PID(0.2, 0.002, 0.01, 10)
	};

	Vec3 angular_accel;
	angular_accel.n[axis] = rotPIDs[axis].update(target_rate, GlobalVars::vehicle_rot_rate.n[axis], GlobalVars::dt);

	return GlobalVars::inertia_tensor * angular_accel;
}

float rotHoldHoverRate(float target_angle, int axis)
{
	static PID rotPIDs[3] =
	{
		PID(4, 0.05, 0),
		PID(4, 0.05, 0)
	};
	return rotPIDs[axis].update(target_angle, GlobalVars::vehicle_rot.eulerAngles().n[axis], GlobalVars::dt);
}

Vec3 rotHoldHover(float target_angle, int axis)		// returns torque require to hold input rotation
{
	//static PID rotPIDs[3] =
	//{
	//	PID(4, 0.05, 0),
	//	PID(4, 0.05, 0)
	//};
	//
	//Vec3 angular_vel;
	//angular_vel.n[axis] = rotPIDs[axis].update(target_angle, GlobalVars::vehicle_rot.eulerAngles().n[axis], GlobalVars::dt);
	Vec3 angular_vel;
	angular_vel.n[axis] = rotHoldHoverRate(target_angle, axis);
	return rotRateHoldHover(angular_vel.n[axis], axis);
}

float sideSlipHoldHover(float target_slip_angle)
{
	static PID side_slip_PID(5, 0.05, 0);

	float slip_angle = rBound(asin(GlobalVars::airflow_rel.unit().y) / GlobalVars::deg2rad, -10, 10);
	float yaw_rate = side_slip_PID.update(target_slip_angle, slip_angle, GlobalVars::dt);
	//GlobalVars::debug.println("target_slip_angle: ", target_slip_angle);
	//GlobalVars::debug.println("current slip angle: ", slip_angle);
	return yaw_rate;
}

Vec3 hoverStabilityControlTorque(Vec3 control_in)
{

	Vec3 torque;
	torque += rotHoldHover(control_in.x, 0);
	torque += rotHoldHover(control_in.y, 1);
	torque += rotRateHoldHover(control_in.z, 2);

	return torque;

	/////	HOLD PITCH AND ROLL	 /////
	//static Vec3 prev_error;
	//const float p = 4;
	//const float d = 0.05;
	//
	//Vec3 error = control_in - GlobalVars::vehicle_rot.eulerAngles();
	//error.z = 0;	// remove yaw component
	//Vec3 error_rate = (error - prev_error) / dt;
	//prev_error = error;
	//
	//Vec3 commanded_rates = error * p + error_rate * d;
	//commanded_rates.z = control_in.z;
	//return rotRateHoldTorque(commanded_rates);
	////static Vec3 prev_error_vel, error_accumulator_vel;
	////const float p_v = 0.1;
	////const float d_v = 0.002;
	////const float i_v = 0.01;
	////return torqueForRotVelHold(commanded_rates, prev_error_vel, error_accumulator_vel, p_v, d_v, i_v, dt);
}

void holdAoA(float angle)
{
	static PID AoA_PID(0.5, 0.05, 0.2, 5);

	float AoA = (asin(GlobalVars::airflow_rel.unit().z) / GlobalVars::deg2rad);
	float pitch_ratio = -AoA_PID.update(-angle, AoA, GlobalVars::dt);
	float air_spd = GlobalVars::airflow_rel.mag();
	if (air_spd == 0) return;
	pitch_ratio *= (75 / air_spd) * (75 / air_spd);
	setControlSurface(pitch_ratio, 1);

	//GlobalVars::debug.println("target AoA: ", -angle);
	//GlobalVars::debug.println("actual AoA: ", AoA);
}

void holdSideSlip(float angle)
{
	static PID slip_PID(0.2, 0.01, 0.1, 10);

	float target_gs = 0.22 * angle;

	//float yaw_ratio = slip_PID.update(angle, asin(GlobalVars::airflow_rel.unit().y) / GlobalVars::deg2rad, GlobalVars::dt);
	float yaw_ratio = slip_PID.update(target_gs * GlobalVars::g0, GlobalVars::vehicle_accel.y, GlobalVars::dt);
	float air_spd = GlobalVars::airflow_rel.mag();
	if (air_spd == 0) return;
	yaw_ratio *= (75 / air_spd) * (75 / air_spd);

	setControlSurface(yaw_ratio, 2);
	GlobalVars::debug.println("target SIDE GS: ", target_gs);
	GlobalVars::debug.println("actual SIDE GS: ", GlobalVars::vehicle_accel.y / GlobalVars::g0);
}

float holdNormalGs(float Gs)
{
	static PID g_force_PID(0.0, 0.000, 0.75, 75);
	
	float air_spd = GlobalVars::airflow_rel.mag();
	if (air_spd == 0) return 0;
	float speed_correction_factor = (75 / air_spd) * (75 / air_spd);
	
	g_force_PID.I = 0.75 / speed_correction_factor;
	
	return -g_force_PID.update(Gs * GlobalVars::g0, GlobalVars::vehicle_accel.z, GlobalVars::dt) * speed_correction_factor;
	
}

void fwdStabilityControl(Vec3 command_input)
{
	float dt = GlobalVars::dt;
	static PID
		roll(0.5, 0.01, 0.0),
		pitch(0.5, 0.02, 0.5);// ,
		//yaw(50, 0.5, 0.0);

	const float max_roll_rate = 360;
	const float max_gs = 6;
	const float min_gs = -2;
	const float max_slip_angle = 5;
	float input = command_input.y;
	bound(command_input, -1, 1);
	command_input.x *= max_roll_rate;
	command_input.y = rBound(-command_input.y * max_gs + 1, min_gs, max_gs);// *GlobalVars::g0;
	command_input.z *= max_slip_angle;

	Vec3 surface_ratios;
	surface_ratios.x = roll.update(command_input.x, GlobalVars::vehicle_rot_rate.x, dt);
	surface_ratios.y = 0;// -pitch.update(command_input.y, GlobalVars::vehicle_accel.z, dt);
	//surface_ratios.z = yaw.update(command_input.z, asin(GlobalVars::airflow_rel.unit().y) / GlobalVars::deg2rad, dt);


	//const float ref_velocity = 25;
	//float proportional_factor = airflow_rel.mag();
	if (GlobalVars::airflow_rel.sqMag() != 0) surface_ratios /= GlobalVars::airflow_rel.sqMag() / 625;

	bound(surface_ratios, -1, 1);

	GlobalVars::debug.println("desired roll rate: ", command_input.x);
	GlobalVars::debug.println("desired normal gs: ", command_input.y);// / GlobalVars::g0);
	GlobalVars::debug.println("desired side slip: ", command_input.z);
	//GlobalVars::debug.println("control deflections: ", surface_ratios);
	//setControlSurfaces(surface_ratios);

	setControlSurface(surface_ratios.x, 0);
	//setControlSurface(surface_ratios.z, 2);

	holdAoA(holdNormalGs(command_input.y));
	holdSideSlip(command_input.z);

	//holdAoA(input * 10);
}



