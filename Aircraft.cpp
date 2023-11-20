#include "Aircraft.h"
#include "GlobalVars.h"
#include "Util.h"

Aircraft aircraft;

void Aircraft::begin()
{
	CoM_position *= 0.3048;
	for (int i = 0; i < 3; i++)
		fan_positions[i] *= 0.3048;

	aircraft.lift_fan_matrix.fillMatrix(fan_positions[0], fan_positions[1], fan_positions[2], CoM_position);

	aircraft.mass = XPLMGetDataf(Global::total_mass);
	for (int i = 0; i < 3; i++)
		aircraft.inertia_tensor.element(i, i) = aircraft.mass * XPLMGetDataf(Global::moments[i]);
}

void Aircraft::update()
{
	findRot();
	findRelativeAirflow();
	findAccel();
	findGroundState();
}

void Aircraft::applyChanges()
{

	Vec3 thrust_vectors[3];
	lift_fan_matrix.compute(hover_forces, hover_torques, thrust_vectors[0], thrust_vectors[1], thrust_vectors[2]);
	
	setHoverMotors(thrust_vectors);
	setForwardMotors();
}

void Aircraft::setHoverMotors(Vec3* thrust_vectors)
{
	static PID fan_pids[3] =
	{
		PID(0.8, 0.02, 0.5, 0, 50), // left fan
		PID(0.8, 0.02, 0.5, 0, 50), // right fan
		PID(0.8, 0.02, 0.5, 0, 50) // nose fan
	};

	const float max_angle = 60;	// degrees +or-

	/// FLIP NOSE MOTOR?
	float angle = 0;
	if (thrust_vectors[2].dot(Vec3::Z) < 0) // if thust is negative, flip fan 180 degrees to simulate running in reverse
		angle = 180;
	XPLMSetDatavf(Global::acf_vertcant, &angle, 4, 1);

	/// GET THROTTLE FROM THRUST
	float actual_thrust[3];
	XPLMGetDatavf(Global::prop_thrust, actual_thrust, 2, 3);

	float throttles[3]{};
	for(int i = 0; i < 3; i++)
	{
		throttles[i] = fan_pids[i].update(thrust_vectors[i].mag(), actual_thrust[i], Global::dt) / 100;
		bound(throttles[i], 0, 1);
	}

	XPLMSetDatavf(Global::throttle_ratio, throttles, 2, 3);

	/// SET WING PROP ANGLES
	for(int i = 0; i < 2; i++)
	{
		float vert_angle = atan2(-thrust_vectors[i].x, thrust_vectors[i].z) / Global::deg2rad;
		float side_angle = atan2(-thrust_vectors[i].y, thrust_vectors[i].z) / Global::deg2rad;

		bound(vert_angle, -max_angle, max_angle);
		bound(side_angle, -max_angle, max_angle);

		XPLMSetDatavf(Global::acf_vertcant, &vert_angle, i, 1);
		XPLMSetDatavf(Global::acf_sidecant, &side_angle, i, 1);
	}
}

void Aircraft::hideProps(float max_rpm)
{
	if(getMotorRPM(2) < max_rpm)
		XPLMSetDataf(Global::thrust_vctr, 1);
}

void Aircraft::showProps()
{
	XPLMSetDataf(Global::thrust_vctr, 0.9);
}

void Aircraft::addHoverAngularAccel(Vec3 alpha)
{
	hover_torques += inertia_tensor * alpha;
}

void Aircraft::addHoverTorque(Vec3 torque)
{
	hover_torques += torque;
}

void Aircraft::addHoverLinearAccel(Vec3 accel)
{
	hover_forces += accel * mass;
}

void Aircraft::addHoverForce(Vec3 force)
{
	hover_forces += force;
}

void Aircraft::addControlSurfaceInput(Vec3 input)
{
	control_surface_inputs += input;
}

void Aircraft::cutHoverThrottles()
{
	float throttle[3] = { 0, 0, 0 };
	XPLMSetDatavf(Global::throttle_ratio, throttle, 2, 3);
}

void Aircraft::cutForwardThrottles()
{
	float throttle[2] = { 0, 0 };
	XPLMSetDatavf(Global::throttle_ratio, throttle, 0, 2);
}

void Aircraft::setControlSurface(float input, int axis)
{
	if (axis == 1 || axis == 2) input = -input;
	bound(input, -1, 1);
	XPLMSetDataf(Global::control_surface_actuators[axis], input);
}

void Aircraft::mixControlSurface(float input, int axis, float mix_ratio)
{
	float starting_ratio = XPLMGetDataf(Global::control_surface_actuators[axis]);
	if (axis == 1 || axis == 2) starting_ratio = -starting_ratio;
	setControlSurface(input * mix_ratio + starting_ratio * (1 - mix_ratio), axis);
}

Vec3 Aircraft::getControlSurfaces()
{
	Vec3 ratios;
	for (int i = 0; i < 3; i++)
	{
		ratios.n[i] = XPLMGetDataf(Global::control_surface_actuators[i]);
		if (i == 1 || i == 2) ratios.n[i] *= -1;
	}
	return ratios;
}

void Aircraft::mixControlSurfaces(Vec3 input, float mix_ratio)
{
	for (int i = 0; i < 3; i++)
		mixControlSurface(input.n[i], i, mix_ratio);
}

void Aircraft::setControlSurfaces(Vec3 input)
{
	bound(input, -1, 1);
	for (int i = 0; i < 3; i++)
		setControlSurface(input.n[i], i);
}

void Aircraft::setForwardThrust(float thrust)
{
	forward_thrust = thrust;
}

void Aircraft::setForwardMotors()
{
	static PID forward_PIDs[2] =
	{
		PID(0.05, 0.00, 0.5, 100), // left fwd prop
		PID(0.05, 0.00, 0.5, 100), // right fwd prop
	};

	/// FLIP MOTORS?
	float angle = 0;
	if (forward_thrust < 0) // if thust is negative, flip fan 180 degrees to simulate running in reverse
		angle = 180;
	XPLMSetDatavf(Global::acf_vertcant, &angle, 0, 2);

	/// GET THROTTLE FROM THRUST
	float actual_thrust[2];
	XPLMGetDatavf(Global::prop_thrust, actual_thrust, 0, 2);

	float throttles[2]{};
	for (int i = 0; i < 2; i++)
	{
		throttles[i] = forward_PIDs[i].update(mag(forward_thrust / 2), actual_thrust[i], Global::dt) / 100;
		bound(throttles[i], 0, 1);
	}

	XPLMSetDatavf(Global::throttle_ratio, throttles, 0, 2);
}

float Aircraft::getMotorRPM(int motor)
{
	float rpm;
	XPLMGetDatavf(Global::engine_speed, &rpm, motor, 1);
	return rpm;
}

void Aircraft::findRot()
{
	float dt = Global::dt;
	float roll_ = XPLMGetDataf(Global::roll);
	float pitch_ = -XPLMGetDataf(Global::pitch);
	float psi_ = 360 - XPLMGetDataf(Global::psi);
	//Global::debug.println("psi: ", psi_);

	attitude = Quat(Vec3(roll_, pitch_, psi_));
	attitude_roll_pitch = Quat(Vec3(roll_, pitch_, 0));

	static Quat prev_attitude;
	rot_rate = (prev_attitude.inverse() * attitude).eulerAngles() / dt;

	prev_attitude = attitude;

	static Vec3 prev_rate;
	rot_accel = (rot_rate - prev_rate) / dt;
	prev_rate = rot_rate;
}

void Aircraft::findAccel()
{
	accel.x = -XPLMGetDataf(Global::g_forces[0]);
	accel.y = -XPLMGetDataf(Global::g_forces[1]);
	accel.z = XPLMGetDataf(Global::g_forces[2]);
	accel *= Global::g0;
}

void Aircraft::findRelativeAirflow()
{
	airflow_rel.x = -XPLMGetDataf(Global::air_relative_velocity[2]);
	airflow_rel.y = -XPLMGetDataf(Global::air_relative_velocity[0]);
	airflow_rel.z = XPLMGetDataf(Global::air_relative_velocity[1]);
}

void Aircraft::findGroundState()
{
	float gear_forces = XPLMGetDataf(Global::gear_nml_forces);
	on_ground = (gear_forces > 1);
	//Global::debug.println("gear forces:", gear_forces);
}
