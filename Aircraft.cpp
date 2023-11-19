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

void Aircraft::hideProps(float max_rpm)
{
	if(getMotorRPM(2) < max_rpm)
		XPLMSetDataf(Global::thrust_vctr, 1);
}

void Aircraft::showProps()
{
	XPLMSetDataf(Global::thrust_vctr, 0.9);
}

void Aircraft::cutHoverThrottles()
{
	float throttle[3] = { 0, 0, 0 };
	XPLMSetDatavf(Global::throttle_ratio, throttle, 2, 3);
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

void Aircraft::setFwdThrust(float thrust)
{
	setMotorThrustDirection(Vec3::Z * thrust / 2, 0);
	setMotorThrustDirection(Vec3::Z * thrust / 2, 1);
}

void Aircraft::setMotorThrustDirection(Vec3 thrust, int motor)
{
	bool can_reverse = false;
	float max_angle = 0;
	static PID throttleFromThrust_PIDs[5] =
	{
		PID(0.05, 0.00, 0.5, 100), // left fwd prop
		PID(0.05, 0.00, 0.5, 100), // right fwd prop
		PID(0.8, 0.02, 0.5, 0, 50), // left fan
		PID(0.8, 0.02, 0.5, 0, 50), // right fan
		PID(0.8, 0.02, 0.5, 0, 50)  // nose fan
	};

	switch (motor)
	{
	case 0:
	case 1:
		can_reverse = true;
		max_angle = 0;
		break;
	case 2:
	case 3:
		can_reverse = false;
		max_angle = 60;
		break;
	case 4:
		can_reverse = true;
		max_angle = 0;
		break;
	}


	if (can_reverse)
	{
		float angle = 0;
		if (thrust.dot(Vec3::Z) < 0) // if thust is negative, flip fan 180 degrees to simulate running in reverse
			angle = 180;
		XPLMSetDatavf(Global::acf_vertcant, &angle, motor, 1);
	}

	/// GET THROTTLE FROM THRUST
	float commanded_thrust = thrust.mag();
	float actual_thrust;
	XPLMGetDatavf(Global::prop_thrust, &actual_thrust, motor, 1);
	float throttle = throttleFromThrust_PIDs[motor].update(commanded_thrust, actual_thrust, Global::dt) / 100;

	///	SET THROTTLE
	//static RollingAvg fwd_avg1(100);
	//static RollingAvg fwd_avg2(100);
	//if (motor == 0) fwd_avg1.apply(throttle);
	//else if (motor == 1) fwd_avg2.apply(throttle);
	
	bound(throttle, 0, 1);
	XPLMSetDatavf(Global::throttle_ratio, &throttle, motor, 1);
	if (max_angle == 0) return;

	/// SET PROP ANGLE
	float vert_angle = atan2(-thrust.x, thrust.z) / Global::deg2rad;
	float side_angle = atan2(-thrust.y, thrust.z) / Global::deg2rad;

	bound(vert_angle, -max_angle, max_angle);
	bound(side_angle, -max_angle, max_angle);

	XPLMSetDatavf(Global::acf_vertcant, &vert_angle, motor, 1);
	XPLMSetDatavf(Global::acf_sidecant, &side_angle, motor, 1);
}

float Aircraft::getMotorRPM(int motor)
{
	float rpm;
	XPLMGetDatavf(Global::engine_speed, &rpm, motor, 1);
	return rpm;
}