#pragma once

#include "Vec3.h"
#include "Matrix.h"
#include "Quat.h"
#include "XPLMDataAccess.h"
#include "ControlMatrix.h"
#include "PID.h"

class Aircraft
{
private:
	// in ft
	Vec3 fan_positions[3] = { Vec3(-1.8, 0, 0.05), Vec3(-11, 8.5, 0.57), Vec3(-11, -8.5, 0.57) }; // nose, left, right
	Vec3 CoM_position = Vec3(-10, 0, 0.5);

public:
	ControlMatrix lift_fan_matrix;
	Matrix inertia_tensor = Matrix(3, 3);

	Quat attitude, attitude_roll_pitch;
	Vec3 rot_rate, rot_accel, airflow_rel, accel;
	float mass = 1;

	bool on_ground = true;

	Aircraft() {}

	void begin();

	void hideProps(float max_rpm);
	void showProps();
	void cutHoverThrottles();
	void setControlSurface(float input, int axis);
	void mixControlSurface(float input, int axis, float mix_ratio);
	void setControlSurfaces(Vec3 input);
	void mixControlSurfaces(Vec3 input, float mix_ratio);
	Vec3 getControlSurfaces();
	void setMotorThrustDirection(Vec3 thrust, int motor);
	void setFwdThrust(float thrust);
	float getMotorRPM(int motor);
};

extern Aircraft aircraft;