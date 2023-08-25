#pragma once

#include "Vec3.h"
#include "Matrix.h"
#include "Quat.h"
#include "XPLMDataAccess.h"
#include "ControlMatrix.h"
#include "PID.h"

class Aircraft
{
public:
	ControlMatrix lift_fan_matrix;
	Matrix inertia_tensor;

	Quat attitude, attitude_roll_pitch;
	Vec3 rot_rate, rot_accel, airflow_rel, accel;
	float mass;

	bool on_ground = true;

	Aircraft();

};

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
