#pragma once

#include "Vec3.h"
#include "Matrix.h"
#include "Quat.h"
#include "XPLMDataAccess.h"
#include "ControlMatrix.h"
#include "PID.h"
#include <vector>

class Aircraft
{
private:
	// in ft
	Vec3 fan_positions[3] = { Vec3(-1.8, 0, 0.05), Vec3(-11, 8.5, 0.57), Vec3(-11, -8.5, 0.57) }; // nose, left, right
	Vec3 CoM_position = Vec3(-10, 0, 0.5);

	Vec3 hover_torques;
	Vec3 hover_forces;

	Vec3 control_surface_inputs;

	float forward_thrust{};

	void setHoverMotors(Vec3* thrust_vectors);
	void setForwardMotors();

	void findAttitude();
	void findAccel();
	void findRelativeAirflow();
	void findGroundState();

public:
	ControlMatrix lift_fan_matrix;
	Matrix inertia_tensor = Matrix(3, 3);

	Quat attitude, attitude_roll_pitch;
	Vec3 rot_rate, rot_accel, airflow_rel, accel;
	float mass = 1;
	float alt_MSL = 0;
	float alt_agl = 0;
	float vert_vel = 0;

	bool on_ground = true;

	Aircraft() {}

	void begin();
	void update();
	void updateMotors();

	void hideProps(float max_rpm);
	void showProps();

	void addHoverAngularAccel(Vec3 alpha);
	void addHoverTorque(Vec3 torque);
	void addHoverLinearAccel(Vec3 accel);
	void addHoverForce(Vec3 force);

	void setForwardThrust(float thrust);

	void cutHoverThrottles();
	void cutForwardThrottles();

	void addControlSurfaceInput(Vec3 input);
	void setControlSurface(float input, int axis);
	void mixControlSurface(float input, int axis, float mix_ratio);
	void setControlSurfaces(Vec3 input);
	void mixControlSurfaces(Vec3 input, float mix_ratio);
	Vec3 getControlSurfaces();

	float getMotorRPM(int motor);

};

extern Aircraft aircraft;