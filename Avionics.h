#pragma once

#include "GlobalVars.h"
#include "Util.h"
#include "PID.h"
#include "Aircraft.h"

float VVIHold(float commanded_VVI);
void updateVehicleInfo();
void findVehicleRotInfo();
void findRelativeAirflow();
void findVehicleAccel();
void findGroundState();
Vec3 rotRateHoldHover(float target_rate, int axis);
float rotHoldHoverRate(float target_angle, int axis);
Vec3 rotHoldHover(float target_angle, int axis);
float sideSlipHoldHover(float target_slip_angle);
void holdAoA(float angle);
float holdNormalGs(float Gs);
PID* holdSideSlip(float angle, bool return_PID_ptr = false);

Vec3 torqueForRateHold(
	Vec3 target_rate,
	Vec3& prev_error,
	Vec3& error_accumulator,
	float p,
	float d,
	float i,
	float dt);

Vec3 attitudeControlTorque(Vec3 control_in);
void fwdStabilityControl(Vec3 command_input);

/////   AUTOPILOT   /////
float holdHeading(float target_heading, float max_bank_angle, float max_roll_rate);

float holdBankAngle(float target_bank_angle, float max_roll_rate);

float holdMSL(float target_alt, float max_vert_vel, float max_norm_gs);

float holdVertVel(float target_vert_v, float max_norm_gs);

void holdAirSpd(float target_spd);
