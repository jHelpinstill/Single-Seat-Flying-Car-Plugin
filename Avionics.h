#pragma once

#include "GlobalVars.h"
#include "Util.h"
#include "PID.h"
#include "Aircraft.h"

/////   HELPERS   /////
float thrustForVVHoldHover(float commanded_VV, PID& vv_hold);
Vec3 torqueForRateHoldHover(float target_rate, int axis, PID& rot_PID);
float rotHoldHoverRate(float target_angle, int axis);
Vec3 torqueForAttitudeHoldHover(float target_angle, int axis, PID& att_PID, PID& rate_PID);
float yawRateForSSHoldHover(float target_slip_angle, PID& ss_PID);
float pitchRateForNormalGsHold(float Gs);
float rollRateForHeadingHold(float target_heading, float max_bank_angle, float max_roll_rate, PID& heading_PID, PID& rate_PID);
float rollRateForBankAngleHold(float target_bank_angle, float max_roll_rate, PID& rate_PID);
float inputForMSLHold(float target_alt, float max_vert_vel, float max_norm_gs, PID& msl_PID, PID& vv_PID);
float inputForVVHold(float target_vert_v, float max_norm_gs, PID& vv_PID);

/////   AUTOPILOT   /////


void holdAirSpd(float target_spd);

void holdPositionVaryAttitude(Vec3 input, float throttle_handle, float max_VV = 10, Vec3 attitude_params = Vec3(30, 30, 90));

void holdSideSlip(float angle);

void fwdStabilityControl(Vec3 command_input);

void holdAoA(float angle);

void holdPitchRateFwd(float rate);

void printAutopilotData(float target_vel, float target_alt, float target_heading);