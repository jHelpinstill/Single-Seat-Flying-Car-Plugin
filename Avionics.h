#pragma once

#include "GlobalVars.h"
#include "Util.h"
#include "PID.h"
#include "Aircraft.h"

float HoldHoverVV(float commanded_VV);
Vec3 rateHoldHover(float target_rate, int axis);
float rotHoldHoverRate(float target_angle, int axis);
Vec3 attitudeHoldHover(float target_angle, int axis);
float sideSlipHoldHover(float target_slip_angle);
void holdAoA(float angle);
void holdPitchRateFwd(float rate);
float holdNormalGs(float Gs);
PID* holdSideSlip(float angle, bool return_PID_ptr = false);

Vec3 attitudeControlTorque(Vec3 control_in);
void fwdStabilityControl(Vec3 command_input);

/////   AUTOPILOT   /////
float holdHeading(float target_heading, float max_bank_angle, float max_roll_rate);

float holdBankAngle(float target_bank_angle, float max_roll_rate);

float holdMSL(float target_alt, float max_vert_vel, float max_norm_gs);

float holdVertVel(float target_vert_v, float max_norm_gs);

void holdAirSpd(float target_spd);
