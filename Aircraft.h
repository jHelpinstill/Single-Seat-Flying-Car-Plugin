#pragma once

#include "Vec3.h"
#include "XPLMDataAccess.h"
#include "GlobalVars.h"
#include "Util.h"
#include "PID.h"

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
