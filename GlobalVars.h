#pragma once

#include "XPLMDataAccess.h"
#include "Vec3.h"
#include "ControlMatrix.h"
#include "Matrix.h"
#include "Quat.h"
#include "TextBox.h"
#include "Joystick.h"
#include "Aircraft.h"
#include <vector>

namespace Global
{
	extern XPLMDataRef throttle_ratio, joystickThrottleAxis, alt_agl_handle, ground_speed,
		frame_time, pitch, roll, psi, gear_nml_forces, thrust_vctr, spd_brk,
		yoke[3], vx, vz, MSL_elevation, override_joystick, override_pitch, ignition_key, ignition, ignition2, fuel_pump_on,
		override_roll, acf_sidecant, acf_vertcant, vertical_velocity, joystick_yaw_deadzone,
		throttle_override, engine_running, engine_speed, engine_amps, prop_thrust, collective, total_mass, cg_x, cg_z,
		engine_positions[3], moments[3], joy_buttons, air_relative_velocity[3], g_forces[3], control_surface_actuators[3],
		joystick_axes, sim_paused, motor_power, atm_density_ratio;

	extern const float deg2rad;// = 3.1415926535 / 180;
	extern const float g0;// = 9.80665;

	extern TextBox debug;
	extern int l, t, r, b;
	extern float font_color[3];// = { 1.0, 1.0, 1.0 }; // red, green, blue

	extern float dt;
}
