#include "XPlaneDataRefs.h"

void getDataRefs()
{
	Global::joystickThrottleAxis = XPLMFindDataRef("sim/joystick/joystick_axis_values");
	Global::throttle_ratio = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio");
	Global::alt_agl_handle = XPLMFindDataRef("sim/flightmodel/position/y_agl");
	Global::ground_speed = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
	Global::frame_time = XPLMFindDataRef("sim/operation/misc/frame_rate_period");
	Global::pitch = XPLMFindDataRef("sim/flightmodel/position/true_theta");
	Global::roll = XPLMFindDataRef("sim/flightmodel/position/true_phi");
	Global::psi = XPLMFindDataRef("sim/flightmodel/position/psi");
	Global::gear_nml_forces = XPLMFindDataRef("sim/flightmodel/forces/fnrml_gear");

	Global::thrust_vctr = XPLMFindDataRef("sim/flightmodel/controls/vectrqst");
	Global::spd_brk = XPLMFindDataRef("sim/flightmodel/controls/sbrkrqst");
	Global::yoke[0] = XPLMFindDataRef("sim/cockpit2/controls/yoke_pitch_ratio");
	Global::yoke[1] = XPLMFindDataRef("sim/cockpit2/controls/yoke_roll_ratio");
	Global::yoke[2] = XPLMFindDataRef("sim/cockpit2/controls/yoke_heading_ratio");
	Global::joy_buttons = XPLMFindDataRef("sim/joystick/joystick_button_values");

	Global::vx = XPLMFindDataRef("sim/flightmodel/position/local_vx");
	Global::vz = XPLMFindDataRef("sim/flightmodel/position/local_vz");
	Global::MSL_elevation = XPLMFindDataRef("sim/flightmodel/position/elevation");
	Global::override_pitch = XPLMFindDataRef("sim/operation/override/override_joystick_pitch");
	Global::override_roll = XPLMFindDataRef("sim/operation/override/override_joystick_roll");
	Global::override_joystick = XPLMFindDataRef("sim/operation/override/override_joystick");
	Global::ignition_key = XPLMFindDataRef("sim/cockpit2/engine/actuators/ignition_key");
	Global::ignition = XPLMFindDataRef("sim/cockpit/engine/ignition_on");
	Global::ignition2 = XPLMFindDataRef("sim/cockpit2/engine/actuators/ignition_on");
	Global::fuel_pump_on = XPLMFindDataRef("sim/cockpit/engine/fuel_pump_on");
	Global::acf_sidecant = XPLMFindDataRef("sim/aircraft/prop/acf_sidecant");
	Global::acf_vertcant = XPLMFindDataRef("sim/aircraft/prop/acf_vertcant");
	Global::vertical_velocity = XPLMFindDataRef("sim/flightmodel/position/vh_ind");
	Global::joystick_yaw_deadzone = XPLMFindDataRef("sim/joystick/joystick_heading_nullzone");
	Global::throttle_override = XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro");
	Global::engine_running = XPLMFindDataRef("sim/flightmodel/engine/ENGN_running");
	Global::engine_speed = XPLMFindDataRef("sim/cockpit2/engine/indicators/engine_speed_rpm");
	Global::engine_amps = XPLMFindDataRef("sim/flightmodel/engine/ENGN_bat_amps");
	Global::prop_thrust = XPLMFindDataRef("sim/flightmodel/engine/POINT_thrust");
	Global::collective = XPLMFindDataRef("sim/cockpit2/engine/actuators/prop_ratio");
	Global::total_mass = XPLMFindDataRef("sim/flightmodel/weight/m_total");
	Global::cg_x = XPLMFindDataRef("sim/flightmodel2/misc/cg_offset_x");
	Global::cg_z = XPLMFindDataRef("sim/flightmodel2/misc/cg_offset_z");
	Global::engine_positions[0] = XPLMFindDataRef("sim/flightmodel2/engines/location_x_mtr");
	Global::engine_positions[1] = XPLMFindDataRef("sim/flightmodel2/engines/location_y_mtr");
	Global::engine_positions[2] = XPLMFindDataRef("sim/flightmodel2/engines/location_z_mtr");

	Global::air_relative_velocity[0] = XPLMFindDataRef("sim/flightmodel/forces/vx_air_on_acf");
	Global::air_relative_velocity[1] = XPLMFindDataRef("sim/flightmodel/forces/vy_air_on_acf");
	Global::air_relative_velocity[2] = XPLMFindDataRef("sim/flightmodel/forces/vz_air_on_acf");

	Global::g_forces[0] = XPLMFindDataRef("sim/flightmodel/forces/g_axil");
	Global::g_forces[1] = XPLMFindDataRef("sim/flightmodel/forces/g_side");
	Global::g_forces[2] = XPLMFindDataRef("sim/flightmodel/forces/g_nrml");

	Global::moments[1] = XPLMFindDataRef("sim/aircraft/weight/acf_Jxx_unitmass");
	Global::moments[2] = XPLMFindDataRef("sim/aircraft/weight/acf_Jyy_unitmass");
	Global::moments[0] = XPLMFindDataRef("sim/aircraft/weight/acf_Jzz_unitmass");

	Global::control_surface_actuators[0] = XPLMFindDataRef("sim/cockpit2/controls/yoke_roll_ratio");
	Global::control_surface_actuators[1] = XPLMFindDataRef("sim/cockpit2/controls/yoke_pitch_ratio");
	Global::control_surface_actuators[2] = XPLMFindDataRef("sim/cockpit2/controls/yoke_heading_ratio");

	Global::motor_power = XPLMFindDataRef("sim/flightmodel/engine/ENGN_power");

	Global::joystick_axes = XPLMFindDataRef("sim/joystick/joystick_axis_values");

	Global::sim_paused = XPLMFindDataRef("sim/time/paused");
}