#include "XPlaneDataRefs.h"

void getDataRefs()
{
	GlobalVars::joystickThrottleAxis = XPLMFindDataRef("sim/joystick/joystick_axis_values");
	GlobalVars::throttle_ratio = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio");
	GlobalVars::alt_agl_handle = XPLMFindDataRef("sim/flightmodel/position/y_agl");
	GlobalVars::ground_speed = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
	GlobalVars::frame_time = XPLMFindDataRef("sim/operation/misc/frame_rate_period");
	GlobalVars::pitch = XPLMFindDataRef("sim/flightmodel/position/true_theta");
	GlobalVars::roll = XPLMFindDataRef("sim/flightmodel/position/true_phi");
	GlobalVars::psi = XPLMFindDataRef("sim/flightmodel/position/psi");
	GlobalVars::gear_nml_forces = XPLMFindDataRef("sim/flightmodel/forces/fnrml_gear");

	GlobalVars::thrust_vctr = XPLMFindDataRef("sim/flightmodel/controls/vectrqst");
	GlobalVars::spd_brk = XPLMFindDataRef("sim/flightmodel/controls/sbrkrqst");
	GlobalVars::yoke[0] = XPLMFindDataRef("sim/cockpit2/controls/yoke_pitch_ratio");
	GlobalVars::yoke[1] = XPLMFindDataRef("sim/cockpit2/controls/yoke_roll_ratio");
	GlobalVars::yoke[2] = XPLMFindDataRef("sim/cockpit2/controls/yoke_heading_ratio");
	GlobalVars::joy_buttons = XPLMFindDataRef("sim/joystick/joystick_button_values");

	GlobalVars::vx = XPLMFindDataRef("sim/flightmodel/position/local_vx");
	GlobalVars::vz = XPLMFindDataRef("sim/flightmodel/position/local_vz");
	GlobalVars::MSL_elevation = XPLMFindDataRef("sim/flightmodel/position/elevation");
	GlobalVars::override_pitch = XPLMFindDataRef("sim/operation/override/override_joystick_pitch");
	GlobalVars::override_roll = XPLMFindDataRef("sim/operation/override/override_joystick_roll");
	GlobalVars::override_joystick = XPLMFindDataRef("sim/operation/override/override_joystick");
	GlobalVars::ignition_key = XPLMFindDataRef("sim/cockpit2/engine/actuators/ignition_key");
	GlobalVars::ignition = XPLMFindDataRef("sim/cockpit/engine/ignition_on");
	GlobalVars::ignition2 = XPLMFindDataRef("sim/cockpit2/engine/actuators/ignition_on");
	GlobalVars::fuel_pump_on = XPLMFindDataRef("sim/cockpit/engine/fuel_pump_on");
	GlobalVars::acf_sidecant = XPLMFindDataRef("sim/aircraft/prop/acf_sidecant");
	GlobalVars::acf_vertcant = XPLMFindDataRef("sim/aircraft/prop/acf_vertcant");
	GlobalVars::vertical_velocity = XPLMFindDataRef("sim/flightmodel/position/vh_ind");
	GlobalVars::joystick_yaw_deadzone = XPLMFindDataRef("sim/joystick/joystick_heading_nullzone");
	GlobalVars::throttle_override = XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro");
	GlobalVars::engine_running = XPLMFindDataRef("sim/flightmodel/engine/ENGN_running");
	GlobalVars::engine_speed = XPLMFindDataRef("sim/cockpit2/engine/indicators/engine_speed_rpm");
	GlobalVars::engine_amps = XPLMFindDataRef("sim/flightmodel/engine/ENGN_bat_amps");
	GlobalVars::prop_thrust = XPLMFindDataRef("sim/flightmodel/engine/POINT_thrust");
	GlobalVars::collective = XPLMFindDataRef("sim/cockpit2/engine/actuators/prop_ratio");
	GlobalVars::total_mass = XPLMFindDataRef("sim/flightmodel/weight/m_total");
	GlobalVars::cg_x = XPLMFindDataRef("sim/flightmodel2/misc/cg_offset_x");
	GlobalVars::cg_z = XPLMFindDataRef("sim/flightmodel2/misc/cg_offset_z");
	GlobalVars::engine_positions[0] = XPLMFindDataRef("sim/flightmodel2/engines/location_x_mtr");
	GlobalVars::engine_positions[1] = XPLMFindDataRef("sim/flightmodel2/engines/location_y_mtr");
	GlobalVars::engine_positions[2] = XPLMFindDataRef("sim/flightmodel2/engines/location_z_mtr");

	GlobalVars::incoming_air_flow[0] = XPLMFindDataRef("sim/flightmodel/forces/vx_air_on_acf");
	GlobalVars::incoming_air_flow[2] = XPLMFindDataRef("sim/flightmodel/forces/vy_air_on_acf");
	GlobalVars::incoming_air_flow[1] = XPLMFindDataRef("sim/flightmodel/forces/vz_air_on_acf");

	GlobalVars::g_forces[0] = XPLMFindDataRef("sim/flightmodel/forces/g_axil");
	GlobalVars::g_forces[1] = XPLMFindDataRef("sim/flightmodel/forces/g_side");
	GlobalVars::g_forces[2] = XPLMFindDataRef("sim/flightmodel/forces/g_nrml");

	GlobalVars::moments[1] = XPLMFindDataRef("sim/aircraft/weight/acf_Jxx_unitmass");
	GlobalVars::moments[2] = XPLMFindDataRef("sim/aircraft/weight/acf_Jyy_unitmass");
	GlobalVars::moments[0] = XPLMFindDataRef("sim/aircraft/weight/acf_Jzz_unitmass");

	GlobalVars::control_surface_actuators[0] = XPLMFindDataRef("sim/cockpit2/controls/yoke_roll_ratio");
	GlobalVars::control_surface_actuators[1] = XPLMFindDataRef("sim/cockpit2/controls/yoke_pitch_ratio");
	GlobalVars::control_surface_actuators[2] = XPLMFindDataRef("sim/cockpit2/controls/yoke_heading_ratio");

	GlobalVars::motor_power = XPLMFindDataRef("sim/flightmodel/engine/ENGN_power");

	GlobalVars::joystick_axes = XPLMFindDataRef("sim/joystick/joystick_axis_values");

	GlobalVars::sim_paused = XPLMFindDataRef("sim/time/paused");
}