// Downloaded from https://developer.x-plane.com/code-sample/hello-world-sdk-3/
#include <cmath>
#include <string.h>
#include <string>
#include <cstdlib>

///// GLOBAL VARIABLES NAMESPACE /////
#include "GlobalVars.h"

///// CLASSES /////
#include "TextBox.h"
#include "Vec3.h"
#include "PID.h"
#include "ControlMatrix.h"
#include "Matrix.h"
#include "Quat.h"
#include "RollingAvg.h"
#include "BinaryScroller.h"
#include "Aircraft.h"

///// FUNCTIONS /////
#include "XPlaneDataRefs.h"
#include "Joystick.h"
#include "Util.h"
#include "Avionics.h"
#include "Aircraft.h"
#include "PluginSetup.h"


void doHover()
{
	float dt = Global::dt;
	aircraft.showProps();
	aircraft.cutForwardThrottles();
	aircraft.setControlSurfaces(Vec3::zero);

	Vec3 joystick_input = joystick.getFilteredAxes(Vec3(0.05, 0.05, 0.3), Vec3(1.5, 1.5, 1.5));
	float throttle_position = joystick.getSignedThrottle(0.005, 3);
	
	if (joystick.button_3.held) // rotate in place w/ no translate
	{
		holdPositionVaryAttitude(joystick_input, throttle_position);
	}
	else // hover_mode1: joystick controls translation with rotation following acceleration at low speeds, and following prograde at high speeds
	{
		Vec3 force, torque;

		Vec3 world_accel = aircraft.attitude_roll_pitch * aircraft.accel;
		Vec3 air_vel = aircraft.attitude_roll_pitch.inverse() * -aircraft.airflow_rel; 
		Vec3 temp = air_vel; temp.z = 0;
		float air_speed_horiz = temp.mag();
		const float min_airspeed_for_high_speed_input = 10;
		const float speed_threshhold = 45;

		float t = rBound((air_speed_horiz - min_airspeed_for_high_speed_input) / (speed_threshhold - min_airspeed_for_high_speed_input), 0, 1);


		/// FORCES ///
		static PID vv_PID(2, 0.5, 0.02, 0.1);
		force.y = -joystick_input.x * aircraft.mass * Global::g0;
		force.z = thrustForVVHoldHover(throttle_position * lerp(10.0, 20.0, t, 1), vv_PID);
		force.x = joystick_input.y * aircraft.mass * Global::g0;

		const float pusher_ratio = 1.5;
		float pusher_force = lerp(0.0f, force.x / pusher_ratio, t, 1);
		if (pusher_force < 0) pusher_force = 0;
		force.x -= pusher_force;
		aircraft.setForwardThrust(pusher_force);

		Global::debug.println("pusher thrust: ", pusher_force);


		
		/// LOW SPEED TORQUES ///	
		const float degrees_per_g = 10;
		const float max_angle = 10;
		float roll = rBound(-world_accel.y * degrees_per_g / Global::g0, -max_angle, max_angle);
		float pitch = rBound(world_accel.x * degrees_per_g / Global::g0, -max_angle, max_angle);
		float yaw_rate = joystick_input.z * 90;

		/// HIGH SPEED TORQUES ///
		const float roll_rate = 120;
		const float max_roll = 45;
		float high_speed_roll = joystick_input.x * roll_rate;
		if (aircraft.attitude_roll_pitch.eulerAngles().x > max_roll)
			bound(high_speed_roll, -roll_rate, 0);
		else if ((aircraft.attitude_roll_pitch.eulerAngles().x < -max_roll))
			bound(high_speed_roll, 0, roll_rate);
		Vec3 proj = (aircraft.airflow_rel).unit();
		float high_speed_pitch = (asin(proj.z) / Global::deg2rad);// -lerp(0, 5, t, 1);
		float side_slip = joystick_input.z * 10;

		/// COMBINING ///
		static PID ss_PID(5, 0.05, 0);
		static PID att_PIDs[2] =
		{
			PID(4, 0.05, 0, 0, 0.02, 0.15),
			PID(4, 0.05, 0, 0, 0.05, 0.15)
		};
		static PID rate_PIDs[3] =
		{
			PID(0.1, 0.002, 0.01, 10, 0.02, 0.15),
			PID(0.1, 0.002, 0.01, 10, 0.05, 0.15),
			PID(0.2, 0.002, 0.01, 10)
		};
		float combined_roll_rate = lerp(rotHoldHoverRate(roll, 0), high_speed_roll, t);
		float combined_pitch = lerp(pitch, high_speed_pitch, t, 1);
		float combined_yaw_rate = lerp(yaw_rate, yawRateForSSHoldHover(side_slip, ss_PID), t, 1);
		torque =	torqueForAttitudeHoldHover(roll, 0, att_PIDs[0], rate_PIDs[0]) +
					torqueForAttitudeHoldHover(combined_pitch, 1, att_PIDs[1], rate_PIDs[1]) +
					torqueForRateHoldHover(combined_yaw_rate, 2, rate_PIDs[2]);
		holdSideSlip(side_slip);

		aircraft.setControlSurfaces(Vec3::zero);
		Global::debug.println("high speed roll: ", high_speed_roll);

		aircraft.addHoverTorque(torque);
		aircraft.addHoverForce(force);
		Global::debug.println("forces : ", force);
	}
	
	//Global::debug.println("joystick input: ", joystick_input); Global::debug.println("torque applied: ", torque);
	//Global::debug.println("CONTROL DEFLECTIONS: ", aircraft.getControlSurfaces());
}

void doForward()
{
	float dt = Global::dt;
	aircraft.hideProps(2000);

	float joy_throttle = joystick.getUnsignedThrottle(0, 1, false);
	Vec3 joystick_input = joystick.getFilteredAxes(Vec3(0.05, 0.05, 0.3), Vec3(2, 2, 2));

	joy_throttle -= 0.1;
	if (joy_throttle < 0) joy_throttle *= 5;
	float thrust = joy_throttle * aircraft.mass * Global::g0;
	
	Global::debug.println("thrust lbs per engine: ", (float)(thrust * 0.224809 / 2));

	static bool auto_pilot = false;
	static float target_vel = 0;
	static float target_alt = 0;
	static float target_heading = 0;
	const float meters2feet = 3.28084;
	if (joystick.button_thumb.pressed)
	{
		auto_pilot = !auto_pilot;
		target_vel = aircraft.airflow_rel.mag();
		target_vel = floor(target_vel * 2.237) / 2.237;

		target_alt = XPLMGetDataf(Global::MSL_elevation) * meters2feet;
		target_alt = (int)target_alt - ((int)target_alt % 10);

		target_heading = (int)round(aircraft.attitude.eulerAngles().z);
		//target_heading = (int)XPLMGetDataf(GlobalVars::psi);
	}
	if (joystick_input.mag() > 0.2)
	{
		auto_pilot = false;
	}
	if (auto_pilot)
	{
		static BinaryScroller hat(&joystick.button_up, &joystick.button_down, 0.3, 20);
		static BinaryScroller left_buttons(&joystick.button_5, &joystick.button_3, 0.3, 50);
		static BinaryScroller hat_left_right(&joystick.button_left, &joystick.button_right, 0.3, 20);

		hat.apply(target_vel, (float)(1 / 2.237));
		left_buttons.apply(target_alt, 10);
		hat_left_right.apply(target_heading, 1);

		if (target_heading > 180) target_heading -= 360;	// bound target heading to -180, 180
		else if (target_heading < -180) target_heading += 360;

		static PID heading_PID(5, 0.0, 0.0);	static PID roll_PID(0.05, 0.002, 0.01, 1);
		static PID alt_PID(0.5, 0.1, 0.0);		static PID vv_PID(0.025, 0.005, 0.01, 1);
		Vec3 auto_input(rollRateForHeadingHold(target_heading, 30, 10, heading_PID, roll_PID), inputForMSLHold(target_alt, 80, 0.2, alt_PID, vv_PID), 0);
		fwdStabilityControl(auto_input);
		holdAirSpd(target_vel);

		printAutopilotData(target_vel, target_alt, target_heading);
		Global::debug.println("autopilot command ratios: ", auto_input);
	}
	else
	{
		aircraft.setForwardThrust(thrust);
		fwdStabilityControl(joystick_input);
		float t = (aircraft.airflow_rel.mag() - 45) / (55 - 45);	// 0 at 45, 1 at 55
		aircraft.mixControlSurfaces(joystick_input, lerp(0.8, 0.0, t, 1));

		if (joystick.button_3.held)
		{
			aircraft.setControlSurfaces(joystick_input);
		}
	}

	Global::debug.println("CONTROL DEFLECTIONS: ", aircraft.getControlSurfaces());
}

void doOnGround()
{
	float dt = Global::dt;
	aircraft.hideProps(2000);
	aircraft.cutForwardThrottles();
	aircraft.cutHoverThrottles();

	float throttle_position = joystick.getSignedThrottle(0.05, 1);
	if (throttle_position > 0)
		doHover();
}

enum class Flight_state
{
	hover,
	forward,
	on_ground
};
Flight_state flight_state = Flight_state::hover;

void findFlightState(Flight_state &flight_state)
{
	if (aircraft.on_ground)
		flight_state = Flight_state::on_ground;

	else
	{
		if (flight_state == Flight_state::on_ground)
			flight_state = Flight_state::hover;

		if (joystick.button_trigger.pressed)
		{
			switch (flight_state)
			{
			case Flight_state::hover:
				flight_state = Flight_state::forward;
				aircraft.cutHoverThrottles();
				break;
			case Flight_state::forward:
				flight_state = Flight_state::hover;
				break;
			}
		}
	}
}

float printPower()
{
	static RollingAvg fwd_power(100);
	float fwd_motor_power[2];
	XPLMGetDatavf(Global::motor_power, fwd_motor_power, 0, 2);
	//GlobalVars::debug.println("motor power", fwd_motor_power[0]);
	float power = fwd_motor_power[0] + fwd_motor_power[1];
	float horse_power = power / 746;
	fwd_power.apply(horse_power);
	Global::debug.println("POWER (hp): ", horse_power);

	return power;
}

void printMPG(float power)
{
	Vec3 ground_speed = aircraft.attitude * (aircraft.airflow_rel * (3600 / 1609.0));
	Global::debug.println(ground_speed);
	ground_speed.z = 0;

	float mph = ground_speed.mag();
	float gph = (power * 3600.0) / (43500000.0 * 0.4535 * 0.35 * 6.7);
	float mpg = mph / gph;
	Global::debug.println("mph: ", mph);
	Global::debug.println("gph: ", gph);
	Global::debug.println("Estimated MPG: ", mpg);
}

void aircraftMAIN()
{
	Global::debug.reset(Global::l, Global::t);

	if (XPLMGetDatai(Global::sim_paused))
	{
		Global::debug.println("PAUSED");
		return;
	}
	Global::debug.println("debug:");

	Global::dt = XPLMGetDataf(Global::frame_time);
	joystick.update();
	aircraft.update();
	findFlightState(flight_state);

	Global::debug.println("vehicle attitude - world	: ", aircraft.attitude.eulerAngles());
	Global::debug.println("vehicle rotation rate	: ", aircraft.rot_rate);
	Global::debug.println("vehicle rotation accel	: ", aircraft.rot_accel);
	Global::debug.println("Relative Air velocity	: ", aircraft.airflow_rel);
	Global::debug.println("Joystick raw axes		: ", joystick.getRawAxes());
	Global::debug.println("Joystick filtered axes	: ", joystick.getFilteredAxes(Vec3(0.05, 0.05, 0.3), Vec3(1.5, 1.5, 1.5)));

	switch (flight_state)
	{
	case Flight_state::hover:
		Global::debug.println("Flight state: hover");
		doHover();
		break;
	case Flight_state::forward:
		Global::debug.println("Flight state: forward");
		doForward();
		break;
	case Flight_state::on_ground:
		Global::debug.println("Flight state: on_ground");
		doOnGround();
		break;
	}

	float power = printPower();
	printMPG(power);
	
	aircraft.updateMotors();
	Global::debug.reset();
}

bool plugin_setup_finished = false;
void draw_hello_world(XPLMWindowID in_window_id, void* in_refcon)
{
	if (!plugin_setup_finished)
	{
		plugin_setup_finished = true;
		pluginSetup();
	}
	// Mandatory: We *must* set the OpenGL state before drawing
	// (we can't make any assumptions about it)
	XPLMSetGraphicsState(
		0 /* no fog */,
		0 /* 0 texture units */,
		0 /* no lighting */,
		0 /* no alpha testing */,
		1 /* do alpha blend */,
		1 /* do depth testing */,
		0 /* no depth writing */
	);
	XPLMGetWindowGeometry(in_window_id, &Global::l, &Global::t, &Global::r, &Global::b);
	
	aircraftMAIN();
}


