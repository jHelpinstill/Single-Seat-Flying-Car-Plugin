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
	Global::vehicle.showProps();
	float fwd_throttle[2] = { 0, 0 };
	XPLMSetDatavf(Global::throttle_ratio, fwd_throttle, 0, 2);
	Global::vehicle.setControlSurfaces(Vec3::zero);

	Vec3 joystick_input = getJoystickRotValues(1.5);
	Vec3 force, torque;

	float throttle_position = applyDeadzone(getSignedJoystickThrottle(3), 0.005);

	if (getButton(Global::hover_mode2)) // rotate in place w/ no translate
	{
		force.z = VVIHold(throttle_position * 10);
		force = Global::vehicle.attitude_roll_pitch.inverse() * force;

		Vec3 rotation = joystick_input;
		rotation.x *= 30;
		rotation.y *= 30;
		rotation.z *= 90;
		torque = attitudeControlTorque(rotation);
	}
	else // hover_mode1: joystick controls translation with rotation following acceleration at low speeds, and following prograde at high speeds
	{
		
		Vec3 world_accel = (Global::vehicle.attitude_roll_pitch * Global::vehicle.accel);
		Vec3 air_vel = Global::vehicle.attitude_roll_pitch.inverse() * -Global::vehicle.airflow_rel; 
		Vec3 temp = air_vel; temp.z = 0;
		float air_speed_horiz = temp.mag();
		const float min_airspeed_for_high_speed_input = 10;
		const float speed_threshhold = 45;

		float t = rBound((air_speed_horiz - min_airspeed_for_high_speed_input) / (speed_threshhold - min_airspeed_for_high_speed_input), 0, 1);//applyDeadzone(air_speed_horiz, min_airspeed_for_high_speed_input, 1, speed_threshhold) / speed_threshhold;


		/// FORCES ///
		force.y = -joystick_input.x * Global::vehicle.mass * Global::g0;
		force.z = VVIHold(throttle_position * lerp(10.0, 20.0, t, 1));
		force.x = joystick_input.y * Global::vehicle.mass * Global::g0;

		const float pusher_ratio = 1.5;
		float pusher_force = lerp(0.0f, force.x / pusher_ratio, t, 1);
		if (pusher_force < 0) pusher_force = 0;
		force.x -= pusher_force;
		Global::vehicle.setFwdThrust(pusher_force);

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
		if (Global::vehicle.attitude_roll_pitch.eulerAngles().x > max_roll)
			bound(high_speed_roll, -roll_rate, 0);
		else if ((Global::vehicle.attitude_roll_pitch.eulerAngles().x < -max_roll))
			bound(high_speed_roll, 0, roll_rate);
		Vec3 proj = (Global::vehicle.airflow_rel).unit();
		float high_speed_pitch = (asin(proj.z) / Global::deg2rad);// -lerp(0, 5, t, 1);
		float side_slip = joystick_input.z * 10;

		float combined_roll_rate = lerp(rotHoldHoverRate(roll, 0), high_speed_roll, t, 1);
		float combined_pitch = lerp(pitch, high_speed_pitch, t, 1);
		float combined_yaw_rate = lerp(yaw_rate, sideSlipHoldHover(side_slip), t, 1);
		torque =	rateHoldHover(combined_roll_rate, 0) +
					attitudeHoldHover(combined_pitch, 1) +
					rateHoldHover(combined_yaw_rate, 2);
		holdSideSlip(side_slip);
		//holdRateFwd(0);
		//holdAoA(0);
		Global::vehicle.setControlSurfaces(Vec3::zero);
		Global::debug.println("high speed roll: ", high_speed_roll);
	}
	

	
	Global::debug.println("joystick input: ", joystick_input); Global::debug.println("torque applied: ", torque);

	Vec3 target_fan_vectors[3];
	///// Transform input forces and torques into left, right, and nose lift fan thrust vectors /////
	Global::vehicle.lift_fan_matrix.compute(force, torque, target_fan_vectors[0], target_fan_vectors[1], target_fan_vectors[2]);
	for (int i = 0; i < 3; i++)
		Global::vehicle.setMotorThrustDirection(target_fan_vectors[i], i + 2);
	Global::debug.println("CONTROL DEFLECTIONS: ", Global::vehicle.getControlSurfaces());
}

void doForward()
{
	float dt = Global::dt;
	Global::vehicle.hideProps(2000);
	float joy_throttle = getUnsignedJoystickThrottle(false, 1);
	Vec3 joystick_input = getJoystickRotValues();
	joystick_input.x = applyDeadzone(joystick_input.x, 0, 3);
	joystick_input.y = applyDeadzone(joystick_input.y, 0, 2);
	joystick_input.z = applyDeadzone(joystick_input.z, 0, 2);

	joy_throttle -= 0.1;
	if (joy_throttle < 0) joy_throttle *= 5;
	float thrust = joy_throttle * Global::vehicle.mass * Global::g0;
	
	Global::debug.println("thrust lbs per engine: ", (float)(thrust * 0.224809 / 2));

	static bool auto_pilot = false;
	static float target_vel = 0;
	static float target_alt = 0;
	static float target_heading = 0;
	const float meters2feet = 3.28084;
	if (Global::joy_thumb.pressed)
	{
		auto_pilot = !auto_pilot;
		target_vel = Global::vehicle.airflow_rel.mag();
		target_vel = floor(target_vel * 2.237) / 2.237;

		target_alt = XPLMGetDataf(Global::MSL_elevation) * meters2feet;
		target_alt = (int)target_alt - ((int)target_alt % 10);

		target_heading = (int)round(Global::vehicle.attitude.eulerAngles().z);
		//target_heading = (int)XPLMGetDataf(GlobalVars::psi);
	}
	if (joystick_input.mag() > 0.2)
	{
		auto_pilot = false;
	}
	if (auto_pilot)
	{
		static BinaryScroller hat(&Global::joy_up, &Global::joy_down, 0.3, 20);
		static BinaryScroller left_buttons(&Global::joy_5, &Global::joy_3, 0.3, 50);
		static BinaryScroller hat_left_right(&Global::joy_left, &Global::joy_right, 0.3, 20);

		hat.apply(target_vel, (float)(1 / 2.237));
		left_buttons.apply(target_alt, 10);
		hat_left_right.apply(target_heading, 1);

		if (target_heading > 180) target_heading -= 360;	// bound target heading to -180, 180
		else if (target_heading < -180) target_heading += 360;

		float heading = Global::vehicle.attitude.eulerAngles().z;
		Global::debug.println("");
		Global::debug.println("AUTO PILOT ON");
		Global::debug.println("Target speed mph: ", (float)(target_vel * 2.237));
		Global::debug.println("Target MSL: ", target_alt);
		Global::debug.println("Target Heading: ", headingCorrection(target_heading));
		Global::debug.println("Heading: ", headingCorrection(heading));
		Global::debug.println("Target Heading RAW: ", target_heading);
		Global::debug.println("Heading RAW: ", heading);
		Global::debug.println("psi: ", XPLMGetDataf(Global::psi));
		Global::debug.println();
		

		Vec3 auto_input(holdHeading(target_heading, 30, 10), holdMSL(target_alt, 80, 0.2), 0);
		holdAirSpd(target_vel);

		fwdStabilityControl(auto_input);
		Global::debug.println("autopilot command ratios: ", auto_input);
	}
	else
	{
		Global::vehicle.setFwdThrust(thrust);
		fwdStabilityControl(joystick_input);
		float t = (Global::vehicle.airflow_rel.mag() - 45) / (55 - 45);	// 0 at 45, 1 at 55
		Global::vehicle.mixControlSurfaces(joystick_input, lerp(0.8, 0.0, t, 1));

		if (Global::joy_3.held)
		{
			Global::vehicle.setControlSurfaces(joystick_input);
		}
	}

	Global::debug.println("CONTROL DEFLECTIONS: ", Global::vehicle.getControlSurfaces());
}

void doOnGround()
{
	float dt = Global::dt;
	Global::vehicle.hideProps(2000);
	float throttle[5] = { 0, 0, 0, 0, 0 };
	XPLMSetDatavf(Global::throttle_ratio, throttle, 0, 5);

	float throttle_position = applyDeadzone(getSignedJoystickThrottle(), 0.05);
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
	//static int prev_trigger_state = false;

	if (Global::vehicle.on_ground)
		flight_state = Flight_state::on_ground;

	else
	{
		if (flight_state == Flight_state::on_ground)
			flight_state = Flight_state::hover;
		//int trigger_state = getButton(GlobalVars::trigger);
		if (Global::joy_trigger.pressed)//getButton(GlobalVars::trigger) && !prev_trigger_state)
		{
			switch (flight_state)
			{
			case Flight_state::hover:
				flight_state = Flight_state::forward;
				Global::vehicle.cutHoverThrottles();
				break;
			case Flight_state::forward:
				flight_state = Flight_state::hover;
				break;
			}
		}//prev_trigger_state = trigger_state;
	}

	//if (Global::vehicle.airflow_rel.mag() > 100)
	//	flight_state = Flight_state::forward;
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
	Vec3 ground_speed = Global::vehicle.attitude * (Global::vehicle.airflow_rel * (3600 / 1609.0));
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
	Global::stack.reset(Global::l + 300, Global::t);

	if (XPLMGetDatai(Global::sim_paused))
	{
		Global::debug.println("PAUSED");
		return;
	}
	Global::debug.println("debug:");
	
	
	updateButtons();
	updateVehicleInfo();
	findFlightState(flight_state);
	Global::debug.println("vehicle attitude - world	: ", Global::vehicle.attitude.eulerAngles());
	Global::debug.println("vehicle rotation rate	: ", Global::vehicle.rot_rate);
	Global::debug.println("vehicle rotation accel	: ", Global::vehicle.rot_accel);
	Global::debug.println("Relative Air velocity	: ", Global::vehicle.airflow_rel);



	showButtonNumbers();
	//showJoystickAxes();
	static bool exception_thrown = false;
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

	PID* side_slip_PID = holdSideSlip(0.0, true);
	adjustPID(side_slip_PID);

	float power = printPower();
	printMPG(power);
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


