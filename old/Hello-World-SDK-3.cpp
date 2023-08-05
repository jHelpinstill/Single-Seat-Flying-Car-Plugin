// Downloaded from https://developer.x-plane.com/code-sample/hello-world-sdk-3/

#include <cmath>
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMDataAccess.h"
#include <string.h>
#if IBM
	#include <windows.h>
#endif
#if LIN
	#include <GL/gl.h>
#elif __GNUC__
	#include <OpenGL/gl.h>
#else
	#include <GL/gl.h>
#endif

#ifndef XPLM300
	#error This is made to be compiled against the XPLM300 SDK
#endif

// An opaque handle to the window we will create
static XPLMWindowID	g_window;

static XPLMDataRef throttle_ratio, joystickThrottleAxis, alt_agl_handle, ground_speed,
                   frame_time, pitch, roll, psi, gear_nml_forces, thrust_vctr, spd_brk,
	               yoke[2], vx, vz, MSL_elevation, override_pitch,
	               override_roll;

float font_color[] = { 1.0, 1.0, 1.0 }; // red, green, blue

double mouseX = 0;
double mouseY = 0;

float PID[2][3] = { { 0.35f, 0.04f, 0.15f }, {0.05f, 0.02f, 0.02f} };
float bias = 0.32;
int joystick_index = 0;


// Callbacks we will register when we create our window
void				draw_hello_world(XPLMWindowID in_window_id, void * in_refcon);
int	dummy_mouse_handler(XPLMWindowID in_window_id, int x, int y, int is_down, void * in_refcon) { 
	static bool toggle = false;
	toggle = !toggle;
	if (toggle) {
		font_color[0] = 0;
		font_color[1] = 0;
		font_color[2] = 0;
	}
	else {
		font_color[0] = 1;
		font_color[1] = 1;
		font_color[2] = 1;
	}

	mouseX = x;
	mouseY = y;
	PID[0][1] = 0;
	return 0; 
}
XPLMCursorStatus	dummy_cursor_status_handler(XPLMWindowID in_window_id, int x, int y, void * in_refcon) { return xplm_CursorDefault; }
int	dummy_wheel_handler(XPLMWindowID in_window_id, int x, int y, int wheel, int clicks, void* in_refcon) {
	int l, t, r, b;
	XPLMGetWindowGeometry(in_window_id, &l, &t, &r, &b);

	for (int i = 0; i < 3; i++) {
		if (t - y > 80 - 20 * i) {
			PID[0][2 - i] += 0.01f * clicks;
			break;
		}
	}
	if (t - y < 40)
		bias += 0.01f * clicks;

	return 1; 
}
void				dummy_key_handler(XPLMWindowID in_window_id, char key, XPLMKeyFlags flags, char virtual_key, void * in_refcon, int losing_focus) { }


PLUGIN_API int XPluginStart(
							char *		outName,
							char *		outSig,
							char *		outDesc)
{
	strcpy(outName, "HelloWorld3Plugin");
	strcpy(outSig, "xpsdk.examples.helloworld3plugin");
	strcpy(outDesc, "A Hello World plug-in for the XPLM300 SDK.");
	
	XPLMCreateWindow_t params;
	params.structSize = sizeof(params);
	params.visible = 1;
	params.drawWindowFunc = draw_hello_world;
	// Note on "dummy" handlers:
	// Even if we don't want to handle these events, we have to register a "do-nothing" callback for them
	params.handleMouseClickFunc = dummy_mouse_handler;
	params.handleRightClickFunc = dummy_mouse_handler;
	params.handleMouseWheelFunc = dummy_wheel_handler;
	params.handleKeyFunc = dummy_key_handler;
	params.handleCursorFunc = dummy_cursor_status_handler;
	params.refcon = NULL;
	params.layer = xplm_WindowLayerFloatingWindows;
	// Opt-in to styling our window like an X-Plane 11 native window
	// If you're on XPLM300, not XPLM301, swap this enum for the literal value 1.
	params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
	
	// Set the window's initial bounds
	// Note that we're not guaranteed that the main monitor's lower left is at (0, 0)...
	// We'll need to query for the global desktop bounds!
	int left, bottom, right, top;
	XPLMGetScreenBoundsGlobal(&left, &top, &right, &bottom);
	params.left = left + 50;
	params.bottom = bottom + 150;
	params.right = params.left + 200;
	params.top = params.bottom + 200;
	
	g_window = XPLMCreateWindowEx(&params);

	//output_throttle = XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use");
	//input_throttle = XPLMFindDataRef("sim/flightmodel2/engines/throttle_used_ratio");
	//joystick_axis_assignments = XPLMFindDataRef("sim/joystick/joystick_axis_assignments");
	//joy_axis_avail = XPLMFindDataRef("sim/joystick/joy_mapped_axis_avail");
	//override_throttles = XPLMFindDataRef("sim/operation/override/override_throttles");
	
	joystickThrottleAxis = XPLMFindDataRef("sim/joystick/joystick_axis_values");
	throttle_ratio = XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio");
	alt_agl_handle = XPLMFindDataRef("sim/flightmodel/position/y_agl");
	ground_speed = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
	frame_time = XPLMFindDataRef("sim/operation/misc/frame_rate_period");
	pitch = XPLMFindDataRef("sim/flightmodel/position/true_theta");
	roll = XPLMFindDataRef("sim/flightmodel/position/true_phi");
	psi = XPLMFindDataRef("sim/flightmodel/position/psi");
	gear_nml_forces = XPLMFindDataRef("sim/flightmodel/forces/fnrml_gear");
	thrust_vctr = XPLMFindDataRef("sim/flightmodel/controls/vectrqst");
	spd_brk = XPLMFindDataRef("sim/flightmodel/controls/sbrkrqst");
	yoke[0] = XPLMFindDataRef("sim/cockpit2/controls/yoke_pitch_ratio");
	yoke[1] = XPLMFindDataRef("sim/cockpit2/controls/yoke_roll_ratio");
	vx = XPLMFindDataRef("sim/flightmodel/position/local_vx");
	vz = XPLMFindDataRef("sim/flightmodel/position/local_vz");
	MSL_elevation = XPLMFindDataRef("sim/flightmodel/position/elevation");
	override_pitch = XPLMFindDataRef("sim/operation/override/override_joystick_pitch");
	override_roll = XPLMFindDataRef("sim/operation/override/override_joystick_roll");

	//XPLMSetDatai(override_pitch, 1);
	//XPLMSetDatai(override_roll, 1);


	
	// Position the window as a "free" floating window, which the user can drag around
	XPLMSetWindowPositioningMode(g_window, xplm_WindowPositionFree, -1);
	// Limit resizing our window: maintain a minimum width/height of 100 boxels and a max width/height of 300 boxels
	XPLMSetWindowResizingLimits(g_window, 200, 200, 300, 300);
	XPLMSetWindowTitle(g_window, "Sample Window");
	
	return g_window != NULL;
}

PLUGIN_API void	XPluginStop(void)
{
	// Since we created the window, we'll be good citizens and clean it up
	XPLMDestroyWindow(g_window);
	g_window = NULL;
	XPLMSetDatai(override_pitch, 0);
	XPLMSetDatai(override_roll, 0);
}

PLUGIN_API void XPluginDisable(void) { }
PLUGIN_API int  XPluginEnable(void)  { return 1; }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam) { }


void local_vels(float& t_vel, float& n_vel) {
	float x_vel = XPLMGetDataf(vx);
	float z_vel = XPLMGetDataf(vz);

	float heading = XPLMGetDataf(psi);
	float delta_time = XPLMGetDataf(frame_time);

	float angle = (360.0 - static_cast<double>(heading)) * 3.14159 / 180;
	t_vel = (z_vel * cos(angle) + x_vel * sin(angle));
	n_vel = -(z_vel * sin(angle) - x_vel * cos(angle));
}

float get_lever_pos(int lever_axis_index) {
	float throttle_lever[1];

	XPLMGetDatavf(joystickThrottleAxis, throttle_lever, lever_axis_index, 1); //get position of throttle lever on joystick
	return 1 - throttle_lever[0];
}

float get_axis_pos(int axis_index) {
	float lever[1];

	XPLMGetDatavf(joystickThrottleAxis, lever, axis_index, 1);
	return lever[0];
}

void throttle_handler(float hover_throttle, float fwd_throttle) {
	float commanded_throttle[6];

	commanded_throttle[0] = fwd_throttle;
	commanded_throttle[1] = fwd_throttle;

	for (int i = 2; i < 6; i++)
		commanded_throttle[i] = hover_throttle;

	XPLMSetDatavf(throttle_ratio, commanded_throttle, 0, 6);
}

float calc_hover_throttle(float lever_pos, float ground_spd, float& rate_out, float& des_rate_out, float* errors) {
	static float prev_alt{};
	static float prev_error{};
	static double error_accumulator{};

	//if (XPLMGetDataf(ground_speed) < 60) {
		float alt = XPLMGetDataf(MSL_elevation);
		float agl = XPLMGetDataf(alt_agl_handle);
		float max_rate = 10;
		if ( agl > 30) {
			max_rate += (agl - 30) / 10;
		}

		if (XPLMGetDataf(thrust_vctr) == 0)
			XPLMSetDataf(thrust_vctr, 0.033);
		float theta = XPLMGetDataf(pitch);
		float phi = XPLMGetDataf(roll);

		float delta_time = XPLMGetDataf(frame_time);
		if (delta_time == 0)
			return 0;

		float mapped_pos;
		if (lever_pos >= 0.45 && lever_pos <= 0.55) {
			mapped_pos = 0.5;
		}
		else if (lever_pos > 0.55)
			mapped_pos = lever_pos - 0.05;
		else
			mapped_pos = lever_pos + 0.05;

		mapped_pos = (mapped_pos * 2 - 1);
		if (mapped_pos > 0)
			mapped_pos = pow(mapped_pos, 1.5);
		else
			mapped_pos = -pow(-mapped_pos, 1.5);

		float rate = (alt - prev_alt) / delta_time;
		prev_alt = alt;
		
		float desired_rate = max_rate * mapped_pos;

		rate_out = rate;
		des_rate_out = desired_rate;

		double error = desired_rate - rate;
		double error_rate = (error - prev_error) / delta_time;
		prev_error = error;
		error_accumulator += error * delta_time;

		if (error_accumulator > 20 || error_accumulator < -20) {
			if (error_accumulator > 20)
				error_accumulator = 20;
			if (error_accumulator < -20)
				error_accumulator = -20;
		}

		if (XPLMGetDataf(gear_nml_forces) > 0.0f) {
			error_accumulator = 0;
		}

		float throttle = bias + error * PID[0][0] + error_accumulator * PID[0][1] + error_rate * PID[0][2];

		errors[0] = error;
		errors[1] = error_accumulator;
		errors[2] = error_rate;

		double cosine_compensation = 1 / cos(theta * 3.14159 / 180) / cos(phi * 3.14159 / 180);


		if (throttle >= 0 && throttle <= 1)
			return throttle * cosine_compensation;
		else if (throttle < 0)
			return 0;
		else
			return 1;
	//}
	if (XPLMGetDataf(thrust_vctr))
		XPLMSetDataf(thrust_vctr, 0);
	return 0;
}

float calc_fwd_throttle(float lever_pos, float ground_spd) {
	return lever_pos;

	float alt = XPLMGetDataf(alt_agl_handle);
	float speed = XPLMGetDataf(ground_speed);

	if (alt > 35) {
		if (speed > 50) {
			float brake = 1 - 2.66667 * lever_pos;
			if (brake < 0)
				brake = 0;
			XPLMSetDataf(spd_brk, brake);
			if(lever_pos > 0.375)
				return 1.6 * lever_pos - 0.6;
			return 0;
		}
		float vect = (1 - XPLMGetDataf(thrust_vctr));
		if(vect > 0.5)
			return 2*(vect - 0.5);
	}
	return 0.0f;
}

void draw_PID(int l, int t, int r, int b, float* data, int offset) {
	float col_white[] = { 1.0, 1.0, 1.0 };
	XPLMDrawString(col_white, l + 10, t - 58, "P", NULL, xplmFont_Proportional);
	XPLMDrawString(col_white, l + 10, t - 78, "I", NULL, xplmFont_Proportional);
	XPLMDrawString(col_white, l + 10, t - 98, "D", NULL, xplmFont_Proportional);

	for(int i = 0; i < 3; i++)
		XPLMDrawNumber(col_white, l + offset, t - 60 - 20*i, data[i], 2, 2, 1, xplmFont_Proportional);

}

void yoke_handler(float* errors_pitch, float* errors_roll) {
	
	XPLMSetDatai(override_pitch, 0);
	XPLMSetDatai(override_roll, 0);
	static float prev_error[2] = { 0,0 };
	static float accumulator[2] = { 0,0 };

	if (XPLMGetDataf(gear_nml_forces) > 0.0f) {
		accumulator[0] = 0;
		accumulator[1] = 0;
	}
	
	
	if (XPLMGetDataf(alt_agl_handle) < 10) {
		XPLMSetDatai(override_pitch, 1);
		XPLMSetDatai(override_roll, 1);
		float delta_time = XPLMGetDataf(frame_time);

		float vel[2];
		local_vels(vel[0], vel[1]);

		for (int axis = 0; axis < 2; axis++) {
			float lever_pos = (2 * get_axis_pos(25 + axis) - 1);
			float desired = lever_pos;
			if (desired > 0)
				desired = pow(desired, 2);
			else
				desired = -pow(-desired, 2);
			desired *= 5;

			float error = desired - vel[axis];
			float error_rate = (error - prev_error[axis]) / delta_time;
			accumulator[axis] += error * delta_time;

			float result = error * PID[1][0] + accumulator[axis] * PID[1][1] + error_rate * PID[1][2];
			if (result > 1)
				result = 1;
			else if (result < -1)
				result = -1;

			result = 0.5 * (static_cast<double>(result) + lever_pos);
			
			
			XPLMSetDataf(yoke[axis], result);




			if (axis) {
				errors_roll[0] = error;
				errors_roll[1] = accumulator[axis];
				errors_roll[2] = error_rate;
			}
			else {
				errors_pitch[0] = error;
				errors_pitch[1] = accumulator[axis];
				errors_pitch[2] = error_rate;
			}
		}
	}
}

void	draw_hello_world(XPLMWindowID in_window_id, void * in_refcon)
{
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
	
	int l, t, r, b;
	XPLMGetWindowGeometry(in_window_id, &l, &t, &r, &b);
	
	float col_white[] = {1.0, 1.0, 1.0}; // red, green, blue

	float agl = XPLMGetDataf(alt_agl_handle);
	float ground_spd = XPLMGetDataf(ground_speed);

	float rate;
	float desired_rate;
	float errors[3];
	
	float lever_pos = get_lever_pos(28);
	float hover_throttle = calc_hover_throttle(lever_pos, ground_spd, rate, desired_rate, errors);
	float fwd_throttle = calc_fwd_throttle(lever_pos, ground_spd);
	
	throttle_handler(hover_throttle, fwd_throttle);

	float gear_forces = XPLMGetDataf(gear_nml_forces);

	float t_vel, n_vel;
	local_vels(t_vel, n_vel);

	XPLMDrawString(font_color, l + 10, t - 20, "Change Test: 24", NULL, xplmFont_Proportional);
	XPLMDrawNumber(col_white, l + 100, t - 60, 0.0, 3, 2, 1, xplmFont_Proportional);
	XPLMDrawNumber(col_white, l + 100, t - 80, t_vel, 3, 8, 1, xplmFont_Proportional);
	XPLMDrawNumber(col_white, l + 100, t - 100, n_vel, 3, 8, 1, xplmFont_Proportional);
	
	float errors_pitch[3];
	float errors_roll[3];
	//yoke_handler(errors_pitch, errors_roll);
	
	draw_PID(l, t, r, b, PID[0], 20);
	draw_PID(l, t, r, b, errors, 50);

	

	XPLMDrawNumber(col_white, l + 10, t - 120, rate, 2, 2, 1, xplmFont_Proportional);
	XPLMDrawNumber(col_white, l + 10, t - 140, desired_rate, 2, 2, 1, xplmFont_Proportional);
	XPLMDrawNumber(col_white, l + 10, t - 40, bias, 2, 4, 1, xplmFont_Proportional);

}



