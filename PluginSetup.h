#pragma once

#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"

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

static XPLMWindowID	g_window;

// Callbacks we will register when we create our window
void				draw_hello_world(XPLMWindowID in_window_id, void* in_refcon);
int	dummy_mouse_handler(XPLMWindowID in_window_id, int x, int y, int is_down, void* in_refcon) {
	return 0;
}
XPLMCursorStatus	dummy_cursor_status_handler(XPLMWindowID in_window_id, int x, int y, void* in_refcon) { return xplm_CursorDefault; }
int	dummy_wheel_handler(XPLMWindowID in_window_id, int x, int y, int wheel, int clicks, void* in_refcon) {
	return 1;
}
void dummy_key_handler(XPLMWindowID in_window_id, char key, XPLMKeyFlags flags, char virtual_key, void* in_refcon, int losing_focus) { }

PLUGIN_API int XPluginStart(
	char* outName,
	char* outSig,
	char* outDesc)
{
	strcpy(outName, "Hover Plane Control Panel");
	strcpy(outSig, " ");
	strcpy(outDesc, "Control Panel for the hover plane");

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
	params.right = params.left + 500;
	params.top = params.bottom + 500;

	g_window = XPLMCreateWindowEx(&params);

	//output_throttle = XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use");
	//input_throttle = XPLMFindDataRef("sim/flightmodel2/engines/throttle_used_ratio");
	//joystick_axis_assignments = XPLMFindDataRef("sim/joystick/joystick_axis_assignments");
	//joy_axis_avail = XPLMFindDataRef("sim/joystick/joy_mapped_axis_avail");
	//override_throttles = XPLMFindDataRef("sim/operation/override/override_throttles");

	getDataRefs();


	//Global::vehicle.mass = XPLMGetDataf(Global::total_mass);
	//for (int i = 0; i < 3; i++)
	//	Global::vehicle.inertia_tensor.a[i][i] = Global::vehicle.mass * XPLMGetDataf(Global::moments[i]);
	//
	//XPLMSetDataf(Global::joystick_yaw_deadzone, 0.2);
	//Vec3 nose_fan_pos(-1.8, 0, 0.05), left_fan_pos(-11, 10, 0.57), right_fan_pos(-11, -10, 0.57), CoM_pos(-10, 0, 0.5);
	//
	//nose_fan_pos *= 0.3048;
	//left_fan_pos *= 0.3048;
	//right_fan_pos *= 0.3048;
	//CoM_pos *= 0.3048;
	//
	//Vec3 fan_positions[3];
	//for (int fan = 0; fan < 3; fan++)
	//{
	//	for (int dir = 0; dir < 3; dir++)
	//	{
	//		float temp;
	//		XPLMGetDatavf(Global::engine_positions[dir], &temp, 2 + fan, 1);
	//		fan_positions[fan].n[dir] = temp;
	//	}
	//}
	//
	//Global::vehicle.lift_fan_matrix.fillMatrix(nose_fan_pos, left_fan_pos, right_fan_pos, CoM_pos);


	// Position the window as a "free" floating window, which the user can drag around
	XPLMSetWindowPositioningMode(g_window, xplm_WindowPositionFree, -1);
	// Limit resizing our window: maintain a minimum width/height of 100 boxels and a max width/height of 300 boxels
	XPLMSetWindowResizingLimits(g_window, 200, 200, 500, 500);
	XPLMSetWindowTitle(g_window, "Sample Window");

	return g_window != NULL;
}

void pluginSetup()
{
	Vec3 nose_fan_pos(-1.8, 0, 0.05), left_fan_pos(-11, 8.5, 0.57), right_fan_pos(-11, -8.5, 0.57), CoM_pos(-10, 0, 0.5);

	nose_fan_pos *= 0.3048;
	left_fan_pos *= 0.3048;
	right_fan_pos *= 0.3048;
	CoM_pos *= 0.3048;

	Global::vehicle.lift_fan_matrix.fillMatrix(nose_fan_pos, left_fan_pos, right_fan_pos, CoM_pos);

	Global::vehicle.mass = XPLMGetDataf(Global::total_mass);
	for (int i = 0; i < 3; i++)
	{
		Global::vehicle.inertia_tensor(i, i) = Global::vehicle.mass * XPLMGetDataf(Global::moments[i]);
	}

	XPLMSetDatai(Global::override_joystick, 1);

	XPLMDebugString("SETUP COMPLETE\n");

	buttonSetup();
}

PLUGIN_API void	XPluginStop(void)
{
	// Since we created the window, we'll be good citizens and clean it up
	XPLMDestroyWindow(g_window);
	g_window = NULL;
	XPLMSetDatai(Global::override_pitch, 0);
	XPLMSetDatai(Global::override_roll, 0);
}

PLUGIN_API void XPluginDisable(void) { }
PLUGIN_API int  XPluginEnable(void) { return 1; }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void* inParam) { }


