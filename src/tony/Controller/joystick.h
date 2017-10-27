
#include <stdio.h>
#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__
//TYPE 1
#define A 0
#define B 1
#define X 2
#define Y 3
#define RB 5
#define LB 4
#define SELECT 6
#define START 7
#define XBOX 8
#define L3 9 // left stick button
#define R3 10 // left stick button

//TYPE 2
#define LS_X 0
#define LS_Y 1
#define RS_X 2
#define RS_Y 3
#define RT 4
#define LT 5
#define DPAD_X 6
#define DPAD_Y 7

#define JOYSTICK_DEVNAME "/dev/input/js0"

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */


struct js_event {
	unsigned int time;	/* event timestamp in milliseconds */
	short value;   /* value */
	unsigned char type;     /* event type */
	unsigned char number;   /* axis/button number */
};

/*struct wwvi_js_event {
	int button[11];
	int stick1_x;
	int stick1_y;
	int stick2_x;
	int stick2_y;
};*/

// struct controller_state{
// 	bool isPressed[11];
// 	int stickL_x;
// 	int stickL_y;
// 	int stickR_x;
// 	int stickR_y;
// 	int dpad_x;
// 	int dpad_y;
// 	int lt;
// 	int rt;
// 	int type;
// } test = {false,1};

extern int open_joystick(char *joystick_device);
extern int read_joystick_event(struct js_event *jse);
extern void set_joystick_y_axis(int axis);
extern void set_joystick_x_axis(int axis);
extern void close_joystick();
// extern int get_joystick_status(struct wwvi_js_event *wjse);

#endif
