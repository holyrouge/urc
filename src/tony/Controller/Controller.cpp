#include <sstream>
#include "joystick.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include <stdlib.h>
#include <string.h>     // string function definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

#include "controller.h"

/**
 * Make sure to include the standard types for any messages you send and any
 * custom types that you define in .msg files (the .h files should be auto
 * build from the .msg files)
 */
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include "tony/controller.h"
#include "tony/arm.h"
#include "tony/motor.h"

using namespace tony;

//File descriptor initialized to -1 because it is unopened
static int joystick_fd = -1;

// These are given by how xboxdrv data show up
const int A = 0;
const int B = 1;
const int X = 2;
const int Y = 3;
const int RB = 5;
const int LB = 4;
const int SELECT = 6;
const int START = 7;
const int XBOX = 8;
const int L3 = 9; // left stick button
const int R3 = 10; // left stick button
const int LS_X = 0;
const int LS_Y = 1;
const int RS_X = 2;
const int RS_Y = 3;
const int RT = 4;
const int LT = 5;
const int DPAD_X = 6;
const int DPAD_Y = 7;


//Opens xboxcontroller as a file
int open_joystick()
{
  joystick_fd = open(JOYSTICK_DEVNAME, O_RDONLY | O_NONBLOCK); /* read write for force feedback? */
  if (joystick_fd < 0)
    return joystick_fd;

  /* maybe ioctls to interrogate features here? */

  return joystick_fd;
}

// Reads most recent controler event into js_event struct
int read_joystick_event(struct js_event *jse)
{
  int bytes;

  bytes = read(joystick_fd, jse, sizeof(*jse));

  if (bytes == -1)
    return 0;

  if (bytes == sizeof(*jse))
    return 1;


  ROS_INFO("Unexpected bytes from joystick:%d\n", bytes);

  return -1;
}


//Closes xbox controller file when done for good practice
void close_joystick()
{
  close(joystick_fd);
}

int get_joystick_status(js_event *jse, controller *cst)
{
  int rc;
  
  // controller is not open, failure
  if (joystick_fd < 0)
    return -1;


  jse->type &= ~JS_EVENT_INIT; // ignore synthetic events

  // Main buttons
  if (jse->type == JS_EVENT_BUTTON) {
    //main buttons

    switch(jse->number){
        case A : cst->isPressed[A] = !cst->isPressed[A];  break;//A
        case B : cst->isPressed[B] = !cst->isPressed[B]; break;//B
        case X : cst->isPressed[X] = !cst->isPressed[X]; break;//X
        case Y : cst->isPressed[Y] = !cst->isPressed[Y]; break;//Y
        case LB : cst->isPressed[LB] = !cst->isPressed[LB]; break;//LEFT bumper
        case RB : cst->isPressed[RB] = !cst->isPressed[RB]; break;//RIGHT bumper
        case START : cst->isPressed[START] = !cst->isPressed[START]; break;
        case L3 : cst->isPressed[L3] = !cst->isPressed[L3]; break;
        case R3 : cst->isPressed[R3] = !cst->isPressed[R3]; break;
        case SELECT : cst->isPressed[SELECT] = !cst->isPressed[SELECT]; break;
        case XBOX : cst->isPressed[XBOX] = !cst->isPressed[XBOX]; break;

        default : /*ROS_INFO("??? pressed");*/ break;
      }
      }else /* ANALOG BUTTONS */{

      //Left and right sticks, ignore deadzone for consistent zero
      switch(jse->number){
        case LS_X : 
        if(jse->value > DEADZONE || jse->value < -1*DEADZONE)
          cst->stickL_x = jse->value;
        else
          cst->stickL_x = 0; 
        break;
        case LS_Y : 
        if(jse->value > DEADZONE || jse->value < -1*DEADZONE)
          cst->stickL_y = jse->value;
        else
          cst->stickL_y = 0;  
        break;
        case RS_X : 
        if(jse->value > DEADZONE || jse->value < -1*DEADZONE)
          cst->stickR_x = jse->value; 
        else
          cst->stickR_x = 0; 
        break;
        case RS_Y : 
        if(jse->value > DEADZONE || jse->value < -1*DEADZONE)
          cst->stickR_y = jse->value; 
        else
          cst->stickR_y = 0; 
        break;
        //Triggers and DPAD
        case LT : cst->lt = jse->value; break;
        case RT : cst->rt = jse->value; break;
        case DPAD_X : cst->dpad_x = jse->value; break;
        case DPAD_Y : cst->dpad_y = jse->value; break;
        default : break;
      }

    }
    return 0;
  }


/* Calculates checksum and prepares data packet for SAR video movement
*
* This is not written to be modified easily... it assumes a few things.
* Takes in array of 6 speeds of motors, adds init data, and checksum. 
* Also multiplies some things by negative 1 if motors are plugged in
* backwards. 
*
* pub: publisher to publish data
* buffer: data to publish
*
* 4/6/18: Writes data to motor.msg. 
*/
void prepare_packet_write(char *buffer, ros::Publisher pub) {
  int len = 6;
  char transmit_data[len];
  transmit_data[FRONT_LEFT] = buffer[FRONT_LEFT] * FRONT_LEFT_SIGN;
  transmit_data[MID_LEFT] = buffer[MID_LEFT] * MID_LEFT_SIGN;
  transmit_data[BACK_LEFT] = buffer[BACK_LEFT] * BACK_LEFT_SIGN;
  transmit_data[FRONT_RIGHT] = buffer[FRONT_RIGHT] * FRONT_RIGHT_SIGN;
  transmit_data[MID_RIGHT] = buffer[MID_RIGHT] * MID_RIGHT_SIGN;
  transmit_data[BACK_RIGHT] = buffer[BACK_RIGHT] * BACK_RIGHT_SIGN;

  motor msg;
  for(int i = 0; i < len; i++)
    msg.motor_packet[i] = transmit_data[i];
  pub.publish(msg);
  return;
 }


int main(int argc, char **argv)
 {

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher controller_pub = n.advertise<controller>("controller_data", 1000);
  ros::Publisher controller_pub_motor = n.advertise<motor>("motor_data", 1000);

  controller state;
//Joystick stuff

  int fd, rc;
  int done = 0;

  struct js_event jse;

// Initialize controller stuff 
  for(int i=0; i<11;i++){
  // cst.isPressed[i] =false;
    state.isPressed[i]=false;
  }

  state.stickL_x=0;
  state.stickL_y=0;
  state.stickR_x=0;
  state.stickR_y=0;
  state.dpad_x=0;
  state.dpad_y=0;
  state.lt=0;
  state.rt=0;
  state.type=0;

  state.NUM_BUTTONS;

  controller_pub.publish(state);

  fd = open_joystick();


  if (fd < 0) {
    ROS_INFO("Controller failed to open.\n");
    exit(1);
  }

  ros::Rate loop_rate(1000);



  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
  rc = read_joystick_event(&jse); // Read data from controller

  if (rc != 1) { // no updates just wait until next time
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  // ROS_INFO("...: %d", jse->type);

    //ROS_INFO("type: %d", jse.type);
  if(jse.type == JS_EVENT_BUTTON){ // Button press 

    if(jse.number == B) {
      ROS_INFO("B presseed Stopping rover");
      char buffer[8];
      // buffer[HEADER] = 0xFF;
      buffer[FRONT_LEFT] = 0;
      buffer[MID_LEFT] = 0;
      buffer[BACK_LEFT] = 0;

      buffer[FRONT_RIGHT] = 0;
      buffer[MID_RIGHT] = 0;
      buffer[BACK_RIGHT] = 0;

      prepare_packet_write(buffer, controller_pub_motor);
    }
    switch(jse.number){
      case START : 
      if(state.isPressed[START]==1)
        ROS_INFO("START is Pressed");
      else
        ROS_INFO("START is NOT Pressed");
      ROS_INFO("A state: %u", !state.isPressed[A]);
      ROS_INFO("B state: %u", !state.isPressed[B]);
      ROS_INFO("X state: %u", !state.isPressed[X]);
      ROS_INFO("Y state: %u", !state.isPressed[Y]);
      ROS_INFO("LB state: %u", !state.isPressed[LB]);
      ROS_INFO("RB state: %u\n", !state.isPressed[RB]);
      ROS_INFO("SELECT state: %u", !state.isPressed[SELECT]);
      ROS_INFO("L3 state: %u", !state.isPressed[L3]);
      ROS_INFO("R3 state: %u", !state.isPressed[R3]);

      break;
      default : /*ROS_INFO(" pressed");*/ break;
    }
  }else if (jse.type == 2){
    switch(jse.number){
      case RS_X : ROS_INFO("Right JS X: %8hd, Right JS, Y: %8hd",  state.stickR_x, state.stickR_y);
      break;
      case RS_Y : ROS_INFO("Right JS X: %8hd, Right JS, Y: %8hd",  state.stickR_x, state.stickR_y);
      break;
      case LS_X : ROS_INFO("Left JS X: %8hd, Left JS, Y: %8hd",  state.stickL_x, state.stickL_y);
      break;
      case LS_Y : ROS_INFO("Left JS X: %8hd, Left JS, Y: %8hd",  state.stickL_x, state.stickL_y);
      break; 
      default : break;
    }


    if(jse.number == LS_X || jse.number == LS_Y) {
      ROS_INFO("Left stick");
      char buffer[8];
      // buffer[HEADER] = 0xFF;
      buffer[FRONT_LEFT] = state.stickL_y/-256;
      buffer[MID_LEFT] = state.stickL_y/-256;
      buffer[BACK_LEFT] = state.stickL_y/-256;

      buffer[FRONT_RIGHT] = state.stickL_y/-256;
      buffer[MID_RIGHT] = state.stickL_y/-256;
      buffer[BACK_RIGHT] = state.stickL_y/-256;

      prepare_packet_write(buffer, controller_pub_motor);
    }

    if(jse.number == RT) {
      ROS_INFO("Left stick");
      char buffer[8];
      // buffer[HEADER] = 0xFF;
      //ROS_INFO("RIGHT TRIGGEr: %8hd",  state.rt);
      if(state.rt > 15000) {
        buffer[FRONT_LEFT] = 64;
        buffer[MID_LEFT] = 64;
        buffer[BACK_LEFT] = 64;

        buffer[FRONT_RIGHT] = -64;
        buffer[MID_RIGHT] = -64;
        buffer[BACK_RIGHT] = -64;
      } else {
        buffer[FRONT_LEFT] = 0;
        buffer[MID_LEFT] = 0;
        buffer[BACK_LEFT] = 0;

        buffer[FRONT_RIGHT] = 0;
        buffer[MID_RIGHT] = 0;
        buffer[BACK_RIGHT] = 0;
      }
      prepare_packet_write(buffer, controller_pub_motor);
    }

    if(jse.number == LT) {
      ROS_INFO("Left stick");
      char buffer[8];
      // buffer[HEADER] = 0xFF;
      //ROS_INFO("RIGHT TRIGGEr: %8hd",  state.rt);
      if(state.lt > 15000) {
        buffer[FRONT_LEFT] = -64;
        buffer[MID_LEFT] = -64;
        buffer[BACK_LEFT] = -64;

        buffer[FRONT_RIGHT] = 64;
        buffer[MID_RIGHT] = 64;
        buffer[BACK_RIGHT] = 64;
      } else {
        buffer[FRONT_LEFT] = 0;
        buffer[MID_LEFT] = 0;
        buffer[BACK_LEFT] = 0;

        buffer[FRONT_RIGHT] = 0;
        buffer[MID_RIGHT] = 0;
        buffer[BACK_RIGHT] = 0;
      }
      prepare_packet_write(buffer, controller_pub_motor);
    }

    //buffer[0]=state.stickL_y/-1024;
    //buffer[0] = 'U';
  }

  get_joystick_status(&jse,&state);
  controller_pub.publish(state);



  // ROS_INFO("Event: time %8u, value %8hd, type: %3u, axis/button: %u\n",
    // jse.time, jse.value, jse.type, jse.number);
    //if(jse.type == 1)
    //ROS_INFO("Controller state: A %d\n", cst.isPressed[0]);

  

  ros::spinOnce();
  loop_rate.sleep();
  ++count;
}


return 0;
}



