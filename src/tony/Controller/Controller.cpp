#include <sstream>
#include "joystick.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>

#include <stdlib.h>
#include <string.h>  // string function definitions
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions


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
const int L3 = 9;  // left stick button
const int R3 = 10; // left stick button
const int LS_X = 0;
const int LS_Y = 1;
const int RS_X = 2;
const int RS_Y = 3;
const int RT = 4;
const int LT = 5;
const int DPAD_X = 6;
const int DPAD_Y = 7;

/**
     * The file descriptor of the Device. This will be any
     * nonnegative number if the Device is open, otherwise -1.
     */
int descriptor;

/**
     * Boolean flag indicating whether the Device is opened or not.
     */
bool opened;

/**
     * Flag indicating that the termios struct for the device has been
     * altered.
     */
bool setTty;

/**
     * Copy of the termios struct to reset after communication ends.
     */
struct termios saveTty;

// if false, control the rover. if true, control the arm
bool toggleControl;

bool verticalControl;


//Opens xboxcontroller as a file
int open_joystick()
{

  std::ifstream jsFile; // a better name may be in order to avoid confusion
  jsFile.open("../urc2018/src/tony/Controller/js.txt");
  char js[256];

  std::string line = "test";
  if (jsFile.is_open())
  {
    std::getline(jsFile, line);
    ROS_INFO(line.c_str());
  }

  joystick_fd = open(line.c_str(), O_RDONLY | O_NONBLOCK); /* read write for force feedback? */
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

  // memset(wjse, 0, sizeof(
  // ROS_INFO("Hello");
  // while ((rc = read_joystick_event(&jse) == 1)) {
  jse->type &= ~JS_EVENT_INIT; // ignore synthetic events
                               // ROS_INFO("Hello!!");

  /*
   if (jse.type == JS_EVENT_AXIS) {
    switch (jse.number) {
    case 0: wjse->stick1_x = jse.value;
      break;
    case 1: wjse->stick1_y = jse.value;
      break;
    case 2: wjse->stick2_x = jse.value;
      break;
    case 3: wjse->stick2_y = jse.value;
    break;
    default:
      break;
    }
  } else*/ if (jse->type == 1)
  {
    //main buttons

    switch (jse->number)
    {
    case A:
      cst->isPressed[A] = !cst->isPressed[A];
      break; //A
    case B:
      cst->isPressed[B] = !cst->isPressed[B];
      break; //B
    case X:
      cst->isPressed[X] = !cst->isPressed[X];
      break; //X
    case Y:
      cst->isPressed[Y] = !cst->isPressed[Y];
      break; //Y
    case LB:
      cst->isPressed[LB] = !cst->isPressed[LB];
      break; //LEFT bumper
    case RB:
      cst->isPressed[RB] = !cst->isPressed[RB];
      break; //RIGHT bumper
    case START:
      cst->isPressed[START] = !cst->isPressed[START];
      break;
    case L3:
      cst->isPressed[L3] = !cst->isPressed[L3];
      break;
    case R3:
      cst->isPressed[R3] = !cst->isPressed[R3];
      break;
    case SELECT:
      cst->isPressed[SELECT] = !cst->isPressed[SELECT];
      break;
    case XBOX:
      cst->isPressed[XBOX] = !cst->isPressed[XBOX];
      break;

    default: /*ROS_INFO("??? pressed");*/
      break;
    }
  }
  else
  {

    ///sticks
    switch (jse->number)
    {
    case LS_X:
      if (jse->value > DEADZONE || jse->value < -1 * DEADZONE)
        cst->stickL_x = jse->value;
      else
        cst->stickL_x = 0;
      break;
    case LS_Y:
      if (jse->value > DEADZONE || jse->value < -1 * DEADZONE)
        cst->stickL_y = jse->value;
      else
        cst->stickL_y = 0;
      break;
    case RS_X:
      if (jse->value > DEADZONE || jse->value < -1 * DEADZONE)
        cst->stickR_x = jse->value;
      else
        cst->stickR_x = 0;
      break;
    case RS_Y:
      if (jse->value > DEADZONE || jse->value < -1 * DEADZONE)
        cst->stickR_y = jse->value;
      else
        cst->stickR_y = 0;
      break;
    case LT:
      cst->lt = jse->value;
      break;
    case RT:
      cst->rt = jse->value;
      break;
    case DPAD_X:
      cst->dpad_x = jse->value;
      break;
    case DPAD_Y:
      cst->dpad_y = jse->value;
      break;
    default:
      break;
    }
  }

  // } // Weird while loop
  //ROS_INFO("Pressed?: %d\n", cst->isPressed[0]);
  // printf("%d\n", wjse->stick1_y);
  return 0;
}


/* 
*
* Takes in array of 6 speeds of motors and adds init data.
* Also multiplies some things by negative 1 if motors are plugged in
* backwards. 
* Writes data to motor.msg. 
* Header, Length, and Magic Number already defined in motor.msg
* Communication will prep header and checksum and actually write packet to radio
*
* pub: publisher to publish data
* buffer: data to publish
*/
void prep_pub_motormsg(char *buffer, ros::Publisher pub) {
  tony::motor msg;
  int len = msg.NUM_BYTES;
  char transmit_data[len]; 
  transmit_data[FRONT_LEFT] = buffer[FRONT_LEFT] * FRONT_LEFT_SIGN;
  transmit_data[MID_LEFT] = buffer[MID_LEFT] * MID_LEFT_SIGN;
  transmit_data[BACK_LEFT] = buffer[BACK_LEFT] * BACK_LEFT_SIGN;
  transmit_data[FRONT_RIGHT] = buffer[FRONT_RIGHT] * FRONT_RIGHT_SIGN;
  transmit_data[MID_RIGHT] = buffer[MID_RIGHT] * MID_RIGHT_SIGN;
  transmit_data[BACK_RIGHT] = buffer[BACK_RIGHT] * BACK_RIGHT_SIGN;

  for (int i = 0; i < len; i++)
    if (transmit_data[i] == 255)
      transmit_data[i] = 254;

  for(int i = 0; i < len; i++)
    msg.motor_packet[i] = transmit_data[i];
  pub.publish(msg);
  return;
 }

/* 
* Writes data to arm.msg. 
* Header, Length, and Magic Number already defined in arm.msg
* Communication will prep header and checksum and actually write packet to radio
*
* pub: publisher to publish data
* buffer: data to publish
*/
void prep_pub_armmsg(char *buffer, ros::Publisher pub) {
  tony::arm msg; 
  int len = msg.NUM_BYTES;
  char transmit_data[len]; 
  //-1 added to fit in 6 len array. Maybe need to modify constants?
  transmit_data[BASE_ARM-1] = buffer[BASE_ARM] + 127;
  transmit_data[VERTICAL-1] = buffer[VERTICAL] + 127;
  transmit_data[VERTICAL_TOGGLE-1] = buffer[VERTICAL_TOGGLE];
  transmit_data[WRIST_PITCH-1] = buffer[WRIST_PITCH] + 127;
  transmit_data[WRIST_ROTATION-1] = buffer[WRIST_ROTATION] + 127;
  transmit_data[HAND_CONTROL-1] = buffer[HAND_CONTROL];

  for (int i = 0; i < len; i++)
    if (transmit_data[i] == 255)
      transmit_data[i] = 254;

  for(int i = 0; i < len; i++)
    msg.arm_packet[i] = transmit_data[i];
  pub.publish(msg);  
  return;
 }

//REMOVE
void prepare_packet_write(char *buffer)
{
  char transmit_data[8];
  transmit_data[0] = 0xFF;
  transmit_data[FRONT_LEFT + 1] = buffer[FRONT_LEFT] * FRONT_LEFT_SIGN;
  transmit_data[MID_LEFT + 1] = buffer[MID_LEFT] * MID_LEFT_SIGN;
  transmit_data[BACK_LEFT + 1] = buffer[BACK_LEFT] * BACK_LEFT_SIGN;
  transmit_data[FRONT_RIGHT + 1] = buffer[FRONT_RIGHT] * FRONT_RIGHT_SIGN;
  transmit_data[MID_RIGHT + 1] = buffer[MID_RIGHT] * MID_RIGHT_SIGN;
  transmit_data[BACK_RIGHT + 1] = buffer[BACK_RIGHT] * BACK_RIGHT_SIGN;

  for (int i = 1; i < 7; i++)
  {
    if (transmit_data[i] == 255)
      transmit_data[i] = 254;
  }

  char sum = 0;
  for (int i = 1; i < 7; i++)
  {
    sum += transmit_data[i];
  }
  //sum = buffer[FRONT_LEFT] + buffer[MID_LEFT] + buffer[BACK_LEFT] + buffer[FRONT_RIGHT] + buffer[MID_RIGHT] + buffer[BACK_RIGHT];

  transmit_data[7] = sum + 0xAA;
  //ROS_INFO("Speed:%d",state.stickL_y/-1024);
  ::write(descriptor, transmit_data, 8);
}

//REMOVE
void prepare_arm_packet_write(char *buffer)
{
  char transmit_data[8];
  transmit_data[0] = 0xFF;
  transmit_data[BASE_ARM] = buffer[BASE_ARM] + 127;
  transmit_data[VERTICAL] = buffer[VERTICAL] + 127;
  transmit_data[VERTICAL_TOGGLE] = buffer[VERTICAL_TOGGLE];
  transmit_data[WRIST_PITCH] = buffer[WRIST_PITCH] + 127;
  transmit_data[WRIST_ROTATION] = buffer[WRIST_ROTATION] + 127;
  transmit_data[HAND_CONTROL] = buffer[HAND_CONTROL];

  for (int i = 1; i < 7; i++)
  {
    if (transmit_data[i] == 255)
      transmit_data[i] = 254;
  }

  char sum = 0;
  for (int i = 1; i < 7; i++)
  {
    sum += transmit_data[i];
  }

  transmit_data[7] = sum + 0xBB;
  //ROS_INFO("Speed:%d",state.stickL_y/-1024);
  ::write(descriptor, transmit_data, 8);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
 {

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher controller_pub = n.advertise<controller>("arm_data", 1000);
  ros::Publisher controller_pub_motor = n.advertise<motor>("motor_data", 1000);

  controller state;
  //Joystick stuff

  int fd, rc;
  int done = 0;

  char killPacket[] = {'0','0','0','0','0','0','0','0'};
  toggleControl = false; // by default we'll control the rover
  verticalControl = false;

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

  if (fd < 0)
  {
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

    rc = read_joystick_event(&jse);

    // usleep(1000);
    if (rc == 1)
    {
      // ROS_INFO("...: %d", jse->type);

      //ROS_INFO("type: %d", jse.type);
      if (jse.type == 1)
      {

        if (jse.number == B)
        {
          ROS_INFO("B presseed");
          char buffer[8];
          // buffer[HEADER] = 0xFF;
          buffer[FRONT_LEFT] = 0;
          buffer[MID_LEFT] = 0;
          buffer[BACK_LEFT] = 0;

          buffer[FRONT_RIGHT] = 0;
          buffer[MID_RIGHT] = 0;
          buffer[BACK_RIGHT] = 0;

          prepare_packet_write(buffer);
        }

        if (jse.number == A)
        {
          ROS_INFO("A pressed");
          char buffer[8];
          buffer[BASE_ARM] = 0;
          buffer[VERTICAL_TOGGLE] = 0;
          buffer[VERTICAL] = 0;
          buffer[WRIST_PITCH] = 0;
          buffer[WRIST_ROTATION] = 0;
          buffer[HAND_CONTROL] = state.isPressed[A];

          ROS_INFO("A: %d", buffer[HAND_CONTROL]);
          prepare_arm_packet_write(buffer);
        }

        if (jse.number == X)
        {
          ROS_INFO("X pressed");
          char buffer[8];
          buffer[BASE_ARM] = 0;
          buffer[VERTICAL_TOGGLE] = 0;
          buffer[VERTICAL] = 0;
          buffer[WRIST_PITCH] = 0;
          buffer[WRIST_ROTATION] = 0;
          buffer[HAND_CONTROL] = state.isPressed[X];

          prepare_arm_packet_write(buffer);
        }

        if (jse.number == Y)
        {
          ROS_INFO("Y pressed");
          char buffer[8];
          buffer[BASE_ARM] = 0;
          buffer[VERTICAL_TOGGLE] = 0;
          buffer[VERTICAL] = 0;
          buffer[WRIST_PITCH] = 0;
          buffer[WRIST_ROTATION] = 0;
          buffer[HAND_CONTROL] = 0;

          prepare_arm_packet_write(buffer);
        }

        switch (jse.number)
        {
          /* case A :if(cst.isPressed[0]==1)
        ROS_INFO("A is Pressed");
        else
          ROS_INFO("A is NOT Pressed");   break;
      case B :  if(cst.isPressed[1]==1)
        ROS_INFO("B is Pressed");
        else
          ROS_INFO("B is NOT Pressed");break;
      case X :if(cst.isPressed[2]==1)
        ROS_INFO("X is Pressed");
        else
          ROS_INFO("X is NOT Pressed");  break;
      case Y : if(cst.isPressed[3]==1)
        ROS_INFO("Y is Pressed");
        else
          ROS_INFO("Y is NOT Pressed"); break;
    case LB :  if(cst.isPressed[4]==1)
        ROS_INFO("LEFT Bumper is Pressed");
        else
          ROS_INFO("LEFT Bumper is NOT Pressed");break;
    case RB :  if(cst.isPressed[5]==1)
        ROS_INFO("RIGHT Bumper is Pressed");
        else
          ROS_INFO("RIGHT Bumper is NOT Pressed");*/
        case LB:
          if (state.isPressed[LB] == 1) 
          {
            ROS_INFO("LB Pressed");
            verticalControl = !verticalControl;
          }
          else 
          {
            ROS_INFO("LB not pressed");
          }
          break;
        case SELECT:
          if (state.isPressed[SELECT] == 1)
          {
            ROS_INFO("SELECT was pressed");
            toggleControl = !toggleControl;
          }
          else
          {
            ROS_INFO("SELECT is NOT pressed");
          }

          if (toggleControl)
          {
            ROS_INFO("arm");
            prepare_packet_write(killPacket);
          }
          else
          {
            ROS_INFO("rover");
            prepare_arm_packet_write(killPacket);
          }
          break;
        case START:
          if (state.isPressed[START] == 1)
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
        default: /*ROS_INFO(" pressed");*/
          break;
        }
      }
      else if (jse.type == 2)
      {
        switch (jse.number)
        {
        case RS_X:
          ROS_INFO("Right JS X: %8hd, Right JS, Y: %8hd", state.stickR_x, state.stickR_y);
          break;
        case RS_Y:
          ROS_INFO("Right JS X: %8hd, Right JS, Y: %8hd", state.stickR_x, state.stickR_y);
          break;
        case LS_X:
          ROS_INFO("Left JS X: %8hd, Left JS, Y: %8hd", state.stickL_x, state.stickL_y);
          break;
        case LS_Y:
          ROS_INFO("Left JS X: %8hd, Left JS, Y: %8hd", state.stickL_x, state.stickL_y);
          break;
        default:
          break;
        }

        if (!toggleControl)
        {
          if (jse.number == LS_X || jse.number == LS_Y)
          {
            ROS_INFO("Left stick");
            char buffer[8];
            // buffer[HEADER] = 0xFF;
            buffer[FRONT_LEFT] = state.stickL_y / -256;
            buffer[MID_LEFT] = state.stickL_y / -256;
            buffer[BACK_LEFT] = state.stickL_y / -256;

            buffer[FRONT_RIGHT] = state.stickL_y / -256;
            buffer[MID_RIGHT] = state.stickL_y / -256;
            buffer[BACK_RIGHT] = state.stickL_y / -256;

            prepare_packet_write(buffer);
          }

          if (jse.number == RT)
          {
            ROS_INFO("Left stick");
            char buffer[8];
            // buffer[HEADER] = 0xFF;
            //ROS_INFO("RIGHT TRIGGEr: %8hd",  state.rt);
            if (state.rt > 15000)
            {
              buffer[FRONT_LEFT] = 64;
              buffer[MID_LEFT] = 64;
              buffer[BACK_LEFT] = 64;

              buffer[FRONT_RIGHT] = -64;
              buffer[MID_RIGHT] = -64;
              buffer[BACK_RIGHT] = -64;
            }
            else
            {
              buffer[FRONT_LEFT] = 0;
              buffer[MID_LEFT] = 0;
              buffer[BACK_LEFT] = 0;

              buffer[FRONT_RIGHT] = 0;
              buffer[MID_RIGHT] = 0;
              buffer[BACK_RIGHT] = 0;
            }
            prepare_packet_write(buffer);
          }

          if (jse.number == LT)
          {
            ROS_INFO("Left stick");
            char buffer[8];
            // buffer[HEADER] = 0xFF;
            //ROS_INFO("RIGHT TRIGGEr: %8hd",  state.rt);
            if (state.lt > 15000)
            {
              buffer[FRONT_LEFT] = -64;
              buffer[MID_LEFT] = -64;
              buffer[BACK_LEFT] = -64;

              buffer[FRONT_RIGHT] = 64;
              buffer[MID_RIGHT] = 64;
              buffer[BACK_RIGHT] = 64;
            }
            else
            {
              buffer[FRONT_LEFT] = 0;
              buffer[MID_LEFT] = 0;
              buffer[BACK_LEFT] = 0;

              buffer[FRONT_RIGHT] = 0;
              buffer[MID_RIGHT] = 0;
              buffer[BACK_RIGHT] = 0;
            }
            prepare_packet_write(buffer);
          }
        }
        else if (toggleControl)
        {
          char buffer[8]; // let's keep everything at a uniform size for now

          buffer[BASE_ARM] = state.stickL_x / -256;
          buffer[VERTICAL] = state.stickL_y / -256;
          buffer[WRIST_PITCH] = state.stickR_y / -256;
          buffer[WRIST_ROTATION] = state.stickR_x / -256;
          buffer[HAND_CONTROL] = 0;

          if (verticalControl) {
            buffer[VERTICAL_TOGGLE] = 1;
          } else if (!verticalControl) {
            buffer[VERTICAL_TOGGLE] = 0;
          } else {
            buffer[VERTICAL_TOGGLE] = 0;
          }

          ROS_INFO("VToggle: %u", buffer[VERTICAL_TOGGLE]);
          ROS_INFO("Stick: %d", buffer[VERTICAL]);
          ROS_INFO("CLamp: %d", buffer[HAND_CONTROL]);

          prepare_arm_packet_write(buffer);
        }

        //buffer[0]=state.stickL_y/-1024;
        //buffer[0] = 'U';
      }

      get_joystick_status(&jse, &state);
      controller_pub.publish(state);

      // ROS_INFO("Event: time %8u, value %8hd, type: %3u, axis/button: %u\n",
      // jse.time, jse.value, jse.type, jse.number);
      //if(jse.type == 1)
      //ROS_INFO("Controller state: A %d\n", cst.isPressed[0]);
    }

    //ROS_INFO("%s", msg.data.c_str());

    // Call this at end of "doing stuff"
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  //buffer[0]=state.stickL_y/-1024;  //send speed value byte through buffer.

  // if(read(buffer, 10)){
  //   ROS_INFO("Read stuff");
  //   ROS_INFO("\n");
  //   int i =0;
  //   while (i<sizeof(buffer)){
  //   ROS_INFO("0x%02x",buffer[i]);
  //   ROS_INFO("\n");
  //   i++;
  //   }

  // }
  // else{
  //   ROS_INFO("No stuff read.");
  // }

  /*while(true) {
  if(read(descriptor,buffer, 4)){
    cout << " Reading stuff ";
    }else
    cout << "Nothing to read.";
  }*/

  return 0;
}
