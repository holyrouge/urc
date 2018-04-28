#include "ros/ros.h"
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

/*
* This file is the code for the controller ros module
* It sends out a new packet every time a joystick event occurs(joystick moved, button pressed)
* TODO: 
*   - add support for more than one joystick
*   - make a better function for building data packets
*   - resolve wierd descrepency between joystick.h definitions and F310 stick numbers
*   - integrate with luke's communication module
*   - live scan for radio and controller(s)
*/

#define DEADZONE 3200

// for the rover
// #define HEADER      0
#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define MID_LEFT 2
#define MID_RIGHT 3
#define BACK_LEFT 4
#define BACK_RIGHT 5
//#define CHECKSUM

// for the arm
#define BASE_ARM 1
#define VERTICAL_TOGGLE 2
#define VERTICAL 3
#define WRIST_PITCH 4    // right y axis
#define WRIST_ROTATION 5 // right x axis
#define HAND_CONTROL 6   // X or B

//1 OR -1
#define FRONT_LEFT_SIGN 1
#define MID_LEFT_SIGN -1
#define BACK_LEFT_SIGN -1

#define FRONT_RIGHT_SIGN 1
#define MID_RIGHT_SIGN -1
#define BACK_RIGHT_SIGN -1

/**
 * Make sure to include the standard types for any messages you send and any
 * custom types that you define in .msg files (the .h files should be auto
 * build from the .msg files)
 */
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include "tony/controller.h"

using namespace tony;

static int joystick_fd = -1;

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
     * The name of the Device in the filesystem.
     */
std::string name;

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

  //ROS_INFO("Opening: %s", line.c_str());
  //ROS_INFO("pwd: %s", system("pwd"));

  joystick_fd = open(line.c_str(), O_RDONLY | O_NONBLOCK); /* read write for force feedback? */
  if (joystick_fd < 0)
    return joystick_fd;

  /* maybe ioctls to interrogate features here? */

  return joystick_fd;
}

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

void close_joystick()
{
  close(joystick_fd);
}

int get_joystick_status(js_event *jse, controller *cst)
{
  int rc;
  // struct js_event jse;
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

  //ROS_INFO("Left trigger: %d", cst->lt);
  //ROS_INFO("Right stick: %d", cst->stickR_y);

  // } // Weird while loop
  //ROS_INFO("Pressed?: %d\n", cst->isPressed[0]);
  // printf("%d\n", wjse->stick1_y);
  return 0;
}

bool read(char *buffer, int numBytes)
{

  int bytesRead = 0;

  while (numBytes > 0)
  {

    bytesRead = ::read(descriptor, buffer, numBytes);

    if (bytesRead > 0)
    {

      buffer += bytesRead;
      numBytes -= bytesRead;
    }
    else
    {

      ROS_INFO("No bytes read from device.");
    }
  }

  // Return whether any bytes have been read.
  if (numBytes > 0)
    return false;
  else
    return true;
}

/*
bool read(std::vector<byte>& bytes) {

  // Grab the number of bytes to read, and a pointer to the buffer.
  uint32 numBytes = bytes.size();
  byte* bytePtr   = bytes.data();
  
  // Continue to read bytes until the desired number have been read, or until
  // the read returns 0 bytes.
  int bytesRead = 0;
  while(numBytes > 0) {

    bytesRead = ::read(descriptor, bytePtr, numBytes);

    if(bytesRead == 0) break;
    else if (bytesRead == -1) {
      std::cout << "`read()` failed: " << errno << " Shit is fucked up." << std::endl;
      break;
    }

    bytePtr += bytesRead;
    numBytes -= bytesRead;

  }

  #ifdef DEBUG

    // DEBUG: Indicate the bytes read from the device.
    std::cout << "Bytes read from " << name << ": [";

    for(auto c = bytes.begin(); c != bytes.end(); ++c) {

      printf(" 0x%x", (int)(*c));

    }

    std::cout << "]" << std::endl;
  #endif

  // Return whether any bytes have been read.
  if(numBytes > 0)
    return true;
  else
    return false;

}

*/

bool testy()
{

  // Copy over the indicated baud rate.
  //int baudRate = 115200;
  int parity = 0; //must be 0
  bool shouldBlock = false;
  int timeout = 100000;
  // Make sure that we can change the file attributes.
  descriptor = open("/dev/ttyACM0", O_RDWR);
  if (descriptor == -1)
  {
    ROS_INFO("FUCK");
    return false;
  }

  opened = true;

  if (!opened)
  {

    return false;
  }

  // Make sure the device is a serial device.
  if (!isatty(descriptor))
  {

    return false;
  }

  // Create tele-type struct and zero out data.
  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  // Grab attributes for the current serial port.
  if (tcgetattr(descriptor, &tty) != 0)
  {

    ROS_INFO("Device: Could not load attributes from device.");
    return false;
  }

  // Flush the port.
  tcflush(descriptor, TCIOFLUSH);

  // Save the termios attributes for later.
  saveTty = tty;
  setTty = true;

  // Set the baud rate.
  cfsetspeed(&tty, 115200);
  //cfsetospeed (&tty, B115200);
  //cfsetispeed (&tty, B115200);

  // Make the serial port "raw".
  cfmakeraw(&tty);

  // Input flags.
  tty.c_iflag |= IGNPAR | BRKINT;         // enable break processing
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  // Ouptput Flags
  tty.c_oflag = 0; // no remapping, no delays

  // Control Flags.
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  tty.c_cc[VMIN] = (shouldBlock) ? 1 : 0;     // Indicate blocking.
  tty.c_cc[VTIME] = timeout;                  // 0.timeout seconds.
  tty.c_cflag |= CLOCAL;                      // ignore modem controls,
  tty.c_cflag |= CREAD;                       // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);          // shut off parity
  tty.c_cflag |= parity;                      // Set parity.
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  // Disable Local Flags.
  tty.c_lflag = 0;

  // Set the attributes for the current serial port.
  if (tcsetattr(descriptor, TCSANOW, &tty) != 0)
  {

    ROS_INFO("Could not save attributes to device.");
    return false;
  }

  // Flush the port.
  tcflush(descriptor, TCIOFLUSH);

  // Successfully configured the port.
  return true;
}

void prepare_packet_write(char *buffer)
{
  tcflush(descriptor, TCIOFLUSH);
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

  for (int i = 0; i < 8; i++)
  {
    ROS_INFO("%d: %d", i, transmit_data[i]);
  }
  //ROS_INFO("Speed:%d",state.stickL_y/-1024);

  ::write(descriptor, transmit_data, 8);
}

void prepare_arm_packet_write(char *buffer)
{
  tcflush(descriptor, TCIOFLUSH);
  char transmit_data[8];
  transmit_data[0] = 0xFF;
  transmit_data[BASE_ARM] = buffer[BASE_ARM];
  transmit_data[VERTICAL] = buffer[VERTICAL];
  transmit_data[VERTICAL_TOGGLE] = buffer[VERTICAL_TOGGLE];
  transmit_data[WRIST_PITCH] = buffer[WRIST_PITCH];
  transmit_data[WRIST_ROTATION] = buffer[WRIST_ROTATION];
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

  for (int i = 0; i < 8; i++)
  {
    ROS_INFO("%d: %d", i, transmit_data[i]);
  }
  for (int i = 0; i < 8; i++)
  {
    ROS_INFO("%d: %d", i, transmit_data[i]);
  }
  ::write(descriptor, transmit_data, 8);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher controller_pub = n.advertise<controller>("controller_data", 1000);

  controller state;
  //Joystick stuff

  int fd, rc;
  int done = 0;

  char killPacket[] = {'0', '0', '0', '0', '0', '0', '0', '0'};
  toggleControl = false; // by default we'll control the rover
  verticalControl = false;

  struct js_event jse;
  // struct wwvi_js_event wjse;
  // struct controller_state cst;

  for (int i = 0; i < 11; i++)
  {
    // cst.isPressed[i] =false;
    state.isPressed[i] = false;
  }

  state.stickL_x = 0;
  state.stickL_y = 0;
  state.stickR_x = 0;
  state.stickR_y = 0;
  state.dpad_x = 0;
  state.dpad_y = 0;
  state.lt = 0;
  state.rt = 0;
  state.type = 0;

  // A = state.A;
  // B = state.B;
  // X = state.X;
  // Y = state.Y;
  // RB = state.RB;
  // LB = state.LB;
  // SELECT = state.SELECT;
  // START = state.START;
  // XBOX = state.XBOX;
  // R3 = state.R3;

  // LS_X = state.LS_X;
  // LS_Y = state.LS_Y;
  // RS_X = state.RS_X;
  // RS_Y = state.RS_Y;
  // RT = state.RT;
  // LT = state.LT;
  // DPAD_X = state.DPAD_X;
  // DPAD_Y = state.DPAD_Y;

  state.NUM_BUTTONS;

  controller_pub.publish(state);

  fd = open_joystick();

  if (fd < 0)
  {
    ROS_INFO("Controller failed to open.\n");
    exit(1);
  }

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on#include <stdio.h>

 that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

  // "chatter" is the name of the 'channel' of communication. The listener will
  // be looking for one called "chatter" for the string type for example
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // ros::Publisher chatter_pub_int = n.advertise<std_msgs::Int8>("chatter_int", 1000);
  // ros::Publisher chatter_struct = n.advertise<beginner_tutorials::coord>("chatter_struct", 1000);

  ros::Rate loop_rate(500);

  if (testy())
    ROS_INFO("Radio Opened Successfully!");
  else
    ROS_INFO("Radio Could not open");

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
          prepare_arm_packet_write(buffer);
        }

        // if (jse.number == A)
        // {
        //   ROS_INFO("A pressed");
        //   char buffer[8];
        //   buffer[BASE_ARM] = 0;
        //   buffer[VERTICAL_TOGGLE] = 0;
        //   buffer[VERTICAL] = 0;
        //   buffer[WRIST_PITCH] = 0;
        //   buffer[WRIST_ROTATION] = 0;
        //   buffer[HAND_CONTROL] = (state.isPressed[A] == 1 ? 2 : 0);

        //   ROS_INFO("A: %d", buffer[HAND_CONTROL]);
        //   prepare_arm_packet_write(buffer);
        // }

        // if (jse.number == X)
        // {
        //   ROS_INFO("X pressed");
        //   char buffer[8];
        //   buffer[BASE_ARM] = 0;
        //   buffer[VERTICAL_TOGGLE] = 0;
        //   buffer[VERTICAL] = 0;
        //   buffer[WRIST_PITCH] = 0;
        //   buffer[WRIST_ROTATION] = 0;
        //   buffer[HAND_CONTROL] = state.isPressed[X];

        //   prepare_arm_packet_write(buffer);
        // }

        // if (jse.number == Y)
        // {
        //   ROS_INFO("Y pressed");
        //   char buffer[8];
        //   buffer[BASE_ARM] = 0;
        //   buffer[VERTICAL_TOGGLE] = 0;
        //   buffer[VERTICAL] = 0;
        //   buffer[WRIST_PITCH] = 0;
        //   buffer[WRIST_ROTATION] = 0;
        //   buffer[HAND_CONTROL] = 0;

        //   prepare_arm_packet_write(buffer);
        // }

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
        // case LB:
        //   if (state.isPressed[LB] == 1)
        //   {
        //     ROS_INFO("LB Pressed");
        //     verticalControl = !verticalControl;
        //   }
        //   else
        //   {
        //     ROS_INFO("LB not pressed");
        //   }
        //   break;
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
        case A:
          if (state.isPressed[A] == 1)
          {
            ROS_INFO("A PRESSED");
            char buffer[8];
            buffer[BASE_ARM] = 1;
            buffer[VERTICAL_TOGGLE] = 1;
            buffer[VERTICAL] = 0;
            buffer[WRIST_PITCH] = 0;
            buffer[WRIST_ROTATION] = 0;
            buffer[HAND_CONTROL] = 0;
            prepare_arm_packet_write(buffer);
          }
          else
          {
            ROS_INFO("A RELEASED");
            char buffer[8];
            buffer[BASE_ARM] = 0;
            buffer[VERTICAL_TOGGLE] = 0;
            buffer[VERTICAL] = 0;
            buffer[WRIST_PITCH] = 0;
            buffer[WRIST_ROTATION] = 0;
            buffer[HAND_CONTROL] = 0;
            prepare_arm_packet_write(buffer);
          }
          break;
        case X:
          if (state.isPressed[X] == 1)
          {
            ROS_INFO("X PRESSED");
            char buffer[8];
            buffer[BASE_ARM] = 0;
            buffer[VERTICAL_TOGGLE] = 2;
            buffer[VERTICAL] = 0;
            buffer[WRIST_PITCH] = 0;
            buffer[WRIST_ROTATION] = 0;
            buffer[HAND_CONTROL] = 0;
            prepare_arm_packet_write(buffer);
          }
          else
          {
            ROS_INFO("X RELEASED");
            char buffer[8];
            buffer[BASE_ARM] = 0;
            buffer[VERTICAL_TOGGLE] = 0;
            buffer[VERTICAL] = 0;
            buffer[WRIST_PITCH] = 0;
            buffer[WRIST_ROTATION] = 0;
            buffer[HAND_CONTROL] = 0;
            prepare_arm_packet_write(buffer);
          }
          break;
        case Y:
          if (state.isPressed[Y] == 1)
          {
            ROS_INFO("Y PRESSED");
            char buffer[8];
            buffer[BASE_ARM] = 0;
            buffer[VERTICAL_TOGGLE] = 3;
            buffer[VERTICAL] = 0;
            buffer[WRIST_PITCH] = 0;
            buffer[WRIST_ROTATION] = 0;
            buffer[HAND_CONTROL] = 0;
            prepare_arm_packet_write(buffer);
          }
          else
          {
            ROS_INFO("Y RELEASED");
            char buffer[8];
            buffer[BASE_ARM] = 0;
            buffer[VERTICAL_TOGGLE] = 0;
            buffer[VERTICAL] = 0;
            buffer[WRIST_PITCH] = 0;
            buffer[WRIST_ROTATION] = 0;
            buffer[HAND_CONTROL] = 0;
            prepare_arm_packet_write(buffer);
          }
          break;
        case LB:
          if (state.isPressed[LB] == 1)
          {
            ROS_INFO("LB PRESSED");
            char buffer[8];
            buffer[BASE_ARM] = 0;
            buffer[VERTICAL_TOGGLE] = 4;
            buffer[VERTICAL] = 0;
            buffer[WRIST_PITCH] = 0;
            buffer[WRIST_ROTATION] = 0;
            buffer[HAND_CONTROL] = 0;
            prepare_arm_packet_write(buffer);
          }
          else
          {
            ROS_INFO("LB RELEASED");
            char buffer[8];
            buffer[BASE_ARM] = 0;
            buffer[VERTICAL_TOGGLE] = 0;
            buffer[VERTICAL] = 0;
            buffer[WRIST_PITCH] = 0;
            buffer[WRIST_ROTATION] = 0;
            buffer[HAND_CONTROL] = 0;
            prepare_arm_packet_write(buffer);
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

        /*
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
        */

        ROS_INFO("STICK: %d", jse.number);

        if (!toggleControl)
        {
          if (jse.number == LS_X || jse.number == LS_Y)
          {
            ROS_INFO("Joy stick");
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
            ROS_INFO("Right stick");
            char buffer[8];
            // buffer[HEADER] = 0xFF;
            //ROS_INFO("RIGHT TRIGGEr: %8hd",  state.rt);
            ROS_INFO("RT: %d", state.rt);
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

          //buffer[BASE_ARM] = state.stickL_x / -256;
          // buffer[VERTICAL] = state.stickL_y / -256;
          buffer[WRIST_PITCH] = state.stickR_y / -256;
          buffer[WRIST_ROTATION] = state.stickL_y / -256;
          buffer[HAND_CONTROL] = state.stickL_x / -256;
          buffer[BASE_ARM] = state.stickR_x / -256;

          if (verticalControl)
          {
            buffer[VERTICAL_TOGGLE] = 1;
          }
          else if (!verticalControl)
          {
            buffer[VERTICAL_TOGGLE] = 0;
          }
          else
          {
            buffer[VERTICAL_TOGGLE] = 0;
          }

          ROS_INFO("VToggle: %u", buffer[VERTICAL_TOGGLE]);
          ROS_INFO("Stick: %d", buffer[VERTICAL]);
          ROS_INFO("CLamp: %d", buffer[HAND_CONTROL]);
          ROS_INFO("Base: %d", buffer[BASE_ARM]);

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
