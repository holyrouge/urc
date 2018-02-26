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

//#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
//#include <unistd.h>     // UNIX standard function definitions
//#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions



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

int open_joystick()
{
  joystick_fd = open(JOYSTICK_DEVNAME, O_RDONLY | O_NONBLOCK); /* read write for force feedback? */
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
  } else*/ if (jse->type == 1) {
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
      }else {

      switch(jse->number){
        case LS_X : cst->stickL_x = jse->value; break;
        case LS_Y : cst->stickL_y = jse->value; break;
        case RS_X : cst->stickR_x = jse->value; break;
        case RS_Y : cst->stickR_y = jse->value; break;
        case LT : cst->lt = jse->value; break;
        case RT : cst->rt = jse->value; break;
        case DPAD_X : cst->dpad_x = jse->value; break;
        case DPAD_Y : cst->dpad_y = jse->value; break;
        default : break;
        }

      }

     // } // Weird while loop
     //ROS_INFO("Pressed?: %d\n", cst->isPressed[0]);
  // printf("%d\n", wjse->stick1_y);
  return 0;
}

bool read(char* buffer, int numBytes) {

  int bytesRead = 0;

  while(numBytes > 0) {

    bytesRead = ::read(descriptor, buffer, numBytes);

    if(bytesRead > 0) {

      buffer += bytesRead;
      numBytes -= bytesRead;

    } else {

      ROS_INFO("No bytes read from device.");

    }

  }

  // Return whether any bytes have been read.
  if(numBytes > 0)
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

bool testy() {
  
  // Copy over the indicated baud rate.
  //int baudRate = 115200;
  int parity = 0; //must be 0
  bool shouldBlock = false;
  int timeout = 100000;
  // Make sure that we can change the file attributes.
  descriptor = open("/dev/ttyUSB0", O_RDWR);
  if(descriptor ==-1){
    return false;
  }

  opened =true;

  if(!opened) {

    return false;

  } 

  // Make sure the device is a serial device.
  if(!isatty(descriptor)) {

    return false;

  }

  // Create tele-type struct and zero out data.
  struct termios tty;
  memset (&tty, 0, sizeof(tty));

  // Grab attributes for the current serial port.
  if(tcgetattr(descriptor, &tty) != 0) {

    ROS_INFO("Device: Could not load attributes from device.");
    return false;
  }

  // Flush the port.
  tcflush(descriptor, TCIOFLUSH);

  // Save the termios attributes for later.
  saveTty = tty;
  setTty  = true;

  // Set the baud rate.
  cfsetspeed(&tty, 115200);
  //cfsetospeed (&tty, B115200);
    //cfsetispeed (&tty, B115200);

  // Make the serial port "raw".
  cfmakeraw(&tty);

  // Input flags.
  tty.c_iflag     |= IGNPAR | BRKINT;              // enable break processing
  tty.c_iflag     &= ~(IXON | IXOFF | IXANY);      // shut off xon/xoff ctrl
  
  // Ouptput Flags
  tty.c_oflag      = 0;                            // no remapping, no delays
  
  // Control Flags.
  tty.c_cflag      = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  tty.c_cc[VMIN]   = (shouldBlock) ? 1 : 0;      // Indicate blocking.
  tty.c_cc[VTIME]  = timeout;                  // 0.timeout seconds.
  tty.c_cflag     |= CLOCAL;             // ignore modem controls,
  tty.c_cflag     |= CREAD;            // enable reading
  tty.c_cflag     &= ~(PARENB | PARODD);       // shut off parity
  tty.c_cflag     |= parity;             // Set parity.
  tty.c_cflag     &= ~CSTOPB;
  tty.c_cflag     &= ~CRTSCTS;

  // Disable Local Flags.
    tty.c_lflag      = 0;
    
  // Set the attributes for the current serial port.
  if(tcsetattr(descriptor, TCSANOW, &tty) != 0) {

    ROS_INFO("Could not save attributes to device.");
    return false;

  }

  // Flush the port.
  tcflush(descriptor, TCIOFLUSH);

  // Successfully configured the port.
  return true;

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

struct js_event jse;
// struct wwvi_js_event wjse;
// struct controller_state cst;

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


if (fd < 0) {
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
  if (rc == 1) {
  // ROS_INFO("...: %d", jse->type);

    ROS_INFO("type: %d", jse.type);
    if(jse.type == 1){
  switch(jse.number){
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
      case RS_X : ROS_INFO("Right JS X: %8hd",  state.stickR_x);
      break;
      case RS_Y : ROS_INFO("Right JS Y: %8hd", state.stickR_y);
      break;
      case LS_X : ROS_INFO("Left JS X: %8hd",  state.stickL_x);
      break;
      case LS_Y : ROS_INFO("Left JS Y: %8hd",  state.stickL_y);
      break; 
      default : break;
    }

    ROS_INFO("Speed:%d",state.stickL_y/-1024);
    




  }

  get_joystick_status(&jse,&state);
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

  if(testy())
    ROS_INFO("Opened Successfully!");
  else
    ROS_INFO("Could not open");

char buffer[10];

buffer[0]=state.stickL_y/-1024;  //send speed value byte through buffer.

  if(read(buffer, 10)){
    ROS_INFO("Read stuff");
    ROS_INFO("\n");
    int i =0;
    while (i<sizeof(buffer)){
    ROS_INFO("0x%02x",buffer[i]);
    ROS_INFO("\n");
    i++;
    }

  }
  else{
    ROS_INFO("No stuff read.");
  }
  ::write(descriptor,buffer,10);


  /*while(true) {
  if(read(descriptor,buffer, 4)){
    cout << " Reading stuff ";
    }else
    cout << "Nothing to read.";
  }*/
  


  return 0;
}






