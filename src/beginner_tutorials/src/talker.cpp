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


/**
 * Make sure to include the standard types for any messages you send and any
 * custom types that you define in .msg files (the .h files should be auto
 * build from the .msg files)
 */
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include "beginner_tutorials/coord.h" // see beginner_tutorials/msg/coord.msg

static int joystick_fd = -1;

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

int get_joystick_status(js_event *jse, controller_state *cst)
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
  ROS_INFO("type: %d", jse->type);



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
        case 0 :ROS_INFO("A pressed"); cst->isPressed[0] = !cst->isPressed[0]; break;
        case 1 : /*ROS_INFO("B pressed");*/ break;
        case 2 :/*ROS_INFO("X pressed");*/ break;
        case 3 : /*ROS_INFO("Y pressed");*/break;
        default : /*ROS_INFO(" pressed");*/ break;
      }



			}

		 // } // Weird while loop
		 //ROS_INFO("Pressed?: %d\n", cst->isPressed[0]);
	// printf("%d\n", wjse->stick1_y);
	return 0;
}




/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

//Joystick stuff

int fd, rc;
int done = 0;

struct js_event jse;
// struct wwvi_js_event wjse;
struct controller_state cst;

for(int i=0; i<11;i++){
  cst.isPressed[i] =false;
}

fd = open_joystick();


if (fd < 0) {
  ROS_INFO("open failed.\n");
  exit(1);
}

while (!done) {
  rc = read_joystick_event(&jse);
  
  usleep(1000);
  if (rc == 1) {
    // ROS_INFO("...: %d", jse->type);
    get_joystick_status(&jse,&cst);


    // ROS_INFO("Event: time %8u, value %8hd, type: %3u, axis/button: %u\n",
      // jse.time, jse.value, jse.type, jse.number);
			//if(jse.type == 1)
			//ROS_INFO("Controller state: A %d\n", cst.isPressed[0]);

    }
  }



  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

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
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher chatter_pub_int = n.advertise<std_msgs::Int8>("chatter_int", 1000);
  ros::Publisher chatter_struct = n.advertise<beginner_tutorials::coord>("chatter_struct", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {

    //ROS_INFO("%s", msg.data.c_str());

    // Call this at end of "doing stuff"
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
