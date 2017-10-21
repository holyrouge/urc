#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>

#include "tony/dummy.h" // see beginner_tutorials/msg/coord.msg
#include "tony/raw_gps.h" // see beginner_tutorials/msg/coord.msg

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

using namespace tony;
// Functions for 
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard the string: [%s]", msg->data.c_str());
}


// void chatterCallback_int(const std_msgs::Int8::ConstPtr& msg)
// {
//   ROS_INFO("Random number: [%d]", msg->data);
// }

void chatterCallbackDummy(const dummy::ConstPtr& msg)
{
  ROS_INFO("[x, y] is: [%d, %d]", msg->x, msg->y);
  ROS_INFO("[Fixed array size is %d]", msg->SIZE);

  for(int i = 0; i < msg->SIZE; i++) {
    ROS_INFO("fixed[%d] = %d", i, msg->sized_demo[i]);
  }

  for(int i = 0; i < msg->no_size_demo.size(); i++) {
    ROS_INFO("non fixed[%d] = %d", i, msg->no_size_demo[i]);
  }
}

void gpsCallback(const raw_gps::ConstPtr& msg)
{
  // *(msg->raw_gps_data);
  ROS_INFO("GPS array is: [%d]", msg->raw_gps_data[0]);
  ROS_INFO("GPS array is: [%d]", msg->size_gps_data[0]);
}

int main(int argc, char **argv)
{
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
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  // "chatter" is the name of the 'channel'. Looking for someone publishing something on "chatter".
  // 'chatterCallback' is the function that will AUTOMATICALLY called whenever a message occurs
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  // ros::Subscriber sub_int = n.subscribe("chatter_int", 1000, chatterCallback_int);
  ros::Subscriber dummy_subscriber = n.subscribe("chatter_dummy", 1000, chatterCallbackDummy);
  ros::Subscriber sub_gps = n.subscribe("gps", 1000, gpsCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
