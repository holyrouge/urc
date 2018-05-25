#ifndef __CONTROLLER_H_INCLUDED__
#define __CONTROLLER_H_INCLUDED__

#include "ros/ros.h"

#define DEADZONE 3200

// #define HEADER      0
#define FRONT_LEFT  0
#define FRONT_RIGHT 1
#define MID_LEFT    2
#define MID_RIGHT   3
#define BACK_LEFT   4
#define BACK_RIGHT  5
//#define CHECKSUM  

//For the arm
#define BASE_ARM 1
#define VERTICAL_TOGGLE 2
#define VERTICAL 3
#define WRIST_PITCH 4
#define WRIST_ROTATION 5
#define HAND_CONTROL 6

//1 OR -1
#define FRONT_LEFT_SIGN 1
#define MID_LEFT_SIGN   -1
#define BACK_LEFT_SIGN  -1

#define FRONT_RIGHT_SIGN 1
#define MID_RIGHT_SIGN   -1
#define BACK_RIGHT_SIGN  -1

/**
 * The name of the Device in the filesystem.
 */
std::string name;


void prep_pub_motormsg(char *, ros::Publisher);
void prep_pub_armmsg(char *, ros::Publisher);

// REMOVE THESE AFTER MAIN CODE FULLY ACCOMODATES PUBLISHING 
void prepare_packet_write(char *); 
void prepare_arm_packet_write(char *);

#endif
