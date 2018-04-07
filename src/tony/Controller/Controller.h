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



void prepare_packet_write(char *, ros::Publisher);


