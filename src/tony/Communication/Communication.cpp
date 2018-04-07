
#include <stdlib.h>
#include <sstream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>     // string function definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

/**
 * Make sure to include the standard types for any messages you send and any
 * custom types that you define in .msg files (the .h files should be auto
 * build from the .msg files)
 */
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include "tony/dummy.h" // see beginner_tutorials/msg/coord.msg
#include "tony/raw_gps.h" // see beginner_tutorials/msg/coord.msg

#include "Communication.h"



/**
* Open radio device
*/
bool open_radio(){

  // Copy over the indicated baud rate.
  //int baudRate = 115200;
  int parity = 0; //must be 0
  bool shouldBlock = false;
  int timeout = 100000;
  // Make sure that we can change the file attributes.
  descriptor = open("/dev/ttyUSB0", O_RDWR);
  if(descriptor ==-1){
    ROS_INFO("RADIO FAILED TO OPEN. IGNORING");
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
* Read from radio
*/
bool read(char* buffer, int numBytes){

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



void motor_function() {
  //Call prepare_and_write_packet(data, header from msg, magic number from msg)
  return;
}

void arm_function() {
  //Call prepare_and_write_packet(data, header from msg, magic number from msg)
  return;
}


void prepare_and_write_packet(uint8 *data, int num_bytes, uint8 header, uint8 magic_number) {
  uint8 *packet;

  packet[0] = header;//Set 0th byte as header
  for(int i = 1; i <= num_bytes; i++){//set bytes 1-N as data
    packet[i] = data[i];
  }
  uint8 checksum = calc_checksum(packet, num_bytes);
  packet[num_bytes+1] = checksum;//set last byte as checksum

  ::write(descriptor, packet, num_bytes+2); // Transmit 8 bits  Call ::write function with new data packet
  return;
}

uint8 calc_checksum(uint8 *packet, uint8 num_bytes) {
  //Write this as specified in the table
  //Don't want to trandmit 255, this is used as special data... to make it easier
  /*for(int i = 1 ; i < 7; i++) {
    if(transmit_data[i] == 255)
      transmit_data[i] = 254;
  }*/


  // Checksum is sum of all data in packet (Not header) + 0xAA
  /*char sum = 0;
  for(int i = 1; i < 7; i++) {
    sum += buffer[i];
  }*/
   //sum = buffer[FRONT_LEFT] + buffer[MID_LEFT] + buffer[BACK_LEFT] + buffer[FRONT_RIGHT] + buffer[MID_RIGHT] + buffer[BACK_RIGHT];
  return 0;

}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "communication");

  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  
  int count = 0;
  while (ros::ok())
  {
  }


  return 0;
}