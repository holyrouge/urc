#ifndef __COMMUNICATION_H_INCLUDED__
#define __COMMUNICATION_H_INCLUDED__

#include "ros/ros.h"

#define MAX_PAYLOAD 1000

typedef unsigned char uint8;


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


bool open_radio();
bool read(char *buffer, int numBytes);
void motor_function();
void arm_function();
void prepare_and_write_packet(uint8 *data, int num_bytes, uint8 header, uint8 magic_number);
uint8 calc_checksum(uint8 *packet, uint8 magic_number, int num_bytes);

#endif