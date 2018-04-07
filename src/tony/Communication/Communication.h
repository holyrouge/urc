#include "ros/ros.h"

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

/**
* Open radio device
*/
bool open_radio();

/**
* Read from radio
*/
bool read(char *buffer, int numBytes);

void motor_function();

void arm_function();

/**
* adds header and checksum to packet
* writes packet to radio file descriptor
*
* data: array of data to be written
* header: header of packet
* magic_number: for checksum
* num_bytes: length of data
*/
void prepare_and_write_packet(uint8 *data, int num_bytes, uint8 header, uint8 magic_number);

/*
* Determines checksum for packet delivery.
*
* packet: the packet to clac checksum for.
* num_bytes: payload length.
*
*/
uint8 calc_checksum(uint8 *packet, uint8 num_bytes);

