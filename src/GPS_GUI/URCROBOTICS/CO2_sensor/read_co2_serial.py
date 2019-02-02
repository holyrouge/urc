import serial

##############
## Script listens to serial port and writes contents into a file
##############
## requires pySerial to be installed 
import serial

serial_port = 'COM11';
baud_rate = 115200; #In arduino, Serial.begin(baud_rate)
write_to_file_path = "co2_output.dat";

output_file = open(write_to_file_path, 'a');
ser = serial.Serial('COM11', 9600)

import time
time.sleep(3)
while True:
    line = ser.readline();
    line = line.decode("ANSI") #ser.readline returns a binary, convert to string
    print(line)
    if len(line) >= 25:
        output_file.write(line)


    output_file.close()

    time.sleep(1)

   
    output_file = open(write_to_file_path, 'a')

    output_file.truncate(0)


    
