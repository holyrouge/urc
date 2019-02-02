import serial

##############
## Script listens to serial port and writes contents into a file
##############
## requires pySerial to be installed 
import serial

serial_port = 'COM5';
baud_rate = 115200; #In arduino, Serial.begin(baud_rate)
write_to_file_path = "newGPSOutput1.txt";
write_to_co2_file_path = "CO2Output.txt"
output_file_gps = open(write_to_file_path, 'a')
output_file_co2 = open(write_to_co2_file_path, 'a')
ser = serial.Serial('COM5', 115200)

import time
time.sleep(3)
while True:
    line = ser.readline();
    line = line.decode("ANSI") #ser.readline returns a binary, convert to string
    line = line.replace('N', ' ')
    line = line.replace("W", ' ')
    print(line)
    if len(line) >= 25:
        output_file_gps.write(line)
    else:
        if line.startswith("Co2 ppm"):
            co2_string = line[10:]
            output_file_co2.write("42069  " + co2_string)

    output_file_gps.close()
    output_file_co2.close()
    time.sleep(1)

   
    output_file_gps = open(write_to_file_path, 'a')
    output_file_gps.truncate(0)

    output_file_co2 = open(write_to_co2_file_path, 'a')
    output_file_co2.truncate(0)

    
