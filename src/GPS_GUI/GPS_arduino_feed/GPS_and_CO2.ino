
//Make sure to install the adafruit GPS library from https://github.com/adafruit/Adafruit-GPS-Library
#include <Adafruit_GPS.h> //Load the GPS Library. Make sure you have installed the library form the adafruit site above
#include <SoftwareSerial.h> //Load the Software Serial Library. This library in effect gives the arduino additional serial ports
SoftwareSerial mySerial(3, 2); //Initialize SoftwareSerial, and tell it you will be connecting through pins 2 and 3
Adafruit_GPS GPS(&mySerial); //Create GPS object

String NMEA1;  //We will use this variable to hold our first NMEA sentence
String NMEA2;  //We will use this variable to hold our second NMEA sentence
char c;       //Used to read the characters spewing from the GPS module


SoftwareSerial K_30_Serial(12, 13); //Sets up a virtual serial port
//Using pin 12 for Rx and pin 13 for Tx
byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2 (see app note)
byte response[] = {0, 0, 0, 0, 0, 0, 0}; //create an array to store the response
//multiplier for value. default is 1. set to 3 for K-30 3% and 10 for K-33 ICB
int valMultiplier = 1;


void setup()
{
  Serial.begin(115200);  //Turn on the Serial Monitor
  K_30_Serial.begin(9600);    //Opens the virtual serial port with a baud of 9600
  delay(200);  //Pause
}


void loop()                     // run over and over again
{
  delay(200);
  sendRequest(readCO2);
  unsigned long valCO2 = getValue(response);
  Serial.print("Co2 ppm = ");
  Serial.println(valCO2);
  delay(200);
  K_30_Serial.end();
  delay(1000);
  GPS.begin(9600);       //Turn GPS on at baud rate of 9600
  GPS.sendCommand("$PGCMD,33,0*6D"); // Turn Off GPS Antenna Update
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Tell GPS we want only $GPRMC and $GPGGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  delay(200);
  readGPS();  //This is a function we define below which reads two NMEA sentences from GPS
  delay(1000);
  mySerial.end();
  K_30_Serial.begin(9600);

 
}


void readGPS(){  //This function will read and remember two NMEA sentences from GPS
  clearGPS();    //Serial port probably has old or corrupt data, so begin by clearing it all out
  while(!GPS.newNMEAreceived()) { //Keep reading characters in this loop until a good NMEA sentence is received
  c=GPS.read(); //read a character from the GPS
  }
GPS.parse(GPS.lastNMEA());  //Once you get a good NMEA, parse it
NMEA1=GPS.lastNMEA();      //Once parsed, save NMEA sentence into NMEA1
while(!GPS.newNMEAreceived()) {  //Go out and get the second NMEA sentence, should be different type than the first one read above.
  c=GPS.read();
  }
GPS.parse(GPS.lastNMEA());
NMEA2=GPS.lastNMEA();
  //Serial.println(NMEA1);
 // Serial.println(NMEA2);
  Serial.print(GPS.latitude,4); //Write measured latitude to file
  //Serial.print(GPS.lat); //Which hemisphere N or S
  Serial.print("  ");
  Serial.print(GPS.longitude,4); //Write measured longitude to file
  //Serial.print(GPS.lon); //Which Hemisphere E or W
  Serial.print("  ");
  Serial.print(GPS.altitude);
  Serial.print("  ");
  Serial.print(GPS.speed);
  //Serial.print(" ");
  Serial.println();

}

void clearGPS() {  //Since between GPS reads, we still have data streaming in, we need to clear the old data by reading a few sentences, and discarding these
  while(!GPS.newNMEAreceived()) {
    c=GPS.read();
    }
  GPS.parse(GPS.lastNMEA());
  while(!GPS.newNMEAreceived()) {
    c=GPS.read();
    }
  GPS.parse(GPS.lastNMEA());

}

void sendRequest(byte packet[])
{
  while (!K_30_Serial.available()) //keep sending request until we start to get a response
  {
    K_30_Serial.write(readCO2, 7);
    delay(50);
  }

  int timeout = 0; //set a timeoute counter
  while (K_30_Serial.available() < 7 ) //Wait to get a 7 byte response
  {
    timeout++;
    if (timeout > 10)   //if it takes to long there was probably an error
    {
      while (K_30_Serial.available()) //flush whatever we have
        K_30_Serial.read();

      break;                        //exit and try again
    }
    delay(50);
  }

  for (int i = 0; i < 7; i++)
  {
    response[i] = K_30_Serial.read();
  }
}

unsigned long getValue(byte packet[])
{
  int high = packet[3];                        //high byte for value is 4th byte in packet in the packet
  int low = packet[4];                         //low byte for value is 5th byte in the packet

  unsigned long val = high * 256 + low;              //Combine high byte and low byte with this formula to get value
  return val * valMultiplier;
}

