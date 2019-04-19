#include <VL53L0X.h>


/*!
   @file DFRobot_VL53L0X.ino
   @brief DFRobot's Laser rangefinder library
   @n The example shows the usage of VL53L0X in a simple way.

   @copyright	[DFRobot](http://www.dfrobot.com), 2016
   @copyright	GNU Lesser General Public License

   @author [LiXin]
   @version  V1.0
   @date  2017-8-21
   @https://github.com/DFRobot/DFRobot_VL53L0X
  timer*/
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
#include <SD.h>
#include <SPI.h>


/*****************Keywords instruction*****************/
//Continuous--->Continuous measurement model
//Single------->Single measurement mode
//High--------->Accuracy of 0.25 mm
//Low---------->Accuracy of 1 mm
/*****************Function instruction*****************/
//setMode(ModeState mode, PrecisionState precision)
//*This function is used to set the VL53L0X mode
//*mode: Set measurement mode       Continuous or Single
//*precision: Set the precision     High or Low
//void start()
//*This function is used to enabled VL53L0X
//float getDistance()
//*This function is used to get the distance
//uint16_t getAmbientCount()
//*This function is used to get the ambient count
//uint16_t getSignalCount()
//*This function is used to get the signal count
//uint8_t getStatus();
//*This function is used to get the status
//void stop()
//*This function is used to stop measuring

//Sensor Specifications
/*
   Sample rate:


*/

DFRobotVL53L0X sensor;
File myFile;


void setup() {
  //Keeping the fie open to write for data logging
  

  //initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  //join i2c bus (address optional for master)
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(4)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");



  Wire.begin();
  //Set I2C sub-device address
  sensor.begin(0x50);
  //Set to Back-to-back mode and high precision mode
  sensor.setMode(Continuous,Low);
  //Laser rangefinder begins to work
  sensor.start();
}

unsigned long duration = 300000;  // Count down 5 min
unsigned long lastTick;


void loop()
{

  sensorRead();
  exit(0);

}

void sensorRead(){


  unsigned long startTime = millis();
  unsigned long endTime = startTime + duration;
  myFile = SD.open("data.csv", FILE_WRITE);
  //uint16_t distance;


  while(millis()<endTime) {
    //countDown--;
    {
      
      if(myFile)
      {
        //Get the distance

        //Serial.println(distance);
        Serial.print("Distance: ");
        Serial.println(sensor.getDistance());
        myFile.println(sensor.getDistance());
        

      }
 
    }

    //lastTick += 1000;
  //delay(100);
  }
  myFile.close();
  Serial.println("-------------------------DONE----------------------------");
  Serial.println("Total Time Lapsed: " + (String)duration + "ms has lapsed");

}
