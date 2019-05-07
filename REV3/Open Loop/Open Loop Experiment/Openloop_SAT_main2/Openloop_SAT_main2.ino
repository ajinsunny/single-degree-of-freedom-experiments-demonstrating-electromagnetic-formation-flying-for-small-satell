#include <DueTimer.h>
#include <SineWaveDue.h>
#include <SD.h>
#include <SPI.h>
#include <VL53L0X.h>
#include "Arduino.h"
#include "DFRobot_VL53L0X.h"


char incomingByte;
DFRobotVL53L0X sensor;
unsigned long period = 70000; 
float dist[2];
unsigned int i = 0;
float vel = 0;
float velocity; 
float delta_pos;
File myFile;

void setup()  
{ 
  analogReadResolution(10);
  analogWriteResolution(10);
  pinMode(9, OUTPUT);
  Serial.begin(115200);
  //delay(20000);
  while (!Serial) 
  {
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
  sensor.setMode(Continuous, High);
  //Laser rangefinder begins to work
  sensor.start();
  myFile = SD.open("sat2.csv", FILE_WRITE);
  while(Serial.available()==0){}
  incomingByte = Serial.read();
  if(incomingByte == 'A')
  {
  Serial.println(incomingByte);
  }
} 

void loop()  
{ 
  while(millis() < period) 
  {
    if(myFile)
    {
    S.startSinusoid1(10);
    delay(84);
    Serial.print("Voltage value: ");
    Serial.println(S.return_voltage());
    dist[i] = (sensor.getDistance()/1000)+0.2;
    vel = velocity_func(dist);
    Serial.print("Distance: ");
    Serial.println(dist[i]);
    myFile.print(dist[i]);
    myFile.print(",");
    Serial.print("Velocity: ");
    Serial.println(vel);
    myFile.print(vel);
    myFile.print(",");
    i++;

    
    if (i >= 2)
    {
      i = 0;
    }
    
    //delay(1000);
    S.stopSinusoid();
    }
  }
    myFile.close();
    exit(0);
}
    
//    sw.playTone2(1000, 1200);
//    delay(1000);
//    sw.stopTone();
//    sw.playTone(5000, 1000);
//    for( int i; i<10; i++){
//      digitalWrite(9, HIGH);
//      sw.playToneDecay(400, .1*i);
//      delay(1000);
//      digitalWrite(9, LOW);
//      delay(100);
//  }
    




float velocity_func(float dist[2])
{
  delta_pos = dist[i] - dist[i - 1];
  velocity = delta_pos/0.016;
  return velocity;
}


  
