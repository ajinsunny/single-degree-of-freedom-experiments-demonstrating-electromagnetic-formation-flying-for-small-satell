#include <DueTimer.h>
#include <SineWaveDue.h>
#include <SD.h>
#include <SPI.h>
#include <VL53L0X.h>
#include "Arduino.h"
#include "DFRobot_VL53L0X.h"


char incomingByte;
DFRobotVL53L0X sensor;
unsigned long period = 700000000; 
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
    //if(myFile)
    {
    S.startSinusoid1(10);
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


    
    delay(1000);
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


  
