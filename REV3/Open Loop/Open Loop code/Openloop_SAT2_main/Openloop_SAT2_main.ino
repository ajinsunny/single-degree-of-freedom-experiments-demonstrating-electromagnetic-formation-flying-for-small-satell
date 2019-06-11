
/*
   Small Satellite Position Control Software.
   Filename: Open_Loop_SAT2_main.ino
   Author: Ajin Sunny
   Last Modified by: Ajin Sunny


   Written for Thesis: One dimensional Electromagnetic Actuation and Pulse Sensing.
   Version: 1.0
   Date: 02-25-2019
   Last Updated: 05-24-2019
*/
//HEADER FILES 

#include <DueTimer.h>
#include <SineWaveDue.h>
#include <SD.h>
#include <SPI.h>
#include <VL53L0X.h>
#include "Arduino.h"
#include "DFRobot_VL53L0X.h"

DFRobotVL53L0X sensor;  //SENSOR OBJECT
File myFile;            // FILE OBJECT

char incomingByte;
unsigned long period = 30000; 
double dist[7] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
unsigned int i = 1;
double V_final = 0.0;
double velocity_final_final = 0.0;
double velocity_final[7] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double vel[7] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0}; 
double previous_velocity = 0.0;
double current_velocity = 0.0;
double a = 0.9;


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
  myFile.println(" ");
  myFile.print("Time");
  myFile.print(",");
  myFile.print("Distance"); 
  myFile.print(",");
  myFile.println("Velocity"); 
  
  while(Serial.available()==0){}
  incomingByte = Serial.read();
  if(incomingByte == 'A')
  {
  Serial.println(incomingByte);
  }
} 

void loop()  
{ 
  S.startSinusoid1(10);
  while(millis() < period) 
  {    
    if(myFile)
    {
    
    //delay(50);
//    //Serial.print("Voltage value: ");
//    //Serial.println(S.return_voltage());
    //dist[i] = sensorRead();
//    double disp = sensor.getDistance()/10;
//    Serial.println(disp);
    //i++;   
    velocity_func(); 
//    Serial.print("Velocity: ");
//    Serial.println(V_final);
//    myFile.print(V_final);
//    myFile.print(",");
//   // myFile.print("Time: ");
//    myFile.println(millis());
    
//    
//    //delay(1000);
    
    }
    
  }
    myFile.close();
    //SD.remove("sat2.csv");
    S.stopSinusoid();
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
    

//double velocity_func(double dist[2])
//{
//  delta_pos = dist[i] - dist[i - 1];
//  velocity = delta_pos/0.016;
//  return velocity;
//}

/*------------- VELOCITY FUNCTION---------------*/

void velocity_func()
{ 
   
  for(int k = 0; k < 7; k++)
  { 
    dist[i] = sensordistRead();   // Captures the distance  
    vel[i] = (dist[i] - dist[i - 1])/0.038;   // Calculates the velocity 
    current_velocity = vel[i+1];        // current velocity 
    previous_velocity = vel[i-1];     // previous velocity
    velocity_final[i+1] = a*velocity_final[i] + (1-a)*current_velocity;    // forward euler formula for the velocity.
    velocity_final_final = velocity_final_final + velocity_final[i+1];   //sum the velocity to a double point variable. 
    i++;
    
//    //Time
//    Serial.print("Time: ");
//    Serial.println(millis());
//    myFile.print(millis());
//    myFile.print(",");
//    
//    //Distance
//    Serial.print("Distance: ");
//    Serial.println(dist[i]);
//    myFile.println(dist[i]);
//    myFile.print(",");
  
    if (i == 7)
      {
        dist[0]=dist[i-1];    //shifts the array back to the 0th element of the array. 
        vel[0] = vel[i-1];
        velocity_final[1] = velocity_final[i-2];
        i = 1;                // sets the counter back to the first position. 
      }   
  }

  //Time
  Serial.print("Time: ");
  Serial.println(millis());
  myFile.print(millis());
  myFile.print(",");
  
  //Distance
  Serial.print("Distance: ");
  Serial.println(dist[i]);
  myFile.print(dist[i]);
  myFile.print(",");

  velocity_final_final = velocity_final_final/7;  //Average the velocity. 

  //Velocity
  Serial.print("Velocity: ");
  Serial.println(velocity_final_final);
  myFile.println(velocity_final_final);
     
}

double sensordistRead()
{
  double relative_dist;
  relative_dist = ((sensor.getDistance()/10)+20);
  return relative_dist; 
} 
