/*
 * Small Satellite Position Control Software. 
 * Author: Ajin Sunny
 * Last Modified by: Ajin Sunny
 *
 * 
 * Written for Thesis: One dimensional Electromagnetic Actuation and Pulse Sensing. 
 * Version: 1.0
 * Date: 02-25-2019
 * 
 * 
 * 
 */

 /*
  *TOF SENSOR MEASUREMENT MODES: CONTINUOUS MODE OR SINGLE MODE
  */


//HEADER FILES 
#include <VL53L0X.h>
#include "Arduino.h"
#include "DFRobot_VL53L0X.h"
#include "math.h"
#include <SD.h>
#include <SPI.h>
#include <DueTimer.h>
#include <SineWaveDue.h>



DFRobotVL53L0X sensor;   // sensor object
File myFile;             // File Object

unsigned long period = 30000;  // Count down 5 min
unsigned long now_time = 0;
//unsigned long lastTick;
unsigned long prev_millis = 0;
unsigned long delta_t = 0;
unsigned long delta_t1 = 0;
long lastMillis = 0;
long loops = 0;
float dist_1;
float dist_2;
float dist;
const float c = 1;
float t1;
float t2;
float delta_pos;
float velocity;
float k1 = 1;
float k2 = 1;
float kr = 1;
float kv = 1;
float vel = 0;
float a1 = 0;
float a2 = 0;
float desired_dist = 200.00;
float Amplitude;
float A = 0;



/*--------------------SETUP-------------------------*/
 
void setup() {

  analogReadResolution(10);
  analogWriteResolution(10);
  //pinMode(9, OUTPUT);
  //Keeping the file open to write for data logging
  

  //initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  //join i2c bus (address optional for master)
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
//
//
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
  sensor.setMode(Continuous,High);
  //Laser rangefinder begins to work
  sensor.start();
}


//struct FB_struct{
//
//double fb1;
//double fb2;
//
//};
//
//
//struct FB_struct feedBack(float measured_dist){
//  
//struct FB_struct new_fb;
//
//new_fb.fb1 = k2*dist*dist;
//new_fb.fb2 = k1*dist*dist*(kr*(dist - desired_dist)) + c*(kr*vel);
//
//return new_fb;
//  
//}


/*--------------------LOOP-----------------------*/

void loop()
{ 
if (millis() < now_time + period)
{
  S.startSinusoid(100);
  delay(37);
  dist = sensor.getDistance();
  //vel = velocity_func(dist);
  //A = feedback_algorithm(dist,vel);
  S.stopSinusoid();
  S.startSinusoid(100,A);
  delay(50)
  S.stopSinusoid();
  now_time = millis();
 
}

}


/*--------------------SENSOR READ FUNCTION--------*/

void sensorRead()
{
  
 
  //S.startSinusoid(100);
  //delay(1000);
  unsigned long startTime = millis();
  unsigned long endTime = startTime + period;
  myFile = SD.open("data.csv", FILE_WRITE);
  
  //unsigned long current_millis = 0;
  //unsigned long dist_1;

  while(millis()<endTime) {
    //countDown--;
    {
      //S.startSinusoid(100, 300);
      if(myFile)
      {
        //S.startSinusoid(100, 300);
        Serial.print("Distance: ");
        Serial.println(sensor.getDistance());
        delta_t = millis() - prev_millis;
        prev_millis = millis();
        Serial.print("Time:");
        Serial.println(delta_t);
        //myFile.println(sensor.getDistance());        
        //computation from feedback control for new sinusoid.
        //S.setAmplitude(700); 
                
      }
      
      
    
    }
     
    //lastTick += 1000;
  //delay(100);
  }
  
  S.stopSinusoid();
  
  //myFile.close();
  Serial.println("-------------------------DONE----------------------------");
  Serial.println("Total Time Lapsed: " + (String)period + "ms has lapsed");
  
    
}

/*-------------SINE FUNCTION---------------*/

float velocity_func(float dist)
{
  //dist_1 = sensor.getDistance();
  t1 = millis();
  dist_2 = sensor.getDistance(); 
  t2 = millis();
  delta_pos = dist_2 - dist; 
  velocity = delta_pos/(t2-t1);
  return velocity; 
}


/*----------------FEEDBACK ALGORITHM FUNCTION -----------------*/

float feedback_algorithm(float dist, float velocity)
{
  
  Amplitude = k1*dist*dist*(kr*(dist - desired_dist)) + c*(kv*vel);
  return Amplitude;
  
}
