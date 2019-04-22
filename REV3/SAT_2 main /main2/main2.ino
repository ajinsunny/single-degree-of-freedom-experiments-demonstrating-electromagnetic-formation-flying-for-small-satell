

/*
   Small Satellite Position Control Software.
   Author: Ajin Sunny
   Last Modified by: Ajin Sunny


   Written for Thesis: One dimensional Electromagnetic Actuation and Pulse Sensing.
   Version: 1.0
   Date: 02-25-2019


*/

/*
  TOF SENSOR MEASUREMENT MODES: CONTINUOUS MODE OR SINGLE MODE
*/

//HEADER FILES
#include <VL53L0X.h>
#include "Arduino.h"
#include "DFRobot_VL53L0X.h"
#include <SD.h>
#include <SPI.h>
#include <DueTimer.h>
#include <SineWaveDue.h>
#include <math.h>



DFRobotVL53L0X sensor;   // sensor object
File myFile;             // File Object

unsigned long period = 7000000000;  // Count down 30 sec
unsigned long startime;
long previousMillis = 0;
unsigned long endtime;
//unsigned long lastTick;
unsigned long prev_millis = 0;
unsigned long delta_t = 0;
unsigned long delta_t1 = 0;
long lastMillis = 0;
long loops = 0;
float dist[2];
const float c = 10;
float t1;
float t2;
float delta_pos;
float velocity;
float k2a = 9000;
float kr = 1;
float kv = 1;
float vel = 0;
float a1 = 0;
float a2 = 0;
float desired_dist = 0.30;
float Amplitude;
float A = 0;
unsigned int i = 0;
char incomingByte;


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
  //delay(20000);
  myFile = SD.open("sat2.csv", FILE_WRITE);
  while (Serial.available() == 0) {}
    incomingByte = Serial.read();
    Serial.println(incomingByte);    
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
  if(incomingByte == 'A') 
  {
  while (millis() < period)
  {
    if(myFile)
    {
    S.startSinusoid(100,A);
    delay(36);
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
    Serial.print("Amplitude: ");
    Serial.println(A);
    myFile.println(A);
    i++;
    if (i >= 2)
    {
      i = 0;
    }
    A = feedback_algorithm(dist[i], vel);
    S.stopSinusoid();

  }
  }
  }
  myFile.close();
  exit(0);

}


/*--------------------SENSOR READ FUNCTION--------*/

//void sensorRead()
//{
//
//
//  //S.startSinusoid(100);
//  //delay(1000);
//  unsigned long startTime = millis();
//  unsigned long endTime = startTime + period;
//  myFile = SD.open("data.csv", FILE_WRITE);
//
//  //unsigned long current_millis = 0;
//  //unsigned long dist_1;
//
//  while (millis() < endTime) {
//    //countDown--;
//    {
//      //S.startSinusoid(100, 300);
//      if (myFile)
//      {
//        //S.startSinusoid(100, 300);
//        Serial.print("Distance: ");
//        Serial.println(sensor.getDistance());
//        delta_t = millis() - prev_millis;
//        prev_millis = millis();
//        Serial.print("Time:");
//        Serial.println(delta_t);
//        //myFile.println(sensor.getDistance());
//        //computation from feedback control for new sinusoid.
//        //S.setAmplitude(700);
//
//      }
//
//
//
//    }
//
//    //lastTick += 1000;
//    //delay(100);
//  }
//
//  S.stopSinusoid();
//
//  //myFile.close();
//  Serial.println("-------------------------DONE----------------------------");
//  Serial.println("Total Time Lapsed: " + (String)period + "ms has lapsed");
//
//
//}

/*-------------VELOCITY FUNCTION---------------*/

float velocity_func(float dist[2])
{
  delta_pos = dist[i] - dist[i - 1];
  velocity = delta_pos/0.016;
  return velocity;
}


/*----------------FEEDBACK ALGORITHM FUNCTION -----------------*/

float feedback_algorithm(float dist, float velocity)
{
  Amplitude = k2a*pow(dist,2);
//  Amplitude = k2a * pow(dist,2) * (pow(abs(tanh(kr * (dist - desired_dist)) + c*tanh(kv * vel)),0.5)) * sign((pow(abs(tanh(kr * (dist - desired_dist)) + c*tanh(kv * vel)),0.5)));
  if (Amplitude >=500)
    {return 490;}
    else{
      return Amplitude;
    }

  

//  if ((tanh(kr * (dist - desired_dist)) + c*tanh(kv * vel))>0)
//  {
//    Amplitude = k2a * pow(dist,2) * (pow(abs(tanh(kr * (dist - desired_dist)) + c*tanh(kv * vel)),0.5));
//  }
//  else
//  {
//    Amplitude = -k2a * pow(dist,2) * (pow(abs(tanh(kr * (dist - desired_dist)) + c*tanh(kv * vel)),0.5));
//  }
//  return Amplitude;
//  if (Amplitude > 500)
//   {return 500;}
//  else
//   {return Amplitude;}
}
