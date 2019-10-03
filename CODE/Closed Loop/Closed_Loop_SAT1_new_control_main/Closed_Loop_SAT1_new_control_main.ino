/*

   Small Satellite Position Control Software.
   Filename: Closed_Loop_SAT1_main.ino
   Author: Ajin Sunny
   Last Modified by: Ajin Sunny


   Written for Thesis: One dimensional Electromagnetic Actuation and Pulse Sensing.
   Version: 1.0
   Date: 02-25-2019
   Last Updated: 09-09-2019

*/

//HEADER FILES
#include <DueTimer.h>
#include <SineWaveDue.h>
#include <SD.h>
#include <SPI.h>
#include <VL53L0X.h>
#include "Arduino.h"
#include "DFRobot_VL53L0X.h"
#include "math.h"



DFRobotVL53L0X sensor;   // Sensor File Object
File myFile;             // File Objec
File raw_File;           // Raw File Object

unsigned long period = 100000;  // Experiment time in milliseconds
unsigned long startime;
unsigned long endtime;
//unsigned long lastTick;
//unsigned long prev_millis = 0;
//unsigned long delta_t = 0;
//unsigned long delta_t1 = 0;
long loops = 0;
double dist[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
const float c = 8.5;
float t1;
float t2;
double k1a = 28.5;
double kr = 1;
double kv = 1;
double vel[9] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double dist_time[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double dist_filtered[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double velocity_final[4] = {0.0,0.0,0.0,0.0};
double return_vel;
double V_final;
double V_sat;
float a1 = 0;
float a2 = 0;
double desired_dist = 0.350;
double A_v = 0.00;
double A_d = 0.00;
double digital_vsine = 0.0;
double Amplitude = 0.00;
unsigned int i = 1;
unsigned int j = 1;
unsigned int k = 1;
char incomingByte;
double relative_dist = 0.0;
double total_dist = 0.0;
double total_relative_dist = 0.00;
double velocity_final_final = 0.00;
double a = 0.97;
//double b = 0.95;
double previous_velocity = 0.00;
double current_velocity = 0.00;

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
  myFile = SD.open("sat1.csv", FILE_WRITE);
  raw_File = SD.open("raw1.csv", FILE_WRITE);
  

  myFile.print("Time");
  myFile.print(",");
  myFile.print("Distance"); 
  myFile.print(",");
  myFile.print("Velocity"); 
  myFile.print(",");
  myFile.print("Saturated_Velocity"); 
  myFile.print(",");
  myFile.print("Amplitude");
  myFile.print(",");
  myFile.println("Amplitude Digital");

  
  raw_File.print("Time");
  raw_File.print(",");
  raw_File.print("Raw Distance");
  raw_File.print(",");
  raw_File.print("Raw Velocity");
  raw_File.print(",");
  raw_File.println("Raw Filtered Velocity");
  
//  raw_File.println("Filtered Velocity");

  
  while (Serial.available() == 0) {}
  incomingByte = Serial.read();
    
  if(incomingByte == 'A')
  {
  Serial.println(incomingByte);
  }

  //delay(20000);
}



/*--------------------LOOP-----------------------*/

void loop()
{
  while (millis() < period)
  {
    S.startSinusoid_update(10,A_d);
    if(myFile)
    {
      startime = millis();
    //delay(18);
//    Serial.print("Voltage value: ");
//    Serial.println(S.return_voltage());

/// COMPUTATION FUNCTION
     
//    for(int i = 0; i < 7; i++)
//    {
//      relative_dist = sensordistRead();
//      total_relative_dist = total_relative_dist + relative_dist;  
//    }
//
//    total_relative_dist = total_relative_dist/7;
      Serial.print("Start: ");
      Serial.println(startime);

      V_final = velocity_func();
      
      if(abs(V_final) <= 0.001)
      {
        V_sat = 0;
      }

      else{
        V_sat = V_final;
      }
      
      //Time
      Serial.print("Time: ");
      Serial.println(millis());
      myFile.print(millis());
      myFile.print(",");
      
    
      //Distance
      Serial.print("Distance: ");
      Serial.println(dist[i-1],8);
      myFile.print(dist[i-1],8);
      myFile.print(",");
    
      //Velocity
      Serial.print("Velocity: ");
      Serial.println(V_final,8);
      myFile.print(V_final,8);
      myFile.print(",");
      
      //Saturated Velocity
      Serial.print("Saturated Velocity: ");
      Serial.println(V_sat,8);
      myFile.print(V_sat,8);
      myFile.print(",");
  
  
      //Feedback Amplitude (u_1)
      Serial.print("Amplitude: ");
      Serial.println(A_v,8);
      myFile.print(A_v,8);
      myFile.print(",");
      
      //Feedback Digital Amplitude (For Arduino)
      Serial.print("Amplitude Digital: ");
      Serial.println(A_d,8);
      myFile.println(A_d,8);
      
        //Sinusoid Signal
//      Serial.print("Sinusoidal Signal");
//      Serial.println(S.return_voltage_signal(10,A_v),8);
//      myFile.println(S.return_voltage_signal(10,A_v),8);
//      
      
      endtime = millis();
      Serial.print("End: ");
      Serial.println(endtime);
      Serial.print("Diff1: ");
      Serial.println(endtime-startime);
      if((endtime-startime) < 100)
      {
        delay(100-(endtime-startime));
        Serial.println("Action1"); 
        A_v = feedback_algorithm(dist[i-1],V_sat);
        A_d = (A_v*490)/2.75;  // Converting voltage to digital
        unsigned int endtime2 = millis();
        Serial.print("Diff2: ");
        Serial.println(endtime2-startime); 
      }
      

//      if((endtime-startime) > 100)
//      {
//        delay((endtime-startime)-100);
//        Serial.println("Action2");
//        A_v = feedback_algorithm(dist[i],V_final);
//        A_d = (A_v*490)/2.75;
//        unsigned int endtime2 = millis();
//        Serial.print("Diff2: ");
//        Serial.println(endtime2-startime);
//       
//      }

      else{
        A_v = feedback_algorithm(dist[i-1],V_sat);
        A_d = (A_v*490)/2.75;  
      }


          
//      int waitime = 100-(endtime-startime);
//      Serial.print("Wait: ");
//      Serial.println(waitime);

    
    
    /// AMPLITUDE UPDATE   
    
//    //Time
//    Serial.print("Time: ");
//    Serial.print(millis(),8);
//    myFile.print(millis(),8);
//    myFile.print(",");
//  
//    //Distance
//    Serial.print("Distance: ");
//    Serial.println(dist[i],8);
//    myFile.print(dist[i],8);
//    myFile.print(",");
//  
//   //Velocity
//    Serial.print("Velocity: ");
//    Serial.println(V_final,8);
//    myFile.print(V_final,8);
//    myFile.print(",");
//
//    //Current Amplitude
//    Serial.print("Amplitude: ");
//    Serial.println(A_v,8);
//    myFile.print(A_v,8);
//    myFile.print(",");
//
//    //Digital Amplitude
//    Serial.print("Digital Voltage Amplitude: ");
//    Serial.println(A_d,8);
//    myFile.print(A_d,8);
//    myFile.print(",");
//
//    //Sinusoid Signal
//    Serial.print("Voltage Signal");
//    Serial.println(S.return_voltage_signal(10,A_v),8);
//    myFile.println(S.return_voltage_signal(10,A_v),8);
//    
//
//    
//   // i++;
//    
////    if (i >= 2)
////    {
////      i = 0;
////    }
////    A = feedback_algorithm(dist[i], vel); 
     
  }
  S.stopSinusoid(); 
  } 
  myFile.close();
  raw_File.close();
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
//  Serial.println("-------------------------DONE----------------------M------");
//  Serial.println("Total Time Lapsed: " + (String)period + "ms has lapsed");
//
//
//}

/*------------- VELOCITY FUNCTION---------------*/

double velocity_func()
{
  
//  for(int k = 0; k < 3; k++)
//  { 

  dist[i] = sensordistRead();
  if(dist[i] > 0.60 | dist[i] < 0.25 )
    {
      dist[i] = dist[i-1]; 
    }
// dist_filtered[i] = b*dist_filtered[i-1] + (1-b)*dist[i];
  dist_time[i] = (double)millis()/1000;
   
//  raw_File.print(dist_time[i],8);  
//  raw_File.print(",");
//  raw_File.print(dist[i],8);
//  raw_File.print(",");

  vel[j] = (dist[i]-dist[i-1])/(dist_time[i]-dist_time[i-1]);
  
  if(vel[j] > 0.20 | vel[j] < -0.20)
  {
    vel[j]=vel[j-1];
  }
  
  velocity_final[i] = a*velocity_final[i-1] + (1-a)*vel[j];
  return_vel = velocity_final[i];
  
  

//  raw_File.print(vel[j],8);
//  raw_File.print(",");
//  raw_File.println(return_vel,8);
  
  //current_velocity = vel[j];
  //previous_velocity = vel[i-1];
  
//  raw_File.println(velocity_final[i],7);
 
//  velocity_final_final = velocity_final_final + vel[j];   //sum the velocity to a double point variable.
   
  if (i == 3)
      {
        dist[0]=dist[i];    //shifts the array back to the 0th element of the array.
        //dist_filtered[0] = dist_filtered[i]; 
        //vel[0] = vel[i-1];    // shifts the velocity array back to the 0th element of the array.
        dist_time[0] = dist_time[i];
        velocity_final[0] = velocity_final[i];
        //vel[0] = vel[j];
        i = 0;              // sets the counter back to the first position. 
        j = 1;
      }
    i++;
    j++;
  
  
//  velocity_final_final = velocity_final_final/5; //Average the velocity. 
  
//  myFile.print(velocity_final_final);
//  myFile.print(",");
//  velocity_final_final=0; //set the sum to zero before next iteration
  //raw_File.print(",");
  //raw_File.print(",");
  //raw_File.print(",");
  //raw_File.println(velocity_final[k],7);
//  return_vel = velocity_final[k];
//  
//  if(k==2)
//  {
//    velocity_final[0] = velocity_final[k];
//    k=0;
//  }
//  
//  k++;
  
  return return_vel;
  
}


/*-----------------SENSOR READ FUNCTION----------------*/ 
double sensordistRead()
{
  
  double sum = 0;
  double final_relative_dist = 0;
  for(int i=1;i<=7;i++)
  {
  double relative_dist;
  relative_dist = ((sensor.getDistance()/1000.00)+0.195005029);
  sum = sum + relative_dist;
  }
  final_relative_dist = sum/7;
  return final_relative_dist; 
  
}




/*----------------FEEDBACK ALGORITHM FUNCTION -----------------*/
double feedback_algorithm(double dist, double V_final)
{
  
  //Current control 
  //Amplitude = k1a * pow(dist,2) * (tanh(kr * (dist - desired_dist)) + c * tanh(kv * V_final));
  //New conrtol
  Amplitude = k1a * pow(dist,2) * (pow(abs(tanh(kr * (dist - desired_dist)) + c*tanh(kv * V_final)),0.5));

  if(Amplitude > 3.50)
  {
  return 3.50;
  }
  else if(Amplitude < -3.50)
  {
    return -3.50;
  }

  else{
    return Amplitude;
  }
//  if (Amplitude >= 500)
//    {return 490;}
//  else if (Amplitude<-500)
//    { return -490;}
//  else
//    {return Amplitude;}

}













//float feedback_algorithm(float dist, float velocity)
//{
//  Amplitude = k1a * pow(dist,2) * (tanh(kr * (dist - desired_dist)) + c*tanh(kv * vel));
//
//  if (Amplitude >= 500)
//    {return 490;}
//  else if (Amplitude<-500)
//    { return -490;}
//  else
//    {return Amplitude;}
//}
  
//  Amplitude = k1a * pow(dist,2) * (pow(abs(tanh(kr * (dist - desired_dist)) + c*tanh(kv * vel)),0.5));
//  return Amplitude; 
//  if (Amplitude<-500)
//    {return -500;}
//  if (Amplitude > 500)
//    {return 500;}
//  else 
//  {
//    return Amplitude;
//  }
