/*
   Small Satellite Position Control Software.
   Filename: Closed_Loop_SAT2_main.ino
   Author: Ajin Sunny
   Last Modified by: Ajin Sunny


   Written for Thesis: One dimensional Electromagnetic Actuation and Pulse Sensing.
   Version: 1.0
   Date: 02-25-2019
   Last Updated: 07-05-2019
*/

//HEADER FILES
#include <DueTimer.h>
#include <VL53L0X.h>
#include <SineWaveDue.h>
#include <SD.h>
#include <SPI.h>
#include "Arduino.h"
#include "DFRobot_VL53L0X.h"
#include "math.h"

DFRobotVL53L0X sensor;   // SENSOR OBJECT
File myFile;             // SAT DATA FILE OBJECT
File raw_File;           // RAW FILE OBJECT



unsigned long period = 60000; // Experiment time in milliseconds
unsigned int startime = 0;
unsigned int endtime = 0;
unsigned int stamp_time;
//unsigned long lastTick;
//unsigned long prev_millis = 0;
//unsigned long delta_t = 0;
//unsigned long delta_t1 = 0;
unsigned int time1 = 0;
long loops = 0;
double dist[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
const float c = 9.5 ;
float t1;
float t2;
float delta_pos;
float velocity;
double k2a = 28;
float kr = 1;
float kv = 1;
double vel[9] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double dist_time[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double velocity_final[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double V_final;
float a1 = 0;
float a2 = 0;
double desired_dist = 0.280;
double Amplitude = 0.00;
double A_v = 0.00;
double A_d = 0.00;
unsigned int i = 0;
unsigned int j = 1;
unsigned int n = 0;
char incomingByte;
double relative_dist = 0.00;
double total_relative_dist = 0.00;
double velocity_final_final = 0.00;
double a = 0.95;
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
  //delay(20000);
  myFile = SD.open("sat2.csv", FILE_WRITE);
  raw_File = SD.open("raw.csv", FILE_WRITE);


  myFile.print("Time");
  myFile.print(",");
  myFile.print("Distance"); 
  myFile.print(",");
  myFile.print("Velocity"); 
  myFile.print(",");
  myFile.print("Amplitude");
  myFile.print(",");
  myFile.print("Amplitude Digital");
  myFile.print(",");
  myFile.println("Sinusoid Signal");

  
  raw_File.print("Time");
  raw_File.print(",");
  raw_File.print("Distance");
  raw_File.print(",");
  raw_File.print("Velocity");
  raw_File.print(",");
  raw_File.println("Filtered Velocity");

//  controlFile.println(" ");
//  controlFile.print("DAC0");
//  controlFile.print(",");
//  controlFile.println("DAC1");
  


//  myFile.print("Start");
//  myFile.print(",");
//  myFile.println("End");

  
  while (Serial.available() == 0) {}
  incomingByte = Serial.read();

    if(incomingByte == 'A')
    {
    Serial.println(incomingByte);    
    }
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
  while (millis() < period)
  {
    S.startSinusoid_update(10,A_d);
    
    if(myFile)
    {
      startime = millis();
      
      //delay(18);
//    Serial.print("Voltage value: ");
//    Serial.println(S.return_voltage());
//    dist[i] = (sensor.getDistance()/1000)+0.2;
//    vel = velocity_func(dist);

//      for(int i=0; i < 7; i++)
//      {
//        relative_dist = sensordistRead();
//        total_relative_dist = total_relative_dist + relative_dist;
//      }
//
//      total_relative_dist = total_relative_dist/7;
      
      Serial.print("Start: ");
      Serial.println(startime);
      
      V_final = velocity_func();
      Serial.print("Time: ");
      Serial.println(millis());
      myFile.print(millis());
      myFile.print(",");
    
      //Distance
      Serial.print("Distance: ");
      Serial.println(dist[4],8);
      myFile.print(dist[4],8);
      myFile.print(",");
    
      //Velocity
      Serial.print("Velocity: ");
      Serial.println(V_final,8);
      myFile.print(V_final,8);
      myFile.print(",");
  
      //Feedback Amplitude (u_2)
      Serial.print("Amplitude: ");
      Serial.println(A_v,8);
      myFile.print(A_v,8);
      myFile.print(",");
      
      //Feedback Digital Amplitude (For Arduino)
      Serial.print("Amplitude Digital: ");
      Serial.println(A_d,8);
      myFile.println(A_d,8);
      
  
//      //Sinusoid Signal
//      Serial.print("Sinusoidal Signal");
//      Serial.println(S.return_voltage_signal(10,A_v),8);
//      myFile.println(S.return_voltage_signal(10,A_v),8);

      
      endtime = millis();
      Serial.print("End: ");
      Serial.println(endtime);
      Serial.print("Diff1: ");
      Serial.println(endtime-startime);
      if((endtime-startime) < 100)
      {
        delay(100-(endtime-startime));
        Serial.println("Action1"); 
        A_v = feedback_algorithm(dist[4],V_final);
        A_d = (A_v*490)/2.75;  // Converting voltage to digital
        unsigned int endtime2 = millis();
        Serial.print("Diff2: ");
        Serial.println(endtime2-startime); 
      }
      
//
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
        A_v = feedback_algorithm(dist[4],V_final);
        A_d = (A_v*490)/2.75;  
      }

      
      
//      int waitime = 100-(endtime-startime);
//      Serial.print("Wait: ");
//      Serial.println(waitime);
        
      
    
    
//    // AMPLITUDE UPDATE 
      
      //Time
//      Serial.print("Time: ");
//      Serial.println(millis());
//      Serial.print(millis()/1000);
//      Serial.print('.');
//      Serial.println(millis()%1000,4);
//      myFile.print(millis()/1000);
//      myFile.print('.');
//      myFile.print(millis()%1000,4);
//      myFile.print(",");
//    
//      //Distance
//      Serial.print("Distance: ");
//      Serial.println(dist[i],4);
//      myFile.print(dist[i],4);
//      myFile.print(",");
//    
//      //Velocity
//      Serial.print("Velocity: ");
//      Serial.println(V_final,4);
//      myFile.print(V_final,4);
//      myFile.print(",");
//  
//      //Current Amplitude
//      Serial.print("Amplitude: ");
//      Serial.println(A_v,4);
//      myFile.print(A_v,4);
//      myFile.print(",");
//      
//      //Digital Amplitude
//      Serial.print("Digital Voltage Amplitude: ");
//      Serial.println(A_d,4);
//      myFile.print(A_d,4);
//      myFile.print(",");
//  
//      //Sinusoid Signal
//      Serial.print("Voltage Signal");
//      Serial.println(S.return_voltage_signal(10,A_v),4);
//      myFile.println(S.return_voltage_signal(10,A_v),4);
      
    
      //stamp_time = millis();
  
    
    //i++;

    
//    if (i >= 2)
//    {
//      i = 0;
//    }
//    A = feedback_algorithm(dist[i], vel);
//    S.stopSinusoid();
  
  }
  S.stopSinusoid();
  //S.print_voltage_signal();
  }
  myFile.close();
  raw_File.close();
//  controlFile.close();
  
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

double velocity_func()
{
//  delta_pos = dist[i] - dist[i - 1];
//  velocity = delta_pos/0.016;
//  return velocity;
//  dist[i] = sensordistRead();
//  dist_time[i] = millis();
  
//  raw_File.print(dist_time[i]);
//  raw_File.print(",");
//  raw_File.println(dist[i]);
  
//  i++;
  
  for(int k = 0; k < 5; k++)
  {
    
  dist[i] = sensordistRead();
  if(dist[i] > 2.20)
    {
      dist[i] = dist[i-1]; 
    }
//  Serial.print(dist[i]);
//  Serial.print(",");
//  Serial.println(i);
  dist_time[i] = (double)millis()/1000;
  
//  raw_File.print(dist_time[i]);
//  raw_File.print(",");
//  raw_File.print(dist[i],7);
//  raw_File.print(",");

  vel[j] = (dist[i]-dist[i-1])/(dist_time[i]-dist_time[i-1]);
  
//  raw_File.print(vel[j],7);
//  raw_File.print(",");

  current_velocity = vel[j];
  previous_velocity = vel[i-1];
  velocity_final[i] = a*velocity_final[i-1] + (1-a)*current_velocity;

  if(velocity_final[i] > 0.20)
  {
    velocity_final[i]=velocity_final[i-1];
  }
  
//  raw_File.println(velocity_final[i],7); 

  velocity_final_final = velocity_final_final + velocity_final[i];   //sum the velocity to a double point variable

  if (i == 4)
    {
        dist[0]=dist[i];    //shifts the array back to the 0th element of the array. 
        //vel[0] = vel[i-1];    // shifts the velocity array back to the 0th element of the array.
        dist_time[0] = dist_time[i];
        velocity_final[0] = velocity_final[i];
        i = 0;                // sets the counter back to the first position. 
        j = 1;
    }
  i++;
  j++;

//  if(dist[i] > 2.20)
//    {
//      dist[i] = dist[i-1]; 
//    }
    
//  raw_File.print(millis());  
//  raw_File.print(",");
//  raw_File.println(dist[i],8);

//  vel[j] = (dist[i] - dist[i-1])/(dist_time[i] - dist_time[i-1]);
// 
//  velocity_final[n+1] = a*velocity_final[n] + (1-a)*vel[j];
//  velocity_final_final = velocity_final_final + velocity_final[n+1];
//  
//  j++;
//  i++;
//  n++;
//  
  
   

//  if(i==5 & j==4 & n==4)
//    {
//      
////    dist[0] = dist[i-1]; //shift the array back to the 0th element of the array.
////    vel[0] = vel[i-1];   // shifts the velocity array back to the 0th element of the array. 
//////  velocity_final[0] = velocity_final[i-1];
//      i = 0;// sets the counter back to the first position. 
//      j = 0;
//      n = 0;
//     
//      
//    }
  
//  current_velocity = vel[i+1];
//  previous_velocity = vel[i-1]; 
//  velocity_final[i+1] = a*velocity_final[i-1] + (1-a)*current_velocity; 
//  velocity_final_final = velocity_final_final + velocity_final[i+1];
 
  

     

//  myFile.println(millis());   
//  Serial.print("End:");
//  Serial.println(millis());
  }
  //raw_File.println("Loop Done");

  //Serial.println(dist[i]);
  velocity_final_final = velocity_final_final/5;  //Average velocity.

  return velocity_final_final;
  
}


/*-------------SENSOR READ FUNCTION-----------*/ 
double sensordistRead()
{ 
  double actual_relative_dist;
  actual_relative_dist = ((sensor.getDistance()/1000)+0.200);
  return actual_relative_dist; 
}


/*----------------FEEDBACK ALGORITHM FUNCTION -----------------*/
double feedback_algorithm(double dist, double V_final)
{
  
  //Amplitude = k2a*pow(dist,2);

  if((tanh(kr * (dist - desired_dist)) + c*tanh(kv * V_final)) > 0)
  {
    Amplitude = k2a * pow(dist,2) * (pow(abs(tanh(kr * (dist - desired_dist)) + c*tanh(kv * V_final)),0.5));
  }
  else{

    Amplitude = -1 * k2a * pow(dist,2) * (pow(abs(tanh(kr * (dist - desired_dist)) + c*tanh(kv * V_final)),0.5));
  }

  if(Amplitude > 3.535)
    {
    return 3.535;
    }
    else if(Amplitude < -3.535)
    {
      return -3.535;
    }
  
    else{
      return Amplitude;
    }

    
//  if (Amplitude > 2.8)
//  {
//  return 2.8; 
//  }
//  else
//  {
//    return Amplitude;
//  }

}












//float feedback_algorithm(float dist, float velocity)
//{
//  Amplitude = k2a*pow(dist,2);
////  Amplitude = k2a * pow(dist,2) * (pow(abs(tanh(kr * (dist - desired_dist)) + c*tanh(kv * vel)),0.5)) * sign((pow(abs(tanh(kr * (dist - desired_dist)) + c*tanh(kv * vel)),0.5)));
//  if (Amplitude >=500)
//    {return 490;}
//    else{
//      return Amplitude;
//    }
////  if ((tanh(kr * (dist - desired_dist)) + c*tanh(kv * vel))>0)
////  {
////    Amplitude = k2a * pow(dist,2) * (pow(abs(tanh(kr * (dist - desired_dist)) + c*tanh(kv * vel)),0.5));
////  }
////  else
////  {
////    Amplitude = -k2a * pow(dist,2) * (pow(abs(tanh(kr * (dist - desired_dist)) + c*tanh(kv * vel)),0.5));
////  }
////  return Amplitude;
////  if (Amplitude > 500)
////   {return 500;}
////  else
////   {return Amplitude;}
//}
