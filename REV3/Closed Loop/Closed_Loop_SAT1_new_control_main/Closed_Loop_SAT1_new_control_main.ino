/*
   Small Satellite Position Control Software.
   Filename: Closed_Loop_SAT1_main.ino
   Author: Ajin Sunny
   Last Modified by: Ajin Sunny


   Written for Thesis: One dimensional Electromagnetic Actuation and Pulse Sensing.
   Version: 1.0
   Date: 02-25-2019
   Last Updated: 05-17-2019

*/

/*
  TOF SENSOR MEASUREMENT MODES: CONTINUOUS MODE OR SINGLE MODE
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



DFRobotVL53L0X sensor;   // SENSOR OBJECT
File myFile;             // FILE OBJECT 

unsigned long period = 200000;  // Count down 15 min
unsigned long startime;
long previousMillis = 0;
unsigned long endtime;
//unsigned long lastTick;
//unsigned long prev_millis = 0;
//unsigned long delta_t = 0;
//unsigned long delta_t1 = 0;
long lastMillis = 0;
long loops = 0;
double dist[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
const float c = 2/22;
float t1;
float t2;
double k1a = 22;
double kr = 1;
double kv = 1;
double vel[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double velocity_final[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double V_final;
float a1 = 0;
float a2 = 0;
double desired_dist = 0.280;
double A_v = 0.00;
double A_d = 0.00;
double digital_vsine = 0.0;
double Amplitude = 0.00;
unsigned int i = 0;
char incomingByte;
double relative_dist = 0.0;
double total_dist = 0.0;
double total_relative_dist = 0.00;
double velocity_final_final = 0.00;
double a = 0.90;
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

  myFile.println(" ");
  myFile.print("Time");
  myFile.print(",");
  myFile.print("Distance"); 
  myFile.print(",");
  myFile.print("Velocity"); 
  myFile.print(",");
  myFile.print("Amplitude Amps");
  myFile.print(",");
  myFile.print("Amplitude Voltage Digital");
  myFile.print(",");
  myFile.print("Sinusoid Signal");

  
  while (Serial.available() == 0) {}
  incomingByte = Serial.read();
    
  if(incomingByte == 'A')
  {
  Serial.println(incomingByte);
  }

  //delay(20000);
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
    S.startSinusoid1(10,A_v);
    if(myFile)
    {
    
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

    V_final = velocity_func();

    A_v = feedback_algorithm(dist[i],V_final);
    A_d = (A_v*490)/2.75; // Converting voltage to digital

    
    
    /// AMPLITUDE UPDATE   
    
    //Time
    Serial.print("Time: ");
    Serial.print(millis()/1000);
    Serial.print('.');
    Serial.println(millis()%1000,4);
    myFile.print(millis()/1000);
    myFile.print('.');
    myFile.print(millis()%1000,4);
    myFile.print(",");
  
    //Distance
    Serial.print("Distance: ");
    Serial.println(dist[i],4);
    myFile.print(dist[i],4);
    myFile.print(",");
  
   //Velocity
    Serial.print("Velocity: ");
    Serial.println(V_final,4);
    myFile.print(V_final,4);
    myFile.print(",");

    //Current Amplitude
    Serial.print("Amplitude: ");
    Serial.println(A_v,4);
    myFile.print(A_v,4);
    myFile.print(",");

    //Digital Amplitude
    Serial.print("Digital Voltage Amplitude: ");
    Serial.println(A_d,4);
    myFile.print(A_d,4);
    myFile.print(",");

    //Sinusoid Signal
    Serial.print("Voltage Signal");
    Serial.println(S.return_voltage_signal(10,A_v),4);
    myFile.println(S.return_voltage_signal(10,A_v),4);
    

    
   // i++;
    
//    if (i >= 2)
//    {
//      i = 0;
//    }
//    A = feedback_algorithm(dist[i], vel); 
     
  }
  S.stopSinusoid(); 
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
//  Serial.println("-------------------------DONE----------------------M------");
//  Serial.println("Total Time Lapsed: " + (String)period + "ms has lapsed");
//
//
//}

/*------------- VELOCITY FUNCTION---------------*/

double velocity_func()
{
  for(int k = 0; k < 8; k++)
  { 
  dist[i] = sensordistRead();
  if(dist[i] > 2.20)
    {
      dist[i] = dist[i-1]; 
    }
  vel[i] = (dist[i] - dist[i-1])/0.01333;
  current_velocity = vel[i+1];
  previous_velocity = vel[i-1];
  velocity_final[i+1] = a*velocity_final[i-1] + (1-a)*current_velocity; 
  velocity_final_final = velocity_final_final + velocity_final[i+1];   //sum the velocity to a double point variable.
 
  
  if (i == 7)
      {
        dist[0]=dist[i-1];    //shifts the array back to the 0th element of the array. 
        vel[0] = vel[i-1];    // shifts the velocity array back to the 0th element of the array.
        velocity_final[0] = velocity_final[i-1];
        i = 0;                // sets the counter back to the first position. 
      }
      i++;
     
  }
  velocity_final_final = velocity_final_final/7; //Average the velocity. 
  
  return velocity_final_final;
}


/*-----------------SENSOR READ FUNCTION----------------*/ 
double sensordistRead()
{
  double actual_relative_dist;
  actual_relative_dist = ((sensor.getDistance()/1000)+0.200);
  return actual_relative_dist; 
}




/*----------------FEEDBACK ALGORITHM FUNCTION -----------------*/
double feedback_algorithm(double dist, double V_final)
{
  //Current control 
  //Amplitude = k1a * pow(dist,2) * (tanh(kr * (dist - desired_dist)) + c * tanh(kv * V_final));
  //New conrtol
  Amplitude = k1a * pow(dist,2) * (pow(abs(tanh(kr * (dist - desired_dist)) + c*tanh(kv * V_final)),0.5));

  if(Amplitude > 2.50)
  {
  return 2.50;
  }
  else if(Amplitude < -2.50)
  {
    return -2.50;
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
