#include <i2c_t3.h>
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

elapsedMillis sincePrint;

void setup()
{
  Wire.begin(); // join i2c bus
  Serial1.begin(115200); // start serial communication at 9600bps
}


void loop(){
  int out ;
  Serial1.print(sincePrint);
  sincePrint = 0;
  Serial1.print(">");
  Serial1.println(out = readDistance());
  if (out == 0){
    delay(5000);
  }
}

int readDistance(){
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
   delay(1); // Wait 1 ms to prevent overpolling
    Wire.beginTransmission(LIDARLite_ADDRESS);
    Wire.write(RegisterMeasure);
    Wire.write(MeasureValue);
    nackack = Wire.endTransmission(I2C_STOP,10000);
    if (nackack!=0){
        Serial1.print("A");
        }
    
  }

 
  nackack = 100;
  uint8_t trys = 0;
  while (nackack != 0){ 
    if (trys>30){
      return 0 ;
    }
    trys++;
  delay(1);
    Wire.beginTransmission(LIDARLite_ADDRESS);
    Wire.write(RegisterHighLowB);
    nackack = Wire.endTransmission(I2C_STOP,10000);
    if (nackack!=0){
      Serial1.print("C");
      continue;
    }
    Wire.requestFrom(LIDARLite_ADDRESS,2,I2C_STOP,10000);
  }
  while (Wire.available()<2){
    Serial1.print("D");
    delay(1);
  }

  int reading = Wire.readByte() ;
  reading = reading << 8;
  reading |= Wire.readByte();
  return reading;
 
}
