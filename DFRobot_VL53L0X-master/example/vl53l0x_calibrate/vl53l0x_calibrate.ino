#include <VL53L0X.h>

#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
#include <SD.h>
#include <SPI.h>

const int ledPin = 13;
uint8_t address = 0;


#define CALIBRATE_DISTANCE_OFFSET                   (0x61)
#define CALIBRATE_SPAD                              (0x75)
#define TEMPERATURE_CALIBRATION                     (0x55)
#define SET_LED_MODE                                (0x6c)
#define LED_OFF                                     (0x66)

void setup() {

  //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100);
  Wire.begin();
  Wire.setClock(100000);
  Serial.begin(115200);

  //Stop bus locking up when I2C glitches occur.
  //Wire.setDefaultTimeout(10000);

  // initialize the LED pin as an output.
  pinMode(ledPin, OUTPUT);

  address = 0x08;
}


void mm_to_bytes(uint8_t *bytes, uint16_t mm)
{
    bytes[0] = (mm >> 8 & 0xFF);
    bytes[1] = (mm & 0xFF);
}


void set_led_mode_off()
{

  Wire.beginTransmission(address);
  Wire.write(SET_LED_MODE);
  Wire.write(LED_OFF);
  Wire.endTransmission();
}

void print_menu()
{

  Serial.println("c - calibration_routine");


  Serial.print("Current address is: ");
  Serial.println(address, DEC);

  Serial.println("");

}


void calibration_routine()
{
  set_led_mode_off();

  delay(500);
  
  /* Calibrate SPADs */
  Wire.beginTransmission(address);
  Wire.write(CALIBRATE_SPAD);
  Wire.endTransmission();

  delay(1000);

  /* Calibrate Temperature */

  Wire.beginTransmission(address);
  Wire.write(TEMPERATURE_CALIBRATION);
  Wire.endTransmission();

  delay(1000);

  /* Calibrate Offset */

  uint8_t calib_dist = 100; //In millimeters
  uint8_t dist_bytes[2];
  mm_to_bytes(dist_bytes, calib_dist);
  Wire.beginTransmission(address);
  Wire.write(CALIBRATE_DISTANCE_OFFSET);
  Wire.write(dist_bytes[0]);
  Wire.write(dist_bytes[1]);
  Wire.endTransmission();

  delay(10000);

  Serial.println("Calibration Complete");
}

void loop() {

  while (!Serial) {
    // Wait for serial port
  }
  //digitalWrite(ledPin, HIGH);   // set the LED on
  while (Serial.available() == 0) {}
  switch (Serial.read())
  {
    
    case 'c': calibration_routine(); break;

    default: print_menu(); break;
  }


  delay(10);
  //digitalWrite(ledPin, LOW); // set the LED off

}
