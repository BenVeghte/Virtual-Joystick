/*
 *  Project     Virtual Joystick 
 *  @author     Ben Veghte
 *  @link       github.com/BenVeghte/Virtual-Joystick
 *  @license    GNU GPLv3
 *  
 *  This code is the first version of the virtual joystick project only using the IMU and 
 *  the teensy 4.0. Teensyduino
 *  
 */

/*********************REQUIRED LIBRARIES*********************/
//The thing about teensys that took me a bit to figure out, is the default libraries 
//selected by choosing USB type from teensyduino are loaded on defaut

//ICM-20948 Libraries
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>

//Adafruit AHRS
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

//General
#include <Wire.h>


/************************************************************/



/***********************OBJECT CREATION**********************/
//ICM-20948
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
Adafruit_ICM20948 icm;


//Adafruit AHRS
Adafruit_Sensor_Calibration_EEPROM cal;
Adafruit_Madgwick filter;

/************************************************************/

/**************************VARIABLES*************************/
//ICM-20948

uint16_t measurement_delay_us = 65535; //This may or may not be neccessary but its purpose is delay data collection so as to not overload the sensor

//Adafruit AHRS
#define FILTER_UPDATE_RATE_HZ 100



/************************************************************/


void setup() {  
  Serial.begin(115200);
  /**************************ICM-20948*************************/
  if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) {}
  }

  icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G); //Set the accelermeter to the correct max magnitude of accel
  icm.setGyroRange(ICM20948_GYRO_RANGE_1000_DPS); //Set the gyro to the right range

  /************************************************************/

  /************************ADAFRUIT AHRS***********************/
  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
    while (1) {} //If the calibration doesn't work
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
    while (1) {} //If the calibration doesn't work
  }

  filter.begin(FILTER_UPDATE_RATE_HZ); //Begin the filter
  
  /************************************************************/


}

void loop() {
  // put your main code here, to run repeatedly:

}
