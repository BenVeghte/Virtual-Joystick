#include <XInput.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_AHRS_FusionInterface.h>
#include <Adafruit_AHRS_Mahony.h>

#include <Adafruit_Simple_AHRS.h>

#include <Adafruit_Sensor_Calibration.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>

//Gyroscope Libraries
#include <Adafruit_L3GD20.h>
#include <Adafruit_L3GD20_U.h>

//Accel Libraries
#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>

//Sensor Unique IDs
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);


Adafruit_Mahony filter;

Adafruit_Sensor_Calibration_EEPROM cal;

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
//#define AHRS_DEBUG_OUTPUT

const int SafetyPin = 0;  // Ground this pin to prevent inputs
const boolean UseLeftJoystick   = true;  // set to true to enable left joystick
const boolean InvertLeftYAxis   = false;  // set to true to use inverted left joy Y

const boolean UseRightJoystick  = false;  // set to true to enable right joystick
const boolean InvertRightYAxis  = false;  // set to true to use inverted right joy Y

const boolean UseTriggerButtons = true;   // set to false if using analog triggers

const int ADC_Max = 1023;  
const float JoyMax = 32767; 

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);

    //Accelerometer


    //Gyro
    gyro.enableAutoRange(true);
  
    /* Initialise the sensor */
    if(!gyro.begin()){
      /* There was a problem detecting the L3GD20 ... check your connections */
      Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
      while(1);
    }
  
    /* Display some basic information on this sensor */



  if(!accel.begin()){
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  /* Enable auto-gain */
  mag.enableAutoRange(true);

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  Wire.setClock(400000); // 400KHz

  pinMode(SafetyPin, INPUT_PULLUP);

  XInput.setJoystickRange(0, 1023);
  XInput.setAutoSend(false);  // Wait for all controls before sending

  XInput.begin();

}
void loop() {
  float roll, pitch, yaw;
  float gx, gy, gz;
  static uint8_t counter = 0;


  // put your main code here, to run repeatedly:

  sensors_event_t accel_event;
  sensors_event_t gyro_event;
  sensors_event_t mag_event;

  gyro.getEvent(&gyro_event);
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  cal.calibrate(mag_event);
  cal.calibrate(accel_event);
  cal.calibrate(gyro_event);

  gx = gyro_event.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro_event.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro_event.gyro.z * SENSORS_RADS_TO_DPS;

  filter.update(gx, gy, gz, 
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z, 
                mag_event.magnetic.x, mag_event.magnetic.y, mag_event.magnetic.z);

  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();
  joystick_send(roll, pitch, yaw);
  /*
  Serial.print("Orientation: ");
  Serial.print(heading);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.println(roll);
  */

  /*
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  Serial.print("Quaternion: ");
  Serial.print(qw, 4);
  Serial.print(", ");
  Serial.print(qx, 4);
  Serial.print(", ");
  Serial.print(qy, 4);
  Serial.print(", ");
  Serial.println(qz, 4);  
  */
  
  
  
}

void joystick_send(float roll, float pitch, float yaw) {
  XInput.setJoystick(JOY_LEFT, roll*JoyMax/360, pitch*JoyMax/360);
  XInput.send();
}
