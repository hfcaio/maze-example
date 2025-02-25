#include <Arduino.h>
#include <LSS.h>
#include <Wire.h>
#include "sensors.h"

#define numberOfSensors 8

// define motors
#define ID_FR 2
#define ID_FL 1
#define ID_BR 3
#define ID_BL 4
#define MAX_VEL 100

#define LSS_BAUD	(LSS_DefaultBaud)
#define LSS_SERIAL	(Serial)

SensorList sensors[numberOfSensors] = {
  {new VL53L0X(), 2 ,0x29 },
  {new VL53L0X(), 3 ,0x30 },
  {new VL53L0X(), 4 ,0x31 },
  {new VL53L0X(), 5 ,0x32 },
  {new VL53L0X(), 6 ,0x33 },
  {new VL53L0X(), 7 ,0x34 },
  {new VL53L0X(), 8 ,0x35 },
  {new VL53L0X(), 9 ,0x36 }
};

// types
enum direction {
  forward = 1,
  backward = -1
};

// global variables
LSS motor_FR(ID_FR), motor_FL(ID_FL), motor_BR(ID_BR), motor_BL(ID_BL);


// Motor movement
bool move_deg(int deg, int time_limite, direction dir = direction::forward);
bool move_vel(int vel, int time, direction dir = forward);

// logic 

void setup() {
  Serial.begin(9600);
  Wire.begin();

  init_tof(sensors);
  //LSS::initBus(LSS_SERIAL, LSS_BAUD);

  //setting Max rpm
  motor_FR.setMaxSpeedRPM(MAX_VEL);
  motor_FL.setMaxSpeedRPM(MAX_VEL);
  motor_BR.setMaxSpeedRPM(MAX_VEL);
  motor_BL.setMaxSpeedRPM(MAX_VEL);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0; i < sizeof(sensors)/sizeof(sensors[0]); i++){
    Serial.println("================================================");
    char buffer[50];
    sprintf(buffer, "Sensor %d: %d mm", i, sensors[i].tof->readRangeSingleMillimeters());
    Serial.println(buffer);
  }
}

bool move_vel(int vel, int time, direction dir = direction::forward) {
  /* 
    function for movement setting a wheel speed in RPM
  */
  bool res;
  res = motor_FR.wheelRPM(vel*MAX_VEL/100 * dir);
  res &= motor_FL.wheelRPM(-vel*MAX_VEL/100 * dir);
  res &= motor_BR.wheelRPM(vel*MAX_VEL/100 * dir);
  res &= motor_BL.wheelRPM(-vel*MAX_VEL/100 * dir);
  
  delay(time);

  return res;
}

bool move_deg(int deg, int time_limite, direction dir = forward) {
  /* 
    function for movement in degress
  */
 bool res;
 res = motor_FR.moveRelativeT(deg*10*dir, time_limite);
 res &= motor_FL.moveRelativeT(-deg*10*dir, time_limite);
 res &= motor_BR.moveRelativeT(deg*10*dir, time_limite);
 res &= motor_BL.moveRelativeT(-deg*10*dir, time_limite);

 return res;
}