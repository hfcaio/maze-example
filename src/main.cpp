#include <Arduino.h>
#include <LSS.h>
#include <Wire.h>
#include <VL53L0X.h>

// define motors
#define ID_FR 2
#define ID_FL 1
#define ID_BR 3
#define ID_BL 4
#define MAX_VEL 100

#define LSS_BAUD	(LSS_DefaultBaud)
#define LSS_SERIAL	(Serial)

// types
enum direction {
  forward = 1,
  backward = -1
};

// global variables
LSS motor_FR(ID_FR), motor_FL(ID_FL), motor_BR(ID_BR), motor_BL(ID_BL);
VL53L0X sensor;

// Motor movement
bool move_deg(int deg, int time_limite, direction dir = direction::forward);
bool move_vel(int vel, int time, direction dir = forward);

// logic 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  //LSS::initBus(LSS_SERIAL, LSS_BAUD);
  
  sensor.setAddress(0x01);

  //setting Max rpm
  motor_FR.setMaxSpeedRPM(MAX_VEL);
  motor_FL.setMaxSpeedRPM(MAX_VEL);
  motor_BR.setMaxSpeedRPM(MAX_VEL);
  motor_BL.setMaxSpeedRPM(MAX_VEL);


}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("dist: ");
  Serial.println(sensor.readRangeSingleMillimeters());
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