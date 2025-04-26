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


void setup(){
  Serial.begin(9600);

  Wire.begin();

  while (!init_tof(sensors)){
    Serial.println("conectando...");
  }
  //LSS::initBus(LSS_SERIAL, LSS_BAUD);

  
  //setting Max rpm
  motor_FR.setMaxSpeedRPM(MAX_VEL);
  motor_FL.setMaxSpeedRPM(MAX_VEL);
  motor_BR.setMaxSpeedRPM(MAX_VEL);
  motor_BL.setMaxSpeedRPM(MAX_VEL);
}
void loop(){
  Serial.println("================================================");
  // put your main code here, to run repeatedly:
  for(unsigned int i = 0; i < numberOfSensors; i++){
    char buffer[50];
    sprintf(buffer, "Sensor %d: %d mm \n", i, sensors[i].tof->readRangeSingleMillimeters());
    Serial.println(buffer);
  }
}