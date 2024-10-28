#include <Arduino.h>
#include <LSS.h>
#include <Wire.h>
#include <VL53L0X.h>

// constants
#define DELAY 3

// define motors
#define ID_FR 2
#define ID_FL 1
#define ID_BR 3
#define ID_BL 4
#define MAX_VEL 100

#define LSS_BAUD	(LSS_DefaultBaud)
#define LSS_SERIAL	(Serial)

// define xshut for tof
#define pin_FR 2
#define pin_FL 3
#define pin_RF 4
#define pin_RB 5
#define pin_BR 6
#define pin_BL 7
#define pin_LB 8
#define pin_LF 9


// types
enum direction {
  forward = 1,
  backward = -1
};

// global variables
LSS motor_FR(ID_FR), motor_FL(ID_FL), motor_BR(ID_BR), motor_BL(ID_BL);
VL53L0X sensor_FR, sensor_FL, sensor_RF, sensor_RB, sensor_BR, sensor_BL, sensor_LB, sensor_LF;

// TOF sensor
void init_tof() {
  pinMode(pin_FR, OUTPUT);
  pinMode(pin_FL, OUTPUT);
  pinMode(pin_RF, OUTPUT);
  pinMode(pin_RB, OUTPUT);
  pinMode(pin_BR, OUTPUT);
  pinMode(pin_BL, OUTPUT);
  pinMode(pin_LB, OUTPUT);
  pinMode(pin_LF, OUTPUT);
  delay(DELAY);

  digitalWrite(pin_FR, LOW);
  sensor_FR.setAddress(pin_FR);
  delay(DELAY);

  digitalWrite(pin_FL, LOW);
  sensor_FR.setAddress(pin_FL);
  delay(DELAY);

  digitalWrite(pin_RF, LOW);
  sensor_FR.setAddress(pin_RF);
  delay(DELAY);

  digitalWrite(pin_RB, LOW);
  sensor_FR.setAddress(pin_RB);
  delay(DELAY);

  digitalWrite(pin_BR, LOW);
  sensor_FR.setAddress(pin_BR);
  delay(DELAY);

  digitalWrite(pin_BL, LOW);
  sensor_FR.setAddress(pin_BL);
  delay(DELAY);

  digitalWrite(pin_LB, LOW);
  sensor_FR.setAddress(pin_LB);
  delay(DELAY);

  digitalWrite(pin_LF, LOW);
  sensor_FR.setAddress(pin_LF);
  delay(DELAY);

  sensor_FR.init();
  FrontA.setTimeout(100);
  FrontA.startContinuous();
}

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