#include <VL53L0X.h>

// constants
#define DELAY 3
#define numberOfSensors 8

typedef struct{
    VL53L0X* tof;
    int xshut;  
    int newAddress;
}SensorList;

// TOF sensor
bool init_tof(SensorList sensors[], bool debug = false) {
  for(unsigned int i = 0; i < numberOfSensors; i++){
      pinMode(sensors[i].xshut, OUTPUT);
      digitalWrite(sensors[i].xshut, LOW);
  }

  for(unsigned int i = 0; i < numberOfSensors; i++){
    digitalWrite(sensors[i].xshut, HIGH);
    delay(DELAY);
    Serial.println("setting sensor");

    sensors[i].tof->init();
    sensors[i].tof->setAddress(sensors[i].newAddress);
    sensors[i].tof->setTimeout(100);
    sensors[i].tof->startContinuous();
  }
  
  return true;
}