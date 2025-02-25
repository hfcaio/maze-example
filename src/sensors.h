#include <VL53L0X.h>

// constants
#define DELAY 3

typedef struct{
    VL53L0X* tof;
    int xshut;  
    int newAddress;
}SensorList;

// TOF sensor
bool init_tof(SensorList sensors[], bool debug = false) {
  for(int i = 0; i < sizeof(sensors)/sizeof(sensors[0]); i++){
      pinMode(sensors[i].xshut, OUTPUT);
      digitalWrite(sensors[i].xshut, LOW);
  }

  for(int i = 0; i < sizeof(sensors)/sizeof(sensors[0]); i++){
    digitalWrite(sensors[i].xshut, HIGH);
    delay(DELAY);

    sensors[i].tof->init();
    sensors[i].tof->setAddress(sensors[i].newAddress);
    sensors[i].tof->setTimeout(100);
    sensors[i].tof->startContinuous();
  }
  
  return true;
}