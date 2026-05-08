

#include "O2Sensor.h"


O2Sensor::O2Sensor()  {
    
}

int O2Sensor::setup(){
   
    //o2Sensor.setCollectNumber(COLLECT_NUMBER);
    return  o2Sensor.begin(OxygenSensorAddress);
}

float O2Sensor::getCurrentO2Value(){
    return o2Sensor.getOxygenData(COLLECT_NUMBER);
}