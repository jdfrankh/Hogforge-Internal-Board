#ifndef O2SENSOR_H
#define O2SENSOR_H

#include "DFRobot_OxygenSensor.h"
#include <vector>
#include <Arduino.h>

#define OxygenSensorAddress ADDRESS_3
#define COLLECT_NUMBER  10             // collect number, the collection range is 1-100.

/* Make sure the sensor is properly heated. A
manual calibration is required before use. There is 
a button on the deivce. Press the button with powered supplied in amibent
to get a baseline for oxygen. 


Goal is to make sure we can calcuate absolute vacuum*/

class O2Sensor{

public:
    O2Sensor();

    int setup();

    void collectData();

    float getCurrentO2Value();
    
    std::vector<float> getO2ValueArray(){
        return o2ValueArray;
    }



private:
    DFRobot_OxygenSensor o2Sensor;
    float o2Value = 0.0;
    std::vector<float> o2ValueArray;
    int collectIndex = 0;
};




#endif 