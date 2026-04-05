#pragma once
#include <LimitSwitch.h>
#include <FastAccelStepper.h>
#include <Axis.h>

class LoaderBar {

    public:

    LoaderBar(uint8_t leftStepPin, uint8_t leftDirPin, uint8_t leftEnPin,
              uint8_t rightStepPin, uint8_t rightDirPin, uint8_t rightEnPin,
              LimitSwitch* _LoadSwitch, LimitSwitch* _ResetSwitch);

    void init();

    void homeAll();

    void stopAll();

    void prepForPrint();

    void setSpeed(int _speed);

    void startSwipe();

    void setSwipe(bool direction);

    void readLimitSwitches();

    bool homing = true;
    bool headingHome = false;

    bool reachedEnd = false;

    FastAccelStepper* LeftArm = nullptr;
    FastAccelStepper* RightArm = nullptr;
    LimitSwitch* LoadLimit = nullptr;
    LimitSwitch* ResetLimit = nullptr;

    bool manualSwipe = false;
    private:

    FastAccelStepperEngine engine;

    uint8_t leftStepPin, leftDirPin, leftEnPin;
    uint8_t rightStepPin, rightDirPin, rightEnPin;

    const int travelDistance = 10000;
    int speed = 1000;

};
