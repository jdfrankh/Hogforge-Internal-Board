#include "LoaderBar.h"


LoaderBar::LoaderBar(uint8_t leftStepPin_, uint8_t leftDirPin_, uint8_t leftEnPin_,
                     uint8_t rightStepPin_, uint8_t rightDirPin_, uint8_t rightEnPin_,
                     LimitSwitch* _LoadSwitch, LimitSwitch* _ResetSwitch)
    : leftStepPin(leftStepPin_), leftDirPin(leftDirPin_), leftEnPin(leftEnPin_),
      rightStepPin(rightStepPin_), rightDirPin(rightDirPin_), rightEnPin(rightEnPin_),
      LoadLimit(_LoadSwitch), ResetLimit(_ResetSwitch) {}

void LoaderBar::init(){

    LoadLimit->init();
    ResetLimit->init();

    engine.init();

    LeftArm = engine.stepperConnectToPin(leftStepPin);
    if (LeftArm) {
        LeftArm->setDirectionPin(leftDirPin);
        LeftArm->setEnablePin(leftEnPin);
        LeftArm->setAutoEnable(true);
        LeftArm->setSpeedInHz(3000);
        LeftArm->setAcceleration(5000);
    }

    RightArm = engine.stepperConnectToPin(rightStepPin);
    if (RightArm) {
        RightArm->setDirectionPin(rightDirPin);
        RightArm->setEnablePin(rightEnPin);
        RightArm->setAutoEnable(true);
        RightArm->setSpeedInHz(3000);
        RightArm->setAcceleration(5000);
    }

    LoadLimit->setCallback([this]() {
        Serial.println("Load limit triggered! Stopping motors.");
        stopAll();
    });

    ResetLimit->setCallback([this]() {
        Serial.println("Reset limit triggered! Stopping motors.");
        stopAll();
    });
}

void LoaderBar::homeAll(){
    if (LeftArm) LeftArm->moveTo(100000);
    if (RightArm) RightArm->moveTo(-100000);

    // Wait for both to finish
    while ((LeftArm && LeftArm->isRunning()) || (RightArm && RightArm->isRunning())) {
        readLimitSwitches();
        delay(1);
    }

    Serial.println("Homed All");
}


void LoaderBar::setSwipe(bool direction){

}

void LoaderBar::stopAll(){
    if (LeftArm) LeftArm->forceStop();
    if (RightArm) RightArm->forceStop();
}


void LoaderBar::setSpeed(int _speed){
    speed = _speed;
    if (LeftArm) LeftArm->setSpeedInHz(speed);
    if (RightArm) RightArm->setSpeedInHz(speed);
}

void LoaderBar::startSwipe(){

}

void LoaderBar::readLimitSwitches(){
    LoadLimit->checkAndCallback();
    ResetLimit->checkAndCallback();
}
