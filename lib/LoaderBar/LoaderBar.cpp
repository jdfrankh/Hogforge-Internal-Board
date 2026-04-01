#include "LoaderBar.h"


LoaderBar::LoaderBar(StepperMotor* _LeftArm, StepperMotor* _RightArm, LimitSwitch* _LoadSwitch, LimitSwitch* _ResetSwitch ){

    LeftArm = _LeftArm;
    RightArm = _RightArm;
    LoadLimit = _LoadSwitch;
    RestLimit = _ResetSwitch;

}

void LoaderBar::init(){
    LeftArm->init();
    //delay(50); 
    RightArm->init();
    LoadLimit->init();
    RestLimit->init();


    setSpeed(speed);

    RightArm->invertDrive(false); // was true
    LeftArm->invertDrive(true);

}

void LoaderBar::setSwipe(bool direction){

    if(direction){
        LeftArm->setDistance(travelDistance);
        RightArm->setDistance(travelDistance);
    }
    else{
        LeftArm->setDistance(-travelDistance);
        RightArm->setDistance(-travelDistance);

    }
    //LeftArm->setDistance(travelDistance);
    //RightArm->setDistance(travelDistance);


}

void LoaderBar::stopAll(){
    LeftArm->setDistance(0);
    RightArm->setDistance(0);
}


void LoaderBar::setSpeed (int _speed){

    speed = _speed;
    LeftArm->setSpeed(_speed);
    RightArm->setSpeed(_speed);

}

void LoaderBar::startSwipe(){
    Serial.printf("Starting swipe!: %d : ", travelDistance);


    setSwipe(1);
    
}
