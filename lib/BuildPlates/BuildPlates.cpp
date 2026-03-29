#include "BuildPlates.h"


LoadingPlates::LoadingPlates(StepperMotor* _BuildPlate, StepperMotor* _LoadPlate){

    BuildPlate = _BuildPlate;
    LoadPlate = _LoadPlate;

}



void LoadingPlates::init(){

    BuildPlate->init();
    LoadPlate->init();

    setSpeed(speed);
}

void LoadingPlates::homeAll(){

    BuildPlate->setDirection(1);
    LoadPlate->setDirection(1);
    BuildPlate->setDistance(BUILD_PLATE_HIEGHT);
    LoadPlate->setDistance(LOAD_PLATE_HIEGHT);

    homing = true;

}

void LoadingPlates::prepForPrint(int printHeight){

    BuildPlate->setSpeed(StepperMotor::FAST);
    LoadPlate->setSpeed(StepperMotor::FAST);

    uint64_t heightDiff = printHeight * 1000;

    Serial.printf("Print hieght: %d", heightDiff);
    Serial.println();

    LoadPlate->setDistance(LOAD_PLATE_HIEGHT - (heightDiff));
    BuildPlate->setDistance(BUILD_PLATE_HIEGHT);
    BuildPlate->setDirection(1);
    LoadPlate->setDirection(1);
   // LoadPlate->setDistance(0);

}


void LoadingPlates::setSpeed(int _speed){
    speed = _speed;
    BuildPlate->setSpeed(_speed);
    LoadPlate->setSpeed(_speed);
}

bool LoadingPlates::update(){
    



}


// Set the distance to be a minature step as according the .h variable
void LoadingPlates::miniStep(int stepSize, bool direction){

    stepSize = stepSize * 10;
    
    if(direction){
        BuildPlate->setDirection(0);
        LoadPlate->setDirection(1);
    }
    else{
        BuildPlate->setDirection(1);
        LoadPlate->setDirection(0);
    }

    BuildPlate->setDistance(stepSize );
    
    LoadPlate->setDistance(stepSize); 

}

void LoadingPlates::liftBothPlates(){
    BuildPlate->setDirection(1);
    LoadPlate->setDirection(1);

    BuildPlate->setDistance(BUILD_PLATE_HIEGHT);
    LoadPlate->setDistance(LOAD_PLATE_HIEGHT);
}

void LoadingPlates::liftAboveLip(){
    BuildPlate->setDirection(1);
    LoadPlate->setDirection(1);

    BuildPlate->setDistance(BUILD_PLATE_HIEGHT + TO_ABOVE_LIP);
    LoadPlate->setDistance(LOAD_PLATE_HIEGHT + TO_ABOVE_LIP);
}