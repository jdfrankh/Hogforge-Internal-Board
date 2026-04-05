#include "BuildPlates.h"


LoadingPlates::LoadingPlates(AccelStepper _BuildPlate, AccelStepper _LoadPlate, LimitSwitch* _loadPlateSwitch, LimitSwitch* _buildPlateSwitch){

    BuildPlate = _BuildPlate;
    LoadPlate = _LoadPlate;
    loadPlateSwitch = _loadPlateSwitch;
    buildPlateSwitch = _buildPlateSwitch;

}



void LoadingPlates::init(){


}

void LoadingPlates::homeAll(){



}

void LoadingPlates::prepForPrint(int printHeight){



}


void LoadingPlates::setSpeed(int _speed){

}

bool LoadingPlates::update(){
    
return false;


}


// Set the distance to be a minature step as according the .h variable
void LoadingPlates::miniStep(int stepSize, bool direction){


}

void LoadingPlates::liftBothPlates(){

}

void LoadingPlates::liftAboveLip(){

}

void LoadingPlates::readLimitSwitches(){


}