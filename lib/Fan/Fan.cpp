
#include "Fan.h"




Fan::Fan(char* id, int pwm1, int pwm2){
    this->id = id;
    _pwm1 = pwm1;
    _pwm2 = pwm2;
    
}


void Fan::setup(){
    pinMode(_pwm1, OUTPUT);
    pinMode(_pwm2, OUTPUT);
    setSpeed(Speeds::OFF);

}

void Fan::setSpeed(Speeds speed, Direction direction){

    if (speed == Speeds::OFF) {
        stop();
        return;
    }

    int speedValue = static_cast<int>(speed);
    if(direction == Direction::FORWARD){
        analogWrite(_pwm1, speedValue);
        analogWrite(_pwm2, 0);
    } else {
        analogWrite(_pwm1, 0);
        analogWrite(_pwm2, speedValue);
    }
    _currentSpeed = speedValue;
}


void Fan::setSpeed(int speed){
    setSpeed(static_cast<Speeds>(speed), static_cast<Direction>(_currentDirection));

}

void Fan::stop( ){
    
    if(_currentBreakType == static_cast<int>(BreakType::BRAKE)){
        analogWrite(_pwm1, 255);
        analogWrite(_pwm2, 255);
    } else {
        analogWrite(_pwm1, 0);
        analogWrite(_pwm2, 0);
    }
    _currentSpeed = 0;

}