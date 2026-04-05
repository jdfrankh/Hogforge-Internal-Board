#include "Axis.h"










void Axis::addLimitSwitch(const char* id, int pin){
    limitSwitches->push_back(LimitSwitchItem{id, pin, this});
}

void Axis::removeLimitSwitch(const char* id){
    for(LimitSwitchItem& limitSwitch : *limitSwitches){
        if(strcmp(limitSwitch.id, id) == 0){
            limitSwitches->erase(std::remove_if(
                limitSwitches->begin(), limitSwitches->end(), [&](LimitSwitchItem& l){
                    return strcmp(l.id, id) == 0;
                }), limitSwitches->end());
            break;
        }
    }
}

void Axis::setLimitSwitchCallback(const char* id, std::function<void()> callback){
    for(LimitSwitchItem& limitSwitch : *limitSwitches){
        if(strcmp(limitSwitch.id, id) == 0){
            limitSwitch.onTriggered = callback;
            break;
        }
    }
}

void Axis::addStepper(const char* id,std::vector<int> pins, int speed, bool direction){
    
    steppers->push_back(StepperMotor{id, speed, direction, pins});

}

void Axis::removeStepper(const char* id){
    for(StepperMotor& stepper : *steppers){
        if(strcmp(stepper.id, id) == 0){ // Check to see if there is any mismatch with the id
            steppers->erase(std::remove_if(
                steppers->begin(), steppers->end(), [&](StepperMotor& s){ 
                    return strcmp(s.id, id) == 0; }), steppers->end());
            break;
        }
    }
}




void Axis::LimitSwitchItem::init(){
    limitSwitch->init();

    limitSwitch->setCallback([this](){
       // Serial.print("Limit switch triggered: ");
       // Serial.println(id);

        axis->stop();
        axis->resetPosition();

        if(onTriggered){
            onTriggered();
        }
    });


}


void Axis::StepperMotor::init(FastAccelStepperEngine& engine) {
    if (pins.size() < 3) {
        Serial.println("Not enough pins provided to initialize stepper");

        return;
    }

    stepper = engine.stepperConnectToPin(pins[STEP_PIN]);
    if (!stepper) {
        Serial.println("Failed to connect stepper to pin ");
        return;
    }

    stepper->setDirectionPin(pins[DIRECTION_PIN]);
    stepper->setEnablePin(pins[ENABLE_PIN]);
    stepper->setAutoEnable(true);
    stepper->setSpeedInHz(speed);
    stepper->setAcceleration(50000);


}

void Axis::StepperMotor::moveTo(long position){
    if(stepper){
     //   Serial.print("Moving stepper ");
     //   Serial.print(id);
     //   Serial.print(" to position ");
     //   Serial.println(position);

        direction ? stepper->moveTo(position) : stepper->moveTo(-position);
    }
}

Axis::Axis(int homingDistance)
    : steppers(new std::vector<StepperMotor>()),
      limitSwitches(new std::vector<LimitSwitchItem>()),
      homingDistance(homingDistance) {
}


void Axis::init(){
    this->engine = new FastAccelStepperEngine();

    this->engine->init();

    for(StepperMotor& stepper : *steppers){
        stepper.init(*engine);
    }
    for(LimitSwitchItem& limitSwitch : *limitSwitches){
        limitSwitch.init();
    }


}


void Axis::home(){

    for (StepperMotor& stepper : *steppers){
        stepper.moveTo(homingDistance);
    }


}


void Axis::stop(){
    for(StepperMotor& stepper : *steppers){
        if(stepper.stepper){
            stepper.stepper->stopMove();
        }
    }

}

void Axis::resetPosition(){
    for(StepperMotor& stepperMotor : *steppers){
        if(stepperMotor.stepper){
            stepperMotor.stepper->setCurrentPosition(0);
        }
    }
}

void Axis::update(){
    for(LimitSwitchItem& limitSwitch : *limitSwitches){
        limitSwitch.limitSwitch->checkAndCallback();
    }
    delay(1);
}

void Axis::printLimitSwitchStates(){
    for(LimitSwitchItem& limitSwitch : *limitSwitches){
        Serial.print("Limit Switch ");
        Serial.print(limitSwitch.id);
        Serial.print(" state: ");
        Serial.println(limitSwitch.limitSwitch->read());
    }
}

void Axis::moveAxis(long position){
    for(StepperMotor& stepper : *steppers){
        stepper.moveTo(position);
    }
}