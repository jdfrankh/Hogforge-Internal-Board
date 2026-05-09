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


        Serial.print("Moving stepper ");
        Serial.print(id);
        Serial.print(" to position ");
        Serial.println(position);


        direction ? stepper->moveTo(position) : stepper->moveTo(-position);
    }
}

Axis::Axis(int homingDistance)
    : steppers(new std::vector<StepperMotor>()),
      limitSwitches(new std::vector<LimitSwitchItem>()),
      homingDistance(homingDistance) {
}


void Axis::init(){
    // FastAccelStepper keeps a process-global StepperQueue array (`fas_queue[]`)
    // and FastAccelStepperEngine::init() unconditionally calls _initVars() on
    // every entry of that array.  _initVars() zeroes driver_data, which is the
    // pointer the MCPWM/PCNT backend dereferences from inside StepperTask
    // (isReadyForCommands_mcpwm_pcnt -> mapping->mcpwm_unit).  If a second
    // FastAccelStepperEngine is ever init()'d, it wipes the queues that the
    // first engine has already bound to real MCPWM units, and the first
    // engine's StepperTask null-derefs on the next motion command
    // (LoadProhibited @ EXCVADDR=0x00000000).
    //
    // The library therefore requires exactly ONE engine per process.  Use a
    // function-local static so every Axis instance shares the same engine and
    // engine.init() is only ever called once.
    static FastAccelStepperEngine sharedEngine;
    static bool sharedEngineInited = false;
    if (!sharedEngineInited) {
        sharedEngine.init();
        sharedEngineInited = true;
    }
    engine = &sharedEngine;

    for(LimitSwitchItem& limitSwitch : *limitSwitches){
        limitSwitch.init();
    }

    for(StepperMotor& stepper : *steppers){
        stepper.init(*engine);
    }


}


void Axis::home(){

    #if DEBUGMODELOOP
        Serial.println("Homing Axis...");
    #endif

    // Not source of crash
    //this->setSpeed(Speed::FAST);

    for (StepperMotor& stepper : *this->steppers){
        Serial.print("Homing stepper ");
        Serial.println(stepper.id);
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

void Axis::setSpeed(Speed speed){
    for(StepperMotor& stepper : *steppers){
        stepper.speed = speed;
        if(stepper.stepper){
            stepper.stepper->setSpeedInHz(speed);
        }
    }
}

bool Axis::update(){
    for(LimitSwitchItem& limitSwitch : *limitSwitches){
        limitSwitch.limitSwitch->checkAndCallback();
    }
    
    return this->isRunning();
    
}

bool Axis::isRunning(){

    for(StepperMotor& stepperMotor : *steppers){
        if(stepperMotor.stepper && stepperMotor.stepper->isRunning()){
            return true;
        }
    }
    return false;
}

void Axis::printLimitSwitchStates(){
    for(LimitSwitchItem& limitSwitch : *limitSwitches){
        Serial.print("Limit Switch ");
        Serial.print(limitSwitch.id);
        Serial.print(" state: ");
        Serial.println(limitSwitch.limitSwitch->read());
    }
}

void Axis::printStepperStates(){
    for(StepperMotor& stepper : *steppers){
        Serial.print("Stepper ");
        Serial.print(stepper.id);
        Serial.print(" current position: ");
        if(stepper.stepper){
            Serial.println(stepper.stepper->getCurrentPosition());
        }
        else{
            Serial.println("Stepper not initialized");
        }
    }
}

void Axis::moveAxis(long position){
    for(StepperMotor& stepper : *steppers){
        stepper.moveTo(position);
    }
}

void Axis::moveSingleStepper(char*id , long position){
    for(StepperMotor& stepper : *steppers){
        
        if(strcmp(stepper.id, id) == 0){
            stepper.moveTo(position);
            break;
        }
    }

}