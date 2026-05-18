#include "Axis.h"
#include <TMCStepper.h>

// VACTUAL is in units of (f_clk / 2^24). At the internal 12 MHz clock this is
// ~0.7152557 microsteps/s per LSB.
static constexpr float VACTUAL_HZ_PER_LSB = 12000000.0f / 16777216.0f;


void Axis::addLimitSwitch(const char* id, int pin, const char* targetStepperId){
    limitSwitches->push_back(LimitSwitchItem{id, pin, this, targetStepperId});
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

void Axis::setStepperUart(const char* id, TMC2209Stepper* driver, int32_t vactual){
    for(StepperMotor& s : *steppers){
        if(strcmp(s.id, id) == 0){
            s.uartMode = true;
            s.tmcDriver = driver;
            s.uartVactual = vactual;
            return;
        }
    }
    Serial.print("setStepperUart: no stepper named ");
    Serial.println(id);
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

        // If a target stepper was supplied, only stop and zero that one.
        // Otherwise fall back to stopping the whole axis.
        //if (targetStepperId) {
        //    axis->stop(const_cast<char*>(targetStepperId));
        //    axis->resetPosition(const_cast<char*>(targetStepperId));
        //} else {
        //    axis->stop();
        //    axis->resetPosition();
        //}

        if(onTriggered){
            onTriggered();
        }
    });


}


void Axis::StepperMotor::init(FastAccelStepperEngine& engine) {
    if (uartMode) {
        // No STEP/DIR backend; the chip is driven via UART VACTUAL writes.
        return;
    }
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
    if (uartMode) {
        if (!tmcDriver) return;

        long delta = position - currentPosition;
        if (delta == 0) { lastMoveSign = 0; return; }

        // Map user direction onto VACTUAL sign the same way the FastAccelStepper
        // backend does for the STEP/DIR variant: DIR_REVERSE (true) keeps the
        // sign, DIR_NORMAL (false) flips it.
        int sign = (delta > 0) ? +1 : -1;
        int vactualSign = direction ? sign : -sign;
        int32_t v = vactualSign * uartVactual;

        positionBeforeMove   = currentPosition;
        targetPosition       = position;
        moveStartMs          = millis();
        expectedDurationMs   = (uint32_t)((float)labs(delta) * 1000.0f /
                                          ((float)uartVactual * VACTUAL_HZ_PER_LSB));
        moving               = true;
        lastMoveSign         = (int8_t)sign;

        Serial.print("Moving (UART) stepper ");
        Serial.print(id);
        Serial.print(" to ");
        Serial.print(position);
        Serial.print(" via VACTUAL=");
        Serial.print(v);
        Serial.print(" for ~");
        Serial.print(expectedDurationMs);
        Serial.println(" ms");

        tmcDriver->VACTUAL(v);
        return;
    }

    if(stepper){


        Serial.print("Moving stepper ");
        Serial.print(id);
        Serial.print(" to position ");
        Serial.println(position);

        long signedTarget = direction ? position : -position;
        long delta = signedTarget - stepper->getCurrentPosition();

        // lastMoveSign is read by Axis::update()'s held-switch failsafe and
        // compared against homingDistance, which is expressed in USER-INTENT
        // space (before the per-stepper DIR_REVERSE flip). If we recorded the
        // post-flip stepper-space sign here, a DIR_REVERSE stepper trying to
        // move AWAY from its limit (e.g. raising the LoadPlate off home)
        // would look like it's moving INTO the limit and get killed every
        // loop. Compute the sign in user space instead.
        long userCurrent = direction
            ? stepper->getCurrentPosition()
            : -stepper->getCurrentPosition();
        long userDelta = position - userCurrent;
        lastMoveSign = (userDelta > 0) ? +1 : (userDelta < 0 ? -1 : 0);
        (void)delta;

        stepper->moveTo(signedTarget);
    }
}

void Axis::StepperMotor::stop(){
    if (uartMode) {
        if (tmcDriver) tmcDriver->VACTUAL(0);
        if (moving) {
            unsigned long elapsed = millis() - moveStartMs;
            if (expectedDurationMs == 0 || elapsed >= expectedDurationMs) {
                currentPosition = targetPosition;
            } else {
                float frac = (float)elapsed / (float)expectedDurationMs;
                currentPosition = positionBeforeMove +
                    (long)(frac * (float)(targetPosition - positionBeforeMove));
            }
            moving = false;
        }
        lastMoveSign = 0;
        return;
    }

    if (stepper) stepper->stopMove();
    lastMoveSign = 0;
}

bool Axis::StepperMotor::isRunning(){
    if (uartMode) return moving;
    return stepper && stepper->isRunning();
}

void Axis::StepperMotor::resetPosition(){
    if (uartMode) {
        currentPosition = 0;
        positionBeforeMove = 0;
        targetPosition = 0;
        moving = false;
        if (tmcDriver) tmcDriver->VACTUAL(0);
        return;
    }
    if (stepper) stepper->setCurrentPosition(0);
}

void Axis::StepperMotor::update(){
    if (!uartMode || !moving) return;
    if (millis() - moveStartMs >= expectedDurationMs) {
        if (tmcDriver) tmcDriver->VACTUAL(0);
        currentPosition = targetPosition;
        moving = false;
    }
}

long Axis::StepperMotor::getCurrentPosition(){
    if (uartMode) {
        if (!moving) return currentPosition;
        unsigned long elapsed = millis() - moveStartMs;
        if (expectedDurationMs == 0) return currentPosition;
        if (elapsed >= expectedDurationMs) return targetPosition;
        float frac = (float)elapsed / (float)expectedDurationMs;
        return positionBeforeMove +
               (long)(frac * (float)(targetPosition - positionBeforeMove));
    }
    return stepper ? stepper->getCurrentPosition() : 0;
}

Axis::Axis(long homingDistance)
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

void Axis::setHome(bool value, char* id){
    if(id){
        for(StepperMotor& stepper : *steppers){
            if(strcmp(stepper.id, id) == 0){
                stepper.isHoming = value;
                break;
            }
        }
        return;
    }

    // Apply to all steppers when no id is supplied. Previously this branch
    // hard-coded `false`, which made `setHome(true)` a silent no-op and
    // meant getHome() could never report the axis as homed via the
    // broadcast call. Honor the caller-supplied value instead.
    for(StepperMotor& stepperMotor : *steppers){
        stepperMotor.isHoming = value;
    }
}


bool Axis::getHome(char* id){
    if(id){
        for(StepperMotor& stepper : *steppers){
            if(strcmp(stepper.id, id) == 0){
                return stepper.isHoming;
            }
        }
        return false;
    }

    // Return true only if all motors are at home
    for(StepperMotor& stepperMotor : *steppers){
        if(!stepperMotor.isHoming){
            return false;
        }
    }
    return true;
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
        stepper.isHoming = true;
    }


}


void Axis::stop(char* id){ // An optional id can be supplied to stop a single stepper
    if(id){
        for(StepperMotor& stepper : *steppers){
            if(strcmp(stepper.id, id) == 0){
                stepper.stop();
                return;
            }
        }
    }

    for(StepperMotor& stepper : *steppers){
        stepper.stop();
    }

}

void Axis::resetPosition(char* id){ // An optional id can be supplied to reset the position of a single stepper

    if(id){
        for(StepperMotor& stepper : *steppers){
            if(strcmp(stepper.id, id) == 0){
                stepper.resetPosition();
                break;
            }
        }
        return;
    }

    // Reset all positions of all motors
    for(StepperMotor& stepperMotor : *steppers){
        stepperMotor.resetPosition();
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
        // Always fire on rising edge. Edge detection inside checkAndCallback
        // (!lastPressed) is enough to prevent the callback from re-firing
        // every loop while the switch is mechanically held, so we don't need
        // to also gate on getHome(). Gating on getHome() previously caused
        // the reset/limit callback to be suppressed during normal MOVEBAR
        // swipes and during manual switch tests, since isHoming is only set
        // by Axis::home().
        limitSwitch.limitSwitch->checkAndCallback(true);
    }

    // Held-switch failsafe: the edge-triggered callback above only fires on a
    // not-pressed -> pressed transition. If a move is commanded while the
    // switch is already held (e.g. you hit "home" twice, or the axis was
    // resting on the switch at boot), the rising edge never happens and the
    // motor would otherwise drive into the limit.
    //
    // Direction-aware: only stop a move that is heading INTO the limit
    // (same sign as the axis homingDistance). A move heading AWAY from the
    // limit is exactly what the user wants when leaving home, so leave it
    // alone. This is what allows commanding a move off a held switch.
    int8_t homingSign = (homingDistance > 0) ? +1 : (homingDistance < 0 ? -1 : 0);
    for(LimitSwitchItem& limitSwitch : *limitSwitches){
        if (!limitSwitch.limitSwitch->read()) continue;

        // Find the protected stepper(s) and check direction of travel.
        bool stopThisOne = false;
        for(StepperMotor& s : *steppers){
            if (limitSwitch.targetStepperId &&
                strcmp(limitSwitch.targetStepperId, s.id) != 0) continue;
            if (!s.isRunning()) continue;
            if (s.lastMoveSign == 0) continue;
            if (s.lastMoveSign == homingSign) {
                stopThisOne = true;
                break;
            }
        }
        if (stopThisOne && limitSwitch.onTriggered) {
            limitSwitch.onTriggered();
        }
    }

    for(StepperMotor& stepperMotor : *steppers){
        stepperMotor.update();
    }
    
    return this->isRunning();
    
}

bool Axis::isLimitSwitchPressed(const char* id){
    if (!id) return false;
    for (LimitSwitchItem& sw : *limitSwitches){
        if (strcmp(sw.id, id) == 0){
            return sw.limitSwitch->read();
        }
    }
    return false;
}

bool Axis::isRunning(char* id){
    if (id)
    {
        for(StepperMotor& stepperMotor : *steppers){
            if(strcmp(stepperMotor.id, id) == 0){
                if(stepperMotor.isRunning()){
                    return true;
                }
                break;
            }
        }
    }
    

    for(StepperMotor& stepperMotor : *steppers){
        if(stepperMotor.isRunning()){
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
        Serial.print(stepper.uartMode ? " [UART] " : " ");
        Serial.print("current position: ");
        if(stepper.uartMode || stepper.stepper){
            Serial.println(stepper.getCurrentPosition());
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