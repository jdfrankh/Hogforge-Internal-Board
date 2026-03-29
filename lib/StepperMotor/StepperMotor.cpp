#include "StepperMotor.h"

// Construct a driver-based AccelStepper with optional microstep control pins.
StepperMotor::StepperMotor(uint8_t enPin_, uint8_t dirPin_, uint8_t stepPin_, uint8_t ms1Pin_, uint8_t ms2Pin_)
    : stepper{AccelStepper::DRIVER, stepPin_, dirPin_},
      enPin{enPin_},
      dirPin{dirPin_},
      stepPin{stepPin_},
      ms1Pin{ms1Pin_},
      ms2Pin{ms2Pin_} {}

// Convenience ctor when you don't care about microstep control pins.
StepperMotor::StepperMotor(uint8_t enPin_, uint8_t dirPin_, uint8_t stepPin_)
    : StepperMotor(enPin_, dirPin_, stepPin_, 0, 0) {}

void StepperMotor::init() {
    pinMode(enPin, OUTPUT);
    // Your wiring expects HIGH here; keep behaviour identical.
    digitalWrite(enPin, HIGH);

    if (ms1Pin) {
        pinMode(ms1Pin, OUTPUT);
        digitalWrite(ms1Pin, LOW);
    }
    if (ms2Pin) {
        pinMode(ms2Pin, OUTPUT);
        digitalWrite(ms2Pin, LOW);
    }

    stepper.setEnablePin(enPin);
    stepper.setPinsInverted(invertDirection, false, true);
    stepper.enableOutputs();
    applySpeed();
}

void StepperMotor::invertDrive(bool invert) {
    invertDirection = invert;
    stepper.setPinsInverted(invertDirection, false, true);
    applySpeed();
}

void StepperMotor::home(int height) {
    resetHome = true;
    setDirection(invertDirection ? 1 : 0);
    setDistance(height);
}

void StepperMotor::setDirection(int dir) {
    directionSign = dir ? 1 : -1;
    stepper.setSpeed(directionSign * speedStepsPerSecond);
}

bool StepperMotor::moveComplete() {
    bool done = (stepper.distanceToGo() == 0);
    if (done && resetHome) {
        resetHome = false;
        stepper.setCurrentPosition(0);
    }
    return done;
}

void StepperMotor::move() {
    Serial.println("Moving to distance: " + String(toDistance) + " from current: " + String(stepper.currentPosition()));
    while (!moveComplete()) {
        stepper.run();
        delayMicroseconds(1);
        
    }
    Serial.println("Move complete. Current position: " + String(stepper.currentPosition()));
}

void StepperMotor::setSpeed(int microsecondsPerStep) {
    if (microsecondsPerStep <= 0) {
        microsecondsPerStep = FAST;
    }
    speedSettingUs = microsecondsPerStep;
    applySpeed();
}

int16_t StepperMotor::getCurrentDistance() {
    return static_cast<int16_t>(stepper.currentPosition());
}

void StepperMotor::setCurrentDistance(int currDistance) {
    stepper.setCurrentPosition(currDistance);

    move();


}

void StepperMotor::setDistance(int distance) {
    toDistance = distance;
    hasTarget = true;
    stepper.move(directionSign * distance);
    move();
}

int StepperMotor::getToDistance() {
    return toDistance;
}


float StepperMotor::toStepsPerSecond(int microsecondsPerStep) {
    if (microsecondsPerStep <= 0) {
        return 0.0f;
    }
    return 1000000.0f / static_cast<float>(microsecondsPerStep);
}

void StepperMotor::applySpeed() {
    speedStepsPerSecond = toStepsPerSecond(speedSettingUs);
    stepper.setMaxSpeed(speedStepsPerSecond);
    stepper.setAcceleration(speedStepsPerSecond * 2.0f);
    stepper.setSpeed(directionSign * speedStepsPerSecond);
}
