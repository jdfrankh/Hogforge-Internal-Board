// StepperMotor: thin wrapper around AccelStepper tuned for your ESP32 belt axis.
//
// Design:
//  - Uses DRIVER mode: separate STEP and DIR pins plus an enable pin.
//  - Speed is expressed as MICROSECONDS PER STEP (smaller = faster).
//  - updateStepper() must be called frequently from loop()/periodic() to generate steps.

#pragma once

#include <Arduino.h>
#include <AccelStepper.h>

class StepperMotor {
public:
    // Handy presets for microseconds-per-step. Tune to your mechanics/driver.
    static constexpr int FAST      = 150;
    static constexpr int MEDIUM    = 300;
    static constexpr int SLOW      = 500;
    static constexpr int SUPERSLOW = 700;

    static constexpr int INITPOS = 2000; // Amount of steps to get out of inital poisition

    static constexpr int TURNSTEPS = 1500; // Inital steps to match corner movements

    static constexpr int LEFTRIGHTMOVE = 500; // Adjust to be moved left or giht

    StepperMotor(uint8_t enPin, uint8_t dirPin, uint8_t stepPin, uint8_t ms1Pin, uint8_t ms2Pin);
    StepperMotor(uint8_t enPin, uint8_t dirPin, uint8_t stepPin);

    // Must be called once at startup after construction.
    void init();

    // dir != 0 => forward, dir == 0 => backward (relative to wiring).
    void setDirection(int dir);

    // Speed in MICROSECONDS PER STEP.
    // Example: 150 = very fast, 650 = medium, 1000 = slow.
    void setSpeed(int microsecondsPerStep);

    // Relative move in steps (sign comes from setDirection()).
    void setDistance(int distance);

    void setCurrentDistance(int currDistance);

    // True when there is no remaining distance to go.
    bool moveComplete();

    int16_t getCurrentDistance();
    int getToDistance();
    void home(int height);

    // Invert logical direction (useful if wiring is flipped).
    void invertDrive(bool invert);

    void move();

private:
    AccelStepper stepper;

    uint8_t enPin  = 0;
    uint8_t dirPin = 0;
    uint8_t stepPin = 0;
    uint8_t ms1Pin = 0;
    uint8_t ms2Pin = 0;

    int toDistance = 0;
    bool invertDirection = false;
    bool resetHome = false;
    bool hasTarget = false;

    int speedSettingUs = MEDIUM; // microseconds per step
    int directionSign = 1;
    float speedStepsPerSecond = 0.0f;

    static float toStepsPerSecond(int microsecondsPerStep);
    void applySpeed();
};


