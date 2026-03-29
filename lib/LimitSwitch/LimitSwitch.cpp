#include "LimitSwitch.h"


volatile bool LimitSwitch::triggered = false;


LimitSwitch::LimitSwitch(uint8_t pin){
    this->pin = pin;
}

// Define the static userCallback variable as std::function<void()>
#include <functional>
std::function<void()> LimitSwitch::userCallback = nullptr;

void LimitSwitch::init() {
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), handleInterrupt, CHANGE);
}

void LimitSwitch::handleInterrupt() {
    triggered = true;
    if (userCallback) {
        userCallback();
    }
}

bool LimitSwitch::isPressed() {
    this->triggered = false; // Reset the triggered state after checking
    return digitalRead(pin) == HIGH;
}

void LimitSwitch::setCallback(std::function<void()> cb) {
    userCallback = cb;
}


