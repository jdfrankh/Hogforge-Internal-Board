// Reads the limit switch 10 times and returns the most occurring value

#include "LimitSwitch.h"


volatile bool LimitSwitch::triggered = false;


LimitSwitch::LimitSwitch(uint8_t pin){
    this->pin = pin;
}


void LimitSwitch::init() {
    Serial.println("Initalizing Limit Switch...");
    pinMode(pin, INPUT_PULLUP);
 
}

bool LimitSwitch::isPressed() {
    this->triggered = digitalRead(pin) == LOW;
    return this->triggered;
}

bool LimitSwitch::read(){
    /*
    int trueCount = 0;
    int falseCount = 0;
    for (int i = 0; i < 20; ++i) {
        if (isPressed()) {
            trueCount++;
        } else {
            falseCount++;
        }
    }
        */
    
    //Serial.print("Limit Switch "); Serial.print(pin); Serial.print(" state: "); Serial.println(isPressed());
    return isPressed();
}

void LimitSwitch::setCallback(std::function<void()> callback) {
    userCallback = callback;
}

void LimitSwitch::checkAndCallback() {
    if (read() && userCallback) {
        userCallback();
    }
}


