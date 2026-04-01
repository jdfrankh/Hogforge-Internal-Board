#include "LimitSwitch.h"


volatile bool LimitSwitch::triggered = false;


LimitSwitch::LimitSwitch(uint8_t pin){
    this->pin = pin;
}


void LimitSwitch::init() {
    Serial.println("Initalizing Limit Switch...");
    pinMode(pin, INPUT);
 
}

bool LimitSwitch::isPressed() {
    this->triggered = false; // Reset the triggered state after checking
    return digitalRead(pin) == HIGH;
}


