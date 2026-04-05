#pragma once


#include <Arduino.h>


class LimitSwitch {

    public:

        LimitSwitch(uint8_t pin);

        void init();

        bool read();

        bool isPressed();

        void setCallback(std::function<void()> callback);
        void checkAndCallback();

        #define LIMIT_SWITCH_TRIGGERED 0
        #define LIMIT_SWITCH_NOT_TRIGGERED 1

        static volatile bool triggered; // Cannot initalize static variables

    private:

        uint8_t pin = 0;
        std::function<void()> userCallback = nullptr;

    

};