#pragma once


#include <Arduino.h>


class LimitSwitch {

    public:

        LimitSwitch(uint8_t pin);

        void init();

        static void setCallback(std::function<void()> cb);
        static void handleInterrupt();

        bool isPressed();

    

        static volatile bool triggered; // Cannot initalize static variables

    private:

        uint8_t pin = 0;
        static std::function<void()> userCallback;

    

};