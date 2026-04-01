#pragma once
#include <StepperMotor.h>
#include <LimitSwitch.h>


class LoaderBar {

    public:

    LoaderBar(StepperMotor* _LeftArm, StepperMotor* _RightArm, LimitSwitch* _LoadSwitch, LimitSwitch* _ResetSwitch);

    void init();

    void homeAll();

    void stopAll();

    void prepForPrint();

    void setSpeed(int _speed);

    void startSwipe();

    void setSwipe(bool direction);



    bool homing = true;
    bool headingHome = false;

    bool reachedEnd = false;

    StepperMotor* LeftArm = nullptr;
    StepperMotor* RightArm = nullptr;
    LimitSwitch* LoadLimit = nullptr;
    LimitSwitch* RestLimit = nullptr;


    bool manualSwipe = false;
    private:

    const int travelDistance =10000; //5500
    int speed = StepperMotor::FAST;


    
 

    

};
