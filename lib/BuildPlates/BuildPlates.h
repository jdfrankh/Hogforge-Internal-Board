#pragma once
#include <StepperMotor.h>
#include <LimitSwitch.h>


class LoadingPlates{

    public:

    bool isHome = false, homing = false, atPosition = false;
    const int BUILD_PLATE_HIEGHT = 52000; //62000
    const int LOAD_PLATE_HIEGHT = BUILD_PLATE_HIEGHT - 3000;
    const int TO_ABOVE_LIP = 5000;



    LoadingPlates(StepperMotor* _BuildPlate, StepperMotor* _LoadPlate, LimitSwitch* _loadPlateSwitch, LimitSwitch* _buildPlateSwitch);

    StepperMotor* BuildPlate = nullptr;
    StepperMotor* LoadPlate = nullptr;
    LimitSwitch* loadPlateSwitch = nullptr;
    LimitSwitch* buildPlateSwitch = nullptr;
    
    void init();

    void homeAll();

    void prepForPrint(int printHeight = 0); // Set build plate to high and load plate to low

    bool update();

    void setSpeed(int _speed);

    void miniStep(int stepSize, bool direction );

    void liftBothPlates();

    void liftAboveLip();

    private:

        

        int speed = StepperMotor::FAST;

        int smallStep = 325; //65000 / 200 for 200 layers
};