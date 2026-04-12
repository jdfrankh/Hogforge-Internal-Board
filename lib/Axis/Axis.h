#ifndef AXIS_H
#define AXIS_H

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <vector>
#include <functional>
#include <LimitSwitch.h>

class Axis {
public:
    enum StepperPins{
        STEP_PIN,
        DIRECTION_PIN,
        ENABLE_PIN

    };

    enum Speed {
        SUPERSLOW = 1000,
        SLOW = 2000,
        FAST = 4000,
        SUPERFAST = 6000
    };

    struct StepperMotor{

        enum Speed {
            SUPERSLOW = 1000,
            SLOW = 2000,
            FAST = 4000,
            SUPERFAST = 6000

        };

        enum Direction{
            DIR_NORMAL = 0,
            DIR_REVERSE = 1
        };

        StepperMotor(const char* id, int speed, bool direction, std::vector<int> pins) : id(id), speed(speed), direction(direction), pins(pins) {}

        void init(FastAccelStepperEngine& engine);

        const char* id;
        FastAccelStepper* stepper = nullptr;
        int speed;
        bool direction;
        std::vector<int> pins;

        void moveTo(long position);

    };

    struct LimitSwitchItem{

        LimitSwitchItem(const char* id, int pin, Axis* axis) : id(id), pin(pin), limitSwitch(new LimitSwitch{static_cast<uint8_t>(pin)}), axis(axis) {
        }


        void init();

        const char* id;
        LimitSwitch* limitSwitch;
        static volatile bool state;
        int pin;
        Axis* axis;
        std::function<void()> onTriggered = nullptr;
    };

    FastAccelStepperEngine* engine = nullptr;

    void addStepper(const char* id ,std::vector<int> pins, int speed, bool direction);

    void removeStepper(const char* id);
    void addLimitSwitch(const char* id, int pin);
    void removeLimitSwitch(const char* id);
    void setLimitSwitchCallback(const char* id, std::function<void()> callback);


    Axis(int homingDistance);
    void init();

    void home();
    void resetPosition();
    bool update(); // Return to see if any command is running
    void moveAxis(long position);
    void moveSingleStepper(char* id, long position);
    void printLimitSwitchStates();
    void printStepperStates();
    void setSpeed(Speed speed);
    void setAcceleration(int acceleration); // Change to struct
    void stop();
    bool isRunning();

private:
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enPin;
    std::vector<StepperMotor>* steppers;
    std::vector<LimitSwitchItem>* limitSwitches;
    std::vector<long> positions;

    long homingDistance = 0;



    //FastAccelStepper* stepper = nullptr;

};

#endif // AXIS_H
