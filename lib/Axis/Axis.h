#ifndef AXIS_H
#define AXIS_H

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <vector>
#include <functional>
#include <LimitSwitch.h>

// Forward declaration; full include only needed in Axis.cpp.
class TMC2209Stepper;

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
            SLOW = 1000,
            MEDIUM = 2000,
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

    
        // ---- UART / TMC2209 dead-reckoning mode ----
        // When uartMode is true, this stepper has no working STEP/DIR pins and
        // is driven by writing VACTUAL over UART. Position is integrated from
        // the commanded VACTUAL and elapsed wall-clock time (open loop).
        bool uartMode = false;
        TMC2209Stepper* tmcDriver = nullptr;
        int32_t uartVactual = 0;          // |VACTUAL| used while moving (LSB)
        long currentPosition = 0;         // user-frame microstep position
        long positionBeforeMove = 0;
        long targetPosition = 0;
        unsigned long moveStartMs = 0;
        unsigned long expectedDurationMs = 0;
        bool moving = false;

        // Sign of the most recently commanded move (+1, -1, or 0 if idle).
        // Used by the held-switch failsafe to decide whether the active move
        // is heading INTO a limit (same sign as Axis::homingDistance) and
        // should be stopped, or AWAY from it (opposite sign) and allowed.
        int8_t lastMoveSign = 0;

        bool isHoming = false; // Whether this stepper is currently performing a homing operation

        void moveTo(long position);
        void stop();
        bool isRunning();
        void resetPosition();
        void update();
        long getCurrentPosition();

    };

    struct LimitSwitchItem{

        LimitSwitchItem(const char* id, int pin, Axis* axis, const char* targetStepperId = nullptr) : id(id), pin(pin), limitSwitch(new LimitSwitch{static_cast<uint8_t>(pin)}), axis(axis), targetStepperId(targetStepperId) {
        }


        void init();

        const char* id;
        LimitSwitch* limitSwitch;
        static volatile bool state;
        int pin;
        Axis* axis;
        const char* targetStepperId = nullptr; // if set, only this stepper is stopped/zeroed
        std::function<void()> onTriggered = nullptr;
    };

    FastAccelStepperEngine* engine = nullptr;

    void addStepper(const char* id ,std::vector<int> pins, int speed, bool direction);

    // Mark an already-added stepper as UART-driven (TMC2209 VACTUAL).
    // Call BEFORE init() so the FastAccelStepper backend skips the STEP pin.
    void setStepperUart(const char* id, TMC2209Stepper* driver, int32_t vactual);

    void removeStepper(const char* id);
    void addLimitSwitch(const char* id, int pin, const char* targetStepperId = nullptr);
    void removeLimitSwitch(const char* id);
    void setLimitSwitchCallback(const char* id, std::function<void()> callback);


    Axis(int homingDistance);
    void init();

    bool isLimitSwitchPressed(const char* id);

    void setHome(bool value, char *id = nullptr); // Set the current position of the specified stepper to be the home position (0). If id is nullptr, all steppers are zeroed.
    bool getHome(char* id = nullptr); // Get whether the specified stepper is currently at the home position. If id is nullptr, returns true if all steppers are at home.

    void home();
    void resetPosition(char* id = nullptr); // Optional ID
    bool update(); // Return to see if any command is running
    void moveAxis(long position);
    void moveSingleStepper(char* id, long position);
    void printLimitSwitchStates();
    void printStepperStates();
    void setSpeed(Speed speed);
    void setAcceleration(int acceleration); // Change to struct
    void stop(char* id = nullptr); // Optional ID, if not supplied, all steppers will be stopped
    bool isRunning(char* id = nullptr); // Optional ID, if not supplied, will check if any stepper is running

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
