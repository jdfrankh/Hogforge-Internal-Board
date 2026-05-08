#ifndef FAN_H

#define FAN_H

#include <Arduino.h>

class Fan {

    public:

    enum class Speeds {
        OFF = 0,
        SPEED_LOW = 75,
        SPEED_MEDIUM = 150,
        SPEED_MAX = 255
    };
    
    enum class Direction {
        BACKWARD = 0,
        FORWARD = 1,
    };

    enum class BreakType {
        COAST = 0,
        BRAKE = 1
    };

        Fan(char* id, int pwm1, int pwm2);

        void setup();

        void setSpeed(Speeds speed, Direction direction = Direction::FORWARD);

        void setSpeed(int speed);

        void stop();

        int getSpeed(){
            return _currentSpeed;
        }

        void setBreakType(BreakType breakType){
            _currentBreakType = static_cast<int>(breakType);
        }

        void setDirection(Direction direction){
            _currentDirection = static_cast<int>(direction);
            setSpeed(static_cast<Speeds>(_currentSpeed), direction);
        }

        int getBreakType(){
            return _currentBreakType;
        }

        int getDirection(){
            return _currentDirection;
        }


        char* getId(){
            return id;
        }


    private:
        char* id;
        int _currentDirection;
        int _currentBreakType = static_cast<int>(BreakType::COAST);
        int _pwm1;
        int _pwm2;
        int _currentSpeed;

};


#endif