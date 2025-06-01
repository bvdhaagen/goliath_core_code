#ifndef MotorController_h
#define MotorController_h

#include <Arduino.h>
#include <AccelStepper.h>

class MotorController {
public:
    // Constructor with pin configuration
    MotorController(const int stepPins[], const int dirPins[], const int limitPins[], int numMotors);

    // Initialization
    void begin(const int homingSpeeds[], const int maxSpeeds[],
        const int stepsToReverse[], const int accelerations[],
        const int directions[]);

    // Core functions
    void homeAll();
    void moveAllJointsSynchronized(const float target_positions[], const float steps_per_rad[], float max_speed_rad);
    void moveToPosition(int motor, long position);
    void moveRelative(int motor, long distance);
    void stopAll();
    void emergencyStop();

    // Status checks
    bool isHomed() const;
    long getPosition(int motor) const;
    bool isMoving() const;

    // Must call in loop()
    void run();

private:
    AccelStepper* steppers;
    const int* limitSwitchPins;
    int motorCount;
    bool homingDone;

    // Configuration
    const int* homingSpeed;
    const int* maxSpeed;
    const int* stepsToReverse;
    const int* acceleration;
    const int* motorDirection;

    // Helper
    void printHomingProgress(int motorIndex, const char* message);

    void setMaxSpeed(int motor, float speed) {
        if(motor >= 0 && motor < motorCount) {
            steppers[motor].setMaxSpeed(speed);
        }
    }
    
    float getMaxSpeed(int motor) const {
        if(motor >= 0 && motor < motorCount) {
            return steppers[motor].maxSpeed();
        }
        return 0;
    }
};

#endif