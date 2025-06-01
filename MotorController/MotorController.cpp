#include "MotorController.h"
#include <Arduino.h>

// Constructor
MotorController::MotorController(const int stepPins[], const int dirPins[], 
                               const int limitPins[], int numMotors) 
    : limitSwitchPins(limitPins), motorCount(numMotors), homingDone(false) {
    
    steppers = new AccelStepper[motorCount];
    for(int i = 0; i < motorCount; i++) {
        steppers[i] = AccelStepper(AccelStepper::DRIVER, stepPins[i], dirPins[i]);
    }
}

// Initialization
void MotorController::begin(const int homingSpeeds[], const int maxSpeeds[],
                          const int stepsToReverse[], const int accelerations[],
                          const int directions[]) {
    this->homingSpeed = homingSpeeds;
    this->maxSpeed = maxSpeeds;
    this->stepsToReverse = stepsToReverse;
    this->acceleration = accelerations;
    this->motorDirection = directions;

    for(int i = 0; i < motorCount; i++) {
        pinMode(limitSwitchPins[i], INPUT_PULLUP);
        steppers[i].setMaxSpeed(homingSpeeds[i]);
        steppers[i].setAcceleration(accelerations[i]);
    }
}

// Homing sequence
void MotorController::homeAll() {
    homingDone = false;
    Serial.println("\n=== STARTING HOMING SEQUENCE ===");
    
    // Home motors 2 and 3 simultaneously
    bool motor2Homed = false, motor3Homed = false;
    while (!motor2Homed || !motor3Homed) {
        if (!motor2Homed) {
            if (digitalRead(limitSwitchPins[1]) == HIGH) {
                steppers[1].setSpeed(homingSpeed[1] * motorDirection[1]);
                steppers[1].runSpeed();
            } else {
                motor2Homed = true;
                steppers[1].setCurrentPosition(0);
                printHomingProgress(1, "Homed (position 0), moving to start...");
            }
        }
        
        if (!motor3Homed) {
            if (digitalRead(limitSwitchPins[2]) == HIGH) {
                steppers[2].setSpeed(homingSpeed[2] * motorDirection[2]);
                steppers[2].runSpeed();
            } else {
                motor3Homed = true;
                steppers[2].setCurrentPosition(0);
                printHomingProgress(2, "Homed (position 0), moving to start...");
            }
        }
    }

    // Move to start positions
    steppers[1].move(-stepsToReverse[1] * motorDirection[1]);
    steppers[2].move(-stepsToReverse[2] * motorDirection[2]);
    steppers[1].setMaxSpeed(maxSpeed[1]);
    steppers[2].setMaxSpeed(maxSpeed[2]);
    
    while (steppers[1].distanceToGo() != 0 || steppers[2].distanceToGo() != 0) {
        steppers[1].run();
        steppers[2].run();
    }
    
    steppers[1].setCurrentPosition(0);
    steppers[2].setCurrentPosition(0);
    printHomingProgress(1, "At start position (position 0)");
    printHomingProgress(2, "At start position (position 0)");

    // Home remaining motors
    int motorsToHome[] = {0, 3, 4, 5};
    for (int i : motorsToHome) {
        while (digitalRead(limitSwitchPins[i]) == HIGH) {
            steppers[i].setSpeed(-homingSpeed[i] * motorDirection[i]);
            steppers[i].runSpeed();
        }
        
        steppers[i].setCurrentPosition(0);
        steppers[i].setMaxSpeed(maxSpeed[i]);
        steppers[i].move(stepsToReverse[i] * motorDirection[i]);
        steppers[i].runToPosition();
        steppers[i].setCurrentPosition(0);
        printHomingProgress(i, "At start position (position 0)");
    }

    homingDone = true;
    Serial.println("\n=== HOMING COMPLETE ===");
    
    // Set all motors to max speed
    for (int i = 0; i < motorCount; i++) {
        steppers[i].setMaxSpeed(maxSpeed[i]);
    }
}



// Movement control
void MotorController::moveToPosition(int motor, long position) {
    if(motor >= 0 && motor < motorCount && homingDone) {
        steppers[motor].moveTo(position * motorDirection[motor]);
    }
}

void MotorController::moveRelative(int motor, long distance) {
    if(motor >= 0 && motor < motorCount && homingDone) {
        steppers[motor].move(distance * motorDirection[motor]);
    }
}

// Safety functions
void MotorController::stopAll() {
    for(int i = 0; i < motorCount; i++) {
        steppers[i].stop();
    }
}

void MotorController::emergencyStop() {
    for(int i = 0; i < motorCount; i++) {
        steppers[i].stop();
        steppers[i].disableOutputs();
    }
    homingDone = false;
    Serial.println("EMERGENCY STOP ACTIVATED");
}

// Status checks
bool MotorController::isHomed() const {
    return homingDone;
}

long MotorController::getPosition(int motor) const {
    if(motor >= 0 && motor < motorCount) {
        return steppers[motor].currentPosition() * motorDirection[motor];
    }
    return 0;
}

bool MotorController::isMoving() const {
    for(int i = 0; i < motorCount; i++) {
        if(steppers[i].distanceToGo() != 0) {
            return true;
        }
    }
    return false;
}

// Main run function
void MotorController::run() {
    for(int i = 0; i < motorCount; i++) {
        steppers[i].run();
    }
}

// Helper function
void MotorController::printHomingProgress(int motorIndex, const char* message) {
    Serial.print("Motor ");
    Serial.print(motorIndex + 1);
    Serial.print(": ");
    Serial.println(message);
}