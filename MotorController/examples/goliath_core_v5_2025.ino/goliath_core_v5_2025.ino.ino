#include "MotorController.h"
#include "IK_Solver.h"

// Configuration (same as before)
const int NUM_MOTORS = 6;
const int limitSwitchPins[NUM_MOTORS] = {23, 22, 14, 37, 36, 33};
const int stepPins[NUM_MOTORS] = {7, 6, 5, 3, 4, 2};
const int dirPins[NUM_MOTORS] = {29, 28, 25, 24, 9, 8};

const int homingSpeed[NUM_MOTORS] = {600, 800, 600, 400, 400, 400};
const int maxSpeed[NUM_MOTORS] = {1000, 8000, 2000, 2000, 800, 1200};
const int stepsToReverse[NUM_MOTORS] = {11450, 36000, 9000, 13300, 7500, 11300};
const int acceleration[NUM_MOTORS] = {1400, 2000, 600, 2000, 1500, 1400};
const int motorDirection[NUM_MOTORS] = {1, 1, 1, -1, -1, -1};

MotorController arm(stepPins, dirPins, limitSwitchPins, NUM_MOTORS);
IK_Solver ikSolver;

void setup() {
    Serial.begin(115200);
    arm.begin(homingSpeed, maxSpeed, stepsToReverse, acceleration, motorDirection);
    Serial.println("System ready. Commands:");
    Serial.println("H - Home");
    Serial.println("M X,Y,Z,R,P,Y - Move to pose");
    Serial.println("D - Toggle demo");
    Serial.println("S - Emergency stop");
}

void loop() {
    arm.run();
    handleSerialCommands();
}

void handleSerialCommands() {
    if(Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if(input.equalsIgnoreCase("H")) {
            arm.homeAll();
        }
        else if(input.equalsIgnoreCase("D")) {
            if(arm.isHomed()) {
                // Toggle demo mode
            }
        }
        else if(input.equalsIgnoreCase("S")) {
            arm.emergencyStop();
        }
        else if(input.startsWith("M")) {
            handleMoveCommand(input);
        }
    }
}

void handleMoveCommand(String input) {
    input = input.substring(1);
    input.trim();
    
    float values[6] = {0};
    int index = 0;
    int lastComma = -1;
    
    for(unsigned int i = 0; i <= input.length(); i++) {
        if(i == input.length() || input.charAt(i) == ',') {
            values[index++] = input.substring(lastComma + 1, i).toFloat();
            lastComma = i;
            if(index >= 6) break;
        }
    }
    
    if(index == 6) {
        Pose target(values[0], values[1], values[2], values[3], values[4], values[5]);
        JointAngles angles = ikSolver.calculateIK(target);
        
        // Convert joint angles to motor positions and move
        for(int i = 0; i < NUM_MOTORS; i++) {
            float steps = angles.angles[i] * (3200.0 / (2.0 * PI)); // Convert radians to steps
            arm.moveToPosition(i, steps);
        }
    }
}