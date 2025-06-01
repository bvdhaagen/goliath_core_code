#include "MotorController.h"
#include <Adafruit_PWMServoDriver.h>

// Create PWM servo driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo settings
#define SERVO_MIN 350  // Minimum pulse length count (out of 4096)
#define SERVO_MAX 560  // Maximum pulse length count (out of 4096)
#define GRIPPER_CHANNEL 0  // PCA9685 channel the gripper is connected to

// Motor configuration
const int NUM_MOTORS = 6;
const int stepPins[NUM_MOTORS] = {7, 6, 5, 3, 4, 2};
const int dirPins[NUM_MOTORS] = {29, 28, 25, 24, 9, 8};
const int limitPins[NUM_MOTORS] = {23, 22, 14, 37, 36, 33};

// Motor parameters
const int homingSpeed[NUM_MOTORS] = {600, 1200, 800, 400, 400, 400};
const int maxSpeed[NUM_MOTORS] = {1000, 5000, 8000, 2000, 800, 1200};
const int stepsToReverse[NUM_MOTORS] = {
  11450,    // J1: Half of full rotation (24000/2)
  53800,    // J2: Half of full rotation (168000/2)
  33000,    // J3: Half of full rotation (70000/2)
  11000,    // J4: Half of full rotation (25000/2)
  7500,     // J5: Half of full rotation (13000/2)
  11500     // J6: Half of full rotation (19000/2)
};
const int acceleration[NUM_MOTORS] = {1400, 2000, 600, 2000, 1500, 1400};
const int motorDirection[NUM_MOTORS] = {1, -1, -1, -1, -1, -1};

// Steps per radian conversion (using full rotation steps)
const float STEPS_PER_RAD[NUM_MOTORS] = {
  24200.0f / (2 * PI),   // J1 = 3819.72 steps/rad    
  200000.0f / (2 * PI),  // J2 = 26738.03 steps/rad
  80000.0f / (2 * PI),   // J3 = 11140.85 steps/rad
  24200.0f / (2 * PI),   // J4 = 3978.87 steps/rad
  24000.0f / (2 * PI),   // J5 = 2069.01 steps/rad
  24000.0f / (2 * PI)    // J6 = 3023.94 steps/rad
};

MotorController arm(stepPins, dirPins, limitPins, NUM_MOTORS);

// Position control
float target_positions[NUM_MOTORS] = {0};
const float MAX_SPEED_RAD = 5.0; // rad/s
const float MOVE_THRESHOLD = 0.001; // rad

// Movement tracking
float movement_start_positions[NUM_MOTORS] = {0};
unsigned long movement_start_time = 0;
bool movement_in_progress = false;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection
  
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  arm.begin(homingSpeed, maxSpeed, stepsToReverse, acceleration, motorDirection);
  arm.homeAll();
}

void executeSynchronizedMovement() {
  // 1. Calculate required movement for each joint
  float movements_rad[NUM_MOTORS];
  float max_movement = 0;
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    movements_rad[i] = fabs(target_positions[i] - movement_start_positions[i]);
    if (movements_rad[i] > max_movement) {
      max_movement = movements_rad[i];
    }
  }

  // 2. Calculate time needed for the longest movement
  float movement_time = max_movement / MAX_SPEED_RAD;
  
  // 3. Calculate progress (0-1)
  float progress = min(1.0, (millis() - movement_start_time) / (movement_time * 1000));
  
  // 4. Move each joint proportionally
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (movements_rad[i] > MOVE_THRESHOLD) {
      float current_target = movement_start_positions[i] + (progress * (target_positions[i] - movement_start_positions[i]));
      long target_steps = current_target * STEPS_PER_RAD[i];
      arm.moveToPosition(i, target_steps);
    }
  }

  // 5. Check if movement is complete
  if (progress >= 1.0) {
    movement_in_progress = false;
    // Ensure final positions are exact
    for (int i = 0; i < NUM_MOTORS; i++) {
      arm.moveToPosition(i, target_positions[i] * STEPS_PER_RAD[i]);
    }
  }
}

void loop() {
  arm.run(); // Critical for motor movement
  
  // 1. Handle incoming commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("M ")) {
      // Parse position commands
      String data = cmd.substring(2);
      int index = 0;
      int lastComma = -1;
      
      for (unsigned int i = 0; i <= data.length(); i++) {
        if (i == data.length() || data.charAt(i) == ',') {
          target_positions[index] = data.substring(lastComma + 1, i).toFloat();
          lastComma = i;
          if (++index >= NUM_MOTORS) break;
        }
      }
      
      // Initialize synchronized movement
      for (int i = 0; i < NUM_MOTORS; i++) {
        movement_start_positions[i] = arm.getPosition(i) / STEPS_PER_RAD[i];
      }
      movement_start_time = millis();
      movement_in_progress = true;
    }
    else if (toupper(cmd[0]) == 'H') {
      arm.homeAll();
      for (int i = 0; i < NUM_MOTORS; i++) {
        target_positions[i] = 0;
      }
      movement_in_progress = false;
    }
  }

  // 2. Execute synchronized movement if active
  if (movement_in_progress) {
    executeSynchronizedMovement();
  }

  // 3. Send position feedback
  static unsigned long last_send = 0;
  if (millis() - last_send >= 20) { // 50Hz feedback rate
    for (int i = 0; i < NUM_MOTORS; i++) {
      float pos_rad = arm.getPosition(i) / STEPS_PER_RAD[i];
      Serial.print("J"); Serial.print(i+1); 
      Serial.print(":"); Serial.print(pos_rad, 3);
      Serial.print(" ");
    }
    Serial.println();
    last_send = millis();
  }
}