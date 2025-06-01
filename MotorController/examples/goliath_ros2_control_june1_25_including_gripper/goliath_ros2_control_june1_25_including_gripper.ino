#include "MotorController.h"
#include <Adafruit_PWMServoDriver.h>

// Constants
constexpr uint8_t NUM_MOTORS = 6;
constexpr uint8_t NUM_GRIPPER_FINGERS = 2;
constexpr uint8_t TOTAL_JOINTS = NUM_MOTORS + NUM_GRIPPER_FINGERS;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;
constexpr uint16_t FEEDBACK_INTERVAL_MS = 20;  // 50Hz feedback rate
constexpr uint16_t SERVO_PWM_FREQ = 60;        // 60Hz for analog servos

// Servo settings
constexpr uint16_t SERVO_MIN = 350;  // Minimum pulse length count
constexpr uint16_t SERVO_MAX = 560;  // Maximum pulse length count
constexpr uint8_t GRIPPER_CHANNEL = 0;  // PCA9685 channel for gripper

// Gripper range in radians
constexpr float FINGER1_MIN = -0.03f;
constexpr float FINGER1_MAX = 0.22f;
constexpr float FINGER2_MIN = 0.22f;
constexpr float FINGER2_MAX = -0.03f;

// Movement parameters
constexpr float DEFAULT_MOVEMENT_DURATION_MS = 3000.0f;
constexpr float MOVE_THRESHOLD = 0.001f;  // rad

// Motor configuration
const int stepPins[NUM_MOTORS] = {7, 6, 5, 3, 4, 2};
const int dirPins[NUM_MOTORS] = {29, 28, 25, 24, 9, 8};
const int limitPins[NUM_MOTORS] = {23, 22, 14, 37, 36, 33};

// Motor parameters
const int homingSpeed[NUM_MOTORS] = {600, 1200, 800, 400, 400, 400};
const int maxSpeed[NUM_MOTORS] = {1000, 5000, 8000, 2000, 800, 1200};
const int stepsToReverse[NUM_MOTORS] = {
  11450,    // J1: Half of full rotation
  53800,    // J2: Half of full rotation
  33000,    // J3: Half of full rotation
  11000,    // J4: Half of full rotation
  7500,     // J5: Half of full rotation
  11500     // J6: Half of full rotation
};
const int acceleration[NUM_MOTORS] = {1400, 2000, 600, 2000, 1500, 1400};
const int motorDirection[NUM_MOTORS] = {1, -1, -1, -1, -1, -1};

// Steps per radian conversion
const float STEPS_PER_RAD[NUM_MOTORS] = {
  24200.0f / (2.0f * PI),   // J1
  200000.0f / (2.0f * PI),  // J2
  80000.0f / (2.0f * PI),   // J3
  24200.0f / (2.0f * PI),   // J4
  24000.0f / (2.0f * PI),   // J5
  24000.0f / (2.0f * PI)    // J6
};

// Objects
Adafruit_PWMServoDriver pwm;
MotorController arm(stepPins, dirPins, limitPins, NUM_MOTORS);

// State variables
float target_positions[TOTAL_JOINTS] = {0};  // 6 arm + 2 gripper
float current_positions[TOTAL_JOINTS] = {0};
float movement_start_positions[TOTAL_JOINTS] = {0};
float movement_duration_ms = DEFAULT_MOVEMENT_DURATION_MS;
uint32_t movement_start_time = 0;
bool movement_in_progress = false;

///////////////////////////////////////////////////////////////////////////////
//                          Helper Functions                                //
///////////////////////////////////////////////////////////////////////////////

inline float mapToPWM(float joint_pos) {
  // Safely map joint position to PWM value
  joint_pos = constrain(joint_pos, FINGER1_MIN, FINGER1_MAX);
  return static_cast<uint16_t>(
    map(joint_pos * 1000.0f, FINGER1_MIN * 1000.0f, FINGER1_MAX * 1000.0f, 
        SERVO_MIN, SERVO_MAX)
  );
}

void setGripper(float position) {
  // Constrain and set both fingers
  position = constrain(position, FINGER1_MIN, FINGER1_MAX);
  
  // Calculate finger positions
  current_positions[NUM_MOTORS] = position;  // Finger 1
  current_positions[NUM_MOTORS+1] = FINGER2_MIN + 
    (position - FINGER1_MIN) * (FINGER2_MAX - FINGER2_MIN) / (FINGER1_MAX - FINGER1_MIN);
  
  // Set PWM
  pwm.setPWM(GRIPPER_CHANNEL, 0, mapToPWM(position));
}

void parseMovementCommand(const String& cmd) {
  // Expecting format: "M val1,val2,...,val7" (6 arm + 1 gripper)
  int startIdx = 2;  // Skip "M "
  int commaIdx;
  uint8_t jointIdx = 0;
  
  while ((commaIdx = cmd.indexOf(',', startIdx)) >= 0 && jointIdx < 7) {
    float value = cmd.substring(startIdx, commaIdx).toFloat();
    
  if (jointIdx < 6) {  // First 6 values: Arm joints
      target_positions[jointIdx] = value;
  } 
  else if (jointIdx == 6) {  // 7th value: Gripper command
      // Set both gripper fingers to the same value (mirroring)
      target_positions[6] = value;     // First gripper finger
      target_positions[7] = value*-1; // Second gripper finger (mirrored)
      setGripper(value);  // Apply the gripper command
  }
    
    startIdx = commaIdx + 1;
    jointIdx++;
    
    if (commaIdx < 0) break;  // No more commas
  }
  
  // Initialize movement
  memcpy(movement_start_positions, current_positions, sizeof(current_positions));
  movement_start_time = millis();
  movement_in_progress = true;
}

void executeSynchronizedMovement() {
  uint32_t elapsed = millis() - movement_start_time;
  float progress = min(1.0f, static_cast<float>(elapsed) / movement_duration_ms);
  
  // Interpolate all joints
  for (uint8_t i = 0; i < TOTAL_JOINTS; i++) {
    current_positions[i] = movement_start_positions[i] + 
                         (target_positions[i] - movement_start_positions[i]) * progress;
    
    // Handle motor movements
    if (i < NUM_MOTORS) {
      arm.moveRelative(i, current_positions[i] * STEPS_PER_RAD[i]);
    }
  }
  
  // Update gripper servo
  pwm.setPWM(GRIPPER_CHANNEL, 0, mapToPWM(current_positions[NUM_MOTORS]));
  
  // Check completion
  if (progress >= 1.0f) {
    movement_in_progress = false;
  }
}

void sendFeedback() {
  static uint32_t last_send = 0;
  uint32_t now = millis();
  
  if (now - last_send >= FEEDBACK_INTERVAL_MS) {
    // Arm joints
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      Serial.print("J"); Serial.print(i+1); 
      Serial.print(":"); Serial.print(current_positions[i], 3);
      Serial.print(" ");
    }
    
    // Gripper fingers
    Serial.print("S1:"); Serial.print(current_positions[NUM_MOTORS], 3);
    Serial.print(" ");
    Serial.print("S2:"); Serial.print(current_positions[NUM_MOTORS+1], 3);
    
    Serial.println();
    last_send = now;
  }
}

///////////////////////////////////////////////////////////////////////////////
//                          Main Arduino Functions                          //
///////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial); // Wait for serial connection

  // Initialize PWM servo driver
  pwm.begin();
  pwm.setPWMFreq(SERVO_PWM_FREQ);

  // Initialize motor controller
  arm.begin(homingSpeed, maxSpeed, stepsToReverse, acceleration, motorDirection);

  // Home the robot
  arm.homeAll();
  
  // Initialize positions
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    current_positions[i] = 0.0f;
    target_positions[i] = 0.0f;
  }
  
  // Initialize gripper to closed position
  setGripper(FINGER1_MIN);
}

void loop() {
  // Critical motor control
  arm.run();
  
  // Handle incoming commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() > 0) {
      switch (toupper(cmd[0])) {
        case 'M':  // Movement command
          if (cmd.length() > 2) {
            parseMovementCommand(cmd);
          }
          break;
          
        case 'H':  // Home command
          arm.homeAll();
          for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            target_positions[i] = 0.0f;
            current_positions[i] = 0.0f;
          }
          setGripper(FINGER1_MIN);
          movement_in_progress = false;
          break;
          
        case 'D':  // Set duration
          if (cmd.length() > 2) {
            movement_duration_ms = cmd.substring(2).toFloat();
          }
          break;
      }
    }
  }
  
  // Execute movement if active
  if (movement_in_progress) {
    executeSynchronizedMovement();
  }
  
  // Send feedback
  sendFeedback();
}