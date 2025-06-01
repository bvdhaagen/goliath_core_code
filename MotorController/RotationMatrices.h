#ifndef ROTATION_MATRICES_H
#define ROTATION_MATRICES_H

#include "Arduino.h"

// Remove these lines - they're already defined in Teensy core
// const double PI = 3.141592653589793;
// const double HALF_PI = 1.5707963267948966;

// 3D Vector Class
class Vec3 {
public:
    float x, y, z;
    Vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
};

// Robot Pose Class
class Pose {
public:
    float x, y, z, roll, pitch, yaw;
    Pose(float x=0, float y=0, float z=0, float roll=0, float pitch=0, float yaw=0)
        : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}
};

// Joint Angles Storage
class JointAngles {
public:
    float angles[6];
    
    JointAngles(float j1=0, float j2=0, float j3=0, float j4=0, float j5=0, float j6=0) {
        angles[0] = j1;
        angles[1] = j2;
        angles[2] = j3;
        angles[3] = j4;
        angles[4] = j5;
        angles[5] = j6;
    }
};

#endif