#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include "Arduino.h"

// Remove PI and HALF_PI definitions since they're in wiring.h
class Vec3 {
public:
    float x, y, z;
    Vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
};

class Pose {
public:
    float x, y, z, roll, pitch, yaw;
    Pose(float x=0, float y=0, float z=0, float roll=0, float pitch=0, float yaw=0)
        : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}
};

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

class IK_Solver {
private:
    // Arm dimensions (modify to match your robot)
    const float linkLengths[6] = {0.104, 0.1694, 0.315, 0.066258, 0.319213, 0.07318};
    
public:
    IK_Solver();
    JointAngles calculateIK(const Pose& target);
};

#endif