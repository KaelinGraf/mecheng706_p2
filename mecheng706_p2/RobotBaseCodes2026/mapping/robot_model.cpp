#include "robot_model.hpp"

// Out-of-line definitions of the RobotModel static tables (see robot_model.hpp:
// avr-g++ under -std=gnu++11 rejects C++17 inline-variable member initialisers).
// Frame: x forward, y LEFT, th positive CCW — matches Pose2D::chain() (a standard
// CCW rotation) and the world/map frame. So a sensor on the robot's LEFT has +y and
// a left-ward splay has +th; right sensors are -y / -th. Magnitudes unchanged; only
// the y/th signs are set so left sensors sit on the left and face left (& vice versa).
const Pose2D RobotModel::SensorPose[5] = {
    {11.5f,   0.0f,    0.0f      },   // ULTRASONIC      (front centre)
    {10.63f,  7.788f,  0.523599f },   // IR_FRONT_LEFT   (left,  faces forward-left  +30 deg)
    {10.63f, -7.788f, -0.523599f },   // IR_FRONT_RIGHT  (right, faces forward-right -30 deg)
    {-6.262f, 9.444f,  0.523599f },   // IR_REAR_LEFT    (left,  faces forward-left  +30 deg)
    {-6.262f,-9.444f, -0.523599f }    // IR_REAR_RIGHT   (right, faces forward-right -30 deg)
};

const Pose2D RobotModel::TurretPose = {7.6f, 0.0f, 0.0f};
