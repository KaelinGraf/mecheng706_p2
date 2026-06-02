#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include "pose.hpp"

struct RobotModel {
    inline static const Pose2D SensorPose[5] = { // Define the poses of the 5 sensors relative to the robot's center
        {0.0f, 0.0f, 0.0f},   // ultrasonic sensor at the front
        {0.0f, 0.0f, 0.0f}, // Left front IR sensor
        {0.0f, 0.0f, 0.0f}, // Right front IR sensor
        {0.0f, 0.0f, 0.0f}, // rear left ir sensor
        {0.0f, 0.0f, 0.0f} // rear right ir sensor
    };
    inline static enum SensorType { ULTRASONIC, IR_FRONT_LEFT, IR_FRONT_RIGHT, IR_REAR_LEFT, IR_REAR_RIGHT}; // Enum to identify sensor types
    
    Pose2D getSensorPose(SensorType type) const { // Function to get the pose of a specific sensor
        return SensorPose[type];
    }

    const Pose2D* getAllSensorPoses() const { // Function to get the poses of all sensors
        return SensorPose;
    }
    
    Pose2D getSensorPoseInWorld(SensorType type, const Pose2D& robotPose) const { // Function to get the pose of a specific sensor in the world frame
        return robotPose.chain(getSensorPose(type));
    }

    void getAllSensorPosesInWorld(const Pose2D& robotPose, Pose2D* sensorPosesInWorld) const { // Function to get the poses of all sensors in the world frame
        for (int i = 0; i < 5; ++i) {
            sensorPosesInWorld[i] = robotPose.chain(getSensorPose(static_cast<SensorType>(i)));
        }
    }

};

#endif // ROBOT_MODEL_H