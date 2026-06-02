#include "mapping/pose.hpp"

Pose2D Pose2D::chain(Pose2D other) const {
    Pose2D result;
    result.x = x + other.x * cos(th) - other.y * sin(th);
    result.y = y + other.x * sin(th) + other.y * cos(th);
    result.th = th + other.th;
    return result;
}

rMatrix2D Pose2D::rAsMatrix() const {
    rMatrix2D result;
    result.m00 = cos(th);
    result.m01 = -sin(th);
    result.m10 = sin(th);
    result.m11 = cos(th);
    return result;
}



DeadReckoner::DeadReckoner() : currentPose() {}

DeadReckoner::~DeadReckoner() {}

Pose2D DeadReckoner::getPose() const {
    return currentPose;
}

Pose2D DeadReckoner::updatePose(float heading, float vx, float vy, float dt) {
    // Update the robot's pose based on its velocity and heading
    //vx, vy are in the robot's local frame, so we need to convert them to world frame using the current heading
    float vx_world = KX * vx * cos(heading) - KY * vy * sin(heading);
    float vy_world = KX * vx * sin(heading) + KY * vy * cos(heading);
    currentPose.x += vx_world * dt;
    currentPose.y += vy_world * dt;
    currentPose.th = heading; // Update heading directly from the input
    
    return currentPose;
}
