#include "fire_localiser.hpp"

FireLocaliser::FireLocaliser(): m00(0), m01(0), m11(0), c0(0), c1(0) {};

FireLocaliser::~FireLocaliser() {};

void FireLocaliser::update_localisation(const Pose2D& robotPose, Pose2D turretPose , const float angleError) {
    // Update the linear system based on the current robot and turret poses
    turretPose.th += angleError; //correct turret pose with angle error
    Pose2D turretInWorld = robotPose.chain(turretPose); //get turret pose in world frame
    float st = sinf(turretInWorld.th);
    float ct = cosf(turretInWorld.th);
    float nx = -st; //normals along new ray from turret
    float ny = ct; 

    float c = nx * turretInWorld.x + ny * turretInWorld.y; //constant term of the line equation
    m00 += nx * nx;
    m01 += nx * ny;
    m11 += ny * ny;
    c0 += nx * c;
    c1 += ny * c;
    return;
}
Pose2D FireLocaliser::solvePose() {
    float det = m00 * m11 - m01 * m01;
    float trace = m00 + m11;
    float confidence = (trace > 0.0f) ? (det / (trace * trace)) * 4.0f : 0.0f;
    if (det <= 1e-6f * trace) {          // degenerate: too few / near-parallel rays
        return {0.0f, 0.0f, 0.0f};       // or whatever your "no fix" sentinel is
    }
    float fireX = (c0 * m11 - c1 * m01) / det;
    float fireY = (m00 * c1 - m01 * c0) / det;
    return {fireX, fireY, confidence};
}

float FireLocaliser::solveDeterminant() {
    // Calculate the determinant of the system 
    float det = (m00 * m11 - m01 * m01);
    return det; 
}