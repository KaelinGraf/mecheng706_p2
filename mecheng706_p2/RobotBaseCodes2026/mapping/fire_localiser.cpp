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
    // Display ASAP: hand back the estimate the moment the system is numerically
    // solvable, even at LOW confidence. The viewer scales the marker size/opacity by
    // confidence (the .th field), so an early, uncertain fix shows up faint and firms
    // up as more (spatially diverse) bearings arrive — we no longer wait for a strong
    // determinant before revealing anything. The only gate left is a numerical floor
    // on det so the divide can't throw the estimate to infinity.
    // NOTE: triangulation needs >= 2 bearings from DIFFERENT robot positions; a
    // STATIONARY robot gives one ray (det stays ~0) so no point fix is possible yet.
    // See implementation.md: plan is to drive along the last lock bearing to build a
    // baseline, then converge.
    float confidence = (trace > 0.0f) ? (det / (trace * trace)) * 4.0f : 0.0f;
    if (det <= 1e-9f) {                  // numerically singular -> no usable fix yet
        return {0.0f, 0.0f, 0.0f};       // "no fix" sentinel (viewer hides conf<=0)
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