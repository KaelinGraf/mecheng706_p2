#include "pose.hpp"
#include <math.h>   

// ---------------------------------------------------------------------------
// Motion-model calibration tables.  CALIBRATED via drive_calibration (5 s timed
// drives; dt in us).  xs = commanded |v| in the driveMotors control-effort scale
// (0..100, each effort capped at 100); ys = gain so world velocity = gain * v
// (cm per effort*us), where gain = measured_cm / (v * 5000 * 1000).
// vx=20 is below the chassis deadband (0 cm). Re-run drive_calibration to refresh;
// keep xs[] ascending.
// ---------------------------------------------------------------------------
static const float KX_XS[] = {  0.f,    20.f,     40.f,     60.f,      80.f,     100.f };  // |vx| <= 100
static const float KX_YS[] = { 0.0f,    0.0f,  2.90e-7f, 3.40e-7f, 3.125e-7f, 2.74e-7f };  // fwd cm: 0,0,58,102,125,137
static const float KY_XS[] = {  0.f,    20.f,     40.f,     60.f,      80.f,     100.f };  // |vy| <= 100
static const float KY_YS[] = { 0.0f, 1.00e-8f, 2.10e-7f, 2.767e-7f, 2.45e-7f,  2.24e-7f };  // right cm: 1,42,83,98,112

const Lut1D KX_LUT = { KX_XS, KX_YS, (int)(sizeof(KX_XS) / sizeof(KX_XS[0])) };
const Lut1D KY_LUT = { KY_XS, KY_YS, (int)(sizeof(KY_XS) / sizeof(KY_XS[0])) };

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

Pose2D DeadReckoner::updatePose(float heading, float vx, float vy, long dt) {
    // Update the robot's pose based on its velocity and heading
    //vx, vy are in the robot's local frame, so we need to convert them to world frame using the current heading
    // Velocity-dependent gains: interpolate the LUT at this tick's command magnitude.
    float kx = KX_LUT.eval(fabsf(vx));
    float ky = KY_LUT.eval(fabsf(vy));
    // Body command convention (single source of truth = the mecanum mixer in
    // servo_control.h): +vx = FORWARD, +vy = RIGHT. Resolve into the world/map frame
    // (X-forward, Y-left at heading 0): forward maps along the heading, right maps to
    // heading-90deg (hence the -right on Y). Heading from the gyro is already correct
    // on the map, so pass it straight through.
    float fwd   = kx * vx;   // +vx = forward
    float right = ky * vy;   // +vy = right
    float vx_world = fwd * cos(heading) + right * sin(heading);
    float vy_world = fwd * sin(heading) - right * cos(heading);
    currentPose.x += vx_world * dt;
    currentPose.y += vy_world * dt;
    currentPose.th = heading; // heading verified correct on the map — pass through
    
    return currentPose;
}
