#ifndef FIRE_LOCALISER_H
#define FIRE_LOCALISER_H

#include "mapping/pose.hpp"
#include "math.h"

class FireLocaliser {
public:
    FireLocaliser();
    ~FireLocaliser();

    //call each iter IF and ONLY IF the "lock" condition is met this iter
    void update_localisation(const Pose2D& robotPose, Pose2D turretPose, const float angleError);
    Pose2D solvePose();//solves for fire pose in world frame, returns as Pose2D with x,y as position and th as confidence (0-1)
    float solveDeterminant(); //solves for determinant of system 
    private:
    float m00, m01, m11, c0, c1; //coefficients of the linear system Ax=b, where A=[[m00,m01],[m10,m11]], x=[fireX, fireY], b=[c0,c1]

};

#endif // FIRE_LOCALISER_H