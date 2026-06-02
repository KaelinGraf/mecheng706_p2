#ifndef NAVIGATOR_H
#define NAVIGATOR_H
#include "pose.hpp"
#include "occupancy_grid.h"

class Navigator {
public:
    Navigator(DeadReckoner* dead_reckoner, OccupancyGrid* local_grid);
    ~Navigator();

    // Main navigation function to be called in the control loop
    void navigate(Pose2D targetPose);
private:
    DeadReckoner* _dead_reckoner;
    OccupancyGrid* _local_grid;
};  

#endif // NAVIGATOR_H