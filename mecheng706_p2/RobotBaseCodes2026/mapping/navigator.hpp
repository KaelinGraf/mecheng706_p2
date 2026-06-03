#ifndef NAVIGATOR_H
#define NAVIGATOR_H
#include "pose.hpp"
#include "occupancy_grid.h"
#include "fire_localiser.hpp"

class Navigator {
public:
    Navigator(DeadReckoner* dead_reckoner, OccupancyGrid* local_grid, FireLocaliser* fire_localiser):
     _dead_reckoner(dead_reckoner), _local_grid(local_grid), _fire_localiser(fire_localiser) {};
    ~Navigator();

    // Main navigation function to be called in the control loop
    void navigate(Pose2D targetPose);
private:
    DeadReckoner* _dead_reckoner;
    OccupancyGrid* _local_grid;
    FireLocaliser* _fire_localiser;
};  

#endif // NAVIGATOR_H