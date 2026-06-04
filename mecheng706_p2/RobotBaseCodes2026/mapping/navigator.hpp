#ifndef NAVIGATOR_H
#define NAVIGATOR_H
#include "pose.hpp"
#include "occupancy_grid.h"
#include "fire_localiser.hpp"
#include "../behavior.h"
#include "../mappings.h"

class Navigator {
public:
    Navigator(DeadReckoner* dead_reckoner, OccupancyGrid* local_grid, FireLocaliser* fire_localiser):
     _dead_reckoner(dead_reckoner), _local_grid(local_grid), _fire_localiser(fire_localiser) {};
    ~Navigator();

    // Main navigation function to be called in the control loop
    void navigate(Pose2D targetPose);
    void setSearchBehavior(BehaviorNS::SearchBehaviour behavior) { _searchBehavior = behavior; }
    void setCloseToFire(bool close_to_fire) { _closeToFire = close_to_fire; }
    BehaviorNS::SearchBehaviour searchBehavior() const { return _searchBehavior; }
    Pose2D suggestCommand() const;
private:
    DeadReckoner* _dead_reckoner;
    OccupancyGrid* _local_grid;
    FireLocaliser* _fire_localiser;
    BehaviorNS::SearchBehaviour _searchBehavior = BehaviorNS::SearchBehaviour::FIND_FIRE;
    bool _closeToFire = false;
};  

#endif // NAVIGATOR_H