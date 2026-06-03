#include "world_model.hpp"

void WorldModel::update(Tap* tap){
    /*
    Performs a full update of the entire dead reckoning -> occupancy map + fire localisation pipeline
    Does NOT interact with the navigator. The navigator is only queried for a command signal by appropriate sub-states in FSM.
    Navigator replans are also called asynchronously as needed.
    */
   long now = micros();
   //First update pose (affects all downstream functions)
    _deadReckoner.updatePose(tap->heading,tap->vx,tap->vy,(now-_lastUpdateTime));
    //compute pose once, reuse downstream
    const Pose2D robotPose = _deadReckoner.getPose();
    //full grid walk (recenter + update cells)
    _occupancyGrid.update(robotPose,tap->range);
    //update fire localisation if locked
    if(tap->fire_locked){
        Pose2D turretPose = _robot->TurretPose; turretPose.th += tap->turret_angle;
        _fireLocaliser.update_localisation(robotPose,turretPose,tap->angle_error);
    }
    _lastUpdateTime = now;
}