#ifndef WORLD_MODEL_H
#define WORLD_MODEL_H

#include "occupancy_grid.h"
#include "fire_localiser.hpp"
#include "pose.hpp"
#include "robot_model.hpp"
#include "navigator.hpp"
#include "../utils.h"
#include "../firefighter.h"



class WorldModel {
/*
WorldModel serves as an interface between the firefighter class and all mapping/localisation/nav components via an API it exposes
The idea being that the main control loop in firefighter class can just call worldModel.update() and worldModel.navigate() 
without needing to know about the internal workings of the mapping/localisation/nav components, 
and we can easily swap out implementations of those components by just changing the WorldModel implementation without needing to change firefighter code.

*/
public:
    WorldModel(FireFighter*ff, RobotModel* robot): 
        _occupancyGrid(robot), 
        _fireLocaliser(), 
        _navigator(&_deadReckoner, &_occupancyGrid, &_fireLocaliser), 
        _ff(ff), 
        _robot(robot) {_lastUpdateTime = micros();};
    ~WorldModel();

    void update(Tap* tap); //updates world model based on new sensor readings, should be called every iter in control loop before navigate

private:
    OccupancyGrid _occupancyGrid;
    FireLocaliser _fireLocaliser;
    DeadReckoner _deadReckoner;
    Navigator _navigator;
    FireFighter* _ff;
    RobotModel* _robot;

    long _lastUpdateTime;
    long _lastNavigateTime = 0;
    long _lastSerialPrintTime = 0;

};

#endif // WORLD_MODEL_H