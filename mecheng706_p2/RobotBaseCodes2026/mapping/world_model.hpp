#ifndef WORLD_MODEL_H
#define WORLD_MODEL_H

#include "mapping/occupancy_grid.h"
#include "mapping/fire_localiser.hpp"
#include "mapping/pose.hpp"
#include "mapping/robot_model.hpp"
#include "mapping/navigator.hpp"
#include "utils.h"



class WorldModel {
public:
    WorldModel();
    ~WorldModel();
}

#endif // WORLD_MODEL_H