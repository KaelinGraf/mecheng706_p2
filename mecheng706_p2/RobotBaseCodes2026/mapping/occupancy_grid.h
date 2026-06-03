#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <string.h>   // memmove, memset


#include "../utils.h"
#include "../firefighter.h"

#include "robot_model.hpp"
#include "pose.hpp"


#define WIN_N 30
#define CELL_CM 5.0f
#define LOC_OCC 4
#define LOC_FREE 3
#define LOC_CLAMP 12
#define LOC_THRESH 5

struct ray {
    float n_x, n_y, c;
};

class OccupancyGrid {
public:
    OccupancyGrid(RobotModel* robot):_robot(robot) {memset(L, 0, sizeof(L));};
    ~OccupancyGrid(){};
     struct idx_xy {
        int x, y;
    };

    void update(const Pose2D& robotPose, float* sensorReadings); //takes in the robot's current pose and updates the occupancy grid based on sensor readings, from deadreckoner
    void voxelTraversal(const Pose2D& sensorPose, const Pose2D& targetPose);


    bool isCellOccupied(int x, int y) const;
    inline int idx(int x, int y) const{ return y * WIN_N + x;}
    inline int8_t& at (int x, int y){ return L[idx(x, y)]; }
    // Read-only accessors for the serial emitter + offline tests: raw log-odds
    // (not thresholded like isCellOccupied) and the window origin in world cells.
    inline int8_t logOddsAt(int x, int y) const { return L[idx(x, y)]; }
    inline int originX() const { return org_cx; }
    inline int originY() const { return org_cy; }

    void shiftX(int dx);
    void shiftY(int dy);
    void recenter(const Pose2D& p);
    bool worldInGrid(float wx, float wy, int& cx, int& cy) const;
    idx_xy worldToIndex(float wx, float wy) const;
    void reset(const Pose2D& robotPose);
   


private:
    RobotModel* _robot;
    int8_t L[WIN_N * WIN_N] = {0}; // 1D array representing the occupancy grid
    int org_cx = -WIN_N/2, org_cy = -WIN_N/2; // origin of the grid in world coordinates (in cells)


};

#endif // OCCUPANCY_GRID_H
