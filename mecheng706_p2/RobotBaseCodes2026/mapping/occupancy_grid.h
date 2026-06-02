#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <string.h>   // memmove, memset

#include "robot_model.hpp"
#include "firefighter.h"
#include "pose.hpp"


#define WIN_N 30
#define CELL_CM 5.0f

struct ray {
    float n_x, n_y, c;
};

class OccupancyGrid {
public:
    OccupancyGrid(RobotModel* robot, Firefighter* firefighter):_robot(robot), _ff(firefighter) {memset(L, 0, sizeof(L));};
    ~OccupancyGrid(){};

    void update(const Pose2D& robotPose); //takes in the robot's current pose and updates the occupancy grid based on sensor readings, from deadreckoner
    void voxelTraversal(const Pose2D& sensorPose, const Pose2D& targetPose);


    bool isCellOccupied(int x, int y) const;
    inline int idx(int x, int y) const{ return y * WIN_N + x;}
    inline int8_t& at (int x, int y){ return L[idx(x, y)]; }

    void shiftX(int dx);
    void shiftY(int dy);
    void recenter(const Pose2D& p);
    bool worldInGrid(float wx, float wy, int& cx, int& cy) const;
    idx_xy worldToIndex(float wx, float wy) const;
    void reset(const Pose2D& robotPose);
    struct idx_xy {
        int x, y;
    };


private:
    RobotModel* _robot;
    Firefighter* _ff;
    int8_t L[WIN_N * WIN_N] = {0}; // 1D array representing the occupancy grid
    int org_cx = -WIN_N/2, org_cy = -WIN_N/2; // origin of the grid in world coordinates (in cells)


};

#endif // OCCUPANCY_GRID_H
