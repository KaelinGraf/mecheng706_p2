#include "mapping/occupancy_grid.h"

static const int8_t UNKNOWN = 0;   // log-odds 0



void OccupancyGrid::update(const Pose2D& robotPose){
    this->recenter(robotPose);
    Pose2D sensorPoses[5];
    _robot->getAllSensorPosesInWorld(robotPose, sensorPoses);
    for (int i = 0; i < 5; ++i) {
        Pose2D sensorPose = sensorPoses[i];
        float _sensor_reading = _ff->getSensorReading(static_cast<RobotModel::SensorType>(i)); //TODO get sensor reading from firefighter class
        if (_sensor_reading <= 0) continue; // Skip if no valid reading
        Pose2D _sensor_target = {_sensor_reading,0,0};//TODO get sensor reading from firefighter class
        Pose2D sensor_target_world = sensorPose.chain(_sensor_target); //calculates target in world coordinates by chaining the sensor pose with the reading (which is in the sensor's local frame)
        //We can now apply voxel traversal + probibalistic occupancy grid mapping
        this->voxelTraversal(sensorPose, sensor_target_world);
    }
}

void OccupancyGrid::voxelTraversal(const Pose2D& sensorPose, const Pose2D& targetPose){
    //amanatides & Woo 3D voxel traversal adapted to 2D grid
    idx_xy startCell = worldToIndex(sensorPose.x, sensorPose.y);
    idx_xy _endCell = worldToIndex(targetPose.x, targetPose.y); //note, this purposely can be out of bounds, we just early-exit traversal
    
    
}

// shift so new[ax] = old[ax+dx]; clear the exposed columns
void OccupancyGrid::shiftX(int dx) {
  if (dx > 0) {                      // robot moved +x
    int keep = WIN_N - dx;
    for (int ay = 0; ay < WIN_N; ++ay) {
      int8_t* row = &L[ay*WIN_N];
      memmove(row, row + dx, keep);          // pull world backward
      memset (row + keep, UNKNOWN, dx);      // clear leading columns
    }
  } else if (dx < 0) {
    int d = -dx, keep = WIN_N - d;
    for (int ay = 0; ay < WIN_N; ++ay) {
      int8_t* row = &L[ay*WIN_N];
      memmove(row + d, row, keep);
      memset (row, UNKNOWN, d);
    }
  }
}

// y shifts move whole rows (contiguous), so one memmove does it
void OccupancyGrid::shiftY(int dy) {
  if (dy > 0) {
    int keep = WIN_N - dy;
    memmove(&L[0], &L[dy*WIN_N], (size_t)keep*WIN_N);
    memset (&L[keep*WIN_N], UNKNOWN, (size_t)dy*WIN_N);
  } else if (dy < 0) {
    int d = -dy, keep = WIN_N - d;
    memmove(&L[d*WIN_N], &L[0], (size_t)keep*WIN_N);
    memset (&L[0], UNKNOWN, (size_t)d*WIN_N);
  }
}

// call every map update, BEFORE integrating new sensor readings
void OccupancyGrid::recenter(const Pose2D& p) {
  int rx = (int)floorf(p.x / CELL_CM);     // robot world cell — floorf, see gotcha
  int ry = (int)floorf(p.y / CELL_CM);
  int dx = (rx - WIN_N/2) - org_cx;        // how far origin must move to re-centre
  int dy = (ry - WIN_N/2) - org_cy;

  if (abs(dx) >= WIN_N || abs(dy) >= WIN_N) {
    memset(L, UNKNOWN, sizeof(L));         // jumped clean off the window -> all new
  } else {
    if (dx) shiftX(dx);
    if (dy) shiftY(dy);
  }
  org_cx += dx;                            // <-- the half your sketch was missing
  org_cy += dy;
}

bool OccupancyGrid::worldInGrid(float wx, float wy, int& cx, int& cy) const {
  cx = (int)floorf(wx / CELL_CM) - org_cx;
  cy = (int)floorf(wy / CELL_CM) - org_cy;
  return (cx >= 0 && cx < WIN_N && cy >= 0 && cy < WIN_N);
}

OccupancyGrid::idx_xy OccupancyGrid::worldToIndex(float wx, float wy) const {
    idx_xy cell;
    cell.x = (int)floorf(wx / CELL_CM) - org_cx;
    cell.y = (int)floorf(wy / CELL_CM) - org_cy;
    return cell;
}


void OccupancyGrid::reset(const Pose2D& robotPose) {
    memset(L, UNKNOWN, sizeof(L)); // Clear the grid
    org_cx = (int)floorf(robotPose.x / CELL_CM) - WIN_N/2;
    org_cy = (int)floorf(robotPose.y / CELL_CM) - WIN_N/2;
}