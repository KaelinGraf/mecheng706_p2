#ifndef POSE_H
#define POSE_H

#define KX 0.5f // correction factor for velocity to account for wheel slip, etc.
#define KY 0.5f  

struct rMatrix;
//represents current 2D pose of the robot (x, y, theta) in the world frame
struct Pose2D {
    float x=0, y=0, th=0, vx=0, vy=0;
    Pose2D chain(Pose2D other) const;
    rMatrix2D rAsMatrix() const;

};

struct rMatrix2D {
    float m00, m01, m10, m11;
};

class DeadReckoner {
public:
    DeadReckoner();
    ~DeadReckoner();
    Pose2D getPose() const;
    Pose2D  updatePose(float heading, float vx, float vy,float dt);
    

 
private:
    Pose2D currentPose;

};

#endif // POSE_H