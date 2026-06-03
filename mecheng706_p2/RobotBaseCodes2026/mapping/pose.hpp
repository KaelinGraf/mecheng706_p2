#ifndef POSE_H
#define POSE_H

#define KX 0.5f // correction factor for velocity to account for wheel slip, etc.
#define KY 0.5f  

struct rMatrix2D {
    float m00, m01, m10, m11;
};

//represents current 2D pose of the robot (x, y, theta) in the world frame
struct Pose2D {
    float x, y, th;
    Pose2D(float x = 0, float y = 0, float th = 0) : x(x), y(y), th(th) {}
    Pose2D chain(Pose2D other) const;
    rMatrix2D rAsMatrix() const;

};

class DeadReckoner {
public:
    DeadReckoner();
    ~DeadReckoner();
    Pose2D getPose() const;
    Pose2D  updatePose(float heading, float vx, float vy,long dt);
    

 
private:
    Pose2D currentPose;

};

#endif // POSE_H