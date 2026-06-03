#ifndef POSE_H
#define POSE_H


struct Lut1D {
    const float* xs;   // ascending sample inputs (command magnitude)
    const float* ys;   // gain at each sample
    int          n;
    // Linear interpolation, clamped flat outside the sampled range.
    float eval(float x) const {
        if (n <= 0)       return 0.0f;
        if (x <= xs[0])   return ys[0];
        if (x >= xs[n-1]) return ys[n-1];
        int i = 1;
        while (i < n && xs[i] < x) ++i;            // first sample >= x
        float t = (x - xs[i-1]) / (xs[i] - xs[i-1]);
        return ys[i-1] + t * (ys[i] - ys[i-1]);
    }
};

extern const Lut1D KX_LUT;   // forward gain vs |vx|
extern const Lut1D KY_LUT;   // strafe  gain vs |vy|

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