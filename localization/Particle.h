#ifndef PARTICLE_H
#define PARTICLE_H

#include <common/Field.h>
#include <math/Geometry.h>
#include <math/Pose2D.h>
#include <math/Vector2.h>

class Particle {
  public:
    Point2D loc;
    AngRad theta;
    float prob;
    void move(Pose2D pose);
    void move(Vector2<float> loc, AngRad theta);
    void move(Point2D loc, AngRad theta);
    void moveRelative(Pose2D pose);
    void moveRelative(Vector2<float> loc, AngRad theta);
    void moveRelative(Point2D loc, AngRad theta);
    void placeRandomly();
    void degradeProbability(float amount);
};
#endif
