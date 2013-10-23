#include <localization/Particle.h>

void Particle::move(Vector2<float> loc, AngRad theta) {
  Pose2D pose(theta, loc);
  move(pose);
}

void Particle::move(Pose2D pose) {
  this->loc += Point2D(pose.translation.x, pose.translation.y);
  this->theta += pose.rotation;
}

void Particle::move(Point2D loc, AngRad theta) {
  Pose2D pose(theta, loc.x, loc.y);
  move(pose);
};

void Particle::moveRelative(Vector2<float> loc, AngRad theta) {
  Pose2D pose(theta, loc);
  moveRelative(pose);
}

void Particle::moveRelative(Pose2D pose) {
  Pose2D self = Pose2D(this->theta, this->loc.x, this->loc.y);
  pose = pose.relativeToGlobal(self);
  pose.translation -= self.translation;
  pose.rotation -= self.rotation;
  move(pose);
}

void Particle::moveRelative(Point2D loc, AngRad theta) {
  Pose2D pose(theta, loc.x, loc.y);
  moveRelative(pose);
};

void Particle::placeRandomly() {
  float 
    rx = 2.0 * drand48() - 1, 
    ry = 2.0 * drand48() - 1,
    ra = 2.0 * drand48() - 1;
  Point2D newLoc(rx * FIELD_CORNER.x, ry * FIELD_CORNER.y);
  this->loc = newLoc;
  this->theta = ra * M_PI;
}

void Particle::degradeProbability(float amount) {
  this->prob *= amount;
}
