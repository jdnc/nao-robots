#ifndef ODOMETRY_
#define ODOMETRY_

#include "MemoryBlock.h"
#include <math/Pose2D.h>

// all info needed by localization about robot motions
// includes kicking, walking, falling

struct Getup {
  enum GetupType {
    NONE,
    UNKNOWN,
    FRONT,
    BACK,
    NUM_GETUPS
  };
};

struct Fall {
  enum FallDir {
    NONE,
    UNKNOWN,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    NUM_FALL_DIRS
  };
};

struct OdometryBlock : public MemoryBlock {
public:

  OdometryBlock() {
    header.version = 3;
    header.size = sizeof(OdometryBlock);

    displacement = Pose2D(0,0,0);
    
    standing = true;
    didKick = false;

    getting_up_side_ = Getup::NONE;
    fall_direction_ = Fall::NONE;
  }

  void reset() {
    displacement = Pose2D(0,0,0);
    didKick = false;
    getting_up_side_ = Getup::NONE;

    // fall dir is not set by motion, so dont do the resetting in sync
    //fall_direction_ = Fall::NONE;
  }

  // walking
  Pose2D displacement;
  bool standing;

  // kicking
  bool didKick;
  float kickVelocity;
  float kickHeading;

  // falling
  Getup::GetupType getting_up_side_;
  Fall::FallDir fall_direction_;
};

#endif 
