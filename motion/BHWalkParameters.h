#ifndef BHWALKPARAMETERS_BENPHJGZ
#define BHWALKPARAMETERS_BENPHJGZ

#include <math/Pose2D.h>

// only a subset of the full parameters, for now
struct BHWalkParameters {
  BHWalkParameters():
    set(false)
  {}

  bool set;
  Pose2D speedMax;
  float speedMaxBackwards;
  Pose2D speedMaxChange;
};


#endif /* end of include guard: BHWALKPARAMETERS_BENPHJGZ */
