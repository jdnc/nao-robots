#ifndef ROBOT_ESTIMATOR_H
#define ROBOT_ESTIMATOR_H

#include <vision/estimators/LikelihoodEstimator.h>

class RobotEstimator : public LikelihoodEstimator<10> {
  public:
    RobotEstimator() { setNames(); }
  protected:
    void setNames() {
      names_.push_back("w/h");
      names_.push_back("h/w");
      names_.push_back("kdist/wdist");
      names_.push_back("kdist/hdist");
      names_.push_back("tf disc");
      names_.push_back("jersey \%");
      names_.push_back("gw \%");
      names_.push_back("w \%");
      names_.push_back("c \%");
      names_.push_back("wheight");
    }
};

#endif
