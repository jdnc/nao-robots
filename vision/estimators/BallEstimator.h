#ifndef BALL_ESTIMATOR_H
#define BALL_ESTIMATOR_H

#include <vision/estimators/LikelihoodEstimator.h>

class BallEstimator : public LikelihoodEstimator<7> {
  public:
    BallEstimator() { setNames(); }
  protected:
    void setNames() {
      names_.push_back("orange \%");
      names_.push_back("g/w \%");
      names_.push_back("circ dev");
      names_.push_back("height");
      names_.push_back("kwdisc");
      names_.push_back("field dist");
      names_.push_back("vel");
    }
};

#endif
