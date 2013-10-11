#ifndef CROSS_ESTIMATOR_H
#define CROSS_ESTIMATOR_H

#include <vision/estimators/LikelihoodEstimator.h>

class CrossEstimator : public LikelihoodEstimator<11> {
  public:
    CrossEstimator() { setNames(); }
  protected:
    void setNames() {
      names_.push_back("g TL");
      names_.push_back("g TM");
      names_.push_back("g TR");
      names_.push_back("g L");
      names_.push_back("g R");
      names_.push_back("g BL");
      names_.push_back("g BM");
      names_.push_back("g BR");
      names_.push_back("p dist");
      names_.push_back("w width");
      names_.push_back("w height");
    }
};

#endif
