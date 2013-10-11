#ifndef SIM_TRUTH_DATA_H_
#define SIM_TRUTH_DATA_H_

#include "MemoryBlock.h"
#include <math/Pose2D.h>

struct SimTruthDataBlock : public MemoryBlock {
public:
  SimTruthDataBlock()
  {
    header.version = 0;
    header.size = sizeof(SimTruthDataBlock);

    has_truth_=false;
  }

  bool has_truth_;
  
  Pose2D robot_pos_;
  Pose2D ball_pos_;
};

#endif 
