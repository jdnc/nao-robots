#ifndef JOINTBLOCK_
#define JOINTBLOCK_

#include <iostream>

#include <common/RobotInfo.h>

#include "MemoryBlock.h"

struct JointBlock : public MemoryBlock {
public:
  JointBlock()  {
    header.version = 1;
    header.size = sizeof(JointBlock);
    for (int i=0; i<NUM_JOINTS; i++) {
      values_[i] = 0;
    }
  }

  float prevValues_[NUM_JOINTS];
  float values_[NUM_JOINTS];
  float changes_[NUM_JOINTS];
  float stiffness_[NUM_JOINTS];
};

#endif
