#ifndef PROCESSEDSONARBLOCK_VCLB37XJ
#define PROCESSEDSONARBLOCK_VCLB37XJ

#include "MemoryBlock.h"

struct ProcessedSonarBlock : public MemoryBlock {

  ProcessedSonarBlock() {

    header.version = 3;
    header.size = sizeof(ProcessedSonarBlock);

    on_left_ = false;
    on_right_ = false;
    on_center_ = false;
    left_distance_ = 0;
    right_distance_ = 0;
    center_distance_ = 0;

    bump_left_ = false;
    bump_right_ = false;

    sonar_module_update_ = false;
    sonar_module_enabled_ = true;
  }

  bool on_left_;
  bool on_right_;
  bool on_center_;

  float left_distance_;
  float right_distance_;
  float center_distance_;

  bool sonar_module_enabled_;
  bool sonar_module_update_;

  bool bump_left_;
  bool bump_right_;
};

#endif /* end of include guard: PROCESSEDSONARBLOCK_VCLB37XJ */
