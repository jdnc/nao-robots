#ifndef SENSORCALIBRATIONBLOCK_
#define SENSORCALIBRATIONBLOCK_

#include <iostream>

#include <common/RobotInfo.h>

#include "MemoryBlock.h"

struct SensorCalibrationBlock : public MemoryBlock {
public:
  SensorCalibrationBlock() {
    header.version = 1;
    header.size = sizeof(SensorCalibrationBlock);
    reset();
  }

  void reset() {
    is_calibrated_ = false;
    for (int i = 0; i < 6; i++)
      inertial_offsets_[i] = 0.0;

    fsr_feet_offset_ = 0.0; // positive for left foot
    fsr_left_side_offset_ = 0.0; // positive for left side of left foot
    fsr_left_front_offset_ = 0.0; // positive for front of left foot
    fsr_right_side_offset_ = 0.0; // positive for left side of right foot
    fsr_right_front_offset_ = 0.0; // positive for front of right foot

  }

  bool is_calibrated_;

  float inertial_offsets_[6];

  float fsr_feet_offset_; // positive for left foot
  float fsr_left_side_offset_; // positive for left side of left foot
  float fsr_left_front_offset_; // positive for front of left foot
  float fsr_right_side_offset_; // positive for left side of right foot
  float fsr_right_front_offset_; // positive for front of right foot
};

#endif 
