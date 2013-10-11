#ifndef ROBOTINFOBLOCK_CBJC25GF
#define ROBOTINFOBLOCK_CBJC25GF

#include <common/RobotDimensions.h>
#include <common/MassCalibration.h>

#include "MemoryBlock.h"

struct RobotInfoBlock : public MemoryBlock {
public:
  RobotInfoBlock()  {
    header.version = 1;
    header.size = sizeof(RobotInfoBlock);
  }  

  RobotDimensions dimensions_;
  MassCalibration mass_calibration_;

};

#endif /* end of include guard: ROBOTINFOBLOCK_CBJC25GF */

