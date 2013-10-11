#ifndef SIMIMAGEBLOCK_
#define SIMIMAGEBLOCK_

#include <common/RobotInfo.h>

#include "MemoryBlock.h"

struct SimImageBlock : public MemoryBlock {
public:
  SimImageBlock()  {
    header.version = 0;
    header.size = sizeof(SimImageBlock);
  }

  char image_[SIM_IMAGE_SIZE];
};

#endif 
