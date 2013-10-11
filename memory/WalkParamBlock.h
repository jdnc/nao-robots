#ifndef WALKPARAMBLOCK_REVJHZ5T
#define WALKPARAMBLOCK_REVJHZ5T

#include "MemoryBlock.h"
#include <motion/WalkEngineParameters.h>
#include <motion/HTWKWalkParameters.h>
#include <motion/BHWalkParameters.h>

struct WalkParamBlock : public MemoryBlock {
  WalkParamBlock():
    send_params_(false),
    params_()
  {
    header.version = 11;
    header.size = sizeof(WalkParamBlock);
  }

  bool send_params_;
  WalkEngineParameters params_;
  HTWKWalkParameters htwk_params_;
  BHWalkParameters bh_params_;
};

#endif /* end of include guard: WALKPARAMBLOCK_REVJHZ5T */
