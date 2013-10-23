#ifndef LOCALIZATIONBLOCK_
#define LOCALIZATIONBLOCK_

#include "MemoryBlock.h"
#include <localization/Particle.h>

#define NUM_PARTICLES 100

struct LocalizationBlock : public MemoryBlock {
public:
  LocalizationBlock()  {
    header.version = 9;
    header.size = sizeof(LocalizationBlock);
  }
  Particle particles[NUM_PARTICLES];
};

#endif 
