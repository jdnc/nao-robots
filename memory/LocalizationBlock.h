#ifndef LOCALIZATIONBLOCK_
#define LOCALIZATIONBLOCK_

#include "MemoryBlock.h"

#define MAX_MODELS_IN_MEM 4


struct Sides {
  enum SideType {
    NOSIDES,
    LEFTSIDE,
    RIGHTSIDE,
    NUM_SIDES
  };
};

struct LocalizationBlock : public MemoryBlock {
public:
  LocalizationBlock()  {
    header.version = 9;
    header.size = sizeof(LocalizationBlock);
    bestModel = 0;
    for (int i = 0; i < MAX_MODELS_IN_MEM; i++){
      modelNumber[i] = i;
      alpha[i] = 0;
    }
    bestAlpha = 0;
    oppositeModels = false;
    fallenModels = false;
    numMateFlippedBalls = 0;
    numBadBallUpdates = 0;
    blueSide = 0;
  }

  int blueSide;

  int kfType;
  int bestModel;
  float bestAlpha;

  // indicate if there are more with significant likelihood that think we're facing the opposite way
  bool oppositeModels;
  bool fallenModels;
  int numMateFlippedBalls;
  int numBadBallUpdates;

  //Model Info
  int modelNumber[MAX_MODELS_IN_MEM];
  float alpha[MAX_MODELS_IN_MEM];
  float X00[MAX_MODELS_IN_MEM];
  float X10[MAX_MODELS_IN_MEM];
  float X20[MAX_MODELS_IN_MEM];
  float X30[MAX_MODELS_IN_MEM];
  float X40[MAX_MODELS_IN_MEM];
  float X50[MAX_MODELS_IN_MEM];
  float X60[MAX_MODELS_IN_MEM];
  float P00[MAX_MODELS_IN_MEM];
  float P01[MAX_MODELS_IN_MEM];
  float P10[MAX_MODELS_IN_MEM];
  float P11[MAX_MODELS_IN_MEM];
  float P22[MAX_MODELS_IN_MEM];
  float P33[MAX_MODELS_IN_MEM];
  float P44[MAX_MODELS_IN_MEM];

  float P34[MAX_MODELS_IN_MEM];

  float SRXX;
  float SRYY;
  float SRXY;
};

#endif 
