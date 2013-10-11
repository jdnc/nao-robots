#ifndef OPPONENTBLOCK_
#define OPPONENTBLOCK_

#include "MemoryBlock.h"
#include <math/Geometry.h>

#define MAX_OPP_MODELS_IN_MEM 8

struct OpponentBlock : public MemoryBlock {
public:
  OpponentBlock()  {
    header.version = 3;
    header.size = sizeof(OpponentBlock);
  }

  Point2D getLocation(int index){
    return Point2D(10.0*X00[index], 10.0*X10[index]);
  }

  Point2D getSD(int index){
    return Point2D(10.0*P00[index], 10.0*P11[index]);
  }

  float getAlpha(int index){
    return alpha[index];
  }

  //Model Info
  int modelNumber[MAX_OPP_MODELS_IN_MEM];
  float alpha[MAX_OPP_MODELS_IN_MEM];
  float X00[MAX_OPP_MODELS_IN_MEM];
  float X10[MAX_OPP_MODELS_IN_MEM];
  float X20[MAX_OPP_MODELS_IN_MEM];
  float X30[MAX_OPP_MODELS_IN_MEM];
  float P00[MAX_OPP_MODELS_IN_MEM];
  float P01[MAX_OPP_MODELS_IN_MEM];
  float P10[MAX_OPP_MODELS_IN_MEM];
  float P11[MAX_OPP_MODELS_IN_MEM];
  float P22[MAX_OPP_MODELS_IN_MEM];
  float P33[MAX_OPP_MODELS_IN_MEM];
  int frameLastObserved[MAX_OPP_MODELS_IN_MEM];

  float SRXX[MAX_OPP_MODELS_IN_MEM];
  float SRXY[MAX_OPP_MODELS_IN_MEM];
  float SRYY[MAX_OPP_MODELS_IN_MEM];

};

#endif 
