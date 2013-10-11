#ifndef lEDBLOCK_
#define LEDBLOCK_

#include <iostream>

#include "MemoryBlock.h"
#include <common/RobotInfo.h>

struct LEDBlock : public MemoryBlock {
public:
  LEDBlock()  {
    header.version = 1;
    header.size = sizeof(LEDBlock);
    
    for (int i=0; i<NUM_LEDS; i++) {
      values_[i]=0.0;
    }
    send_leds_ = true;
  }
  bool send_leds_;
  float values_[NUM_LEDS];
};


/*struct LEDBlock : public MemoryBlock {
public:
  LEDBlock()  {
    header.version = 0;
    header.size = sizeof(LEDBlock);
    
    for (int i=0; i<10; i++) {
      left_ear_[i]=0;
      right_ear_[i]=0;
    }
    for (int i=0; i<8; i++) {
      left_eye_[i]=Vector3<float>(0,0,0);
      right_eye_[i]=Vector3<float>(0,0,0);
    }
    chest_ = Vector3<float>(0,0,0);
    left_foot_ = Vector3<float>(0,0,0);
    right_foot_ = Vector3<float>(0,0,0);
  }

  float left_ear_[10];
  float right_ear_[10];

  Vector3<float> left_eye_[8];
  Vector3<float> right_eye_[8];

  Vector3<float> chest_;

  Vector3<float> left_foot_;
  Vector3<float> right_foot_;
};*/


#endif 
