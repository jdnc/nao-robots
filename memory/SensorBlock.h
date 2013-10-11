#ifndef SENSORBLOCK_
#define SENSORBLOCK_

#include <iostream>

#include <common/RobotInfo.h>

#include "MemoryBlock.h"

struct SensorBlock : public MemoryBlock {
public:
  SensorBlock()  {
    header.version = 4;
    header.size = sizeof(SensorBlock);

    // initialize things
    for (int i = 0; i < NUM_SENSORS; i++){
      values_[i] = 0;
    }

    for (int i = 0; i < NUM_JOINTS; i++){
      joint_temperatures_[i] = 0;
    }

    fsr_feet_ = 0;
    fsr_left_side_ = 0;
    fsr_left_front_ = 0;
    fsr_right_side_ = 0;
    fsr_right_front_ = 0;
  }

  float getJointTemperature(int i){
    return joint_temperatures_[i];
  }

  float values_[NUM_SENSORS];
  float angleXVel;
  float angleYVel;
  float futureRoll;
  float futureTilt;

  float sonar_left_[NUM_SONAR_VALS];
  float sonar_right_[NUM_SONAR_VALS];

  float joint_temperatures_[NUM_JOINTS];

  float fsr_feet_; // positive means left foot has more weight
  float fsr_left_side_; // positive means left side of left foot has more weight
  float fsr_left_front_; // positive means front side of left foot has more weight
  float fsr_right_side_; // positive means left side of right foot has more weight
  float fsr_right_front_; // positive means front side of right foot has more weight

  float getValue(int index) { return values_[index]; };
};

#endif 
