#include "MotionModule.h"

#include <common/RobotInfo.h>

void MotionModule::specifyMemoryDependency() {
  requiresMemoryBlock("body_model");
  requiresMemoryBlock("processed_joint_angles");
  requiresMemoryBlock("processed_joint_commands");
  requiresMemoryBlock("processed_sensors");
  requiresMemoryBlock("walk_request");
}

void MotionModule::specifyMemoryBlocks() {
  getMemoryBlock(body_model_,"body_model");
  getMemoryBlock(joint_angles_,"processed_joint_angles");
  getMemoryBlock(commands_,"processed_joint_commands");
  getMemoryBlock(sensors_,"processed_sensors");
  getMemoryBlock(walk_request_,"walk_request");
}

void MotionModule::initSpecificModule() {
  //walk_engine_.init(memory_);
}

void MotionModule::processFrame() {
}

void MotionModule::processWalkFrame() {
  /*
  if (walk_request_->motion_ == WalkRequestBlock::NONE)
    return;

  float command_time;
  if (walk_request_->motion_ == WalkRequestBlock::STAND) {
    command_time = 1000;
    walk_engine_.update(walk_engine_output_,true,false);
  } else {
    command_time = 100;
    walk_engine_.update(walk_engine_output_,false,true);
  }
  
  commands_->setPoseRad(walk_engine_output_.angles);
  commands_->angles_[HeadYaw] = 0;
  commands_->angles_[HeadPitch] = 0;
  commands_->angle_time_ = command_time;
  */
}
