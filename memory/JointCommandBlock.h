#ifndef JOINT_COMMAND_BLOCK_
#define JOINT_COMMAND_BLOCK_

#include <iostream>

#include <common/RobotInfo.h>

#include "MemoryBlock.h"

struct JointCommandBlock : public MemoryBlock {
public:
  JointCommandBlock()  {
    header.version = 7;
    header.size = sizeof(JointCommandBlock);
    body_angle_time_ = 1000;
    head_pitch_angle_time_ = 1000;
    head_yaw_angle_time_ = 1000;
    stiffness_time_ = 1000;
    for (int i=0; i<NUM_JOINTS; i++) {
      angles_[i] = 0;
      stiffness_[i] = 0;
    }
    send_sonar_command_ = false;
    send_body_angles_ = false;
    send_arm_angles_ = false;
    send_head_pitch_angle_ = false;
    send_head_yaw_angle_ = false;
    send_stiffness_ = false;
    sonar_command_ = 0.f;
    send_back_standup_ = false;

    head_pitch_angle_change_ = false;
    head_yaw_angle_change_ = false;
  }

  void setSendAllAngles(bool send, float angle_time = 1000.0f) {
    send_body_angles_ = send;
    send_head_pitch_angle_ = send;
    send_head_yaw_angle_ = send;

    body_angle_time_ = angle_time;
    head_pitch_angle_time_ = angle_time;
    head_yaw_angle_time_ = angle_time;

    head_pitch_angle_change_ = false;
    head_yaw_angle_change_ = false;
  }

  void setPoseRad(float src[NUM_JOINTS]) {
    for (int i=0; i<NUM_JOINTS; i++) {
      angles_[i]=src[i];
    }
  }

  void setPoseDeg(float src[NUM_JOINTS]) {
    for (int i=0; i<NUM_JOINTS; i++) angles_[i]=DEG_T_RAD*src[i];
  }

  void setJointCommand(int i, float val){
    angles_[i] = val;
  }
  
  void setJointCommandDeg(int i, float val){
    angles_[i] = DEG_T_RAD * val;
  }

  void setJointStiffness(int i, float val){
    stiffness_[i] = val;
  }

  void setHeadPan(float target, float time, bool is_change) {
    send_head_yaw_angle_ = true;
    angles_[HeadYaw] = target;
    head_yaw_angle_time_ = time;
    head_yaw_angle_change_ = is_change;
  }

  void setHeadTilt(float target, float time, bool is_change){
    send_head_pitch_angle_ = true;
    angles_[HeadPitch] = target;
    head_pitch_angle_time_ = time;
    head_pitch_angle_change_ = is_change;
  }

  void setAllStiffness(float target, float time) {
    send_stiffness_ = true;
    stiffness_time_ = time;
    for (int i = 0; i < NUM_JOINTS; i++)
      stiffness_[i] = target;
  }

  // head stuff (separate for yaw and pitch)
  bool send_head_pitch_angle_;
  bool send_head_yaw_angle_;
  bool head_pitch_angle_change_;
  bool head_yaw_angle_change_;
  float head_pitch_angle_time_;
  float head_yaw_angle_time_;

  // body stuff
  bool send_body_angles_;
  bool send_stiffness_;

  bool send_arm_angles_;
  float arm_command_time_;

  float angles_[NUM_JOINTS];
  float stiffness_[NUM_JOINTS];

  float body_angle_time_;
  float stiffness_time_;

  bool send_back_standup_;

  // sonar
  bool send_sonar_command_;
  float sonar_command_;
};

#endif 
