#include "BHWalkModule.h"

#include <math/Geometry.h>

// BH
#include "WalkingEngine.h"
#include "Representations/Configuration/MassCalibration.h"

#include <memory/FrameInfoBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/KickRequestBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/RobotInfoBlock.h>
#include <memory/SensorBlock.h>
#include <memory/WalkInfoBlock.h>
#include <memory/WalkParamBlock.h>
#include <memory/WalkRequestBlock.h>

BHWalkModule::BHWalkModule():
  slow_stand_start(-1),
  slow_stand_end(-1),
  walk_requested_start_time(-1),
  prev_kick_active_(false),
  arms_close_to_targets_(false),
  arm_state_(-1),
  arm_state_change_(-1),
  last_walk_or_stand_(-1),
  step_into_kick_state_(NONE),
  time_step_into_kick_finished_(0)
{
  utJointToBHJoint[HeadYaw] = JointData::HeadYaw;
  utJointToBHJoint[HeadPitch] = JointData::HeadPitch;

  utJointToBHJoint[LShoulderPitch] = JointData::LShoulderPitch;
  utJointToBHJoint[LShoulderRoll] = JointData::LShoulderRoll;
  utJointToBHJoint[LElbowYaw] = JointData::LElbowYaw;
  utJointToBHJoint[LElbowRoll] = JointData::LElbowRoll;

  utJointToBHJoint[RShoulderPitch] = JointData::RShoulderPitch;
  utJointToBHJoint[RShoulderRoll] = JointData::RShoulderRoll;
  utJointToBHJoint[RElbowYaw] = JointData::RElbowYaw;
  utJointToBHJoint[RElbowRoll] = JointData::RElbowRoll;

  utJointToBHJoint[LHipYawPitch] = JointData::LHipYawPitch;
  utJointToBHJoint[LHipRoll] = JointData::LHipRoll;
  utJointToBHJoint[LHipPitch] = JointData::LHipPitch;
  utJointToBHJoint[LKneePitch] = JointData::LKneePitch;
  utJointToBHJoint[LAnklePitch] = JointData::LAnklePitch;
  utJointToBHJoint[LAnkleRoll] = JointData::LAnkleRoll;
  
  utJointToBHJoint[RHipYawPitch] = JointData::RHipYawPitch;
  utJointToBHJoint[RHipRoll] = JointData::RHipRoll;
  utJointToBHJoint[RHipPitch] = JointData::RHipPitch;
  utJointToBHJoint[RKneePitch] = JointData::RKneePitch;
  utJointToBHJoint[RAnklePitch] = JointData::RAnklePitch;
  utJointToBHJoint[RAnkleRoll] = JointData::RAnkleRoll;
}

BHWalkModule::~BHWalkModule() {
}

void BHWalkModule::specifyMemoryDependency() {
  requiresMemoryBlock("frame_info");
  requiresMemoryBlock("processed_joint_angles");
  requiresMemoryBlock("raw_joint_angles");
  requiresMemoryBlock("processed_joint_commands");
  requiresMemoryBlock("kick_request");
  requiresMemoryBlock("odometry");
  requiresMemoryBlock("robot_info");
  requiresMemoryBlock("raw_sensors");
  requiresMemoryBlock("walk_info");
  requiresMemoryBlock("walk_param");
  requiresMemoryBlock("walk_request");
}

void BHWalkModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(frame_info_,"frame_info");
  getOrAddMemoryBlock(joints_,"processed_joint_angles");
  getOrAddMemoryBlock(raw_joints_,"raw_joint_angles");
  getOrAddMemoryBlock(commands_,"processed_joint_commands");
  getOrAddMemoryBlock(kick_request_,"kick_request");
  getOrAddMemoryBlock(odometry_,"odometry");
  getOrAddMemoryBlock(robot_info_,"robot_info");
  getOrAddMemoryBlock(sensors_,"raw_sensors");
  getOrAddMemoryBlock(walk_info_,"walk_info");
  getOrAddMemoryBlock(walk_params_,"walk_param");
  getOrAddMemoryBlock(walk_request_,"walk_request");
}

void BHWalkModule::initSpecificModule() {
  std::string config_path = memory_->data_path_;
  config_path += "config/";
  walk_engine_ = new WalkingEngine(config_path);
  if (frame_info_->source == MEMORY_SIM) {
    walk_engine_->theFrameInfo.cycleTime = 0.02f;
  } else {
    walk_engine_->theFrameInfo.cycleTime = 0.01f;
  }
  walk_engine_->currentMotionType = WalkingEngine::stand;
  walk_engine_->theMotionRequest.motion = MotionRequest::specialAction;
  for (int i = 0; i < MotionRequest::numOfMotions; i++)
    walk_engine_->theMotionSelection.ratios[i] = 0;
  walk_engine_->theMotionSelection.ratios[MotionRequest::specialAction] = 1.0;


  setMassCalibration();
  setRobotDimensions();
}

void BHWalkModule::processWalkParams() {
  if (!walk_params_->send_params_)
    return;
  walk_params_->send_params_ = false;
  BHWalkParameters &params = walk_params_->bh_params_;
  walk_engine_->init(params); // reloads parameters from files
  setMassCalibration();
  setRobotDimensions();
}

void BHWalkModule::processFrame() {
  // times
  unsigned int time = 1000 * frame_info_->seconds_since_start + 1000; // + 1000 so that it's never 0, because bhuman says they don't like that
  walk_engine_->theFrameInfo.time = walk_engine_->theJointData.timeStamp = walk_engine_->theSensorData.timeStamp = time;


  processWalkParams();
  processWalkRequest();

  assert((int)JointData::numOfJoints == NUM_JOINTS);

  JointData &joint_data_BH = walk_engine_->theJointData;
  SensorData &sensors_BH = walk_engine_->theSensorData;

  // inputs to walk
  // joints
  for (int ut_ind = 0; ut_ind < NUM_JOINTS; ut_ind++) {
    //int ind = jointMapping[i];
    //joint_data_BH.angles[i] = robot_joint_signs[ind] * raw_joints_->values_[ind];
    int bh_ind = utJointToBHJoint[ut_ind];
    //joint_data_BH.angles[bh_ind] = robot_joint_signs[ut_ind] * raw_joints_->values_[ut_ind];
    joint_data_BH.angles[bh_ind] = raw_joints_->values_[ut_ind];
  }
  // other sensors
  sensors_BH.data[SensorData::gyroX] = sensors_->values_[gyroX];
  sensors_BH.data[SensorData::gyroY] = sensors_->values_[gyroY];
  sensors_BH.data[SensorData::accX] = sensors_->values_[accelX];
  sensors_BH.data[SensorData::accY] = sensors_->values_[accelY];
  sensors_BH.data[SensorData::accZ] = sensors_->values_[accelZ];
  sensors_BH.data[SensorData::angleX] = sensors_->values_[angleX];
  sensors_BH.data[SensorData::angleY] = sensors_->values_[angleY];
  sensors_BH.data[SensorData::fsrLFL] = sensors_->values_[fsrLFL];
  sensors_BH.data[SensorData::fsrLFR] = sensors_->values_[fsrLFR];
  sensors_BH.data[SensorData::fsrLBL] = sensors_->values_[fsrLRL];
  sensors_BH.data[SensorData::fsrLBR] = sensors_->values_[fsrLRR];
  sensors_BH.data[SensorData::fsrRFL] = sensors_->values_[fsrRFL];
  sensors_BH.data[SensorData::fsrRFR] = sensors_->values_[fsrRFR];
  sensors_BH.data[SensorData::fsrRBL] = sensors_->values_[fsrRRL];
  sensors_BH.data[SensorData::fsrRBR] = sensors_->values_[fsrRRR];
 
  // roboEireann hacks
  //sensors_BH.data[SensorData::accY] *= -1;
  //sensors_BH.data[SensorData::gyroX] *= 0.7;
  
  // do our version of ground contact state
  float minForce = 0.5;
  bool contact = true; //arms_close_to_targets_; // only think we're stable if we're not moving our arms
  int contactInds[4][4] = {
    {fsrLFL,fsrLFR,fsrLRL,fsrLRR}, // left
    {fsrRFL,fsrRFR,fsrRRL,fsrRRR}, // right
    {fsrLFL,fsrLFR,fsrRFL,fsrRFR}, // front
    {fsrLRL,fsrLRR,fsrRRL,fsrRRR} // back
  };
  //std::string contactNames[4] = {"left","right","front","back"};
  //if (!contact)
    //std::cout << "unstable arms" << std::endl;
  for (int i = 0; i < 4; i++) {
    float force = 0;
    for (int j = 0; j < 4; j++)
      force += sensors_->values_[contactInds[i][j]];
    if (force < minForce) {
      //std::cout << "unstable " << contactNames[i] << std::endl;
      contact = false;
      break;
    }
  }
  
  // say contact and arms are good if we've been here for a long time
  if ((!walk_engine_->theInertiaSensorData.calibrated) && (walk_requested_start_time > 0) && (frame_info_->seconds_since_start - walk_requested_start_time > 3.0)) {
    //std::cout << "forcing contact on" << std::endl;
    contact = true;
  }

  walk_engine_->theGroundContactState.contact = contact;

/* 
  if (walk_engine_->theMotionRequest.motion == MotionRequest::specialAction) {
    setWalkHeight(258.0f);
  } else {
    float instability = walk_engine_->instability.getAverage();
    float stabilityFrac = (instability - walk_engine_->p.stabilizerOffThreshold) / (walk_engine_->p.stabilizerOnThreshold - walk_engine_->p.stabilizerOffThreshold);
    float z = 258.0f - 20 * stabilityFrac;
    z = crop(z,238,258);
    setWalkHeight(z);
  }
*/

  // DO IT
  walk_engine_->update(walk_request_->is_penalised_,walk_engine_->theMotionRequest.motion == MotionRequest::specialAction);
  
  MotionRequest::Motion &motion = walk_engine_->theMotionRequest.motion;
  float *motion_ratios = walk_engine_->theMotionSelection.ratios;
  
  // slow stand
  static bool prev_slow_stand = false;
  if (doingSlowStand()) {
    doSlowStand();
    last_walk_or_stand_ = frame_info_->seconds_since_start;
    prev_slow_stand = true;
  } else if (prev_slow_stand) {
    std::cout << frame_info_->frame_id << " done slow stand" << std::endl;
    prev_slow_stand = false;
  }
  
  if ((frame_info_->seconds_since_start - last_walk_or_stand_) > 0.3) {
    arm_state_ = -1;
  }

  // odometry
  //odometry_->standing = (motion != MotionRequest::walk);
  odometry_->standing = (walk_engine_->currentMotionType != WalkingEngine::stepping);

  Pose2DBH &odom = walk_engine_->walkingEngineOutput.odometryOffset;
  Pose2D delta(odom.rotation,odom.translation.x,odom.translation.y);
  odometry_->displacement += delta;
  bool kick_active = walk_engine_->kickPlayer.isActive();
  if (!kick_active && prev_kick_active_) { 
    odometry_->didKick = true;
    odometry_->kickVelocity = kick_distance_ / 1.2;
    odometry_->kickHeading = kick_angle_;
  }
  prev_kick_active_ = kick_active;

  //static Pose2D odom;
  //odom += walk_engine_->walkingEngineOutput.odometryOffset;
  //std::cout << walk_engine_->walkingEngineOutput.odometryOffset.translation.x << " " << walk_engine_->walkingEngineOutput.odometryOffset.translation.y << " " << walk_engine_->walkingEngineOutput.odometryOffset.rotation << std::endl;
  //std::cout << odom.translation.x << " " << odom.translation.y << " " << odom.rotation << std::endl;
  
  walk_info_->finished_with_target_ = walk_engine_->finishedWithTarget;
  walk_info_->walk_is_active_ = (walk_engine_->currentMotionType == WalkingEngine::stepping);
  walk_info_->instability_ = walk_engine_->instability.getAverage();
  walk_info_->instable_ = walk_engine_->instable;
  walk_info_->stabilizer_on_threshold_ = walk_engine_->p.stabilizerOnThreshold;
  walk_info_->stabilizer_off_threshold_ = walk_engine_->p.stabilizerOffThreshold;
  setPose2D(walk_info_->robot_velocity_,walk_engine_->walkingEngineOutput.speed);
  setPose2D(walk_info_->robot_relative_next_position_,walk_engine_->walkingEngineOutput.upcomingOdometryOffset);
  walk_info_->is_stance_left_ = (walk_engine_->observedPendulumPlayer.supportLeg == WalkingEngine::left);
  walk_info_->frac_of_step_completed_ = ((walk_engine_->observedPendulumPlayer.t - walk_engine_->observedPendulumPlayer.tb) / (walk_engine_->observedPendulumPlayer.te - walk_engine_->observedPendulumPlayer.tb));
  walk_info_->time_remaining_in_step_ = walk_engine_->observedPendulumPlayer.te - walk_engine_->observedPendulumPlayer.t;

  if ((motion_ratios[MotionRequest::walk] < 0.01f) && (motion_ratios[MotionRequest::stand] < 0.01f)) {
    return;
  }
  last_walk_or_stand_ = frame_info_->seconds_since_start;
  //if ((motion != MotionRequest::stand) && (motion != MotionRequest::walk) && (walk_engine_->walkingEngineOutput.isLeavingPossible))
    //return;

  // outputs from walk
  for (int ut_ind = BODY_JOINT_OFFSET; ut_ind < NUM_JOINTS; ut_ind++) {
    //int ind = jointMapping[i];
    //commands_->angles_[ind] = robot_joint_signs[ind] * walk_engine_->joint_angles[i];
    //commands_->stiffness_[ind] = walk_engine_->joint_hardnesses[i];
    int bh_ind = utJointToBHJoint[ut_ind];
    commands_->angles_[ut_ind] = robot_joint_signs[ut_ind] * walk_engine_->joint_angles[bh_ind];
    commands_->stiffness_[ut_ind] = walk_engine_->joint_hardnesses[bh_ind];
  }
  commands_->stiffness_[HeadPitch] = 1.0;
  commands_->stiffness_[HeadYaw] = 1.0;
  selectivelySendStiffness();

  setArms(commands_->angles_,0.01);
  commands_->send_body_angles_ = true;
  commands_->body_angle_time_ = 10;
}

void BHWalkModule::setKickStepParams() {
  int type = walk_request_->step_kick_type_;
  bool mirrored = (type - 1) % 2 != 0;
  assert(!mirrored);
  WalkingEngineKick& kick = walk_engine_->kickPlayer.kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
  kick.preStepSizeXValue = walk_request_->pre_kick_step_.translation.x;
  kick.preStepSizeYValue = walk_request_->pre_kick_step_.translation.y;
  kick.preStepSizeRValue = walk_request_->pre_kick_step_.rotation;
  kick.stepSizeXValue = walk_request_->kick_step_.translation.x;
  kick.stepSizeYValue = walk_request_->kick_step_.translation.y;
  kick.stepSizeRValue = walk_request_->kick_step_.rotation;
  kick.refXValue = walk_request_->kick_step_ref_x_;
}

void BHWalkModule::processWalkRequest() {
  if (walk_request_->set_kick_step_params_)
    setKickStepParams();


  if ((!walk_request_->new_command_) || (step_into_kick_state_ == PERFORMING))
    return;

  WalkRequest &walk_request_BH = walk_engine_->theMotionRequest.walkRequest;
  MotionRequest::Motion &motion = walk_engine_->theMotionRequest.motion;


  if (walk_request_->motion_ == WalkRequestBlock::STAND) {
    motion = MotionRequest::stand;
    walk_request_BH = WalkRequest();
    walk_requested_start_time = -1;
    //std::cout << "STAND" << std::endl;
  } else if (walk_request_->motion_ == WalkRequestBlock::WALK) {
    if (walk_requested_start_time < 0) {
      walk_requested_start_time = frame_info_->seconds_since_start;
    }
    // don't walk until we're calibrated
    if (!walk_engine_->theInertiaSensorData.calibrated)
      motion = MotionRequest::stand;
    else
      motion = MotionRequest::walk;
    //std::cout << frame_info_->frame_id << " WALK" << std::endl;
  } else {
    motion = MotionRequest::specialAction;
    walk_requested_start_time = -1;
  }

  //if (walk_request_->motion_ == WalkRequestBlock::NONE) {
    //std::cout << "ABORT INSTANT" << std::endl;
    //// ABORT INSTANTANEOUSLY
    //float *ratios = walk_engine_->theMotionSelection.ratios;
    //for (int i = 0; i < MotionRequest::numOfMotions; i++)
      //ratios[i] = 0;
    //ratios[MotionRequest::specialAction] = 1.0;
  //}

  if (walk_request_->walk_to_target_) {
    if (walk_request_BH.mode != WalkRequest::targetMode) {
      walk_engine_->time_walk_target_started_ = walk_engine_->theFrameInfo.time;
    }
    walk_request_BH.mode = WalkRequest::targetMode;
  } else if (walk_request_->percentage_speed_)
    walk_request_BH.mode = WalkRequest::percentageSpeedMode;
  else
    walk_request_BH.mode = WalkRequest::speedMode;
  
  walk_request_BH.pedantic = walk_request_->pedantic_walk_;


  walk_request_BH.speed.rotation = walk_request_->speed_.rotation;
  walk_request_BH.speed.translation.x = walk_request_->speed_.translation.x;
  walk_request_BH.speed.translation.y = walk_request_->speed_.translation.y;
  
  walk_request_BH.target.rotation = walk_request_->target_point_.rotation;
  walk_request_BH.target.translation.x = walk_request_->target_point_.translation.x;
  walk_request_BH.target.translation.y = walk_request_->target_point_.translation.y;
  if (walk_request_->walk_to_target_) {
    walk_request_BH.speed.translation.x = 1.0;
    walk_request_BH.speed.translation.y = 1.0;
    walk_request_BH.speed.rotation = 1.0;
  }

  walk_request_BH.kickBallPosition.x = kick_request_->ball_rel_x_;
  walk_request_BH.kickBallPosition.y = kick_request_->ball_rel_y_;

  // kicks
  if (walk_request_->perform_kick_) {
    if (fabs(walk_request_->kick_heading_) < DEG_T_RAD * 15) {
      if (walk_request_->kick_with_left_)
        walk_request_BH.kickType = WalkRequest::left;
      else
        walk_request_BH.kickType = WalkRequest::right;
    } else {
      if (walk_request_->kick_heading_ > DEG_T_RAD*70)
        walk_request_BH.kickType = WalkRequest::sidewardsRight;
      else if (walk_request_->kick_heading_ > 0)
        walk_request_BH.kickType = WalkRequest::angleRight;
      else if (walk_request_->kick_heading_ < -DEG_T_RAD*50)
        walk_request_BH.kickType = WalkRequest::sidewardsLeft;
      else
        walk_request_BH.kickType = WalkRequest::angleLeft;
    }
    //std::cout << "PERFORM KICK: " << WalkRequest::getName(walk_request_BH.kickType) << std::endl;
    kick_distance_ = walk_request_->kick_distance_;
    kick_angle_ = walk_request_->kick_heading_;
  } else {
    walk_request_BH.kickType = WalkRequest::none;
  }
 
  if (!doingSlowStand() && shouldStartSlowStand()) {
    startSlowStand();
  }

  if (doingSlowStand()) {
    motion = MotionRequest::specialAction; // to make sure bhuman doesn't recalibrate during this time
  } 

  walk_engine_->walk_decides_finished_with_target_ = walk_request_->walk_decides_finished_with_target_;
  walk_engine_->finished_with_target_max_x_error_ = walk_request_->finished_with_target_max_x_error_;
  walk_engine_->finished_with_target_max_y_error_ = walk_request_->finished_with_target_max_y_error_;
  walk_engine_->finished_with_target_min_y_error_ = walk_request_->finished_with_target_min_y_error_;
}

void BHWalkModule::getArmsForState(int state, Joints angles) {
  if (state <= 1) {
    angles[LShoulderPitch] = DEG_T_RAD * -116;
    angles[LShoulderRoll] = DEG_T_RAD * 12;
    angles[LElbowYaw] = DEG_T_RAD * -85;
    angles[LElbowRoll] = DEG_T_RAD * -0;
    angles[RShoulderPitch] = DEG_T_RAD * -116;
    angles[RShoulderRoll] = DEG_T_RAD * 12;
    angles[RElbowYaw] = DEG_T_RAD * -85;
    angles[RElbowRoll] = DEG_T_RAD * -0;
    if (state == 1) {
      angles[LElbowYaw] = DEG_T_RAD * 25;
      angles[RElbowYaw] = DEG_T_RAD * 25;
    }
  } else {
    angles[LShoulderPitch] = DEG_T_RAD * -116;
    angles[LShoulderRoll] = DEG_T_RAD * 8;
    angles[LElbowYaw] = DEG_T_RAD * 25;
    angles[LElbowRoll] = DEG_T_RAD * -53;
    angles[RShoulderPitch] = DEG_T_RAD * -116;
    angles[RShoulderRoll] = DEG_T_RAD * 8;
    angles[RElbowYaw] = DEG_T_RAD * 25;
    angles[RElbowRoll] = DEG_T_RAD * -53;
  }
}
  
void BHWalkModule::determineStartingArmState() {
  // start from the current joints
  for (int i = ARM_JOINT_FIRST; i <= ARM_JOINT_LAST; i++) {
    armStart[i] = joints_->values_[i];
  }
  
  // if arms are far out, start at 0
  //for (int i = 0; i < 2; i++) {
    //int shoulderPitch = LShoulderPitch;
    //int shoulderRoll = LShoulderRoll;
    //if (i == 1) {
      //shoulderPitch = RShoulderPitch;
      //shoulderRoll = RShoulderRoll;
    //}
    //if ((joints_->values_[shoulderRoll] > DEG_T_RAD * 45) || // arm is up (from cross)
        //(joints_->values_[shoulderPitch] > DEG_T_RAD * -90)) { // arm is in front
      //arm_state_ = 0;
      //return;
    //}
  //}

  for (int state = 2; state >= 1; state--) {
    Joints temp;
    getArmsForState(state,temp);
    bool acceptable = true;
    for (int i = ARM_JOINT_FIRST; i <= ARM_JOINT_LAST; i++) {
      if (fabs(joints_->values_[i] - temp[i]) > DEG_T_RAD * 10) {
        //std::cout << JointNames[i] << " is too far for state " << state << " sensed: " << RAD_T_DEG * joints_->values_[i] << " " << " desired: " << RAD_T_DEG * temp[i] << std::endl;
        acceptable = false;
        break;
      }
    }
    if (acceptable) {
      arm_state_ = state;
      //std::cout << "selected: " << arm_state_ << std::endl;
      return;
    }
  }
  // default to 0 if everything else has been bad
  arm_state_ = 0;
}

void BHWalkModule::setArms(Joints angles, float timeInSeconds) {
  float armStateTimes[3] = {1.0,0.5,0.5};

  if (timeInSeconds < 0.01)
    timeInSeconds = 0.01;

  float timePassed = frame_info_->seconds_since_start - arm_state_change_;
  
  int prevState = arm_state_;
  if (arm_state_ < 0) {
    determineStartingArmState();
  } else if (arm_state_ >= 2)
    arm_state_ = 2;
  else if (timePassed > armStateTimes[arm_state_]) {
    arm_state_ += 1;
  }
  
  // goal keeper only does state 0 ever
  if (walk_request_->keep_arms_out_){
    arm_state_ = 0;
  }

  if (arm_state_ != prevState) {
    //std::cout << frame_info_->frame_id << " changing state from " << prevState << " to " << arm_state_ << " after " << timePassed << " seconds" << std::endl;
    arm_state_change_ = frame_info_->seconds_since_start;
    timePassed = 0;
    // save previous commands as start
    if (prevState >= 0) {
      getArmsForState(prevState,armStart);
    }
  }

  
  // calculate the fraction we're into this state
  float frac = (timePassed + timeInSeconds) / armStateTimes[arm_state_];
  frac = crop(frac,0,1.0);

  // get desired angles
  getArmsForState(arm_state_,angles);

  // set the values
  for (int i = ARM_JOINT_FIRST; i <= ARM_JOINT_LAST; i++) {
    float des = angles[i];
    float orig = armStart[i];
    float val = frac * (des - orig) + orig;
    angles[i] = val;
  }

  // see if the arms are stable
  float maxDeltaDesired = 0;
  float maxDeltaDetected = 0;
  for (int i = LShoulderPitch; i <= RElbowRoll; i++) {
    float delta = angles[i] - joints_->values_[i];
    maxDeltaDesired = max(fabs(delta),maxDeltaDesired);
    maxDeltaDetected = max(fabs(joints_->changes_[i]),maxDeltaDetected);
  }
  arms_close_to_targets_ = (maxDeltaDesired < DEG_T_RAD * 20) || (maxDeltaDetected < DEG_T_RAD * 0.35);
}

const float BHWalkModule::STAND_ANGLES[NUM_JOINTS] = {
  0,
  -0.366519,
  0,
  0.00669175,
  -0.548284,
  1.04734,
  -0.499061,
  -0.00669175,
  0,
  -0.00669175,
  -0.548284,
  1.04734,
  -0.499061,
  0.00669175,
  -1.5708,
  0.2,
  -1.5708,
  -0.2,
  -1.5708,
  0.2,
  -1.5708,
  -0.2
};


//const int BHWalkModule::jointMapping[NUM_JOINTS] = {
  //HeadYaw,
  //HeadPitch,
  //LShoulderPitch,
  //LShoulderRoll,
  //LElbowYaw,
  //LElbowRoll,
  //RShoulderPitch,
  //RShoulderRoll,
  //RElbowYaw,
  //RElbowRoll,
  //LHipYawPitch,
  //LHipRoll,
  //LHipPitch,
  //LKneePitch,
  //LAnklePitch,
  //LAnkleRoll,
  //RHipYawPitch,
  //RHipRoll,
  //RHipPitch,
  //RKneePitch,
  //RAnklePitch,
  //RAnkleRoll
//};

void BHWalkModule::setMassCalibration() {
  int bhuman_inds[MassCalibrationBH::numOfLimbs] = {
    MassCalibrationBH::neck,
    MassCalibrationBH::head,
    MassCalibrationBH::shoulderLeft,
    MassCalibrationBH::bicepsLeft,
    MassCalibrationBH::elbowLeft,
    MassCalibrationBH::foreArmLeft,
    MassCalibrationBH::shoulderRight,
    MassCalibrationBH::bicepsRight,
    MassCalibrationBH::elbowRight,
    MassCalibrationBH::foreArmRight,
    MassCalibrationBH::pelvisLeft,
    MassCalibrationBH::hipLeft,
    MassCalibrationBH::thighLeft,
    MassCalibrationBH::tibiaLeft,
    MassCalibrationBH::ankleLeft,
    MassCalibrationBH::footLeft,
    MassCalibrationBH::pelvisRight,
    MassCalibrationBH::hipRight,
    MassCalibrationBH::thighRight,
    MassCalibrationBH::tibiaRight,
    MassCalibrationBH::ankleRight,
    MassCalibrationBH::footRight,
    MassCalibrationBH::torso
  };
  
  int ut_inds[MassCalibrationBH::numOfLimbs] = {
    BodyPart::neck,
    BodyPart::head,
    BodyPart::left_shoulder,
    BodyPart::left_bicep,
    BodyPart::left_elbow,
    BodyPart::left_forearm,
    BodyPart::right_shoulder,
    BodyPart::right_bicep,
    BodyPart::right_elbow,
    BodyPart::right_forearm,
    BodyPart::left_pelvis,
    BodyPart::left_hip,
    BodyPart::left_thigh,
    BodyPart::left_tibia,
    BodyPart::left_ankle,
    BodyPart::left_foot,
    BodyPart::right_pelvis,
    BodyPart::right_hip,
    BodyPart::right_thigh,
    BodyPart::right_tibia,
    BodyPart::right_ankle,
    BodyPart::right_foot,
    BodyPart::torso
  };

  for (int i = 0; i < MassCalibrationBH::numOfLimbs; i++) {
    walk_engine_->theMassCalibration.masses[bhuman_inds[i]].mass = robot_info_->mass_calibration_.masses[ut_inds[i]].mass;
    for (int j = 0; j < 3; j++)
      walk_engine_->theMassCalibration.masses[bhuman_inds[i]].offset[j] = robot_info_->mass_calibration_.masses[ut_inds[i]].offset[j];
  }
}

void BHWalkModule::setRobotDimensions() {
  RobotDimensionsBH &bh = walk_engine_->theRobotDimensions;
  RobotDimensions &ut = robot_info_->dimensions_;

  bh.xHeadTiltToCamera = ut.values_[RobotDimensions::xHeadTiltToBottomCamera];
  bh.zHeadTiltToCamera = ut.values_[RobotDimensions::zHeadTiltToBottomCamera];
  bh.headTiltToCameraTilt = ut.values_[RobotDimensions::tiltOffsetToBottomCamera]; // TODO is this correct
  bh.xHeadTiltToUpperCamera = ut.values_[RobotDimensions::xHeadTiltToTopCamera];
  bh.zHeadTiltToUpperCamera = ut.values_[RobotDimensions::zHeadTiltToTopCamera];
  bh.headTiltToUpperCameraTilt = ut.values_[RobotDimensions::tiltOffsetToTopCamera]; // TODO is this correct

  bh.lengthBetweenLegs = ut.values_[RobotDimensions::lengthBetweenLegs];
  bh.upperLegLength = ut.values_[RobotDimensions::upperLegLength];
  bh.lowerLegLength = ut.values_[RobotDimensions::lowerLegLength];
  bh.heightLeg5Joint = ut.values_[RobotDimensions::footHeight];
  bh.zLegJoint1ToHeadPan = ut.values_[RobotDimensions::zLegJoint1ToHeadPan];
  bh.armOffset[0] = ut.values_[RobotDimensions::armOffset1];
  bh.armOffset[1] = ut.values_[RobotDimensions::armOffset2];
  bh.armOffset[2] = ut.values_[RobotDimensions::armOffset3];
  bh.yElbowShoulder = ut.values_[RobotDimensions::elbowOffsetY];
  bh.upperArmLength = ut.values_[RobotDimensions::upperArmLength];
  bh.lowerArmLength = ut.values_[RobotDimensions::lowerArmLength];
}

void BHWalkModule::selectivelySendStiffness() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (fabs(joints_->stiffness_[i] - commands_->stiffness_[i]) > 0.01) {
      commands_->send_stiffness_ = true;
      commands_->stiffness_time_ = 10;
      return;
    }
  }
}
  
void BHWalkModule::setPose2D(Pose2D &dest, const Pose2DBH &src) {
  dest.translation.x = src.translation.x;
  dest.translation.y = src.translation.y;
  dest.rotation = src.rotation;
}

bool BHWalkModule::doingSlowStand() {
  return frame_info_->seconds_since_start < slow_stand_end;
}

void BHWalkModule::doSlowStand() {
  float dt = slow_stand_end - frame_info_->seconds_since_start;
  for (int i = BODY_JOINT_OFFSET; i < NUM_JOINTS; i++) {
    commands_->angles_[i] = STAND_ANGLES[i];
    commands_->stiffness_[i] = 1.0;
  }
  setArms(commands_->angles_,dt);
  commands_->send_body_angles_ = true;
  commands_->body_angle_time_ = 1000 * dt;
  selectivelySendStiffness();
}

void BHWalkModule::setWalkHeight(float z) {
  float maxDec = 10.0 / 100.0; // 1 cm every second (i.e. 100 frames)
  float maxInc = 30.0 / 100.0; // 3 cm every second (i.e. 100 frames)

  float delta = (z - walk_engine_->p.standComPosition.z);
  delta = crop(delta,-maxDec,maxDec);
  z = walk_engine_->p.standComPosition.z + delta;

  walk_engine_->p.standComPosition.z = z;
  walk_engine_->p.walkHeight.x = z;
  walk_engine_->p.kickHeight.x = z;
  if (frame_info_->frame_id % 100 == 0)
    std::cout << "setting walk height to " << z << std::endl;
}

bool BHWalkModule::shouldStartSlowStand() {
  //std::cout << frame_info_->frame_id << " shouldStartSlowStand: " << walkNames[walk_request_->motion_] << " " << WalkingEngine::getName(walk_engine_->currentMotionType) << " " << frame_info_->seconds_since_start << " " << slow_stand_end << " "  << RAD_T_DEG * standJointErr(LKneePitch) << " " << RAD_T_DEG * standJointErr(RKneePitch) << std::endl;
  if (walk_request_->slow_stand_) // if it's requested
    return true;
  if ((walk_request_->motion_ != WalkRequestBlock::STAND)
  &&  (walk_request_->motion_ != WalkRequestBlock::WALK))
    return false;
  if (walk_engine_->currentMotionType == WalkingEngine::stepping) // we're walking
    return false;
  if (frame_info_->seconds_since_start - slow_stand_end < 5.0) // recently did a slow stand
    return false;
  float kneeErrAllowed = DEG_T_RAD * 30;
  if ((standJointErr(LKneePitch) > kneeErrAllowed)
  ||  (standJointErr(RKneePitch) > kneeErrAllowed)) // knees far from what we want
    return true;
  return false;
}

void BHWalkModule::startSlowStand() {
  std::cout << "starting slow stand" << std::endl;
  float err = 0;
  for (int i = LHipYawPitch; i <= RAnkleRoll; i++)
    err = max(err,standJointErr(i));
  float maxJointSpeed = DEG_T_RAD * 45;
  float duration = err / maxJointSpeed;
  if (duration < 0.5)
    duration = 0.5;
  if (walk_request_->slow_stand_)
    duration = 0.5;

  slow_stand_start = frame_info_->seconds_since_start;
  slow_stand_end = slow_stand_start + duration;
}

float BHWalkModule::standJointErr(int joint) {
  return fabs(STAND_ANGLES[joint] - joints_->values_[joint]);
}
  
bool BHWalkModule::readyToStartKickAfterStep() {
  // at stepForLeftKick or stepForRightKick and we're walking
  return ((walk_engine_->pendulumPlayer.kickType == WalkRequest::stepForLeftKick) || (walk_engine_->pendulumPlayer.kickType == WalkRequest::stepForRightKick)) && (walk_engine_->currentMotionType == WalkingEngine::stepping) && (step_into_kick_state_ == PERFORMING);
}

void BHWalkModule::handleStepIntoKick() {
  if ((walk_request_->new_command_) && (walk_request_->perform_kick_) && (walk_request_->step_into_kick_) && (step_into_kick_state_ == NONE)) {
    //std::cout << frame_info_->frame_id << " RECEIVED STEP_INTO_KICK" << std::endl;
    WalkRequest &walk_request_BH = walk_engine_->theMotionRequest.walkRequest;
    MotionRequest::Motion &motion = walk_engine_->theMotionRequest.motion;
    kick_request_->finished_with_step_ = false;
    step_into_kick_state_ = PERFORMING;
    motion = MotionRequest::walk;
    walk_request_BH.mode = WalkRequest::percentageSpeedMode;
    walk_request_BH.speed = Pose2DBH(0,0,0);
    if (walk_request_->kick_with_left_)
      walk_request_BH.kickType = WalkRequest::stepForLeftKick;
    else
      walk_request_BH.kickType = WalkRequest::stepForRightKick;
  } 

  if (readyToStartKickAfterStep()) {
    step_into_kick_state_ = FINISHED_WITH_STEP;
    walk_request_->noWalk();
    kick_request_->finished_with_step_ = true;
    time_step_into_kick_finished_ = frame_info_->seconds_since_start;
    //std::cout << frame_info_->frame_id << " finished with step into kick" << std::endl;
    return;
  }

  if (step_into_kick_state_ == FINISHED_WITH_STEP) {
    if (kick_request_->kick_running_ || kick_request_->vision_kick_running_ || (frame_info_->seconds_since_start - time_step_into_kick_finished_ < 0.1)) {
      //std::cout << frame_info_->frame_id << " kick is running: " << kick_request_->kick_running_ << " " << kick_request_->vision_kick_running_ << std::endl;
    } else {
      //std::cout << frame_info_->frame_id << " done" << std::endl;
      step_into_kick_state_ = NONE;
    }
    walk_request_->noWalk();
    walk_engine_->reset();
  }

  if (step_into_kick_state_ == PERFORMING) {
    kick_request_->setNoKick();
  }
}
