#include "KickModule.h"
#include <kinematics/ForwardKinematics.h>
#include <memory/FrameInfoBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/KickModuleBlock.h>
#include <memory/KickParamBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/KickRequestBlock.h>
#include <memory/RobotInfoBlock.h>
#include <memory/SensorBlock.h>
#include <memory/SpeechBlock.h>
#include <memory/WalkInfoBlock.h>
#include <memory/WalkRequestBlock.h>

KickModule::KickModule() {
}

KickModule::~KickModule() {
}

void KickModule::specifyMemoryDependency() {
  requiresMemoryBlock("frame_info");
  requiresMemoryBlock("processed_joint_commands");
  requiresMemoryBlock("processed_joint_angles");
  requiresMemoryBlock("kick_module");
  requiresMemoryBlock("kick_params");
  requiresMemoryBlock("kick_request");
  requiresMemoryBlock("odometry");
  requiresMemoryBlock("robot_info");
  requiresMemoryBlock("processed_sensors");
  requiresMemoryBlock("speech");
  requiresMemoryBlock("walk_info");
  requiresMemoryBlock("walk_request");
}

void KickModule::specifyMemoryBlocks() {
  getMemoryBlock(frame_info_,"frame_info");
  getMemoryBlock(commands_,"processed_joint_commands");
  getMemoryBlock(joint_angles_,"processed_joint_angles");
  getOrAddMemoryBlock(kick_module_,"kick_module");
  getOrAddMemoryBlock(kick_params_,"kick_params");
  getMemoryBlock(kick_request_,"kick_request");
  getMemoryBlock(odometry_,"odometry");
  getMemoryBlock(robot_info_, "robot_info");
  getMemoryBlock(sensors_,"processed_sensors");
  getMemoryBlock(speech_, "speech");
  getMemoryBlock(walk_info_,"walk_info");
  getMemoryBlock(walk_request_,"walk_request");
}

void KickModule::initSpecificModule() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    previous_commands_[i] = joint_angles_->values_[i];
  }
  kick_module_->state_ = KickState::NONE;
  kick_module_->kick_type_ = Kick::NO_KICK;
  kick_module_->swing_leg_ = Kick::LEFT;
  
  params_normal_ = &kick_params_->params_;
  params_super_ = &kick_params_->params_super_;
}

void KickModule::processFrame() {
  processKickRequest();

  if (kick_module_->state_ == KickState::STAND) {
    kick_request_->kick_running_ = true;
    // only transition into kick if we think we're stable
    //std::cout << "walk_info_->instability_: " << walk_info_->instability_ << std::endl;    
    if ((walk_info_->instability_ < walk_info_->stabilizer_off_threshold_) && (!walk_info_->walk_is_active_)) {
      if (getFramesInState() >= state_params_->state_time / 10) { // divide by 10 to convert from ms to frames.
        walk_request_->noWalk();
        transitionToState((KickState::State)(kick_module_->state_ + 1));
      } else {
        walk_request_->stand();
      }
    } else {
      walk_request_->stand();
      transitionToState(KickState::STAND); // restart this state
      return;
    }
  }

  if ((kick_module_->state_ == KickState::WALK) && (!walk_info_->walk_is_active_) && (frame_info_->seconds_since_start > kick_module_->state_start_time_ + 0.5)) {
    startKick();
  }

  // handle transitions
  while ((kick_module_->state_ != KickState::NONE) && (getFramesInState() >= state_params_->state_time / 10)) { // divide by 10 to convert from ms to frames.
    if (kick_module_->state_ == KickState::WALK)
      break;
    transitionToState((KickState::State)(kick_module_->state_ + 1));
  }
  
  if (kick_module_->state_ == KickState::NONE) {
    // not in a kick, let vision know
    kick_request_->kick_running_ = false;
    kick_module_->kick_type_ = Kick::NO_KICK;
  } else {
    // if we're still in a kick, do it
    kick();
    setKickOdometry();
    kick_request_->kick_running_ = true;
  }
}

int KickModule::getFramesInState() {
  return frame_info_->frame_id - kick_module_->state_start_frame_;
}

float KickModule::getTimeInState() {
  return frame_info_->seconds_since_start - kick_module_->state_start_time_;
}

float KickModule::getMillisecondsInState() {
  return 1000.0f * (frame_info_->seconds_since_start - kick_module_->state_start_time_);
}

void KickModule::processKickRequest() {
  if (kick_request_->kick_type_ == Kick::ABORT) {
    kick_module_->state_ = KickState::NONE;
  } else if ((kick_request_->kick_type_ != Kick::NO_KICK) && (kick_module_->state_ == KickState::NONE)) {
    startKick();
  }
}

void KickModule::startKick() {
  kick_module_->kick_type_ = kick_request_->kick_type_;
  kick_module_->swing_leg_ = kick_request_->kick_leg_;
  kick_module_->set_kick_odometry_ = false;
  kick_module_->desired_kick_distance_ = kick_request_->desired_distance_;
  kick_module_->desired_kick_angle_ = kick_request_->desired_angle_;
  params_ = params_normal_; // do normal kick

  invalidCount = 0;
  initStiffness();
  if (params_->step_into_kick_)
    transitionToState(KickState::SHIFT);
  else
    transitionToState(KickState::STAND);
  kick_module_->kick_start_time_ = frame_info_->seconds_since_start;
  std::cout << "Kick requested: " <<  kick_module_->kick_type_ << "  requested distance: " << kick_module_->desired_kick_distance_ << "  requested angle: " << kick_module_->desired_kick_angle_ << std::endl;
  
  setHead();
}

void KickModule::setHead() {
  if ((kick_module_->state_ == KickState::STAND) || (kick_module_->state_ == KickState::SHIFT) || (kick_module_->state_ == KickState::LIFT) || (kick_module_->state_ == KickState::ALIGN) || (kick_module_->state_ == KickState::SPLINE)) {
    commands_->setHeadPan(DEG_T_RAD*0, 0.1, false);
    commands_->setHeadTilt(DEG_T_RAD*-21, 0.1, false);
  }
}

void KickModule::initStiffness() {
  for (int i = 0; i < NUM_JOINTS; i++)
    commands_->stiffness_[i] = 1.0;
  commands_->send_stiffness_ = true;
  commands_->stiffness_time_ = 30;
}

void KickModule::setLegStiffness(float stiff) {
  for (int i = 2; i < 14; i++)
    commands_->stiffness_[i] = stiff;
  commands_->send_stiffness_ = true;
  commands_->stiffness_time_ = 30;
}

void KickModule::transitionToState(KickState::State state) {
  //std::cout << "transitionToState: " << KickState::getName(kick_module_->state_) << " -> " << KickState::getName(state) << " " << frame_info_->frame_id << std::endl;
  if (kick_module_->state_ == KickState::STAND && !walk_info_->walk_is_active_) {
    bool continue_kick = handleAiming();
    if (continue_kick) {
      invalidCount = 0;
    } else {
      invalidCount = invalidCount + 1;
    }
    if (invalidCount > 10 || (state != KickState::STAND && invalidCount > 5)) {
      kick_module_->state_ = KickState::NONE;
      std::cout << "Abandon kick - invalid count too high" << std::endl;
      return;
    }
  }

  if (state == KickState::SHIFT) { 
    if (params_->states[KickState::SPLINE].state_time > 0) {
      calcSwingSplinePts();
    }
  }

  kick_module_->state_ = state;
  kick_module_->sent_command_ = false;
  kick_module_->sent_steady_state_command_ = false;
  kick_module_->state_start_time_ = frame_info_->seconds_since_start;
  kick_module_->state_start_frame_ = frame_info_->frame_id;
  state_params_ = &(params_->states[kick_module_->state_]);
}

bool KickModule::chooseKickLeg() {
  // choose a leg if switchable
  if (kick_module_->swing_leg_ == Kick::SWITCHABLE) {
    if (kick_request_->ball_rel_y_ > 0) {
      kick_module_->swing_leg_ = Kick::LEFT;
    } else {
      kick_module_->swing_leg_ = Kick::RIGHT;
    }
  }
  return true;
}

bool KickModule::checkKickValidity() {
  // std::cout << "Kick request - forward: " << kick_request_->ball_rel_x_ << ", side: " << kick_request_->ball_rel_y_ << ", dist side: " << kick_module_->ball_dist_side_ << std::endl; 
  if((kick_module_->swing_leg_ == Kick::LEFT && kick_module_->ball_dist_side_ > 30) || (kick_module_->swing_leg_ == Kick::RIGHT && kick_module_->ball_dist_side_ < -50)) {
    std::cout << "Ball too far sideways: " << kick_module_->ball_dist_side_ << std::endl;
    return false;
  }
  //forward
  if (kick_module_->ball_dist_forward_ > 70) {
    std::cout << "Ball too far forwards: " << kick_module_->ball_dist_forward_ << std::endl;
    return false;
  }
  return true;
}

bool KickModule::handleAiming() {
  if (!kick_request_->ball_seen_) {
    std::cout << "Ball not seen." << std::endl;
    return false;
  }
  if (!chooseKickLeg()) {
    return false;
  }
  calcBallPosWRTSwingLeg();
  if (!checkKickValidity()) {
    return false;
  }
  return true;
}

void KickModule::kick() {
  Vector3<float> com = state_params_->com;

  if (kick_module_->state_ == KickState::SPLINE) {
    sendSplineCOMCommands(com);
    return;
  }

  bool send_commands = false;
  Vector3<float> swing;
  bool move_com = true;
  float command_time = state_params_->joint_time;

  if (!kick_module_->sent_command_) {
    send_commands = true;
    swing = state_params_->swing;
    kick_module_->sent_command_ = true;
  } else if ((!kick_module_->sent_steady_state_command_) && (getMillisecondsInState() > command_time)) {
    send_commands = true;
    swing = state_params_->swing;
    command_time = state_params_->state_time - getMillisecondsInState();
    kick_module_->sent_steady_state_command_ = true;
  }

  if (send_commands) {
    setHead();
    bool is_left_swing = (kick_module_->swing_leg_ == Kick::LEFT);
    int dir = 1;
    if (!is_left_swing)
      dir = -1;
  
    if (kick_module_->state_ == KickState::ALIGN) {
      Vector3<float> temp;
      getSwingTargets(swing,temp);
    } 

    com.y *= dir;
    swing.y *= dir;
  
    Pose3D swing_target(swing);
    float roll = DEG_T_RAD * 0;

    calcJointTargets(com,swing_target,is_left_swing,commands_->angles_,move_com,roll);
  
    commands_->send_body_angles_ = true;
    commands_->body_angle_time_ = command_time;
  } else {
    commands_->send_body_angles_ = false;
  }
}

void KickModule::calcSwingSplinePts() {
  Vector3<float> align = params_->states[KickState::ALIGN].swing;
  Vector3<float> kick = params_->states[KickState::KICK2].swing;

  getSwingTargets(align,kick);

  double time = 0;  //default
  if (kick_module_->swing_leg_ == Kick::RIGHT) {
    time = -0.0698*kick_module_->desired_kick_distance_ + 399.4; // tuned on Alison
  } else {
    time = -0.0685*kick_module_->desired_kick_distance_ + 382.5;
  }
  printf("desired: %2.f\n", kick_module_->desired_kick_distance_);
  time = crop(time,200,400);
  std::cout << "time: " << time << std::endl; 
  //cout<<"Time for kick"<<time<<endl;
  
  int num_pts = 5;
  params_->states[KickState::SPLINE].state_time = time;
  params_->states[KickState::SPLINE].joint_time = time;
  double timesInMs[] = {0,10,20,time-10,time};
  double xs[] = {align.x,align.x,align.x,kick.x,kick.x};
  double ys[] = {align.y,align.y,align.y,kick.y,kick.y};
  double zs[] = {align.z,align.z,align.z,kick.z,kick.z};
  setSwingSpline(num_pts,timesInMs,xs,ys,zs);
}

void KickModule::setSwingSpline(int num_pts,double timesInMs[], double xs[], double ys[], double zs[]) {
  swing_spline_.set(num_pts,timesInMs,xs,ys,zs,true);
}

void KickModule::sendSplineCOMCommands(const Vector3<float> &com_in) {
  float time = getMillisecondsInState();
  Vector3<float> swing;
  bool is_left_swing = (kick_module_->swing_leg_ == Kick::LEFT);
  int dir = 1;
  Vector3<float> com(com_in);
  if (!is_left_swing)
    dir = -1;
 
  swing_spline_.calc(time,swing);
  swing.y *= dir;
  com.y *= dir;

  Pose3D swing_target(swing);

  calcJointTargets(com,swing_target,is_left_swing,commands_->angles_,true,0);
  commands_->send_body_angles_ = true;
  commands_->body_angle_time_ = 10;
}

void KickModule::getSwingTargets(Vector3<float> &align, Vector3<float> &kick) {
  float ideal_ball_side_left_swing_ = params_->ideal_ball_side_left_swing_;
  float ideal_ball_side_right_swing_ = params_->ideal_ball_side_right_swing_;
  if (kick_module_->swing_leg_ == Kick::RIGHT) {
    align.y = align.y - (kick_module_->ball_dist_side_ - ideal_ball_side_right_swing_);
    kick.y = kick.y - (kick_module_->ball_dist_side_ - ideal_ball_side_right_swing_);
  } else {
    align.y = align.y + (kick_module_->ball_dist_side_ - ideal_ball_side_left_swing_);
    kick.y = kick.y + (kick_module_->ball_dist_side_ - ideal_ball_side_left_swing_);
  }
}

void KickModule::calcBallPosWRTSwingLeg() {
  int offset = 50;
  if (kick_module_->swing_leg_ == Kick::LEFT) {
    kick_module_->ball_dist_side_ = kick_request_->ball_rel_y_ - offset;
  } else {
    kick_module_->ball_dist_side_ = kick_request_->ball_rel_y_ + offset;
  }
  kick_module_->ball_dist_forward_ = kick_request_->ball_rel_x_;
  //std::cout << "rel side: " << kick_request_->ball_rel_y_ << ", rel forward: " << kick_request_->ball_rel_x_ << std::endl;
}

void KickModule::setKickOdometry() {
  if (!kick_module_->set_kick_odometry_ && (kick_module_->state_ > KickState::SPLINE) && (kick_module_->state_ != KickState::WALK)) {
    kick_module_->set_kick_odometry_ = true;
    odometry_->didKick = true;
    odometry_->kickVelocity = kick_module_->desired_kick_distance_ / 1.2; // TODO: make this more intelligent
    odometry_->kickHeading = kick_module_->desired_kick_angle_;
  }
}

void KickModule::calcJointTargets(const Vector3<float> &com_target, const Pose3D &swing_rel_stance, bool is_left_swing, float command_angles[NUM_JOINTS], bool move_com, float roll) {
  if (is_left_swing)
    roll *= 1;

  // calculate body model from last commands
  Vector3<float> com;
  calcCenterOfMass(previous_commands_, com, !is_left_swing,0.0f);

  // figure out which leg is stance && which is swing
  Pose3D left_target;
  Pose3D right_target;
  Pose3D *stance_target = &right_target;
  Pose3D *swing_target = &left_target;

  BodyPart::Part stance_foot = BodyPart::right_foot;
  //BodyPart::Part swing_foot = BodyPart::left_foot;
  //int stance_hip_roll = RHipRoll;

  if (!is_left_swing) {
    stance_target = &(left_target);
    swing_target = &(right_target);
    stance_foot = BodyPart::left_foot;
    //swing_foot = BodyPart::right_foot;
    //stance_hip_roll = LHipRoll;
  }
  // the offset from stance leg to torso
  Vector3<float> stance_to_torso_offset;
  stance_to_torso_offset = command_body_model_.abs_parts_[stance_foot].translation - command_body_model_.abs_parts_[BodyPart::torso].translation;

  Vector3<float> abs_desired_com;
  abs_desired_com.x = com_target.x + stance_to_torso_offset.x;
  abs_desired_com.y = com_target.y + stance_to_torso_offset.y;
  abs_desired_com.z = com_target.z;


  // stance leg starts out at current position
  // && will be offset later by com change we want
  stance_target->rotation = RotationMatrix(0,0,0);
  stance_target->translation = command_body_model_.abs_parts_[stance_foot].translation;
  stance_target->rotation.rotateZ(command_body_model_.abs_parts_[stance_foot].rotation.getZAngle());


  // convert from stance in abs frame to stance in torso frame
  stance_target->translation -= command_body_model_.abs_parts_[BodyPart::torso].translation;
  stance_target->translation.x = -com_target.x;
  stance_target->translation.z = -com_target.z;

  // convert swing leg from stance to torso frame
  *swing_target = swing_rel_stance;
  Vector2<float> swingXY;
  swingXY.x = swing_target->translation.x;
  swingXY.y = swing_target->translation.y;
  swingXY.rotate(stance_target->rotation.getZAngle());
  swing_target->translation.x = swingXY.x;
  swing_target->translation.y = swingXY.y;
  swing_target->translation += stance_target->translation;
  swing_target->rotation.rotateZ(stance_target->rotation.getZAngle());

  Vector3<float> com_err;
  float max_acceptable_com_err = 0.25; //1.0;
  for (int i = 0; i < 100; i++) {
    com_err = abs_desired_com - com;
    if (!move_com)
      com_err = Vector3<float>(0,0,0);
    //if (com_err.abs() < max_acceptable_com_err && i > 0)
    if (fabs(com_err.y) < max_acceptable_com_err && i > 0)
      break;

    // so we don't wildly overshoot 
    // (z movements kind of result in double com movements... torso lowers
    // && we lower our weight)
    com_err.z *= 0.5;

    // get stance foot position relative to torso
    // move stance foot opposite direction from com error
    //stance_target->translation.x -= com_err.x;
    stance_target->translation.y -= com_err.y;
    //stance_target->translation.z -= com_err.z;

    // also move desired com by this amount...
    // since we moved stance leg
    //abs_desired_com.x -= com_err.x;
    abs_desired_com.y -= com_err.y;

    // Todd: desired com z is absolute now
    //abs_desired_com.z -= pen_err.z;

    // as stance leg moves one way, we have to move the swing leg relative to
    // the torso to match it
    // so swing leg to stance leg distance remains the same
    //swing_target->translation.x -= com_err.x;
    swing_target->translation.y -= com_err.y;
    //swing_target->translation.z -= com_err.z;

    if (stance_target->translation.z < -203) stance_target->translation.z = -203;
    if (swing_target->translation.z < -203) swing_target->translation.z = -203;

    bool left_compliant  = true;//false;
    bool right_compliant = true;//false;

    // calculate tilt roll from stance leg
    TiltRoll tr = ForwardKinematics::calculateTiltRollFromLeg(!is_left_swing, commands_->angles_, robot_info_->dimensions_);
    tr.roll_ = roll;
    
    commandLegsRelativeToTorso(command_angles, left_target, right_target, tr.tilt_, tr.roll_, 0.0f, left_compliant,right_compliant);
    setArms(command_angles);

    // calculate new com based on these joint commands
    calcCenterOfMass(command_angles, com, !is_left_swing, 0.0f);
  } 
  
  for (int i = 0; i < NUM_JOINTS; i++) {
    previous_commands_[i] = commands_->angles_[i];
  }
}

void KickModule::setArms(float command_angles[NUM_JOINTS]) {
  if (walk_request_->keep_arms_out_) {
    command_angles[LShoulderPitch] = DEG_T_RAD * -116;
    command_angles[LShoulderRoll] = DEG_T_RAD * 12;
    command_angles[LElbowYaw] = DEG_T_RAD * -85;
    command_angles[LElbowRoll] = DEG_T_RAD * -0;
    command_angles[RShoulderPitch] = DEG_T_RAD * -116;
    command_angles[RShoulderRoll] = DEG_T_RAD * 12;
    command_angles[RElbowYaw] = DEG_T_RAD * -85;
    command_angles[RElbowRoll] = DEG_T_RAD * -0;
  } else {
    command_angles[LShoulderPitch] = DEG_T_RAD*-116;
    command_angles[RShoulderPitch] = DEG_T_RAD*-116;

    command_angles[LShoulderRoll] = DEG_T_RAD*8;
    command_angles[RShoulderRoll] = DEG_T_RAD*8;

    command_angles[LElbowRoll] = DEG_T_RAD*-53;
    command_angles[RElbowRoll] = DEG_T_RAD*-53;

    command_angles[LElbowYaw] = DEG_T_RAD*25;
    command_angles[RElbowYaw] = DEG_T_RAD*25;
  }
}

void KickModule::calcCenterOfMass(float *command_angles, Vector3<float> &center_of_mass, bool stance_is_left, float tilt_roll_factor){

  // calculate tilt roll from stance leg
  TiltRoll tr = ForwardKinematics::calculateTiltRollFromLeg(stance_is_left, command_angles, robot_info_->dimensions_);

  // use some fraction of tilt && roll
  tr.tilt_ *= tilt_roll_factor;
  tr.roll_ *= tilt_roll_factor;

  ForwardKinematics::calculateRelativePose(command_angles, tr.tilt_, tr.roll_, command_body_model_.rel_parts_, robot_info_->dimensions_.values_);
  Pose3D base = ForwardKinematics::calculateVirtualBase(stance_is_left, command_body_model_.rel_parts_);
  ForwardKinematics::calculateAbsolutePose(base, command_body_model_.rel_parts_, command_body_model_.abs_parts_);
  ForwardKinematics::calculateCoM(command_body_model_.abs_parts_, command_body_model_.center_of_mass_, robot_info_->mass_calibration_);
  center_of_mass = command_body_model_.center_of_mass_;
}

void KickModule::commandLegsRelativeToTorso(float *command_angles, Pose3D left_target, Pose3D right_target, float /*tilt*/, float roll, float /*tilt_roll_factor*/, bool left_compliant, bool right_compliant) {

  RotationMatrix rot;
  RotationMatrix foot_rotation;
  rot.rotateX(roll);
  foot_rotation.rotateX(-roll);
  if (left_compliant) {
    left_target.translation = rot * left_target.translation;
    left_target.rotation = left_target.rotation * foot_rotation;
  }
  if (right_compliant) {
    right_target.translation = rot * right_target.translation;
    right_target.rotation = right_target.rotation * foot_rotation;
  }

  inverse_kinematics_.calcLegJoints(left_target, right_target, command_angles, robot_info_->dimensions_);
  command_angles[LHipYawPitch] = 0.0;
  command_angles[RHipYawPitch] = 0.0;
}
