#include "WalkModule.h"

#include <kinematics/ForwardKinematics.h>

#include <memory/BodyModelBlock.h>
#include <memory/FrameInfoBlock.h>
#include <memory/GraphableBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/SensorBlock.h>
//#include <memory/WalkEngineBlock.h>
#include <memory/WalkParamBlock.h>
#include <memory/WalkRequestBlock.h>
#include <memory/RobotInfoBlock.h>

#include <math/Geometry.h>

WalkModule::WalkModule():
  Gx_(1,3),
  A0_(3,3),
  b0_(3,1),
  L_(3,2),
  c0_(1,3)
{
  current_state_mat_[0] = NMatrix(3,1);
  current_state_mat_[1] = NMatrix(3,1);
  command_body_model_ = new BodyModelBlock();
}

WalkModule::~WalkModule(){
  delete command_body_model_;
}

void WalkModule::specifyMemoryDependency() {
  requiresMemoryBlock("body_model");
  requiresMemoryBlock("frame_info");
  requiresMemoryBlock("graphable");
  requiresMemoryBlock("processed_joint_angles");
  requiresMemoryBlock("processed_joint_commands");
  requiresMemoryBlock("odometry");
  requiresMemoryBlock("processed_sensors");
  requiresMemoryBlock("walk_engine");
  requiresMemoryBlock("walk_param");
  requiresMemoryBlock("walk_request");
  requiresMemoryBlock("robot_info");
}

void WalkModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(body_model_,"body_model");
  getOrAddMemoryBlock(frame_info_,"frame_info");
  getOrAddMemoryBlock(graph_,"graphable");
  getOrAddMemoryBlock(joints_,"processed_joint_angles");
  getOrAddMemoryBlock(commands_,"processed_joint_commands");
  getOrAddMemoryBlock(odometry_,"odometry");
  getOrAddMemoryBlock(sensors_,"processed_sensors");
  getOrAddMemoryBlock(walk_mem_,"walk_engine");
  getOrAddMemoryBlock(walk_param_,"walk_param");
  getOrAddMemoryBlock(walk_request_,"walk_request");
  getOrAddMemoryBlock(robot_info_,"robot_info");

  step_prev_ = &(walk_mem_->step_prev_);
  step_current_ = &(walk_mem_->step_current_);
  step_next_ = &(walk_mem_->step_next_);
  step_after_next_ = &(walk_mem_->step_after_next_);
  step_two_after_next_ = &(walk_mem_->step_two_after_next_);
  params_ = &(walk_param_->params_);
  walk_mem_->motion_prev_ = WalkRequestBlock::NONE;
  walk_mem_->motion_current_ = WalkRequestBlock::NONE;

  if (frame_info_->source == MEMORY_SIM)
    motion_time_ = 0.02;
  else
    motion_time_ = 0.01;
}

void WalkModule::initSpecificModule() {
  initFrames();
  initMatricesWrapper();
}

void WalkModule::initFrames() {
  walk_mem_->num_step_frames_ = int(params_->phase_length_ / motion_time_ + 0.5);
  walk_mem_->num_double_support_frames_ = int(params_->double_support_frac_ * params_->phase_length_/ motion_time_ + 0.5);
  std::cout << "FRAMES, step: " << walk_mem_->num_step_frames_ << " double: " << walk_mem_->num_double_support_frames_ << std::endl;
}

void WalkModule::initMatricesWrapper() {
  int pend_height = params_->pendulum_height_;
  pend_height = 5 * (int)((pend_height / 5.0) + 0.5); // convert to a multiple of 5
  std::cout << "ORIG PEND HEIGHT: " << params_->pendulum_height_ << " Rounded: " << pend_height << std::endl;
  initMatrices(pend_height);
}

void WalkModule::processFrame() {
  //std::cout << "TIMES: " << calcPhaseFrac() << " " << calcSingleSupportFrac() << std::endl;
  //walk_request_->set(WalkRequestBlock::WALK,Pose2D(0,1.00,0),true,false);
  processWalkRequest();

  bool new_in_motion = getTimeInMotion() < 0.00001;

  //std::cout << "CURRENT COM(abs): " << body_model_->center_of_mass_ << std::endl;

  // from none to standing or walk
  if (new_in_motion){
    initWalk();
  }

  switch (walk_mem_->motion_current_) {
  case WalkRequestBlock::NONE:
    break;

    // walk and stand call the same stuff
  case WalkRequestBlock::STAND:
  case WalkRequestBlock::WALK:
    walk();
    break;

  default:
    break;
  }

  // this should be called at the end
  updateOdometry();

}

void WalkModule::initStand() {
  initStiffness(0.5);
  // find pendulum point, which is our torso position
  Vector3<float> pendulum = body_model_->abs_parts_[BodyPart::torso].translation;
  stand_position_ = Vector3<float>(0,0.5 * params_->foot_separation_,-params_->walk_height_);

  commands_->body_angle_time_ = 1000;
  walk_mem_->abs_left_foot_ = Pose3D(stand_position_);
  walk_mem_->abs_right_foot_ = Pose3D(stand_position_);
  const float max_z_change = 100.0;

  // update those targets based on pendulum height
  float pendulum_z_err = pendulum.z - params_->walk_height_;
  pendulum_z_err = crop(pendulum_z_err,-max_z_change,max_z_change);
  float current_foot_height = body_model_->abs_parts_[BodyPart::left_foot].translation.z - body_model_->abs_parts_[BodyPart::torso].translation.z + pendulum_z_err;

  // current + error
  walk_mem_->abs_left_foot_.translation.z = current_foot_height + pendulum_z_err;
  walk_mem_->abs_right_foot_.translation.z = current_foot_height + pendulum_z_err;

  walk_mem_->abs_right_foot_.translation.y *= -1;
  commandLegsRelativeToTorso(commands_->angles_,walk_mem_->abs_left_foot_,walk_mem_->abs_right_foot_,0,0,false,false);
  setArms(commands_->angles_,Vector3<float>(0,0,0));
  for (int i = 0; i < NUM_JOINTS; i++)
    previous_commands_[i] = commands_->angles_[i];
  commands_->send_body_angles_ = true;
}

void WalkModule::walk() {

  while (frame_info_->frame_id >= step_next_->frame_) {
    switchSupport();
  }

  // X(k) now equals X(k+1) since we've moved ahead
  walk_mem_->current_state_ = walk_mem_->desired_next_state_;

  // handle delayed sensors
  handleSensorDelay();

  // estimate sensor feedback
  estimateSensorPendulum();
  estimateSensorZmp();

  Vector2<float> saved_pen = walk_mem_->current_state_.pen_pos_;

  // adjust the current state com to account for sensor feedback
  if ((!step_current_->is_stand_) && (params_->closed_loop_pen_))
    adjustStateFromSensorPendulum();

  // make the state matrix for the current state - easier to work with
  for (int i = 0; i < 2; i++) {
    current_state_mat_[i][0][0] = walk_mem_->current_state_.pen_pos_[i];
    current_state_mat_[i][1][0] = walk_mem_->current_state_.pen_vel_[i];
    current_state_mat_[i][2][0] = walk_mem_->current_state_.zmp_[i];
  }
  
  // calculate optimal control (Joho thesis eq 5)
  calculateOptimalControl();

  walk_mem_->current_state_.pen_pos_ = saved_pen;
  // make the state matrix for the current state - easier to work with
  for (int i = 0; i < 2; i++) {
    current_state_mat_[i][0][0] = walk_mem_->current_state_.pen_pos_[i];
    current_state_mat_[i][1][0] = walk_mem_->current_state_.pen_vel_[i];
    current_state_mat_[i][2][0] = walk_mem_->current_state_.zmp_[i];
  }


  // update state equation (Joho thesis eq 4)
  calculateDesiredNextState();

  // possibly re-plan steps if desired vel changed
  /*
  if (walk_mem_->previous_step_size_ != walk_mem_->desired_step_size_ &&
      frame_info_->frame_id % 25 == 0){
    replanSteps();
  }
  */
  
  float num_frames_passed = frame_info_->frame_id - last_walk_frame_;
  unsigned int frame = last_walk_frame_ + (walk_mem_->zmp_ref_.getNumberOfEntries()-1);
  for (int i = 0; i < num_frames_passed; i++) {
    frame++;
    walk_mem_->zmp_ref_.pop();
    if (frame < (step_two_after_next_->frame_ - 1))
      walk_mem_->zmp_ref_.push(calcZMPRef(frame));
  }

  //float dt = frame_info_->seconds_since_start - last_walk_time_;
  //// add on to end of zmp ref
  //int num_frames_passed = int((dt / motion_time_) + 0.5);
  //// Todd: what is the time of the next zmp ref to calculate?
  //float t = last_walk_time_ + (walk_mem_->zmp_ref_.getNumberOfEntries()-1) * motion_time_;
  //for (int i = 0; i < num_frames_passed; i++) {
    //t += motion_time_;
    //walk_mem_->zmp_ref_.pop();
    //if (t < (step_two_after_next_->time_ - motion_time_))
      //walk_mem_->zmp_ref_.push(calcZMPRef(t));
  //}

  // use inverse kinematics to move body to match desired pendulum pos
  calcSwingTarget();

  // move swing leg to reach target step
  calcJointTargets();

  // set stiffness (1 if walking, 0.5 if standing)
  if (step_current_->is_stand_ && step_next_->is_stand_ && getTimeInMotion() > 3.0){
    initStiffness(0.5);
  } else {
    initStiffness(1.0);
  } 

  last_walk_time_ = frame_info_->seconds_since_start;
  last_walk_frame_ = frame_info_->frame_id;
}

void WalkModule::replanSteps(){
  //if ((step_two_after_next_->time_ - frame_info_->seconds_since_start) < params_->min_step_change_time_){
    //// step after next is too soon to try re-planning it
    //return;
  //}

  float time = (step_two_after_next_->frame_ - frame_info_->frame_id) / motion_time_; 
  if (time < params_->min_step_change_time_) {
    // step after next is too soon to try re-planning it
    return;
  }

  calcStep(*step_two_after_next_,*step_after_next_,*step_next_);

  // re-do ref zmp calculations for step_after_next on
  //int offset = (step_after_next_->time_ - frame_info_->seconds_since_start) / motion_time_;
  int offset = (step_after_next_->frame_ - frame_info_->frame_id);
  float frame = step_after_next_->frame_;
  for (int i = offset; i < walk_mem_->zmp_ref_.getNumberOfEntries(); i++){
    walk_mem_->zmp_ref_[i] = calcZMPRef(frame);
    frame++;
  }
}


void WalkModule::calculateOptimalControl() {
  // update walk_mem_->current_control_
  // using walk_mem_->sum_zmp_errors_, walk_mem->current_state_
  // and walk_mem_->zmp_ref_ out to some preview horizon

  for (int i = 0; i < 2; i++) {
    walk_mem_->current_control_[i] = -Gi_ * walk_mem_->sum_zmp_errors_[i] - (Gx_ * current_state_mat_[i])[0][0];
    for (unsigned int j = 1; ((int)j  < walk_mem_->zmp_ref_.getNumberOfEntries()) && (j < num_preview_frames_); j++) {
      walk_mem_->current_control_[i] -= Gd_[j] * walk_mem_->zmp_ref_[j][i];
    }
  }
}

void WalkModule::handleSensorDelay() {
  // save current state of zmp/pen in a delay buffer
  // so when we get delayed estimate of true position x frames ago
  // we can match it up with where we thought it would be then
  walk_mem_->delayed_zmp_state_buffer_.push(walk_mem_->current_state_.zmp_);

  // similarly the com
  Vector3<float> pen;
  pen.x = walk_mem_->current_state_.pen_pos_.x;
  pen.y = walk_mem_->current_state_.pen_pos_.y;
  pen.z = params_->walk_height_;
  walk_mem_->delayed_pen_state_buffer_.push(pen);

  // and hold stance step to match with pen
  walk_mem_->delayed_step_buffer_.push(*step_current_);

  // hold accel readings as well, to match with pen when we get it
  // Todd: accel are in meters, convert them to mm
  // and rotate them into global frame. 
  Vector2<float> accels(-1000.0 * sensors_->values_[accelX], -1000.0 * sensors_->values_[accelY]);
  accels.rotate(walk_mem_->global_frame_offset_.rotation);
  walk_mem_->delayed_accel_sensor_buffer_.push(accels);

  if (params_->pen_sensor_delay_frames_ < params_->accel_sensor_delay_frames_) {
    cout << "ERROR: assumed that pen delay is greater than accel delay" << endl;
  }

  // get delayed states to compare with sensor readings
  if (walk_mem_->delayed_zmp_state_buffer_.getNumberOfEntries() > params_->pen_sensor_delay_frames_) {
    walk_mem_->delayed_zmp_state_ = walk_mem_->delayed_zmp_state_buffer_.pop();
    walk_mem_->delayed_pen_state_ = walk_mem_->delayed_pen_state_buffer_.pop();
    walk_mem_->delayed_stance_step_ = walk_mem_->delayed_step_buffer_.pop();
  } else {
    walk_mem_->delayed_zmp_state_ = walk_mem_->delayed_zmp_state_buffer_.front();
    walk_mem_->delayed_pen_state_ = walk_mem_->delayed_pen_state_buffer_.front();
    walk_mem_->delayed_stance_step_ = walk_mem_->delayed_step_buffer_.front();
  }

  // get delayed accels to match with com reading for zmp estimation
  if (walk_mem_->delayed_accel_sensor_buffer_.getNumberOfEntries() > (params_->pen_sensor_delay_frames_ - params_->accel_sensor_delay_frames_)) {
    walk_mem_->delayed_accel_sensor_ = walk_mem_->delayed_accel_sensor_buffer_.pop();
  } else {
    walk_mem_->delayed_accel_sensor_ = walk_mem_->delayed_accel_sensor_buffer_.front();
  }
  
}

Vector3<float> WalkModule::getGlobalPendulum() {
  Vector3<float> abs_to_stance_offset;
  if (step_current_->is_left_foot_)
    abs_to_stance_offset = -body_model_->abs_parts_[BodyPart::left_foot].translation;
  else
    abs_to_stance_offset = -body_model_->abs_parts_[BodyPart::right_foot].translation;

  // calculate it
  Pose3D pen_pose(0,0,0);
  pen_pose.translation = body_model_->abs_parts_[BodyPart::torso].translation;

  // convert from abs to stance
  pen_pose.translation += abs_to_stance_offset;

  // convert from stance back to global
  pen_pose = pen_pose.relativeToGlobal(walk_mem_->global_frame_offset_);
  pen_pose.translation.z += robot_info_->dimensions_.values_[RobotDimensions::footHeight];

  return pen_pose.translation;
}

void WalkModule::estimateSensorPendulum() {

  // use stance foot from when values came from? or from now?
  //Step* stance = step_current_;
  Step* stance = &(walk_mem_->delayed_stance_step_);

  // get com from perceived joint angles using tilt/roll from stance foot
  // Note: the stance foot we were using x frames ago
  Vector3<float> sensedPen;
  calculatePendulum(joints_->values_, sensedPen, stance->is_left_foot_);

  // use stance step and global offset (step position) from x frames ago
  // and use body from the sensed joint angles we just calculated
  Vector3<float> abs_to_stance_offset;
  if (stance->is_left_foot_)
    abs_to_stance_offset = -command_body_model_->abs_parts_[BodyPart::left_foot].translation;
  else
    abs_to_stance_offset = -command_body_model_->abs_parts_[BodyPart::right_foot].translation;

  // convert from abs to stance
  Pose3D pen_pose(0,0,0);
  pen_pose.translation = sensedPen + abs_to_stance_offset;
  // convert from stance back to global
  pen_pose = pen_pose.relativeToGlobal(stance->position_);
  pen_pose.translation.z += robot_info_->dimensions_.values_[RobotDimensions::footHeight];
  
  //walk_mem_->sensor_pen_ = pen_pose.translation;
  walk_mem_->buffered_sensor_pen_.push(pen_pose.translation);
  // make sure the buffer is holding the right number of entries
  while (walk_mem_->buffered_sensor_pen_.getNumberOfEntries() > params_->num_averaged_sensor_pen_frames_)
    walk_mem_->buffered_sensor_pen_.pop();
  // get out the average
  walk_mem_->sensor_pen_ = walk_mem_->buffered_sensor_pen_.getAverage();
}

void WalkModule::adjustStateFromSensorPendulum() {
  Vector3<float> pen_diff = walk_mem_->delayed_pen_state_ - walk_mem_->sensor_pen_;
  float threshold = 3; // in mm

  for (int i = 0; i < 2; i++) {
    if (pen_diff[i] > threshold)
      pen_diff[i] -= threshold;
    else if (pen_diff[i] < -threshold)
      pen_diff[i] += threshold;
    else
      pen_diff[i] = 0.0f;
    
    pen_diff[i] = crop(pen_diff[i],-10.0,10.0);
    //std::cout << pen_diff[i] << " ";
    walk_mem_->current_state_.pen_pos_[i] -= params_->pen_sensor_control_ratio_ * pen_diff[i];
  }
  //std::cout << std::endl;
}

void WalkModule::estimateSensorZmp() {
  // without sensors, this should match the zmp we calculated x frames ago
  //walk_mem_->sensor_zmp_ = walk_mem_->delayed_zmp_;

  Vector3<float> pen;

  // Todd: lets try using sensor pen
  // and match with accel reading from however long ago com takes
  // to match when the com sensation was from
  pen = walk_mem_->sensor_pen_;
  
  // calc zmp with current com sensor and delayed acc readings
  walk_mem_->sensor_zmp_.x = pen.x - ( pen.z / 9806.65) * walk_mem_->delayed_accel_sensor_.x;
  walk_mem_->sensor_zmp_.y = pen.y - ( pen.z / 9806.65) * walk_mem_->delayed_accel_sensor_.y;

  walk_mem_->buffered_sensor_zmp_.push(walk_mem_->sensor_zmp_);
  // make sure the buffer is holding the right number of entries
  while (walk_mem_->buffered_sensor_zmp_.getNumberOfEntries() > params_->num_averaged_sensor_zmp_frames_)
    walk_mem_->buffered_sensor_zmp_.pop();

  // get out the average
  walk_mem_->sensor_zmp_ = walk_mem_->buffered_sensor_zmp_.getAverage();

  // update zmp error sums
  walk_mem_->sum_zmp_errors_ += (walk_mem_->current_state_.zmp_ - walk_mem_->zmp_ref_[0]);
}


void WalkModule::calculateDesiredNextState(){

  // calculate walk_mem_->desired_next_state_
  // using walk_mem_->current_state_, walk_mem_->sensor_zmp_, and walk_mem_->current_control_

  NMatrix next_state(3,1);

  // find difference from sensed and current zmp x frames ago
  Vector2<float> zmp_diff = walk_mem_->sensor_zmp_ - walk_mem_->delayed_zmp_state_;

  // find differnece from sensed and current com x frames ago
  //Vector3<float> pen_diff = walk_mem_->sensor_pen_ - walk_mem_->delayed_pen_state_;

  //cout << "frame: " << frame_info_->frame_id << " using delayed zmp: " << old_state_zmp << endl;
  //cout << " sensed: " << walk_mem_->sensor_zmp_ << " diff: " << zmp_diff << endl;

  // Todd: the sensor feedback should be +L
  for (int i = 0; i < 2; i++) {

    //if (params_->closed_loop_com_) {
    //current_state_mat_[i][0][0] = current_state_mat_[i][0][0] + 0.5 * com_diff[i];
    //next_state = next_state - L_.getCol(0) * com_diff[i];
    //}

    next_state = A0_ * current_state_mat_[i] + b0_ * walk_mem_->current_control_[i];

    // lets save what we would have done if not doing closed loop
    walk_mem_->desired_next_without_closed_loop_.pen_pos_[i] = next_state[0][0];
    walk_mem_->desired_next_without_closed_loop_.pen_vel_[i] = next_state[1][0];
    walk_mem_->desired_next_without_closed_loop_.zmp_[i] = next_state[2][0];

    if (params_->closed_loop_zmp_) {
      zmp_diff[i] *= params_->zmp_sensor_control_ratio_;
      next_state = next_state + L_.getCol(1) * zmp_diff[i];
    }

    
    //if (params_->closed_loop_com_) {
    //com_diff[i] *= params_->com_sensor_control_ratio_;
    //next_state = next_state + L_.getCol(0) * com_diff[i];
    //}

    walk_mem_->desired_next_state_.pen_pos_[i] = next_state[0][0];
    walk_mem_->desired_next_state_.pen_vel_[i] = next_state[1][0];
    walk_mem_->desired_next_state_.zmp_[i] = next_state[2][0];
  }
}


void WalkModule::switchSupport() {

  *step_prev_ = *step_current_;
  *step_current_ = *step_next_;
  *step_next_ = *step_after_next_;
  *step_after_next_ = *step_two_after_next_;

  // keep track of where stance foot is in global frame
  walk_mem_->global_frame_offset_ = step_current_->position_;

  calcStep(*step_two_after_next_,*step_after_next_,*step_next_);

}

void WalkModule::initStiffness(float val) {

  // only if its changed
  if (walk_mem_->current_stiffness_ == val)
    return;

  // initialize the stiffnesses
  commands_->send_stiffness_ = true;
  commands_->stiffness_time_ = 10;
  for (int i = 0; i < NUM_JOINTS; i++)
    commands_->stiffness_[i] = val;
  commands_->stiffness_[HeadPitch] = 1.0;
  commands_->stiffness_[HeadYaw] = 1.0;

  walk_mem_->current_stiffness_ = val;

  //commands_->stiffness_[LAnkleRoll] = 0.60;
  //commands_->stiffness_[RAnkleRoll] = 0.60;
  //commands_->stiffness_[LAnklePitch] = 0.60;
  //commands_->stiffness_[RAnklePitch] = 0.60;
  //commands_->stiffness_[LKneePitch] = 0.60;
  //commands_->stiffness_[RKneePitch] = 0.60;
  //commands_->stiffness_[LShoulderPitch] = 0;
  //commands_->stiffness_[LShoulderRoll] = 0;
  //commands_->stiffness_[RShoulderPitch] = 0;
  //commands_->stiffness_[RShoulderRoll] = 0;
}


void WalkModule::initWalk() {
  initStiffness(1.0);

  // initialize the current step (as standing step)
  step_current_->position_ = Pose2D(0,0,0);
  //step_current_->time_ = frame_info_->seconds_since_start;
  step_current_->frame_ = frame_info_->frame_id;
  step_current_->is_left_foot_ = true;
  step_current_->is_stand_ = true;
  walk_mem_->global_frame_offset_ = Pose2D(0,0,0);

  // if we want to walk left, start with right foot as stance
  if (walk_mem_->desired_step_size_.translation.y > 0)
    step_current_->is_left_foot_ = false;

  // if we want to turn left, start with right foot as stance
  if (walk_mem_->desired_step_size_.rotation > 0)
    step_current_->is_left_foot_ = false;

  // handle the prev step by faking it
  int dir = 1;
  if (step_current_->is_left_foot_)
    dir *= -1;
  step_prev_->position_ = Pose2D(0,stand_position_.x,params_->foot_separation_ * dir);
  //step_prev_->time_ = step_current_->time_ - params_->phase_length_;
  step_prev_->frame_ = step_current_->frame_ - walk_mem_->num_step_frames_;
  step_prev_->is_left_foot_ = !step_current_->is_left_foot_;
  step_prev_->is_stand_ = true;

  // calculate the next 3 steps
  calcStep(*step_next_,*step_current_,*step_prev_);
  calcStep(*step_after_next_,*step_next_,*step_current_);
  calcStep(*step_two_after_next_,*step_after_next_,*step_next_);

  //std::cout << "current: " << step_current_->position_.translation << std::endl;
  //std::cout << "next: " << step_next_->position_.translation << std::endl;
  //std::cout << "after next: " << step_after_next_->position_.translation << std::endl;

  Vector3<float> *foot;
  if (step_current_->is_left_foot_) {
    foot = &(body_model_->abs_parts_[BodyPart::left_foot].translation);
  } else {
    foot = &(body_model_->abs_parts_[BodyPart::right_foot].translation);
  }

  for (int i = 0; i < 2; i++) {
    walk_mem_->current_state_.pen_pos_[i] = body_model_->center_of_mass_[i] - (*foot)[i];
    walk_mem_->current_state_.pen_vel_[i] = 0;
    walk_mem_->current_state_.zmp_[i] = walk_mem_->current_state_.pen_pos_[i];

  }
  walk_mem_->desired_next_state_ = walk_mem_->current_state_;

  initZMPRef();
  last_walk_time_ = frame_info_->seconds_since_start;
  last_walk_frame_ = frame_info_->frame_id;

  for (int i = 0; i < 2; i++)
    walk_mem_->sum_zmp_errors_[i] = 0;

  // to init odometry
  calculateGlobalTorsoLocation();

  // set up previous commands
  for (int i = 0; i < NUM_JOINTS; i++) {
    previous_commands_[i] = joints_->values_[i];
  }
}

void WalkModule::processWalkRequest() {
  if (walk_param_->send_params_) {
    walk_param_->send_params_ = false;
    initMatricesWrapper();
    initFrames();
  }

  walk_mem_->motion_prev_ = walk_mem_->motion_current_;
  walk_mem_->motion_current_ = walk_request_->motion_;

  if (walk_request_->motion_ != walk_mem_->motion_prev_ && (walk_mem_->motion_prev_ == WalkRequestBlock::NONE || walk_mem_->motion_prev_ == WalkRequestBlock::FALLING)) {
    // TODO handle motion change
    resetTimeInMotion();
  }

}

void WalkModule::calcKickStepSize() {
  walk_mem_->previous_step_size_ = walk_mem_->desired_step_size_;
  Pose2D &desired = walk_mem_->desired_step_size_;
  desired.rotation = 0;
  desired.translation.y = 0;
  desired.translation.x = 120;//params_->max_step_size_.translation.x;
}

void WalkModule::calcDesiredStepSize() {
  walk_mem_->previous_step_size_ = walk_mem_->desired_step_size_;
  Pose2D &desired = walk_mem_->desired_step_size_;

  if (walk_request_->percentage_speed_) {
    // convert from percentage speeds to step sizes
    desired.translation.x = walk_request_->speed_.translation.x * params_->max_step_size_.translation.x;
    desired.translation.y = walk_request_->speed_.translation.y * params_->max_step_size_.translation.y;
    desired.rotation = walk_request_->speed_.rotation * params_->max_step_size_.rotation;
  } else {
    // convert from speeds to step sizes
    desired.translation.x = walk_request_->speed_.translation.x * params_->phase_length_;
    desired.translation.y = walk_request_->speed_.translation.y * params_->phase_length_;
    desired.rotation = walk_request_->speed_.rotation * params_->phase_length_;
  }

  if (walk_mem_->motion_current_ == WalkRequestBlock::STAND){
    desired.translation.x = 0;
    desired.translation.y = 0;
    desired.rotation = 0;
  }
}

void WalkModule::calcStep(Step &next, const Step &ref, const Step &prev) {
  next.is_left_foot_ = !ref.is_left_foot_;
  //next.time_ = ref.time_ + params_->phase_length_;
  next.frame_ = ref.frame_ + walk_mem_->num_step_frames_;

  float next_time = (next.frame_ - frame_info_->frame_id) / motion_time_;
  if (walk_mem_->motion_current_ == WalkRequestBlock::STAND || (next_time - walk_mem_->time_motion_started_) < 1.0)
    next.is_stand_ = true;
  else
    next.is_stand_ = false;

  // calculate the position
  int dir = 1;
  if (!next.is_left_foot_)
    dir *= -1;

  // calculate offset from ref foot
  Pose2D foot_offset(0,0,0);
  foot_offset.translation.y = dir * params_->foot_separation_;

  if (walk_request_->perform_kick_ && next.is_left_foot_ == walk_request_->kick_with_left_){
    // its a kick!!!
    calcKickStepSize();
    //next.time_ = ref.time_ + 0.15;
    next.frame_ = ref.frame_ + 8;
    walk_request_->perform_kick_ = false;
  }
  else if (walk_request_->walk_to_target_){
    calcTargetStepSizes(ref);
    // if they're both less than 20, stand
    /*
    if (fabs(walk_mem_->desired_step_size_.translation.x) < 20 && fabs(walk_mem_->desired_step_size_.translation.y) < 20)
      next.is_stand_ = true;
    */

  } 
  else if (walk_request_->rotate_around_target_) {
    calcRotateStepSizes(ref);
  } 
  else {
    calcDesiredStepSize();
  }

  // now add on step amounts
  // only if ref step is not stand (if it is, this step is just a weight shift)
  if (!ref.is_stand_){
    // x
    foot_offset.translation.x += walk_mem_->desired_step_size_.translation.x * 0.5;

    // y
    // side step, move lead foot out, following foot just pulls even again
    // if we just stepped one way, make sure following foot does follow
    // even if we now want to go opposite way
    bool follow_side = false;
    // get ref step position in terms of prev step position
    Pose2D feet_diff = ref.position_.globalToRelative(prev.position_);
    if (fabs(feet_diff.translation.y) > (params_->foot_separation_ + 1.0)){
      follow_side = true;
    }

    if (next.is_left_foot_ && walk_mem_->desired_step_size_.translation.y > 0 && !follow_side)
      foot_offset.translation.y += walk_mem_->desired_step_size_.translation.y;
    else if (!next.is_left_foot_ && walk_mem_->desired_step_size_.translation.y < 0 && !follow_side)
      foot_offset.translation.y += walk_mem_->desired_step_size_.translation.y;

    // rot
    // turn step, move lead foot out, following foot just pulls even again
    // if we just stepped one way, make sure following foot does follow
    // even if we now want to go opposite way
    bool follow_rot = false;
    if (fabs(feet_diff.rotation) > 0.01){
      follow_rot = true;
    }

    if (next.is_left_foot_ && walk_mem_->desired_step_size_.rotation > 0 && !follow_rot)
      foot_offset.rotation += walk_mem_->desired_step_size_.rotation;
    else if (!next.is_left_foot_ && walk_mem_->desired_step_size_.rotation < 0 && !follow_rot)
      foot_offset.rotation += walk_mem_->desired_step_size_.rotation;
  }

  // now convert from these coordinates relative to ref
  // to ones relative to the current frame
  next.position_ = foot_offset.relativeToGlobal(ref.position_);

}

void WalkModule::calcTargetStepSizes(const Step &ref){
  walk_mem_->previous_step_size_ = walk_mem_->desired_step_size_;
  Pose2D &desired = walk_mem_->desired_step_size_;
  // get target position relative to ref foot

  Pose3D stance_foot;
  if (step_current_->is_left_foot_) {
    stance_foot = (body_model_->abs_parts_[BodyPart::left_foot]);
  } else {
    stance_foot = (body_model_->abs_parts_[BodyPart::right_foot]);
  }
  Pose3D torso_target;
  torso_target.translation.x = walk_request_->target_point_.translation.x;
  torso_target.translation.y = walk_request_->target_point_.translation.y;
  // TODO ignoring target point rotation
  // from torso ref to stance ref
  torso_target = torso_target.globalToRelative(stance_foot);
  // from stance ref to global ref
  torso_target = torso_target.relativeToGlobal(walk_mem_->global_frame_offset_);
  // from global ref to ref step ref
  torso_target = torso_target.globalToRelative(ref.position_);

  // now lets target the point at the center of the foot (rather than ankle)
  torso_target.translation.x -= robot_info_->dimensions_.values_[RobotDimensions::ankleToFootCenter];

  // have to double it since each step actually walks half this from ref foot
  desired.translation.x = torso_target.translation.x * 2;

  // deal with translation y
  if (ref.is_left_foot_){
    // left foot should be at half foot sep left of target (to center it)
    desired.translation.y = torso_target.translation.y + (params_->foot_separation_ / 2.0);
  } else {
    desired.translation.y = torso_target.translation.y - (params_->foot_separation_ / 2.0); 
  }  
  desired.rotation = 0;

  // now make sure we're not over max step size
  desired.translation.x = crop(desired.translation.x, -params_->max_step_size_.translation.x, params_->max_step_size_.translation.x); 
  desired.translation.y = crop(desired.translation.y, -params_->max_step_size_.translation.y, params_->max_step_size_.translation.y); 
  

}


void WalkModule::calcRotateStepSizes(const Step &ref){
  walk_mem_->previous_step_size_ = walk_mem_->desired_step_size_;
  Pose2D &desired = walk_mem_->desired_step_size_;
  // get target position relative to ref foot

  Pose3D stance_foot;
  if (step_current_->is_left_foot_) {
    stance_foot = (body_model_->abs_parts_[BodyPart::left_foot]);
  } else {
    stance_foot = (body_model_->abs_parts_[BodyPart::right_foot]);
  }
  Pose3D torso_target;
  torso_target.translation.x = walk_request_->target_point_.translation.x;
  torso_target.translation.y = walk_request_->target_point_.translation.y;
  // TODO ignoring target point rotation
  // from torso ref to stance ref
  torso_target = torso_target.globalToRelative(stance_foot);
  // from stance ref to global ref
  torso_target = torso_target.relativeToGlobal(walk_mem_->global_frame_offset_);
  // from global ref to ref step ref
  torso_target = torso_target.globalToRelative(ref.position_);

  // now lets target the point at the center of the foot (rather than ankle)
  torso_target.translation.x -= robot_info_->dimensions_.values_[RobotDimensions::ankleToFootCenter];

  // and keep at desired distance
  torso_target.translation.x -= walk_request_->rotate_distance_;

  // have to double it since each step actually walks half this from ref foot
  desired.translation.x = torso_target.translation.x * 2;

  // deal with translation y to center target between feet
  if (ref.is_left_foot_){
    // left foot should be at half foot sep left of target (to center it)
    desired.translation.y = torso_target.translation.y + (params_->foot_separation_ / 2.0);
  } else {
    desired.translation.y = torso_target.translation.y - (params_->foot_separation_ / 2.0); 
  }  

  // strafe to match rotation
  float strafe_mag = 0.0;
  if (fabs(walk_request_->rotate_heading_) > 1.57){
    strafe_mag = params_->max_step_size_.translation.y;
  } else {
    strafe_mag = fabs(walk_request_->rotate_distance_ * tanf(walk_request_->rotate_heading_));
  }

  // figure out sign
  float desired_strafe = strafe_mag;
  if (walk_request_->rotate_heading_ > 0)
    desired_strafe = -strafe_mag;
    
  desired.translation.y += desired_strafe;

  // now make sure we're not over max step size
  desired.translation.x = crop(desired.translation.x, -params_->max_step_size_.translation.x, params_->max_step_size_.translation.x); 
  desired.rotation = crop(desired.rotation, -params_->max_step_size_.rotation, params_->max_step_size_.rotation);
  desired.translation.y = crop(desired.translation.y, -params_->max_step_size_.translation.y, params_->max_step_size_.translation.y); 
  

  // try to rotate to face target point
  desired.rotation = atan(walk_request_->target_point_.translation.y / walk_request_->target_point_.translation.x);

  desired.translation.x = crop(desired.translation.x, -params_->max_step_size_.translation.x, params_->max_step_size_.translation.x); 
  desired.rotation = crop(desired.rotation, -params_->max_step_size_.rotation, params_->max_step_size_.rotation);
}


void WalkModule::initZMPRef() {
  walk_mem_->zmp_ref_.init();
  float frame = frame_info_->frame_id;
  //float t = frame_info_->seconds_since_start;
  //while (t < (step_two_after_next_->time_ - motion_time_)) {
  while (frame < (step_two_after_next_->frame_ - 1)) {
    if (walk_mem_->zmp_ref_.getNumberOfEntries() >= walk_mem_->zmp_ref_.getMaxEntries())
      return;
    walk_mem_->zmp_ref_.push(calcZMPRef(frame));
    //t += motion_time_;
    frame++;
  }
}

Vector2<float> WalkModule::calcZMPRef(unsigned int frame) {
  Vector2<float> zmp_ref;

  // lets calculate relative to stance foot and then rotate???
  if (frame < step_current_->frame_){
    // in double support from prev to current
    zmp_ref = getZMPRefForStep(*step_prev_,*step_prev_,*step_current_,frame);
  } else if (frame <= step_current_->frame_ + walk_mem_->num_double_support_frames_) {
    // in double support from prev to current
    interpDoubleSupportZMP(zmp_ref,frame,*step_prev_,*step_current_,*step_prev_);
  } else if (frame <= step_next_->frame_) {
    zmp_ref = getZMPRefForStep(*step_current_,*step_prev_,*step_next_,frame);
  } else if (frame <= step_next_->frame_ + walk_mem_->num_double_support_frames_) {
    // in double support from current to next
    interpDoubleSupportZMP(zmp_ref,frame,*step_current_,*step_next_,*step_prev_);
  } else if (frame <= step_after_next_->frame_) {
    // in single support of next
    zmp_ref = getZMPRefForStep(*step_next_,*step_current_,*step_after_next_,frame);
  } else if (frame <= step_after_next_->frame_ + walk_mem_->num_double_support_frames_) {
    // in double support from next to after next
    interpDoubleSupportZMP(zmp_ref,frame,*step_next_,*step_after_next_,*step_current_);
  } else if (frame <= step_two_after_next_->frame_) {
    // in single support of next
    zmp_ref = getZMPRefForStep(*step_after_next_,*step_next_,*step_two_after_next_,frame);
  } else if (frame <= step_two_after_next_->frame_ + walk_mem_->num_double_support_frames_) {
    // in double support from next to after next
    interpDoubleSupportZMP(zmp_ref,frame,*step_after_next_,*step_two_after_next_,*step_next_);
  } else {
    std::cerr << "WalkModule::calcZMPRef called with bad time, should only happen in initWalk, but not\n  step_two_after_next_->frame_ - frame  = " << step_two_after_next_->frame_ - frame << std::endl;
  }

  //float double_support_time = params_->double_support_frac_ * params_->phase_length_;
  //if (t < step_current_->time_){
    //// in double support from prev to current
    //zmp_ref = getZMPRefForStep(*step_prev_,*step_prev_,*step_current_,t);
  //} else if (t <= step_current_->time_ + double_support_time) {
    //// in double support from prev to current
    //interpDoubleSupportZMP(zmp_ref,t,*step_prev_,*step_current_,*step_prev_);
  //} else if (t <= step_next_->time_) {
    //zmp_ref = getZMPRefForStep(*step_current_,*step_prev_,*step_next_,t);
  //} else if (t <= step_next_->time_ + double_support_time) {
    //// in double support from current to next
    //interpDoubleSupportZMP(zmp_ref,t,*step_current_,*step_next_,*step_prev_);
  //} else if (t <= step_after_next_->time_) {
    //// in single support of next
    //zmp_ref = getZMPRefForStep(*step_next_,*step_current_,*step_after_next_,t);
  //} else if (t <= step_after_next_->time_ + double_support_time) {
    //// in double support from next to after next
    //interpDoubleSupportZMP(zmp_ref,t,*step_next_,*step_after_next_,*step_current_);
  //} else if (t <= step_two_after_next_->time_) {
    //// in single support of next
    //zmp_ref = getZMPRefForStep(*step_after_next_,*step_next_,*step_two_after_next_,t);
  //} else if (t <= step_two_after_next_->time_ + double_support_time) {
    //// in double support from next to after next
    //interpDoubleSupportZMP(zmp_ref,t,*step_after_next_,*step_two_after_next_,*step_next_);
  //} else {
    //std::cerr << "WalkModule::calcZMPRef called with bad time, should only happen in initWalk, but not\n  step_two_after_next_->time_ -t  = " << step_two_after_next_->time_ - t << std::endl;
  //}

  return zmp_ref;
}


Vector2<float> WalkModule::getZMPRefForStep(const Step &step, const Step &prev, const Step &next, unsigned int frame){
  // calculate what the zmp reference should be for this step
  Vector2<float> zmp_ref;
  zmp_ref.x = step.position_.translation.x;
  zmp_ref.y = step.position_.translation.y;
    
  float curr_step_start_frame = (step.frame_);//  + double_support_time);
  float curr_step_frame_length = next.frame_ - curr_step_start_frame;
  float curr_step_fraction = (frame - curr_step_start_frame) / curr_step_frame_length;
    
  // get offset to prev and next in this steps frame of ref
  Pose2D prev_offset = prev.position_.globalToRelative(step.position_);
  Pose2D next_offset = next.position_.globalToRelative(step.position_);
    

  // possibly smoothly interp zmp forward all the time
  float x_offset = 0;
  if (params_->interp_zmp_forward_){
    
    // if we're less than halfway through this step's time
    if (curr_step_fraction < 0.5){
      x_offset = prev_offset.translation.x * (0.5-curr_step_fraction);
    } else {
      x_offset = next_offset.translation.x * (curr_step_fraction-0.5);
    }
  }

  // have to figure out offset based on foot rotation
  Vector2<float> ankleFootOffset(robot_info_->dimensions_.values_[RobotDimensions::ankleToFootCenter]+x_offset,0);
  ankleFootOffset.rotate(step.position_.rotation);
  zmp_ref += ankleFootOffset;

  float shift_amount = params_->interp_zmp_side_amount_ * 0.5 * (next_offset.translation.y - prev_offset.translation.y) / params_->max_step_size_.translation.y;
  //std::cout << t << " " << shift_amount << " " << params_->interp_zmp_side_amount_ << " " << next_offset.translation.y << " " << prev_offset.translation.y << " " << params_->max_step_size_.translation.y << std::endl;

  // add offsets from parameters
  Vector2<float> offset(0,0);
  // if stand... move it over between the two feet
  if (step.is_stand_){
    if (step.is_left_foot_)
      offset.y = -params_->foot_separation_ / 2.0;
    else
      offset.y = params_->foot_separation_ / 2.0;
  } else {
    if (step.is_left_foot_)
      offset = params_->left_foot_zmp_offset_;
    else
      offset = params_->right_foot_zmp_offset_;
    // alter the ys
    float start_zmp = -shift_amount;
    float stop_zmp = shift_amount;
    offset.y += curr_step_fraction * (stop_zmp - start_zmp) + start_zmp;
  }
  //std::cout << offset.y << std::endl;

  zmp_ref += offset.rotate(step.position_.rotation);

  return zmp_ref;
}

inline float sigmoid(float t) {
  return 1.0 / (1.0 + exp(-t));
}

inline float sigmoidScaled(float t, float minT, float factor) {
  float minVal = sigmoid(minT * factor);
  return (sigmoid(t * factor) - minVal) / (1.0 - 2.0 * minVal);
}

void WalkModule::interpDoubleSupportZMP(Vector2<float> &zmp, unsigned int frame, const Step &current, const Step &next, const Step &prev) {
  //float double_support_time = params_->double_support_frac_ * params_->phase_length_;
  float frac = (frame - next.frame_) / (float)walk_mem_->num_double_support_frames_;
  float factor = 5;
  frac = sigmoidScaled(frac-0.5,-0.5,factor);
  Vector2<float> init_pos = getZMPRefForStep(current, prev, next, next.frame_);
  Step fake_next = current;
  fake_next.frame_ = next.frame_ + walk_mem_->num_step_frames_;
  Vector2<float> next_pos = getZMPRefForStep(next, current, fake_next, next.frame_+walk_mem_->num_double_support_frames_);
  Vector2<float> diff = next_pos - init_pos;
  zmp = diff*frac + init_pos;
}

float bspline(float t, float times[], float pts[], int num_pts) {
  float mat[4][4];
  mat[0][0] = -1; mat[0][1] = 3; mat[0][2] = -3; mat[0][3] = 1;
  mat[1][0] = 3; mat[1][1] = -6; mat[1][2] = 3; mat[1][3] = 0;
  mat[2][0] = -3; mat[2][1] = 0; mat[2][2] = 3; mat[2][3] = 0;
  mat[3][0] = 1; mat[3][1] = 4; mat[3][2] = 1; mat[3][3] = 0;
  
  t += 0.00001;
  int ind = 0;
  while (t > times[ind]) {
    ind++;
  }
  ind--;

  if (ind + 4 > num_pts) {
    std::cout << "BAD NUM PTS " << ind+4 << " " << num_pts << std::endl;
  }
  if (ind < 0)
    std::cout << "BAD IND: " << ind << std::endl;

  float res[4];
  for (int i = 0; i < 4; i++) {
    res[i] = 0;
    for (int j = 0; j < 4; j++) {
      res[i] += mat[i][j] * pts[j];
    }
  }

  float ts[4] = {t * t * t, t * t, t, 1};
  float val = 0;
  for (int i = 0; i < 4; i++) {
    val += ts[i] * res[i];
  }
  val /= 6;

  //std::cout << "t: " << t << " ind: " << ind << " time: " << times[ind] << " val: " << val << std::endl;
  return val;
}

void WalkModule::calcSwingTarget() {
  Pose3D *swing_foot = &(walk_mem_->swing_foot_);

  float lift = calcSwingLift();
  float t = calcSingleSupportFrac();
  // if current step is stand, then we're not balanced on other leg and
  // can't lift this one
  if (t < 0 || step_current_->is_stand_) {
    *swing_foot = Pose3D(Vector3<float>(step_prev_->position_.translation.x,step_prev_->position_.translation.y,0));
    swing_foot->rotation = RotationMatrix(0,0,-step_prev_->position_.rotation);
  } else {
    t = (t - params_->step_start_time_) / (params_->step_stop_time_ - params_->step_start_time_);
    t = crop(t,0.0,1.0);
    float change_frac = sigmoidScaled(t - 0.5, -0.5,params_->step_speed_factor_);
    //float times[] = {0,params_->step_start_time_,params_->step_stop_time_,1.0,1.1,1.2};
    //float pts[] = {-0.5,0.0,1.0,1.0,1.5,1.5};
    //float change_frac = bspline(t,times,pts,6);
    //std::cout << t << " " << change_frac << std::endl;
    Pose2D target;
    target.translation = (step_next_->position_.translation - step_prev_->position_.translation) * change_frac + step_prev_->position_.translation;
    target.rotation = (step_next_->position_.rotation - step_prev_->position_.rotation) * change_frac + step_prev_->position_.rotation;

    float foot_tilt = calcSwingTilt();

    swing_foot->translation = Vector3<float>(target.translation.x, target.translation.y, lift);
    swing_foot->rotation = RotationMatrix(0,foot_tilt,-target.rotation);
  }
}

float WalkModule::calcSwingLift() {
  float t = calcSingleSupportFrac();
  if ((t < params_->lift_start_time_) || (t >= params_->lift_stop_time_)) {
    // still in double support
    return 0;
  }
  t = (t - params_->lift_start_time_) / (params_->lift_stop_time_ - params_->lift_start_time_);
  float val;
  val = 0.5 * (1 - cos(2 * M_PI * t));
  val *= params_->step_height_;
  return val;
}

float WalkModule::calcSwingTilt() {
  float t = calcSingleSupportFrac();
  if ((t < params_->swing_tilt_start_frac_) || (t >= params_->swing_tilt_stop_frac_)) {
    return 0;
  }
  t = (t - params_->swing_tilt_start_frac_) / (params_->swing_tilt_stop_frac_ - params_->swing_tilt_start_frac_);
  return params_->swing_tilt_amount_ * 0.5 * (1 - cos(2 * M_PI * t));
}

void WalkModule::calcJointTargets() {

  // calculate body model from last commands
  Vector3<float> pen;
  calculatePendulum(previous_commands_, pen, step_current_->is_left_foot_);

  // Todd: we calculate leg x,y,z based on the ones we sent last time
  // (in the command_body_model_)
  // we can calculate them based on perceived angles by setting this to
  // body_model_
  BodyModelBlock* target_body_model = command_body_model_;

  // figure out which leg is stance and which is swing
  Pose3D *stance = &(walk_mem_->abs_right_foot_);
  Pose3D *swing = &(walk_mem_->abs_left_foot_);
  BodyPart::Part stance_foot = BodyPart::right_foot;
  //BodyPart::Part swing_foot = BodyPart::left_foot;
  int stance_hip_roll = RHipRoll;

  if (step_current_->is_left_foot_) {
    stance = &(walk_mem_->abs_left_foot_);
    swing = &(walk_mem_->abs_right_foot_);
    stance_foot = BodyPart::left_foot;
    //swing_foot = BodyPart::right_foot;
    stance_hip_roll = LHipRoll;
  }

  Vector3<float> pen_err;
  float max_acceptable_pen_err = 0.25; //1.0;

  // the offset from stance leg to torso
  Vector3<float> stance_to_torso_offset;
  stance_to_torso_offset = target_body_model->abs_parts_[stance_foot].translation - target_body_model->abs_parts_[BodyPart::torso].translation;

  Vector3<float> abs_desired_pen;
  // convert to stance, then from stance to torso
  Pose2D des_pen(0,0,0);
  des_pen.translation = walk_mem_->desired_next_state_.pen_pos_;
  des_pen = des_pen.globalToRelative(walk_mem_->global_frame_offset_);

  abs_desired_pen.x = des_pen.translation.x + stance_to_torso_offset.x;
  abs_desired_pen.y = des_pen.translation.y + stance_to_torso_offset.y;
  abs_desired_pen.z = params_->walk_height_;

  // set them both equal to swing for now, we'll over ride the stance later
  walk_mem_->abs_left_foot_ = walk_mem_->swing_foot_;
  walk_mem_->abs_right_foot_ = walk_mem_->swing_foot_;

  // stance leg starts out at current position
  // and will be offset later by com change we want
  stance->rotation = RotationMatrix(0,0,0);
  stance->translation = target_body_model->abs_parts_[stance_foot].translation;
  stance->rotation.rotateZ(target_body_model->abs_parts_[stance_foot].rotation.getZAngle());

  // convert from stance in abs frame to stance in torso frame
  //stance->translation -= target_body_model->abs_parts_[BodyPart::torso].translation;
  //Vector2<float> stanceXY;
  //stanceXY.x = stance->translation.x;
  //stanceXY.y = stance->translation.y;
  //stanceXY.rotate(-target_body_model->abs_parts_[BodyPart::torso].rotation.getZAngle());
  //stance->translation.x = stanceXY.x;
  //stance->translation.y = stanceXY.y;
  //stance->rotation.rotateZ(-target_body_model->abs_parts_[BodyPart::torso].rotation.getZAngle());

  // convert swing leg from global to stance to torso frame
  *swing = swing->globalToRelative(walk_mem_->global_frame_offset_);
  Vector2<float> swingXY;
  swingXY.x = swing->translation.x;
  swingXY.y = swing->translation.y;
  swingXY.rotate(stance->rotation.getZAngle());
  swing->translation.x = swingXY.x;
  swing->translation.y = swingXY.y;
  swing->translation += stance->translation;
  swing->rotation.rotateZ(stance->rotation.getZAngle());

  Vector3<float> arm_offsets(0,0,0);

  // put a cap of 0.5mm z movements
  pen_err = abs_desired_pen - pen;
  if (step_current_->is_stand_){
    float pen_cap = 0.5;
    if (fabs(pen_err.z) > pen_cap){
      if (pen_err.z > 0)
        abs_desired_pen.z = pen.z + pen_cap;
      else
        abs_desired_pen.z = pen.z - pen_cap;
      //      cout << "pen z movement capped to " << (abs_desired_pen.z - pen.z) << " new des: " << abs_desired_pen.z <<  endl;
    }
  }

  for (int i = 0; i < 100; i++) {
    //    std::cout << "ITER: " << i << " " << stance->translation.z << std::endl;
    pen_err = abs_desired_pen - pen;
    //cout << i << " pen error: " << pen_err << " stance Z: " << stance->translation.z << " " << pen_err.abs() << endl;
    if (pen_err.abs() < max_acceptable_pen_err && i > 0)
      break;

    // so we don't wildly overshoot 
    // (z movements kind of result in double com movements... torso lowers
    // and we lower our weight)
    pen_err.z *= 0.5;

    //arm_offsets.x += pen_err.x * 1;
    //arm_offsets.y += pen_err.y * 1;
    //arm_offsets.z += pen_err.z * 5;

    // get stance foot position relative to torso
    // move stance foot opposite direction from com error
    stance->translation.x -= pen_err.x;
    stance->translation.y -= pen_err.y;
    stance->translation.z -= pen_err.z;

    // also move desired com by this amount...
    // since we moved stance leg
    abs_desired_pen.x -= pen_err.x;
    abs_desired_pen.y -= pen_err.y;

    // Todd: desired com z is absolute now
    //abs_desired_com.z -= pen_err.z;

    // as stance leg moves one way, we have to move the swing leg relative to
    // the torso to match it
    // so swing leg to stance leg distance remains the same
    swing->translation.x -= pen_err.x;
    swing->translation.y -= pen_err.y;
    swing->translation.z -= pen_err.z;

    if (stance->translation.z < -203) stance->translation.z = -203;
    if (swing->translation.z < -203) swing->translation.z = -203;

    bool left_compliant  = true;//false;
    bool right_compliant = true;//false;

    // calculate tilt roll from stance leg
    TiltRoll tr = ForwardKinematics::calculateTiltRollFromLeg(step_current_->is_left_foot_, commands_->angles_, robot_info_->dimensions_);

    commandLegsRelativeToTorso(commands_->angles_, walk_mem_->abs_left_foot_, walk_mem_->abs_right_foot_, tr.tilt_, tr.roll_,left_compliant,right_compliant);
    setArms(commands_->angles_,arm_offsets);

    // calculate new com based on these joint commands
    calculatePendulum(commands_->angles_, pen, step_current_->is_left_foot_);
    //cout << "new pen: " << pen << endl;
  }
  


  for (int i = 0; i < NUM_JOINTS; i++) {
    previous_commands_[i] = commands_->angles_[i];
  }
  
  float hip_offset = 0.0;
  if ((!step_current_->is_stand_) && (!step_prev_->is_stand_) && (!step_next_->is_stand_)){
    // add hip offset trapezoid
    float t = calcSingleSupportFrac();
    //t -= params_->hip_roll_offset_start_frac_;
    if (t < params_->hip_roll_offset_start_frac_) {
      // pass
    } else if (t < params_->hip_roll_offset_start_frac_ + params_->hip_roll_offset_rise_frac_) {
      // in rise
      float t2 = t - params_->hip_roll_offset_start_frac_;
      hip_offset = params_->hip_roll_offset_amount_ * (t2 / params_->hip_roll_offset_rise_frac_);
    } else if (t < params_->hip_roll_offset_stop_frac_ - params_->hip_roll_offset_fall_frac_) {
      // in steady
      hip_offset = params_->hip_roll_offset_amount_;
    } else if (t < params_->hip_roll_offset_stop_frac_) {
      // in decay
      float t2 = params_->hip_roll_offset_stop_frac_ - t;
      hip_offset = params_->hip_roll_offset_amount_ * (t2 / params_->hip_roll_offset_fall_frac_);
    }
    //std::cout << t << " " << step_current_->is_left_foot_ << " " <<  RAD_T_DEG*commands_->angles_[stance_hip_roll] << " " << RAD_T_DEG * hip_offset << std::endl;
    commands_->angles_[stance_hip_roll] += hip_offset;
  }


  commands_->send_body_angles_ = true;
  commands_->body_angle_time_ = 10;
}

void WalkModule::calculateCenterOfMass(float *command_angles, Vector3<float> &center_of_mass, bool stance_is_left){

  // calculate tilt roll from stance leg
  TiltRoll tr = ForwardKinematics::calculateTiltRollFromLeg(stance_is_left, command_angles, robot_info_->dimensions_);
  //TiltRoll tr = body_model_->sensors_tilt_roll_;

  // use some fraction of tilt and roll
  tr.tilt_ *= params_->tilt_roll_factor_;
  tr.roll_ *= params_->tilt_roll_factor_;

  ForwardKinematics::calculateRelativePose(command_angles, tr.tilt_, tr.roll_, command_body_model_->rel_parts_, robot_info_->dimensions_.values_);
  Pose3D base = ForwardKinematics::calculateVirtualBase(stance_is_left, command_body_model_->rel_parts_);
  ForwardKinematics::calculateAbsolutePose(base, command_body_model_->rel_parts_, command_body_model_->abs_parts_);
  ForwardKinematics::calculateCoM(command_body_model_->abs_parts_, command_body_model_->center_of_mass_, robot_info_->mass_calibration_);
  center_of_mass = command_body_model_->center_of_mass_;

}

void WalkModule::calculatePendulum(float *command_angles, Vector3<float> &pendulum, bool stance_is_left){

  // calculate tilt roll from stance leg
  TiltRoll tr = ForwardKinematics::calculateTiltRollFromLeg(stance_is_left, command_angles, robot_info_->dimensions_);
  //TiltRoll tr = body_model_->sensors_tilt_roll_;

  // use some fraction of tilt and roll
  tr.tilt_ *= params_->tilt_roll_factor_;
  tr.roll_ *= params_->tilt_roll_factor_;

  ForwardKinematics::calculateRelativePose(command_angles, tr.tilt_, tr.roll_, command_body_model_->rel_parts_, robot_info_->dimensions_.values_);
  Pose3D base = ForwardKinematics::calculateVirtualBase(stance_is_left, command_body_model_->rel_parts_);
  ForwardKinematics::calculateAbsolutePose(base, command_body_model_->rel_parts_, command_body_model_->abs_parts_);


  pendulum = command_body_model_->abs_parts_[BodyPart::torso].translation;

}

void WalkModule::commandLegsRelativeToTorso(float *command_angles, Pose3D left_target, Pose3D right_target, float tilt, float roll, bool left_compliant, bool right_compliant) {

  // use some fraction of tilt and roll
  tilt *= params_->tilt_roll_factor_;
  roll *= params_->tilt_roll_factor_;

  RotationMatrix rot;
  RotationMatrix foot_rotation;

  rot.rotateY(tilt);
  rot.rotateX(roll);

  foot_rotation.rotateY(-tilt);
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
  //std::cout << "OLD GROIN: " << command_angles[LHipYawPitch] << std::endl;
  //float groin = right_target.rotation.getZAngle() - left_target.rotation.getZAngle();
  //std::cout << "NEW GROIN: " << groin << std::endl;
  //inverse_kinematics_.calcLegJoints(left_target, command_angles, groin, true, robot_info_->dimensions_);
  //inverse_kinematics_.calcLegJoints(right_target, command_angles, groin, false, robot_info_->dimensions_);

}

void WalkModule::setArms(float *command_angles, const Vector3<float> &/*offset*/) {
  /*
  float x = 0;
  float y = robot_info_->dimensions_.armOffset.y + 30;
  float z = -20;
  RotationMatrix left_rot;
  left_rot.rotateZ(DEG_T_RAD * 90);
  RotationMatrix right_rot;
  right_rot.rotateZ(DEG_T_RAD * -90);

  Pose3D left(left_rot,Vector3<float>(x,y,z) + offset);
  Pose3D right(right_rot,Vector3<float>(x,-y,z) + offset);
  
  bool res = inverse_kinematics_.calcArmJoints(left,right,command_angles,robot_info_->dimensions_);
  if (!res) {
    std::cout << "Arm inv kinematics failed" << std::endl;
    left = Pose3D(left_rot,Vector3<float>(x,y,z));
    right = Pose3D(right_rot,Vector3<float>(x,-y,z));
    res = inverse_kinematics_.calcArmJoints(left,right,command_angles,robot_info_->dimensions_);
    if (!res)
      std::cout << "Booo..., bad arm inv kinematics" << std::endl;
  }
  */
  command_angles[LShoulderPitch] = DEG_T_RAD*-80;
  command_angles[RShoulderPitch] = DEG_T_RAD*-80;

  command_angles[LShoulderRoll] = DEG_T_RAD*15;
  command_angles[RShoulderRoll] = DEG_T_RAD*15;

  command_angles[LElbowRoll] = DEG_T_RAD*-15;
  command_angles[RElbowRoll] = DEG_T_RAD*-15;

  command_angles[LElbowYaw] = DEG_T_RAD*-70;
  command_angles[RElbowYaw] = DEG_T_RAD*-70;
}

float WalkModule::calcPhaseFrac() {
  //return (frame_info_->seconds_since_start - step_current_->time_ - 0.015 / params_->phase_length_) / (step_next_->time_ - step_current_->time_);
  return ((int)frame_info_->frame_id - (int)step_current_->frame_ - 1) / (float)((int)step_next_->frame_ - (int)step_current_->frame_ - 1);
}

float WalkModule::calcSingleSupportFrac() {
  float denom = max(1,(int)step_next_->frame_ - (int)step_current_->frame_ - (int)walk_mem_->num_double_support_frames_ - 1);
  return ((int)frame_info_->frame_id - (int)step_current_->frame_ - (int)walk_mem_->num_double_support_frames_ - 1) / denom;
  //return (phase_frac - params_->double_support_frac_) / (1.0 - params_->double_support_frac_);
  //return (phase_frac - params_->double_support_frac_ - 0.01 / params_->phase_length_) / (1.0 - params_->double_support_frac_);
}

void WalkModule::resetTimeInMotion() {
  walk_mem_->time_motion_started_ = frame_info_->seconds_since_start;
}

float WalkModule::getTimeInMotion() {
  return frame_info_->seconds_since_start - walk_mem_->time_motion_started_;
}

void WalkModule::updateOdometry() {

  // save where torso was last time vision updated odom
  if (odometry_->displacement.translation.x == 0 && odometry_->displacement.translation.y == 0 && odometry_->displacement.rotation == 0){
    walk_mem_->global_to_odometry_frame_offset_ = walk_mem_->global_last_torso_;
  }

  // figure out current torso location in global frame
  calculateGlobalTorsoLocation();

  // odometry is how much torso has moved last time vision used it
  odometry_->displacement = walk_mem_->global_last_torso_.globalToRelative(walk_mem_->global_to_odometry_frame_offset_);

  // set if standing or walking
  odometry_->standing = step_current_->is_stand_;

}


// for odometry
void WalkModule::calculateGlobalTorsoLocation(){
  // figure out current torso location in global frame
  Pose3D abs_to_stance_offset;
  if (step_current_->is_left_foot_)
    abs_to_stance_offset = body_model_->abs_parts_[BodyPart::left_foot];
  else
    abs_to_stance_offset = body_model_->abs_parts_[BodyPart::right_foot];

  // convert from abs to stance
  Pose3D torso_stance_frame = body_model_->abs_parts_[BodyPart::torso].globalToRelative(abs_to_stance_offset);
  // convert from stance back to global
  Pose3D torso_global_frame = torso_stance_frame.relativeToGlobal(walk_mem_->global_frame_offset_);

  Pose2D torso_global(0,0,0);
  torso_global.rotation = torso_global_frame.rotation.getZAngle();
  torso_global.translation.x = torso_global_frame.translation.x;
  torso_global.translation.y = torso_global_frame.translation.y;

  // save global torso location
  walk_mem_->global_last_torso_ = torso_global;

}

