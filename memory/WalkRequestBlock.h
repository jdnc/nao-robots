#ifndef WALKREQUESTBLOCK_RE8SDRLN
#define WALKREQUESTBLOCK_RE8SDRLN

#include <math/Pose2D.h>
#include "MemoryBlock.h"


const std::string walkNames[] = {
  "WALK",
  "STAND",
  "FALLING",
  "GETUP",
  "FALL_PREVENTION",
  "NONE",
  "STEP_LEFT",
  "STEP_RIGHT",
  "WAIT"
};


struct WalkRequestBlock : public MemoryBlock {
public:
  enum Motion {
    WALK,
    STAND,
    FALLING,
    GETUP,
    FALL_PREVENTION,
    NONE,
    STEP_LEFT,
    STEP_RIGHT,
    WAIT,
    NUM_OF_MOTIONS
  };

  WalkRequestBlock():
    new_command_(false),
    motion_(NONE),
    speed_(),
    percentage_speed_(true),
    pedantic_walk_(false),
    walk_to_target_(false),
    target_walk_is_active_(false),
    rotate_around_target_(false),
    rotate_heading_(0),
    perform_kick_(false),
    kick_heading_(0),
    kick_distance_(3000),
    kick_with_left_(false),
    step_into_kick_(false),
    set_kick_step_params_(false),
    tilt_fallen_counter_(0),
    roll_fallen_counter_(0),
    getup_from_keeper_dive_(false),
    odometry_fwd_offset_(0),
    odometry_side_offset_(0),
    odometry_turn_offset_(0),
    keep_arms_out_(false),
    slow_stand_(false)
  {
    header.version = 16;
    header.size = sizeof(WalkRequestBlock);
  }
    
  void set(Motion m, Pose2D speed, bool percentage_speed, bool pedantic) {
    new_command_ = true;
    motion_ = m;
    speed_ = speed;
    percentage_speed_ = percentage_speed;
    pedantic_walk_ = pedantic;
    walk_to_target_ = false;
    rotate_around_target_ = false;
    perform_kick_ = false;
    kick_heading_ = 0;
    kick_distance_ = 3000;
    kick_with_left_ = false;
    step_into_kick_ = false;
    rotate_distance_ = 0;
    rotate_heading_ = 0;
  }

  void noWalk() {
    set(NONE,Pose2D(0,0,0),true,false);
  }

  void stand() {
    set(STAND,Pose2D(0,0,0),true,false);
  }

  void wait() {
    set(WAIT,Pose2D(0,0,0),true,false);
  }

  void setStep(bool isLeft, float x, float y, float rotation) {
    Motion walk = STEP_RIGHT;
    if (isLeft) walk = STEP_LEFT;
    set(walk,Pose2D(rotation,x,y),false,false);
  }

  void setWalk(float x, float y, float rotation) {
    set(WALK,Pose2D(rotation,x,y),true,false);
  }

  void setPedanticWalk(float x, float y, float rotation) {
    set(WALK,Pose2D(rotation,x,y),true,true);
  }
  
  void setFalling() {
    set(FALLING,Pose2D(0,0,0),true,false);
  }

  void setKick(float distance, float heading, bool with_left, bool step_into_kick){
    new_command_ = true;
    motion_ = WALK;
    perform_kick_ = true;
    kick_heading_ = heading;
    kick_distance_ = distance;
    kick_with_left_ = with_left;
    step_into_kick_ = step_into_kick;
  }

  void setOdometryOffsets(float fwd, float side, float turn) {
    odometry_fwd_offset_ = fwd;
    odometry_side_offset_ = side;
    odometry_turn_offset_ = turn;
  }

  void setWalkTarget(float relx, float rely, float relang, bool pedantic = false) {
    new_command_ = true;
    motion_ = WALK;
    walk_to_target_ = true;
    rotate_around_target_ = false;
    target_point_.translation.x = relx;
    target_point_.translation.y = rely;
    target_point_.rotation = relang;
    pedantic_walk_ = pedantic;
  }

  void setKickStepParams(int type, const Pose2D &preStep, const Pose2D &step, float refX) {
    set_kick_step_params_ = true;
    step_kick_type_ = type;
    pre_kick_step_ = preStep;
    kick_step_ = step;
    kick_step_ref_x_ = refX;
  }

  bool new_command_;
  Motion motion_; // what type of motion, walk/stand/kick
  Pose2D speed_; // the speed of the walk
  bool percentage_speed_; // true if speed is percentage rather than absolute vel
  bool pedantic_walk_; // true disables the step size stabilization.  "Set it when precision is indispensable"

  bool is_penalised_;

  // target point walk 
  Pose2D target_point_;
  bool walk_to_target_;
  bool target_walk_is_active_;
  
  bool rotate_around_target_;
  float rotate_distance_;
  float rotate_heading_;

  // for kicking from walk
  bool perform_kick_;
  float kick_heading_;
  float kick_distance_;
  bool kick_with_left_;
  bool step_into_kick_;

  bool set_kick_step_params_;
  int step_kick_type_;
  Pose2D pre_kick_step_;
  Pose2D kick_step_;
  float kick_step_ref_x_;

  int tilt_fallen_counter_;
  int roll_fallen_counter_;
  bool getup_from_keeper_dive_;
  
  // walk odometry offsets
  float odometry_fwd_offset_;
  float odometry_side_offset_;
  float odometry_turn_offset_;

  // goalie arms
  bool keep_arms_out_;

  bool slow_stand_; // true if we need a slow stand

  bool walk_decides_finished_with_target_;
  float finished_with_target_max_x_error_;
  float finished_with_target_min_y_error_;
  float finished_with_target_max_y_error_;
};

#endif /* end of include guard: WALKREQUESTBLOCK_RE8SDRLN */
