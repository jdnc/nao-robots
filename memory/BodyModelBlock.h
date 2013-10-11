#ifndef BODY_MODEL_BLOCK_
#define BODY_MODEL_BLOCK_

#include <common/RobotInfo.h>
#include <math/Pose3D.h>
#include <math/Vector3.h>
#include <math/Vector2.h>
#include <common/TiltRoll.h>
#include <kinematics/TorsoMatrix.h>

#include "MemoryBlock.h"

struct BodyModelBlock : public MemoryBlock {
public:
  BodyModelBlock()  {
    header.version = 11;
    header.size = sizeof(BodyModelBlock);
    is_calculated_ = false;
    feet_on_ground_ = true;
    feet_on_ground_inst_ = true;
  }  
  bool is_calculated_;

  // Body relative to origin // no rotations
  Pose3D rel_parts_[BodyPart::NUM_PARTS];

  // Body translated and relative to ground
  Pose3D abs_parts_[BodyPart::NUM_PARTS];

  TorsoMatrix torso_matrix_;
  
  Vector3<float> center_of_mass_;

  TiltRoll left_foot_body_tilt_roll_;
  TiltRoll right_foot_body_tilt_roll_;
  TiltRoll sensors_tilt_roll_;

  Vector2<float> zmpFromFSRs;

  bool feet_on_ground_; // if up for over 25 frames
  bool feet_on_ground_inst_; // instantaneous
};

#endif 
