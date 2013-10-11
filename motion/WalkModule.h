#ifndef WALKMODULE_N60V149W
#define WALKMODULE_N60V149W

#include <Module.h>

#include "WalkEngineParameters.h"
#include <common/RobotDimensions.h>
#include <kinematics/InverseKinematics.h>
#include <common/NMatrix.h>
#include <common/MassCalibration.h>

#include <memory/WalkEngineBlock.h>

class BodyModelBlock;
class FrameInfoBlock;
class GraphableBlock;
class JointBlock;
class JointCommandBlock;
class OdometryBlock;
class SensorBlock;
//class WalkEngineBlock;
class WalkParamBlock;
class WalkRequestBlock;
class RobotInfoBlock;

typedef WalkEngineBlock::Step Step;

class WalkModule: public Module {
public:
  WalkModule();
  ~WalkModule();
  void specifyMemoryDependency();
  void specifyMemoryBlocks();
  void initSpecificModule();

  void processFrame();

  void initStiffness(float val);
private:
  // internal functions

  // main functions
  void walk();

  // main helpers
  void switchSupport();

  // initialization
  void initWalk();
  void initStand();

  // walk request
  void processWalkRequest();
  void calcDesiredStepSize();
  void calcTargetStepSizes(const Step &ref);
  void calcRotateStepSizes(const Step &ref);
  void calcKickStepSize();

  // odometry
  void updateOdometry();
  void calculateGlobalTorsoLocation();

  // step positions
  void calcStep(Step &next, const Step &ref, const Step &prev);
  void replanSteps();
  
  // reference zmp
  void initZMPRef();
  //Vector2<float> calcZMPRef(float t);
  //void interpDoubleSupportZMP(Vector2<float> &zmp, float t, const Step &current, const Step &next, const Step &prev);
  //Vector2<float> getZMPRefForStep(const Step &step, const Step &prev, const Step &next, float t);
  Vector2<float> calcZMPRef(unsigned int frame);
  void interpDoubleSupportZMP(Vector2<float> &zmp, unsigned int frame, const Step &current, const Step &next, const Step &prev);
  Vector2<float> getZMPRefForStep(const Step &step, const Step &prev, const Step &next, unsigned int frame);

  // walk com state updates
  void calculateOptimalControl();
  void calculateDesiredNextState();

  // closed loop
  void estimateSensorZmp();
  void estimateSensorPendulum();
  void adjustStateFromSensorPendulum();

  void handleSensorDelay();
  Vector3<float> getGlobalPendulum();

  // swing leg
  void calcSwingTarget();
  float calcSwingLift();
  float calcSwingTilt();

  // calculate joint targets
  void calcJointTargets();
  void calculateCenterOfMass(float *command_angles, Vector3<float> &center_of_mass, bool stance_is_left);
  void calculatePendulum(float *command_angles, Vector3<float> &pendulum, bool stance_is_left);

  // calculating joint commands
  void commandLegsRelativeToTorso(float *command_angles, Pose3D left_target, Pose3D right_target, float tilt, float roll, bool left_compliant, bool right_compliant);
  void setArms(float *command_angles, const Vector3<float> &offset);

  // time helpers
  float calcPhaseFrac();
  float calcSingleSupportFrac();
  void resetTimeInMotion();
  float getTimeInMotion();
  
private:
  float previous_commands_[NUM_JOINTS];

  // some helpful pointers, so that we don't have to always use the memory ptrs
  Step *step_prev_;
  Step *step_current_;
  Step *step_next_;
  Step *step_after_next_;
  Step *step_two_after_next_;

  WalkEngineParameters *params_;


  Vector3<float> stand_position_;
  float motion_time_;
  float last_walk_time_;
  unsigned int last_walk_frame_;

  // kinematics
  InverseKinematics inverse_kinematics_;

private:
  void initFrames();

private:
  void initMatricesWrapper();
  void initMatrices(int pendulum_height);

  // Matrices for zmp and com calculations
  unsigned int num_preview_frames_;
  float Gd_[MAX_PREVIEW_FRAMES];
  float Gi_;
  NMatrix Gx_;
  NMatrix A0_;
  NMatrix b0_;
  NMatrix L_;
  NMatrix c0_;

  NMatrix current_state_mat_[2];

private:
  // memory blocks
  BodyModelBlock *body_model_;
  FrameInfoBlock *frame_info_;
  GraphableBlock *graph_;
  JointBlock *joints_;
  JointCommandBlock *commands_;
  OdometryBlock *odometry_;
  SensorBlock *sensors_;
  WalkEngineBlock *walk_mem_;
  WalkParamBlock *walk_param_;
  WalkRequestBlock *walk_request_;
  RobotInfoBlock *robot_info_;

  // spare one to calculate command body models
  BodyModelBlock *command_body_model_;
};

#endif /* end of include guard: WALKMODULE_N60V149W */
