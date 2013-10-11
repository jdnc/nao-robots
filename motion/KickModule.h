#ifndef KICKMODULE
#define KICKMODULE

#include <memory/Memory.h>
#include <Module.h>
#include <common/RobotDimensions.h>
#include <common/MassCalibration.h>
#include <kinematics/InverseKinematics.h>
#include <motion/KickParameters.h>
#include <common/RingBufferWithSum.h>
#include <memory/BodyModelBlock.h>
#include <math/Spline3D.h>

class FrameInfoBlock;
class JointBlock;
class JointCommandBlock;
class KickModuleBlock;
class KickParamBlock;
class KickRequestBlock;
class OdometryBlock;
class RobotInfoBlock;
class SensorBlock;
class SpeechBlock;
class WalkInfoBlock;
class WalkRequestBlock;

class KickModule : public Module {
public:
  KickModule ();
  virtual ~KickModule ();
  void specifyMemoryDependency();
  void specifyMemoryBlocks();
  void initSpecificModule();
  void setStand(const float *stand_angles) {
    for (int i = 0; i < NUM_JOINTS; i++)
      STAND_ANGLES[i] = stand_angles[i];
  }

  void processFrame();

private:
  int getFramesInState();
  float getTimeInState();
  float getMillisecondsInState();
  void processKickRequest();
  void startKick();
  void setHead();
  void initStiffness();
  void setLegStiffness(float stiff);
  void transitionToState(KickState::State state);
  bool chooseKickLeg();
  bool checkKickValidity();
  bool handleAiming();
  void kick();
  void calcSwingSplinePts();
  void setSwingSpline(int num_pts,double timesInMs[], double xs[], double ys[], double zs[]);
  void sendSplineCOMCommands(const Vector3<float> &com_target);
  void getSwingTargets(Vector3<float> &align, Vector3<float> &kick);
  void calcBallPosWRTSwingLeg();
  void setKickOdometry();
  void calcJointTargets(const Vector3<float> &com_target, const Pose3D &swing_rel_stance, bool left_swing, float command_angles[NUM_JOINTS],bool move_com,float roll);
  void setArms(float command_angles[NUM_JOINTS]);
  void calcCenterOfMass(float *command_angles, Vector3<float> &center_of_mass, bool stance_is_left, float tilt_roll_factor);
  void commandLegsRelativeToTorso(float *command_angles, Pose3D left_target, Pose3D right_target, float tilt, float roll, float tilt_roll_factor, bool left_compliant, bool right_compliant);


private:
  float previous_commands_[NUM_JOINTS];
public:
  Spline3D swing_spline_;
  Spline3D stance_spline_;
  int invalidCount;

private:
  InverseKinematics inverse_kinematics_;
  float STAND_ANGLES[NUM_JOINTS];

private:
  KickStateInfo *state_params_;
  KickParameters *params_;
  KickParameters *params_normal_;
  KickParameters *params_super_;
  BodyModelBlock command_body_model_; // for local computation about commands

  FrameInfoBlock *frame_info_;
  JointCommandBlock *commands_;
  JointBlock *joint_angles_;
  KickModuleBlock *kick_module_;
  KickParamBlock *kick_params_;
  KickRequestBlock *kick_request_;
  OdometryBlock *odometry_;
  RobotInfoBlock *robot_info_;
  SensorBlock *sensors_;
  SpeechBlock *speech_;
  WalkInfoBlock *walk_info_;
  WalkRequestBlock *walk_request_;
};

#endif
