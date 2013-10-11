#ifndef LUA_99KDYFIX5
#define LUA_99KDYFIX5

#include <Module.h>
#include "LuaInterp.h"
#include <math/Pose3D.h>
#include <common/RobotDimensions.h>

class BehaviorBlock;
class CameraBlock;
class FrameInfoBlock;
class GameStateBlock;
class JointBlock;
class KickRequestBlock;
class OdometryBlock;
class RobotStateBlock;
class SensorBlock;
class KickParamBlock;
class WalkParamBlock;
class WalkRequestBlock;
class WorldObjectBlock;
class TeamPacketsBlock;
class OpponentBlock;
class BehaviorParamBlock;
class JointCommandBlock;
class ProcessedSonarBlock;
class ALWalkParamBlock;
class WalkInfoBlock;
class RobotVisionBlock;
class BodyModelBlock;
class RobotInfoBlock;
class SpeechBlock;
class LocalizationBlock;

class LuaModule: public Module {
public:
  LuaModule();
  ~LuaModule();

  void specifyMemoryDependency();
  void specifyMemoryBlocks();
  void initSpecificModule();
  void processFrame();
  void behaviorProcessFrame();
  void initLua();
  void doStrategyCalculations();
  void call(const std::string &cmd);

  void updateModuleMemory(Memory *memory);

  void startLua();
  void triggerRestart(bool trigger) {lua_restart_requested_ = trigger;};
  bool isOk() {return lua_ok_; };

  // memory
  BehaviorBlock *behavior_;
  CameraBlock *camera_block_;
  FrameInfoBlock *vision_frame_info_;
  GameStateBlock *game_state_;
  JointBlock *joint_angles_;
  KickRequestBlock *kick_request_;
  OdometryBlock *odometry_;
  RobotStateBlock *robot_state_;
  SensorBlock *sensors_;
  KickParamBlock *kick_params_;
  WalkParamBlock *walk_param_;
  WalkRequestBlock *walk_request_;
  WorldObjectBlock *world_objects_;
  TeamPacketsBlock *team_packets_;
  OpponentBlock *opponents_;
  BehaviorParamBlock *behavior_params_;
  JointCommandBlock *joint_commands_;
  ProcessedSonarBlock *vision_processed_sonar_;
  ALWalkParamBlock *al_walk_param_;
  WalkInfoBlock *walk_info_;
  RobotVisionBlock *robot_vision_;
  BodyModelBlock *body_model_;
  RobotInfoBlock *robot_info_;
  SpeechBlock *speech_;
  LocalizationBlock *localization_;

  // helpers
  bool getBool(bool *arr, int ind);
  void setBool(bool *arr, int ind, bool val);
  float getFloat(float *arr,int ind);
  void setFloat(float *arr, int ind, float val);
  double getDouble(double *arr,int ind);
  void setDouble(double *arr, int ind, double val);
  int getInt(int *arr,int ind);
  void setInt(int *arr, int ind, int val);
  std::string getString(std::string *arr, int ind);

  Pose2D getPose2D(Pose2D *arr,int ind);
  void setPose2D(Pose2D *arr,int ind, Pose2D val);
  Pose3D* getPose3DPtr(Pose3D *arr,int ind);

private:
  void setLuaPath();

  LuaInterp *lua_interp_;
  bool lua_ok_;
public:
  bool lua_restart_requested_;
};

#endif /* end of include guard: VISION_99KDYIX5 */
