#ifndef VISION_CORE_2H7QHCY7
#define VISION_CORE_2H7QHCY7

#include <lua/LuaInterp.h>
#include <python/PythonInterp.h>
#include <common/InterfaceInfo.h> // for core_type
#include <memory/Memory.h>
#include <memory/Logger.h>
#include <memory/TextLogger.h>
#include <math/Vector2.h>

class BodyModelBlock;
class CameraBlock;
class FrameInfoBlock;
class GameStateBlock;
class JointBlock;
class KickRequestBlock;
class OdometryBlock;
class SensorBlock;
class WalkRequestBlock;
class RobotStateBlock;
class JointCommandBlock;
class ProcessedSonarBlock;
class KickParamBlock;
class WalkParamBlock;
class ALWalkParamBlock;
class WalkInfoBlock;
class RobotInfoBlock;
class RobotVisionBlock;
class WorldObjectBlock;

class CommunicationModule;
class LuaModule;
class PythonModule;
class PerfectLocalizationModule;
class OffFieldLocalizationModule;
class OppModule;
class LocalizationModule;
class VisionModule;
class BehaviorModule;
class ButtonModule;
class LEDModule;

class ImageCapture;

class VisionCore {
public:
  VisionCore(CoreType type, bool use_shared_memory, int team_num, int player_num);
  ~VisionCore();

  void processVisionFrame();

  void preVision();
  void postVision();

  void logMemory();
  void enableLogging(int frames, double frequency); //Set these both to 0 to bypass frame/freq settings 
  void startDisableLogging();
  void enableTextLogging(const char *filename = NULL);
  void disableTextLogging();

  void updateMemory(Memory* memory, bool locOnly = false);
  void setMemoryVariables();

  Memory *memory_;
  bool delete_memory_on_destruct_;
  CoreType type_;
  unsigned last_frame_processed_;

  CommunicationModule *communications_;
  LuaModule *lua_;
  PythonModule *python_;
  VisionModule *vision_;
  LocalizationModule *localization_;
  OppModule *opponents_;
  BehaviorModule *behavior_;
  ButtonModule *buttons_;
  LEDModule *leds_;

#ifndef SWIG   // Lua can't handle the file IO
  Logger* log_;
#endif
  TextLogger textlog_;

  ImageCapture *image_capture_;

  static VisionCore *inst_;

  FrameInfoBlock *vision_frame_info_;

  BodyModelBlock *vision_body_model_;
  CameraBlock *camera_info_;
  JointBlock *vision_joint_angles_;
  KickRequestBlock *vision_kick_request_;
  OdometryBlock *vision_odometry_;
  SensorBlock *vision_sensors_;
  WalkRequestBlock *vision_walk_request_;
  GameStateBlock *game_state_;
  JointCommandBlock *vision_joint_commands_;
  ProcessedSonarBlock *vision_processed_sonar_;
  KickParamBlock *vision_kick_params_;
  WalkParamBlock *vision_walk_param_;
  ALWalkParamBlock *vision_al_walk_param_;
  WalkInfoBlock *vision_walk_info_;
  WorldObjectBlock *world_objects_;
  RobotVisionBlock *robot_vision_;

  CameraBlock *raw_camera_info_;
  FrameInfoBlock *raw_vision_frame_info_;
  RobotStateBlock *robot_state_;
  RobotInfoBlock *robot_info_;

  void publishData();
  void receiveData();
  void motionLock();
  void motionUnlock();

private:
  // synchronized data
  BodyModelBlock *sync_body_model_;
  JointBlock *sync_joint_angles_;
  KickRequestBlock *sync_kick_request_;
  OdometryBlock *sync_odometry_;
  SensorBlock *sync_sensors_;
  WalkRequestBlock *sync_walk_request_;
  JointCommandBlock *sync_joint_commands_;
  ProcessedSonarBlock *sync_processed_sonar_;
  KickParamBlock *sync_kick_params_;
  WalkParamBlock *sync_walk_param_;
  ALWalkParamBlock *sync_al_walk_param_;
  WalkInfoBlock *sync_walk_info_;

  double fps_time_, last_frame_time_, seconds_since_log_;
  unsigned int fps_frames_processed_;
  unsigned int frames_to_log_;
  double log_frequency_;
  bool log_by_frame_;
  bool disable_log_, is_logging_;
  void enableMemoryLogging(const char *filename = NULL);
  void disableMemoryLogging();
  void enableLogging();
  void disableLogging();
  void optionallyWriteLog();

  static const bool useOffFieldLocalization;
private:
  void init(int team_num, int player_num);
  void initMemory();
  void initModules();
  bool isToolCore();
 };

#endif /* end of include guard: CORE_2H7QHCY7 */
