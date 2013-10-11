#include <cstdlib>
#include "PythonModule.h"
//#include <Core.h>

#include <memory/CameraBlock.h>
#include <memory/FrameInfoBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/JointBlock.h>
#include <memory/KickRequestBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/SensorBlock.h>
#include <memory/WalkRequestBlock.h>
#include <memory/WorldObjectBlock.h>
#include <memory/OpponentBlock.h>
#include <memory/BehaviorParamBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/WalkInfoBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/RobotVisionBlock.h>
#include <memory/BodyModelBlock.h>
#include <memory/SpeechBlock.h>
#include <memory/LocalizationBlock.h>

#include <memory/WalkParamBlock.h>
#include <memory/ALWalkParamBlock.h>

#include <common/Config.h>

PythonModule::PythonModule():
  python_interp_(NULL),
  python_ok_(true),
  python_restart_requested_(false) {
}

PythonModule::~PythonModule(){
  if (python_interp_!=NULL) {
    delete python_interp_;
  }
}

void PythonModule::specifyMemoryDependency() {
  requiresMemoryBlock("behavior");
  requiresMemoryBlock("camera_info");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("vision_joint_angles");
  requiresMemoryBlock("vision_kick_request");
  requiresMemoryBlock("vision_odometry");
  requiresMemoryBlock("vision_sensors");
  requiresMemoryBlock("vision_walk_request");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("vision_kick_params");
  requiresMemoryBlock("vision_walk_param");
  requiresMemoryBlock("vision_al_walk_param");
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("team_packets");
  requiresMemoryBlock("opponents");
  requiresMemoryBlock("behavior_params");
  requiresMemoryBlock("vision_joint_commands");
  requiresMemoryBlock("vision_processed_sonar");
  requiresMemoryBlock("vision_walk_info");
  requiresMemoryBlock("robot_vision");
  requiresMemoryBlock("vision_body_model");
  requiresMemoryBlock("robot_info");
  requiresMemoryBlock("speech");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("raw_image");
}

void PythonModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(behavior_,"behavior");
  getOrAddMemoryBlock(camera_block_,"camera_info");
  getOrAddMemoryBlock(game_state_,"game_state");
  getOrAddMemoryBlock(robot_state_,"robot_state");
  getOrAddMemoryBlock(joint_angles_,"vision_joint_angles");
  getOrAddMemoryBlock(kick_request_,"vision_kick_request");
  getOrAddMemoryBlock(odometry_,"vision_odometry");
  getOrAddMemoryBlock(sensors_,"vision_sensors");
  getOrAddMemoryBlock(vision_frame_info_,"vision_frame_info");
  getOrAddMemoryBlock(walk_request_,"vision_walk_request");
  getOrAddMemoryBlock(world_objects_,"world_objects");
  getOrAddMemoryBlock(team_packets_,"team_packets");
  getOrAddMemoryBlock(opponents_,"opponents");
  getOrAddMemoryBlock(behavior_params_,"behavior_params");
  getOrAddMemoryBlock(joint_commands_,"vision_joint_commands");
  getOrAddMemoryBlock(vision_processed_sonar_,"vision_processed_sonar");

  getOrAddMemoryBlock(kick_params_,"vision_kick_params");
  getOrAddMemoryBlock(walk_param_,"vision_walk_param");
  getOrAddMemoryBlock(al_walk_param_,"vision_al_walk_param");
  getOrAddMemoryBlock(walk_info_,"vision_walk_info");
  getOrAddMemoryBlock(robot_vision_,"robot_vision");
  getOrAddMemoryBlock(body_model_,"vision_body_model");

  getOrAddMemoryBlock(robot_info_,"robot_info");
  getOrAddMemoryBlock(speech_,"speech");
  getOrAddMemoryBlock(localization_,"localization");
  getOrAddMemoryBlock(image_,"raw_image");
}

void PythonModule::initSpecificModule() {
  startPython(); 
}

void PythonModule::updateModuleMemory(Memory *memory) {
  memory_ = memory;
  specifyMemoryBlocks();

  // if python is broken, don't run
  if (!checkPython()) return;
  python_interp_->call("initMemory()");
}

bool PythonModule::checkPython() {
  if(!python_ok_) {
    if(python_interp_) {
      delete python_interp_;
      python_interp_ = NULL;
    }
    return false;
  }
  return true;
}

void PythonModule::processFrame() {
  // restart python if requested
  if (python_restart_requested_) {
    python_interp_->finalize();
    python_interp_->init();
    startPython();
  }
  
  call("processFrame()");
}

void PythonModule::initPython(){
  if(!checkPython()) return;
  python_interp_->call("init()");
}

void PythonModule::call(const std::string &cmd) {
  if(!checkPython()) return;
  python_interp_->call(cmd.c_str());
}

void PythonModule::startPython() {
  // Read configuration file and place appropriate values in memory
  if (memory_->core_type_ == CORE_ROBOT) {
    Config config;
    std::cout << "Reading config file: " << memory_->data_path_ + "config.txt" << std::endl;
    // Todd: the team num here is the gc team number, not which color we are
    // and the role # is our array index WO_SELF, not our role, which can change
    if (config.readFromFile(memory_->data_path_ + "config.txt")) {
      robot_state_->robot_id_ = config.robot_id_;
      game_state_->gameContTeamNum = config.team_;
      robot_state_->WO_SELF = config.role_;
      robot_state_->role_ = config.role_;
      std::cout << "From config file, read robot id: " << robot_state_->robot_id_ << ", GC team: " << game_state_->gameContTeamNum << ", wo_self: " << robot_state_->WO_SELF << std::endl;
    }
  }
  python_ok_ = true;
  if(python_interp_ == NULL) python_interp_ = new PythonInterp();
  python_restart_requested_ = false;
}

bool PythonModule::getBool(bool *arr, int ind) {
  return arr[ind];
}

void PythonModule::setBool(bool *arr, int ind, bool val) {
  arr[ind] = val;
}

float PythonModule::getFloat(float *arr, int ind) {
  return arr[ind];
}

void PythonModule::setFloat(float *arr, int ind, float val) {
  arr[ind] = val;
}

double PythonModule::getDouble(double *arr, int ind) {
  return arr[ind];
}

void PythonModule::setDouble(double *arr, int ind, double val) {
  arr[ind] = val;
}

int PythonModule::getInt(int *arr, int ind) {
  return arr[ind];
}

void PythonModule::setInt(int *arr, int ind, int val) {
  arr[ind] = val;
}

unsigned char PythonModule::getUchar(unsigned char *arr, int ind) {
  return arr[ind];
}

void PythonModule::setUchar(unsigned char *arr, int ind, unsigned char val) {
  arr[ind] = val;
}

Pose3D PythonModule::getPose3D(Pose3D* arr, int ind) {
  return arr[ind];
}

void PythonModule::setPose3D(Pose3D* arr, int ind, Pose3D val) {
  arr[ind] = val;
}
