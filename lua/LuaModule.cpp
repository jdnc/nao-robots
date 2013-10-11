#include <cstdlib>
#include "LuaModule.h"
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

LuaModule::LuaModule():
  lua_interp_(NULL),
  lua_ok_(true),
  lua_restart_requested_(false)
{
  setLuaPath();
}

LuaModule::~LuaModule(){
  if (lua_interp_!=NULL) {
    delete lua_interp_;
  }
}

void LuaModule::specifyMemoryDependency() {
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
}

void LuaModule::specifyMemoryBlocks() {
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
}

void LuaModule::updateModuleMemory(Memory *memory) {
  memory_ = memory;
  specifyMemoryBlocks();

  // if lua is broken, don't run
  if (!lua_ok_)
    return;
  try {
    lua_interp_->call("initMemory()");
  }
  catch (int e) {
    std::cerr << "Lua crash with exception on call of initMemory  - Upload new lua!" << e << std::endl << std::flush;
    lua_ok_ = false;
  }
}

void LuaModule::initSpecificModule() {
  startLua(); 
}

void LuaModule::processFrame() {

  // restart lua if requested
  if (lua_restart_requested_)
    startLua();
  
  // if lua is broken, don't run
  if (!lua_ok_) {
    usleep(1/30.0 * 1000000); // don't run too fast
    return;
  }
  // try to run process frame
  try {
    lua_interp_->call("xpcall(processFrame,errHandler)");
  }
  catch (int e) {
    std::cerr << "Lua crash with exception  - Upload new lua!" << e << std::endl << std::flush;
    lua_ok_ = false;
  }
}


void LuaModule::initLua(){
  
  // if lua is broken, don't run
  if (!lua_ok_)
    return;
  
  // try to run behavior process frame
  try {
    lua_interp_->call("init()");
  }
  catch (int e) {
    std::cerr << "Lua crash with exception  - Upload new lua!" << e << std::endl << std::flush;
    lua_ok_ = false;
  }
}


void LuaModule::call(const std::string &cmd) {
  // if lua is broken, don't run
  if (!lua_ok_)
    return;

  try {
    lua_interp_->call(cmd.c_str());
  }
  catch (int e) {
    std::cerr << "Lua crash with exception  - Upload new lua!" << e << std::endl << std::flush;
    lua_ok_ = false;
  }
}

void LuaModule::doStrategyCalculations(){
  // if lua is broken, don't run
  if (!lua_ok_)
    return;

  // try to run behavior process frame
  try {
    lua_interp_->call("strategy.initKickRegion()");
    lua_interp_->call("strategy.setKickAngles()");
  }
  catch (int e) {
    std::cerr << "Lua crash with exception  - Upload new lua!" << e << std::endl << std::flush;
    lua_ok_ = false;
  }
}

void LuaModule::behaviorProcessFrame() {
 
  // if lua is broken, don't run
  if (!lua_ok_)
    return;

  // try to run behavior process frame
  try {
    lua_interp_->call("behavior.processFrame()");
  }
  catch (int e) {
    std::cerr << "Lua crash with exception  - Upload new lua!" << e << std::endl << std::flush;
    lua_ok_ = false;
  }
}

void LuaModule::setLuaPath() {
  // Set the lua path
  char * pPath;
  pPath = getenv ("NAO_HOME");
  char* luapath=new char[256];
  sprintf(luapath,"LUA_PATH=%s/core/lua/?.lua;/home/nao/lua/?.lua",pPath);
  std::cout << "LuaModule:: Setting " << luapath << std::endl;
  putenv(luapath);
}

void LuaModule::startLua() {

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

  if (lua_interp_ != NULL) {
    delete lua_interp_;
    std::cerr << "*** Restarting Lua *** " << std::endl << std::flush;
  }
  try{
    lua_interp_ = new LuaInterp();
    //localization->setLua(lua_);
    lua_ok_ = true;
    if (!lua_interp_->require("init")) {
      std::cerr << "Can't start lua - Upload new lua!" << std::endl << std::flush;
      lua_ok_ = false;
      lua_interp_ = NULL;
    }
  }
  catch (int e){
    std::cerr << "Lua crashed with exception - Upload new lua!" << std::endl << std::flush;
    lua_ok_ = false;
    lua_interp_ = NULL;
  }
  lua_restart_requested_ = false;
}

bool LuaModule::getBool(bool *arr, int ind) {
  return arr[ind];
}

void LuaModule::setBool(bool *arr, int ind, bool val) {
  arr[ind] = val;
}

float LuaModule::getFloat(float *arr, int ind) {
  return arr[ind];
}

void LuaModule::setFloat(float *arr, int ind, float val) {
  arr[ind] = val;
}

double LuaModule::getDouble(double *arr, int ind) {
  return arr[ind];
}

void LuaModule::setDouble(double *arr, int ind, double val) {
  arr[ind] = val;
}

int LuaModule::getInt(int *arr, int ind) {
  return arr[ind];
}

void LuaModule::setInt(int *arr, int ind, int val) {
  arr[ind] = val;
}
  
std::string LuaModule::getString(std::string *arr, int ind) {
  return arr[ind];
}

void LuaModule::setPose2D(Pose2D *arr,int ind, Pose2D val) {
  arr[ind] = val;
}
  
Pose2D LuaModule::getPose2D(Pose2D *arr,int ind) {
  return arr[ind];
}

Pose3D* LuaModule::getPose3DPtr(Pose3D *arr,int ind) {
  return &(arr[ind]);
}
