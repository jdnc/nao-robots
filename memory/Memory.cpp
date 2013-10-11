#include "Memory.h"

#include "BehaviorBlock.h"
#include "BodyModelBlock.h"
#include "CameraBlock.h"
#include "FrameInfoBlock.h"
#include "GraphableBlock.h"
#include "ImageBlock.h"
#include "JointBlock.h"
#include "JointCommandBlock.h"
#include "KickEngineBlock.h"
#include "KickModuleBlock.h"
#include "KickRequestBlock.h"
#include "SensorBlock.h"
#include "SimEffectorBlock.h"
#include "KickParamBlock.h"
#include "WalkParamBlock.h"
#include "WalkRequestBlock.h"
#include "WalkEngineBlock.h"
#include "SimImageBlock.h"
#include "WorldObjectBlock.h"
#include "LocalizationBlock.h"
#include "DelayedLocalizationBlock.h"
#include "OpponentBlock.h"
#include "TeamPacketsBlock.h"
#include "GameStateBlock.h"
#include "RobotStateBlock.h"
#include "OdometryBlock.h"
#include "RobotVisionBlock.h"
#include "SimTruthDataBlock.h"
#include "BehaviorParamBlock.h"
#include "LEDBlock.h"
#include "ProcessedSonarBlock.h"
#include "SensorCalibrationBlock.h"
#include "ALWalkParamBlock.h"
#include "WalkInfoBlock.h"
#include "RobotInfoBlock.h"
#include "SpeechBlock.h"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <sstream>
std::string intToString(int i) {
  std::stringstream out;
  out << i;
  return out.str();
}

Memory::Memory(bool use_shared_memory, MemoryOwner::Owner owner, int team_num, int player_num, bool server):
  use_shared_memory_(use_shared_memory),
  owner_(owner),
  vision_lock_(NULL),
  motion_vision_lock_(NULL),
  suffix_("TEAM" + intToString(team_num) + "PLAYER" + intToString(player_num))
{
  if (use_shared_memory_) {
    shared_memory_ = new SharedMemory(suffix_,server);
    private_memory_ = NULL;
  } else {
    shared_memory_ = NULL;
    private_memory_ = new PrivateMemory();
  }
}

Memory::Memory(const Memory &old):
  vision_lock_(NULL),
  motion_vision_lock_(NULL),
  suffix_(old.suffix_)
{
  use_shared_memory_ = old.use_shared_memory_;
  if (use_shared_memory_) {
    std::cout << "CAN'T HANDLE COPYING SHARED MEMORY AT THIS TIME" << std::endl;
    exit(1);
  }
  private_memory_ = new PrivateMemory(*old.private_memory_);
  shared_memory_ = NULL;
  owner_ = old.owner_;
}

Memory::~Memory() {
  if (shared_memory_ != NULL) {
    delete shared_memory_;
    shared_memory_ = NULL;
  }
  if (private_memory_ != NULL) {
    delete private_memory_;
    private_memory_ = NULL;
  }
}

Memory& Memory::operator=(const Memory &old) {
  if (shared_memory_ != NULL) {
    delete shared_memory_;
    shared_memory_ = NULL;
  }
  if (private_memory_ != NULL) {
    delete private_memory_;
    private_memory_ = NULL;
  }
  
  use_shared_memory_ = old.use_shared_memory_;
  if (use_shared_memory_) {
    std::cout << "CAN'T HANDLE COPYING SHARED MEMORY AT THIS TIME" << std::endl;
    exit(1);
  }
  private_memory_ = new PrivateMemory(*old.private_memory_);
  shared_memory_ = NULL;
  owner_ = old.owner_;
  return *this;
}


MemoryBlock* Memory::getBlockPtr(const std::string &name, MemoryOwner::Owner expect_owner) {
  MemoryBlock *ptr;
  if (use_shared_memory_)
    ptr = shared_memory_->getBlockPtr(name);
  else
    ptr = private_memory_->getBlockPtr(name);

  if (ptr != NULL) {
    if (expect_owner == MemoryOwner::UNKNOWN)
      expect_owner = owner_;
    ptr->checkOwner(name,expect_owner);
  }
  return ptr;
}

const MemoryBlock* Memory::getBlockPtr(const std::string &name, MemoryOwner::Owner expect_owner) const {
  MemoryBlock *ptr;
  if (use_shared_memory_)
    ptr = shared_memory_->getBlockPtr(name);
  else
    ptr = private_memory_->getBlockPtr(name);

  if (ptr != NULL) {
    if (expect_owner == MemoryOwner::UNKNOWN)
      expect_owner = owner_;
    ptr->checkOwner(name,expect_owner);
  }
  return ptr;
}

void Memory::getBlockNames(std::vector<std::string> &module_names, bool only_log) const {
  if (use_shared_memory_)
    shared_memory_->getBlockNames(module_names,only_log,owner_);
  else
    private_memory_->getBlockNames(module_names,only_log,owner_);
}

void Memory::setBlockLogging(const std::string &name, bool log_block) {
  MemoryBlock *block = getBlockPtr(name,MemoryOwner::UNKNOWN);
  if (block != NULL)
    block->log_block = log_block;
}

MemoryBlock* Memory::getBlockPtrByName(const std::string &name){
  return getBlockPtr(name,MemoryOwner::UNKNOWN);
}

const MemoryBlock* Memory::getBlockPtrByName(const std::string &name) const {
  return getBlockPtr(name,MemoryOwner::UNKNOWN);
}

bool Memory::addBlockByName(const std::string &name, MemoryOwner::Owner owner) {
  if (owner == MemoryOwner::UNKNOWN) {
    owner = owner_;
  }
  temp_add_owner_ = owner;

  if (name == "frame_info")
    return addBlock(name,new FrameInfoBlock(0,0,MEMORY_ROBOT));
  else if (name == "vision_frame_info")
    return addBlock(name,new FrameInfoBlock(0,0,MEMORY_ROBOT));
  else if (name == "raw_vision_frame_info")
    return addBlock(name,new FrameInfoBlock(0,0,MEMORY_ROBOT));
  else if (name == "body_model")
    return addBlock(name,new BodyModelBlock());
  else if (name == "graphable")
    return addBlock(name,new GraphableBlock());
  // joints
  else if (name == "raw_joint_angles")
    return addBlock(name,new JointBlock());
  else if (name == "processed_joint_angles")
    return addBlock(name,new JointBlock());
  // commands
  else if (name == "raw_joint_commands")
    return addBlock(name,new JointCommandBlock());
  else if (name == "processed_joint_commands")
    return addBlock(name,new JointCommandBlock());
  else if (name == "led_commands")
    return addBlock(name,new LEDBlock());
  // sensors
  else if (name == "raw_sensors")
    return addBlock(name,new SensorBlock());
  else if (name == "processed_sensors")
    return addBlock(name,new SensorBlock());
  else if (name == "sensor_calibration")
    return addBlock(name,new SensorCalibrationBlock());
  // Sim
  else if (name == "sim_effectors")
    return addBlock(name, new SimEffectorBlock());
  else if (name == "sim_image")
    return addBlock(name,new SimImageBlock());
  else if (name == "sim_world_objects")
    return addBlock(name,new WorldObjectBlock());
  else if (name == "sim_truth_data")
    return addBlock(name,new SimTruthDataBlock());
  // motion
  else if (name == "walk_engine")
    return addBlock(name,new WalkEngineBlock());
  else if (name == "kick_params")
    return addBlock(name,new KickParamBlock());
  else if (name == "walk_param")
    return addBlock(name,new WalkParamBlock());
  else if (name == "al_walk_param")
    return addBlock(name,new ALWalkParamBlock());
  else if (name == "walk_request")
    return addBlock(name,new WalkRequestBlock());
  else if (name == "kick_request")
    return addBlock(name,new KickRequestBlock());
  else if (name == "kick_engine")
    return addBlock(name,new KickEngineBlock());
  else if (name == "kick_module")
    return addBlock(name,new KickModuleBlock());
  else if (name == "odometry")
    return addBlock(name,new OdometryBlock());
  else if (name == "processed_sonar")
    return addBlock(name,new ProcessedSonarBlock());
  else if (name == "walk_info")
    return addBlock(name,new WalkInfoBlock());
  // odometry for localization (includes kick, walk, fall info)
  else if (name == "vision_odometry")
    return addBlock(name,new OdometryBlock());
  // camera 
  else if (name == "raw_camera_info")
    return addBlock(name,new CameraBlock());
  else if (name == "camera_info")
    return addBlock(name,new CameraBlock());
  // vision / localisation
  else if (name == "vision_body_model")
    return addBlock(name,new BodyModelBlock());
  else if (name == "vision_sensors")
    return addBlock(name,new SensorBlock());
  else if (name == "vision_joint_angles")
    return addBlock(name,new JointBlock());
  else if (name == "vision_kick_request")
    return addBlock(name,new KickRequestBlock());
  else if (name == "vision_walk_request")
    return addBlock(name,new WalkRequestBlock());
  else if (name == "vision_joint_commands")
    return addBlock(name,new JointCommandBlock());
  else if (name == "world_objects")
    return addBlock(name,new WorldObjectBlock());
  else if (name == "robot_vision")
    return addBlock(name, new RobotVisionBlock());
  else if (name == "raw_image")
    return addBlock(name, new ImageBlock());
  else if (name == "vision_processed_sonar")
    return addBlock(name, new ProcessedSonarBlock());
  else if (name == "vision_kick_params")
    return addBlock(name,new KickParamBlock());
  else if (name == "vision_walk_param")
    return addBlock(name,new WalkParamBlock());
  else if (name == "vision_al_walk_param")
    return addBlock(name,new ALWalkParamBlock());
  else if (name == "vision_walk_info")
    return addBlock(name,new WalkInfoBlock());
  // behavior
  else if (name == "behavior")
    return addBlock(name,new BehaviorBlock());
  else if (name == "behavior_params")
    return addBlock(name,new BehaviorParamBlock());
  // localization / world model
  else if (name == "localization")
    return addBlock(name,new LocalizationBlock());
  else if (name == "delayed_localization")
    return addBlock(name,new DelayedLocalizationBlock());
  else if (name == "opponents")
    return addBlock(name,new OpponentBlock());
  // team packets
  else if (name == "team_packets")
    return addBlock(name,new TeamPacketsBlock());
  // state
  else if (name == "robot_state")
    return addBlock(name, new RobotStateBlock());
  else if (name == "game_state")
    return addBlock(name, new GameStateBlock());
  // shared data
  else if (name == "robot_info")
    return addBlock(name, new RobotInfoBlock());
  else if (name == "speech")
    return addBlock(name, new SpeechBlock());
  // synchronized data
  else if (name == "sync_body_model")
    return addBlock(name, new BodyModelBlock());
  else if (name == "sync_joint_angles")
    return addBlock(name, new JointBlock());
  else if (name == "sync_kick_request")
    return addBlock(name, new KickRequestBlock());
  else if (name == "sync_odometry")
    return addBlock(name, new OdometryBlock());
  else if (name == "sync_sensors")
    return addBlock(name, new SensorBlock());
  else if (name == "sync_walk_request")
    return addBlock(name, new WalkRequestBlock());
  else if (name == "sync_joint_commands")
    return addBlock(name, new JointCommandBlock());
  else if (name == "sync_processed_sonar")
    return addBlock(name, new ProcessedSonarBlock());
  else if (name == "sync_kick_params")
    return addBlock(name,new KickParamBlock());
  else if (name == "sync_walk_param")
    return addBlock(name, new WalkParamBlock());
  else if (name == "sync_al_walk_param")
    return addBlock(name, new ALWalkParamBlock());
  else if (name == "sync_walk_info")
    return addBlock(name, new WalkInfoBlock());
  else {
    std::cerr << "Memory::addBlockByName: Error: Unknown memory block for name: " << name << std::endl << std::flush;
    return false;
  }
}
