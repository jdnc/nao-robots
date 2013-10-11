#include "VisionCore.h"

#include <common/States.h>

#include <memory/BodyModelBlock.h>
#include <memory/CameraBlock.h>
#include <memory/FrameInfoBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/JointBlock.h>
#include <memory/KickRequestBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/SensorBlock.h>
#include <memory/WalkRequestBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/ProcessedSonarBlock.h>
#include <memory/KickParamBlock.h>
#include <memory/WalkParamBlock.h>
#include <memory/ALWalkParamBlock.h>
#include <memory/WalkInfoBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/RobotVisionBlock.h>
#include <memory/WorldObjectBlock.h>

#include <communications/CommunicationModule.h>
#include <kinematics/KinematicsModule.h>
#include <lua/LuaModule.h>
#include <python/PythonModule.h>
#include <sensor/SensorModule.h>
#include <vision/VisionModule.h>
#include <lights/LEDModule.h>
#include <localization/LocalizationModule.h>
#include <opponents/OppModule.h>
#include <behavior/BehaviorModule.h>
#include <sensor/ButtonModule.h>

#include <imageCapture/ImageCapture.h>

#include <python/PythonInterface.h>

#include <iostream>

#define USE_PYTHON 1
#define USE_LUA !USE_PYTHON

VisionCore *VisionCore::inst_ = NULL;
const bool VisionCore::useOffFieldLocalization = false;

VisionCore::VisionCore (CoreType type, bool use_shared_memory, int team_num, int player_num):
  memory_(new Memory(use_shared_memory,MemoryOwner::VISION,team_num,player_num,false)),
  delete_memory_on_destruct_(true),
  type_(type),
  last_frame_processed_(0),
  // modules
  communications_(NULL),
  lua_(NULL),
  python_(NULL),
  vision_(NULL),
  localization_(NULL),
  opponents_(NULL),
  behavior_(NULL),
  buttons_(NULL),
  leds_(NULL),
  log_(new FileLogger()),
  textlog_(NULL),
  image_capture_(NULL),
  // sync memory
  sync_joint_angles_(NULL),
  sync_kick_request_(NULL),
  sync_odometry_(NULL),
  sync_sensors_(NULL),
  sync_walk_request_(NULL),
  sync_joint_commands_(NULL),
  sync_processed_sonar_(NULL),
  sync_kick_params_(NULL),
  sync_walk_param_(NULL),
  sync_al_walk_param_(NULL),
  sync_walk_info_(NULL),
  fps_frames_processed_(0),
  frames_to_log_(0),
  disable_log_(false),
  is_logging_(false)
{
  init(team_num, player_num);
}

VisionCore::~VisionCore() {
  // stop walking
  if ((sync_walk_request_ != NULL)  && (sync_walk_request_->motion_ == WalkRequestBlock::WALK)) {
    sync_walk_request_->stand();
  }

  // clean up modules
  if (lua_ != NULL)
    delete lua_;
  if (python_ != NULL)
    delete python_;
  if (vision_ != NULL)
    delete vision_;
  if (leds_ != NULL)
    delete leds_;
  if (localization_ != NULL)
    delete localization_;
  if (opponents_ != NULL)
    delete opponents_;
  if (communications_ != NULL)
    delete communications_;
  if (behavior_ != NULL)
    delete behavior_;
  if (buttons_ != NULL)
    delete buttons_;

  // clean up the entire memory block
  if (delete_memory_on_destruct_ && memory_ != NULL)
    delete memory_;

  if (image_capture_ != NULL)
    delete image_capture_;
}

void VisionCore::processVisionFrame() {
  if(USE_LUA)
    lua_->processFrame(); // main control is done in lua
  if(USE_PYTHON)
    python_->processFrame();

  if (communications_ != NULL) {
    communications_->processFrame();
  }

  if (buttons_ !=NULL) {
    buttons_->processButtons(); // check for any button related changes
  }

  // frame rate
  double fseconds = vision_frame_info_->seconds_since_start;
  double time_passed = fseconds - fps_time_;
  seconds_since_log_ += fseconds - last_frame_time_;
  last_frame_time_ = fseconds;
  fps_frames_processed_++;
  if ((time_passed >= 5.0) && !isToolCore()) {
    std::cout << "VISION FRAME RATE: " << fps_frames_processed_ / time_passed << std::endl;
    fps_frames_processed_ = 0;
    fps_time_ = fseconds;
  }

  unsigned int &frame_id = vision_frame_info_->frame_id;
  last_frame_processed_ = frame_id;

  optionallyWriteLog();
  if (communications_ != NULL)
    communications_->optionallyStream();

  if(is_logging_ && disable_log_) {
    disable_log_ = false;
    disableLogging();
    disableTextLogging();
    if(communications_) {
        communications_->sendToolUDP(std::string("LE"));
    }
  }
}

void VisionCore::optionallyWriteLog() {
  if(!is_logging_)
    return;
  if ((log_frequency_ > 1e-5) && (seconds_since_log_ < log_frequency_)) // not long enough since log
    return;
  bool headReady = (!robot_vision_->reported_head_moving) && (vision_frame_info_->seconds_since_start - robot_vision_->reported_head_stop_time > HEAD_STOP_THRESHOLD);
  if ((log_frequency_ > 1e-5) && !headReady && (seconds_since_log_ - log_frequency_ < 0.34)) // logging by frequency, but head is moving, so let's wait a bit (at most 0.34s)
    return;
  logMemory();
  seconds_since_log_ = 0;
  if(log_by_frame_) frames_to_log_--;
  if(frames_to_log_ == 0 && log_by_frame_) disable_log_ = true;
}

void VisionCore::init(int team_num, int player_num) {
  if (type_ == CORE_INIT) {
    FrameInfoBlock *frame_info;
    memory_->getBlockByName(frame_info,"raw_vision_frame_info");
    if (frame_info == NULL) {
      std::cout << "VISION CORE: ROBOT" << std::endl;
      type_ = CORE_ROBOT;
    } else {
      if (frame_info->source == MEMORY_SIM) {
        std::cout << "VISION CORE: SIM" << std::endl;
        type_ = CORE_SIM;
      } else if (frame_info->source == MEMORY_ROBOT) {
        std::cout << "VISION CORE: ROBOT" << std::endl;
        type_ = CORE_ROBOT;
      } else {
        std::cerr << "Unknown memory type when init vision core" << std::endl;
        exit(1);
      }
    }
  }

  inst_ = PythonInterface::CORE_INSTANCE = this;

  if (type_ == CORE_ROBOT) {
    image_capture_ = new ImageCapture(memory_);
    image_capture_->initVision();
    std::cout << "DOING INIT VISION" << std::endl;
  }

  setMemoryVariables();

  initMemory();

  robot_state_->team_ = team_num;
  robot_state_->WO_SELF = player_num;
  robot_state_->clock_offset_ = 0;
  std::ifstream in("/home/nao/time.txt");
  if (in.good()) {
    in >> robot_state_->clock_offset_;
    in.close();
  } else {
    robot_state_->clock_offset_ = 0;
  }
  std::cout << "CLOCK OFFSET: " << robot_state_->clock_offset_ << std::endl;

  initModules();

  fps_time_ = vision_frame_info_->seconds_since_start;

}

void VisionCore::initModules() {

  if (!isToolCore()) {
    communications_ = new CommunicationModule(this);
    communications_->init(memory_,&textlog_);
  }

  if ((type_ != CORE_TOOLSIM) && (type_ != CORE_TOOL_NO_VISION)){
    vision_ = new VisionModule();
    vision_->init(memory_,&textlog_);
  }

  leds_ = new LEDModule();
  leds_->init(memory_,&textlog_);

  localization_ = new LocalizationModule();
  localization_->init(memory_,&textlog_);

  opponents_ = new OppModule();
  opponents_->init(memory_,&textlog_);

  behavior_ = new BehaviorModule();
  behavior_->init(memory_,&textlog_);

  buttons_ = new ButtonModule();
  buttons_->init(memory_,&textlog_);

  /* MUST DO LUA LAST - OTHERWISE ALL THE MEMORY BLOCK POINTERS ARE NOT INITIALISED
     MQ 3/16/2011 */
  if(USE_LUA) {
    lua_ = new LuaModule();
    lua_->init(memory_,&textlog_);
  }
  if(USE_PYTHON) {
    python_ = new PythonModule();
    python_->init(memory_,&textlog_);
  }

  if (communications_ != NULL) {
    if(USE_LUA) communications_->lua_restart_requested_ = &(lua_->lua_restart_requested_);
    if(USE_PYTHON) communications_->lua_restart_requested_ = &(python_->python_restart_requested_);
  }
}

void VisionCore::initMemory() {

  // create all the modules that are used
  //MemorySource mem_source = MEMORY_SIM;
  //if (type_ == CORE_ROBOT)
    //mem_source = MEMORY_ROBOT;

  // get raw info for vision interface
  memory_->getOrAddBlockByName(raw_vision_frame_info_,"raw_vision_frame_info");
  memory_->getOrAddBlockByName(raw_camera_info_,"raw_camera_info");
  /*
    ImageBlock *imageBlockPtr;
    memory_->getOrAddBlockByName(imageBlockPtr, "raw_image");
    imageBlockPtr->log_block = true;
  */
  //Add required memory blocks
  memory_->getOrAddBlockByName(camera_info_,"camera_info");
  memory_->getOrAddBlockByName(vision_frame_info_,"vision_frame_info");
  memory_->getOrAddBlockByName(game_state_,"game_state");
  memory_->getOrAddBlockByName(vision_body_model_,"vision_body_model");
  memory_->getOrAddBlockByName(vision_joint_angles_,"vision_joint_angles");
  memory_->getOrAddBlockByName(vision_kick_request_,"vision_kick_request");
  memory_->getOrAddBlockByName(vision_odometry_,"vision_odometry");
  memory_->getOrAddBlockByName(vision_sensors_,"vision_sensors");
  memory_->getOrAddBlockByName(vision_walk_request_,"vision_walk_request");
  memory_->getOrAddBlockByName(vision_joint_commands_,"vision_joint_commands");
  memory_->getOrAddBlockByName(vision_processed_sonar_,"vision_processed_sonar");
  memory_->getOrAddBlockByName(vision_kick_params_,"vision_kick_params");
  memory_->getOrAddBlockByName(vision_walk_param_,"vision_walk_param");
  memory_->getOrAddBlockByName(vision_al_walk_param_,"vision_al_walk_param");
  memory_->getOrAddBlockByName(vision_walk_info_,"vision_walk_info");
  memory_->getOrAddBlockByName(robot_info_,"robot_info",MemoryOwner::SHARED);
  memory_->getOrAddBlockByName(robot_state_,"robot_state");
  memory_->getOrAddBlockByName(robot_vision_,"robot_vision");
  memory_->getOrAddBlockByName(world_objects_,"world_objects");

  // synchronized blocks
  if (!isToolCore()) {
    memory_->getOrAddBlockByName(sync_body_model_,"sync_body_model",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_joint_angles_,"sync_joint_angles",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_kick_request_,"sync_kick_request",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_odometry_,"sync_odometry",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_sensors_,"sync_sensors",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_walk_request_,"sync_walk_request",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_joint_commands_,"sync_joint_commands",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_processed_sonar_,"sync_processed_sonar",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_kick_params_,"sync_kick_params",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_walk_param_,"sync_walk_param",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_al_walk_param_,"sync_al_walk_param",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_walk_info_,"sync_walk_info",MemoryOwner::SYNC);
  }

  // print out all the memory blocks we're using
  std::vector<std::string> memory_block_names;
  memory_->getBlockNames(memory_block_names,false);
  std::cout << "INITIAL MEMORY BLOCKS:" << std::endl;
  for (unsigned int i = 0; i < memory_block_names.size(); i++)
    std::cout << memory_block_names[i] << std::endl;
  std::cout << "--------------" << std::endl;

  textlog_.setFrameInfo(vision_frame_info_);
  log_->setType(type_);
  textlog_.setType(type_);
}

void VisionCore::logMemory() {
  log_->writeMemory(*memory_);
}

void VisionCore::enableLogging() {
  if(is_logging_) {
    std::cout << "Logging is already in progress\n";
    return;
  }
  is_logging_ = true;
  enableMemoryLogging();
  enableTextLogging();
}

void VisionCore::enableLogging(int frames, double frequency) {
  if(is_logging_) {
    std::cout << "Logging is already in progress\n";
    return;
  }
  log_by_frame_ = (frames > 0);
  frames_to_log_ = frames;
  log_frequency_ = frequency;
  seconds_since_log_ = 0;
  enableLogging();
}

void VisionCore::enableMemoryLogging(const char *filename) {
  is_logging_ = true;
  if (filename) {
    log_->open(filename);
  } else {
    log_->open("vision", true);
  }
}

void VisionCore::enableTextLogging(const char *filename) {
  if (filename) {
    textlog_.open(filename);
  } else {
    textlog_.open("vision", true);
  }
}

void VisionCore::startDisableLogging(){
  if (is_logging_) {
    disable_log_ = true;
  } else {
    // logging is already off, but tool probably doesn't know it, send it again
    if(communications_) {
      communications_->sendToolUDP(std::string("LE"));
    }
  }
}

void VisionCore::disableLogging() {
  if(!is_logging_) return;
  disableMemoryLogging();
  disableTextLogging();
  disable_log_ = false;
  is_logging_ = false;
  frames_to_log_ = 0;
}

void VisionCore::disableMemoryLogging() {
  log_->close();
}

void VisionCore::disableTextLogging() {
  textlog_.close();
}

void VisionCore::updateMemory(Memory* memory, bool locOnly) {
  if (memory_ && delete_memory_on_destruct_) {
    delete memory_;
  }
  delete_memory_on_destruct_ = false;
  memory_ = memory;
  setMemoryVariables();
  if (vision_ && !locOnly)
    vision_->updateModuleMemory(memory_);
  if (localization_)
    localization_->updateModuleMemory(memory_);
  if (opponents_)
    opponents_->updateModuleMemory(memory_);
  if (behavior_)
    behavior_->updateModuleMemory(memory_);
  if (buttons_ && !locOnly)
    buttons_->updateModuleMemory(memory_);
  if (leds_)
    leds_->updateModuleMemory(memory_);

  //if (communications_)
  //communications_->updateModuleMemory(memory_);
  if (USE_LUA && lua_)
    lua_->updateModuleMemory(memory_);
  if (USE_PYTHON && python_)
    python_->updateModuleMemory(memory_);

  FrameInfoBlock *frame_info;
  memory_->getBlockByName(frame_info,"vision_frame_info");
  textlog_.setFrameInfo(frame_info);
}

void VisionCore::setMemoryVariables() {

  // Set the data path for the data folder
  switch (type_) {
  case CORE_ROBOT:
    memory_->data_path_ = "/home/nao/data/";
    break;
  case CORE_SIM:
  case CORE_TOOLSIM:
  case CORE_TOOL:
  case CORE_TOOL_NO_VISION:
    memory_->data_path_ = (std::string(getenv("NAO_HOME")) + "/data/").c_str();
    break;
  default:
    std::cerr << "VisionCore::init - problem setting data_path" << std::endl;
  }

  // Pass the coretype to the memory so that it can be accessed by different modules
  memory_->core_type_ = type_;
}

void VisionCore::preVision() {
  // called from lua befor vision module
  //std::cout << "pre vision" << std::endl << std::flush;

  // don't need to lock or receive data in tool
  if (isToolCore())
    return;

  // get the vision lock
  memory_->vision_lock_->lock();

  // see if we've already processed this frame
  while (raw_vision_frame_info_->frame_id == last_frame_processed_) {
    //std::cout << "WAITING" << std::endl << std::flush;
    memory_->vision_lock_->wait();
  }

  receiveData();
}

void VisionCore::postVision() {
  // don't need unlock for tool
  if (isToolCore())
    return;
  memory_->vision_lock_->unlock();
}

void VisionCore::publishData() {
  // don't publish data for tool
  if (isToolCore())
    return;

  vision_walk_request_->is_penalised_ = (game_state_->state == PENALISED);

  //std::cout << "publishDATA" << std::endl;
  // SHOULD BE CALLED WHILE VISION LOCK IS NOT HELD

  memory_->motion_vision_lock_->lock();
  // copy over data to the motion process
  bool temp_kick_running = sync_kick_request_->kick_running_;
  bool temp_finished_with_step = sync_kick_request_->finished_with_step_;
  *sync_kick_request_ = *vision_kick_request_;
  sync_kick_request_->kick_running_ = temp_kick_running;
  sync_kick_request_->finished_with_step_ = temp_finished_with_step;
  //std::cout << "vis: " << vision_walk_request_->step_into_kick_ << std::endl;
  *sync_walk_request_ = *vision_walk_request_;
  // just so motion has this info from vision
  sync_odometry_->fall_direction_ = vision_odometry_->fall_direction_;
  //std::cout << vision_walk_request_->odometry_fwd_offset_ << std::endl;
  *sync_joint_commands_ = *vision_joint_commands_;
  if (vision_kick_params_->send_params_)
    *sync_kick_params_ = *vision_kick_params_;
  if (vision_walk_param_->send_params_)
    *sync_walk_param_ = *vision_walk_param_;
  if (vision_al_walk_param_->send_params_)
    *sync_al_walk_param_ = *vision_al_walk_param_;
  memory_->motion_vision_lock_->unlock();

  // copy over data to the interface's vision thread
  memory_->vision_lock_->lock();
  camera_info_->copyToImageCapture(raw_camera_info_);
  memory_->vision_lock_->unlock();
}

void VisionCore::receiveData() {
  // don't receive data for tool
  if (isToolCore())
    return;
  // SHOULD BE CALLED WHILE VISION LOCK IS HELD
  // copy over data from the interface's vision thread
  *vision_frame_info_ = *raw_vision_frame_info_;
  camera_info_->copyFromImageCapture(raw_camera_info_);

  memory_->motion_vision_lock_->lock();

  if (type_ != CORE_TOOLSIM){
    // copy over data from the motion process
    *vision_body_model_ = *sync_body_model_;
    *vision_joint_angles_ = *sync_joint_angles_;
    *vision_sensors_ = *sync_sensors_;
    // odom
    *vision_odometry_ = *sync_odometry_;
    sync_odometry_->reset();
  }

  // kick request
  vision_kick_request_->kick_running_ = sync_kick_request_->kick_running_;
  vision_kick_request_->finished_with_step_ = sync_kick_request_->finished_with_step_;
  vision_kick_request_->kick_type_ = Kick::NO_KICK;
  vision_kick_request_->vision_kick_running_ = false;
  float fwdOffset = vision_walk_request_->odometry_fwd_offset_;
  float sideOffset = vision_walk_request_->odometry_side_offset_;
  float turnOffset = vision_walk_request_->odometry_turn_offset_;
  *vision_walk_request_ = *sync_walk_request_;
  vision_walk_request_->odometry_fwd_offset_ = fwdOffset;
  vision_walk_request_->odometry_side_offset_ = sideOffset;
  vision_walk_request_->odometry_turn_offset_ = turnOffset;
  vision_walk_request_->new_command_ = false;
  vision_walk_request_->perform_kick_ = false;
  vision_walk_request_->set_kick_step_params_ = false;
  vision_walk_request_->walk_decides_finished_with_target_ = false;

  vision_walk_param_->send_params_ = false;
  vision_al_walk_param_->send_params_ = false;
  vision_kick_params_->send_params_ = false;

  // walk
  *vision_walk_info_ = *sync_walk_info_;

  //sonar
  *vision_processed_sonar_ = *sync_processed_sonar_;

  // joint commands
  // Todd: no need to receive these here
  // just reset them to sending no commands
  vision_joint_commands_->send_body_angles_ = false;
  vision_joint_commands_->send_head_pitch_angle_ = false;
  vision_joint_commands_->send_head_yaw_angle_ = false;
  vision_joint_commands_->send_stiffness_ = false;
  vision_joint_commands_->send_sonar_command_ = false;
  vision_joint_commands_->send_arm_angles_  = false;

  memory_->motion_vision_lock_->unlock();

}

void VisionCore::motionLock() {
  if (!isToolCore())
    memory_->motion_vision_lock_->lock();
}

void VisionCore::motionUnlock() {
  if (!isToolCore())
    memory_->motion_vision_lock_->unlock();
}

bool VisionCore::isToolCore() {
  return (type_ == CORE_TOOL) || (type_ == CORE_TOOL_NO_VISION);
}
