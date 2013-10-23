#include "CommunicationModule.h"

#include <common/RoboCupGameControlData.h>
#include <VisionCore.h>
#include <memory/Logger.h>
#include <memory/Lock.h>

#include <memory/FrameInfoBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/TeamPacketsBlock.h>
#include <memory/WorldObjectBlock.h>
#include <memory/OpponentBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/CameraBlock.h>
#include <memory/BehaviorBlock.h>

//#include <../lib/zlib/zlib.h>
#include "StreamingMessage.h"

//#include <sys/types.h>
//#include <sys/socket.h>
//#include <arpa/inet.h>
#include <netdb.h>
#include <boost/lexical_cast.hpp>

#include <iostream>

#define MAX_MEM_WRITE_SIZE MAX_STREAMING_MESSAGE_LEN

bool* CommunicationModule::lua_restart_requested_(NULL);
bool* CommunicationModule::python_restart_requested_(NULL);

CommunicationModule::CommunicationModule(VisionCore *core):
  teamUDP(TEAM_UDP_PORT,true,"10.0.255.255"),
  toolUDP(TOOL_UDP_PORT,false,"127.0.0.1"),
  gameControllerUDP(GAMECONTROLLER_PORT,false,"127.0.0.1"),
  io_service(),
  sock(io_service),
  streaming_logger_(NULL),
  stream_msg_(NULL),
  log_buffer_(NULL),
  stream_lock_(NULL)
{
  core_ = core;

  //teamUDP.start((char*)"Team",TEAM_UDP_PORT,SO_BROADCAST,sizeof(TeamPacket),&(CommunicationModule::listenTeamUDP),this);

  //toolUDP.start((char*)"Tool",TOOL_UDP_PORT,INADDR_ANY,1024,&(CommunicationModule::listenToolUDP),this);

  //gameControllerUDP.start((char*)"Game Controller",GAMECONTROLLER_PORT,INADDR_ANY,sizeof(RoboCupGameControlData),&(CommunicationModule::listenGameControllerUDP),this);
  tcp_connected_ = false;
}

CommunicationModule::~CommunicationModule() {
  if (streaming_logger_ != NULL)
    delete streaming_logger_;
  if (stream_msg_ != NULL)
    delete stream_msg_;
  if (log_buffer_ != NULL)
    delete log_buffer_;
  if (stream_lock_ != NULL)
    delete stream_lock_;
}

void CommunicationModule::specifyMemoryDependency() {
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("team_packets");
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("opponents");
  requiresMemoryBlock("vision_odometry");
  requiresMemoryBlock("camera_info");
  requiresMemoryBlock("behavior");
}

void CommunicationModule::specifyMemoryBlocks() {
  getMemoryBlock(frame_info_,"vision_frame_info");
  getOrAddMemoryBlock(game_state_,"game_state");
  getOrAddMemoryBlock(robot_state_,"robot_state");
  getOrAddMemoryBlock(world_objects_,"world_objects");

  getOrAddMemoryBlock(localization_,"localization");
  getOrAddMemoryBlock(team_packets_,"team_packets");
  getOrAddMemoryBlock(opponents_,"opponents");
  getOrAddMemoryBlock(odometry_,"vision_odometry");
  getOrAddMemoryBlock(camera_,"camera_info");
  getOrAddMemoryBlock(behavior_,"behavior");

}

void CommunicationModule::initSpecificModule() {
  teamUDP.startListenThread(&(CommunicationModule::listenTeamUDP),this);
  toolUDP.startListenThread(&(CommunicationModule::listenToolUDP),this);
  gameControllerUDP.startListenThread(&(CommunicationModule::listenGameControllerUDP),this);
  if ((stream_lock_ == NULL) && (core_->type_ != CORE_TOOLSIM)) {
    stream_lock_ = new Lock(Lock::getLockName(memory_,"STREAMLOCK"),true);
  }
}

void CommunicationModule::processFrame() {
  sendTeamUDP();
}


void* CommunicationModule::listenTeamUDP(void* arg ) {
  CommunicationModule* module = reinterpret_cast<CommunicationModule*>(arg);
  bool res;
  TeamPacket tp;
  while (1) {
    sleep(0.1);
    res = module->teamUDP.recv(tp);
    if (!res) {
      //std::cerr << "Reading team UDP failed" << std::endl;
      continue;
    }
    
    if (module->robot_state_->ignore_comms_)
      continue;

    if (tp.sentTime - module->getCurrentTime() > 10.0)
      continue;
    
    //std::cout << "got pkt from robot " << tp.robotNumber << " gc team: " << tp.gcTeam << " rb team: " << tp.rbTeam << " self: " << module->robot_state_->WO_SELF << std::endl;
    // not us but our team, and valid player num
    if (tp.robotNumber != module->robot_state_->WO_SELF &&
        tp.gcTeam == module->game_state_->gameContTeamNum &&
        tp.rbTeam == module->robot_state_->team_ &&
        tp.robotNumber > 0 && tp.robotNumber <= WO_TEAM_LAST) {

      //cout << "Team Packet from someone else\n" << flush;

      // copy team packet to memory
      TeamPacket* tpMem = &(module->team_packets_->tp[tp.robotNumber]);
      memcpy(tpMem, &tp, sizeof(TeamPacket));

      // set some flags about when we received it, etc
      module->team_packets_->frameReceived[tp.robotNumber] = module->frame_info_->frame_id;
      module->team_packets_->ballUpdated[tp.robotNumber] = true;
      module->team_packets_->oppUpdated[tp.robotNumber] = true;


      // Populate world objects for team mate position
      module->world_objects_->objects_[tp.robotNumber].loc.x = tp.locData.robotX;
      module->world_objects_->objects_[tp.robotNumber].loc.y = tp.locData.robotY;
      module->world_objects_->objects_[tp.robotNumber].orientation = tp.locData.orient;

      module->world_objects_->objects_[tp.robotNumber].sd.x = tp.locData.robotSDX;
      module->world_objects_->objects_[tp.robotNumber].sd.y = tp.locData.robotSDY;
      module->world_objects_->objects_[tp.robotNumber].sdOrientation = tp.locData.sdOrient;
    }
    //else if (tp.robotNumber != module->robot_state_->WO_SELF && module->frame_info_->source == MEMORY_ROBOT) {
    //std::cout << "ERROR: received invalid team pkt from robot " << tp.robotNumber << " gc team: " << tp.gcTeam << " rb team: " << tp.rbTeam << " self: " << module->robot_state_->WO_SELF << std::endl;
    //}
  }

  return NULL;
}

// send messages to teammates
void CommunicationModule::sendTeamUDP() {
  // Contruct the team packet, done before checking connect
  // so we can use the filled in data in the simulator

  int WO_SELF = robot_state_->WO_SELF;

  if (WO_SELF < WO_TEAM_FIRST || WO_SELF > WO_TEAM_LAST) return;

  TeamPacket &tp = team_packets_->tp[WO_SELF];

  tp.robotNumber = WO_SELF;
  tp.rbTeam  = robot_state_->team_;
  tp.gcTeam  = game_state_->gameContTeamNum;
  tp.robotIP = robot_state_->robot_id_;

  // our loc
  // these are in mm
  WorldObject* self = &(world_objects_->objects_[WO_SELF]);
  tp.locData.robotX = self->loc.x;
  tp.locData.robotY = self->loc.y;
  tp.locData.robotSDX = self->sd.x;
  tp.locData.robotSDY = self->sd.y;
  tp.locData.orient = self->orientation;

  // ball loc
  // these are in cm (how its used by the filter)
  WorldObject* ball = &(world_objects_->objects_[WO_BALL]);
  //tp.locData.ballX = localization_->X30[localization_->bestModel];
  //tp.locData.ballY = localization_->X40[localization_->bestModel];
  //tp.locData.ballSDX = localization_->SRXX;
  //tp.locData.ballSDY = localization_->SRYY;
  //tp.locData.ballSDXY = localization_->SRXY;

  // send info about when we've heard from each teammate
  for (int j = WO_TEAM_FIRST; j <= WO_TEAM_LAST; j++){
    tp.packetsMissed[j] = (frame_info_->frame_id - team_packets_->frameReceived[j]);
  }

  // fill in opponents from opponent tracking
  // first set them all to unfilled
  for (int i = 0; i < MAX_OPP_MODELS_IN_MEM; i++){
    tp.oppData[i].filled = false;
  }
  int fillIndex = 0;
  for (int j = 0; j < MAX_OPP_MODELS_IN_MEM; j++){
    if (opponents_->alpha[j] == -1000) continue;

    // dont send garbage info
    if (fabs(opponents_->X00[j]) > 3000) continue;
    if (fabs(opponents_->X10[j]) > 3000) continue;
    if (opponents_->SRXX[j] == 0 || opponents_->SRYY[j] == 0) continue;
    if (opponents_->SRXX[j] > 10000 || opponents_->SRYY[j] > 10000) continue;
    if (isnan(opponents_->X00[j]) || isnan(opponents_->X10[j])) continue;
    if (isnan(opponents_->SRXX[j]) || isnan(opponents_->SRYY[j])) continue;

    // these are all in cm as well (how its used by the filter)
    tp.oppData[fillIndex].x = opponents_->X00[j];
    tp.oppData[fillIndex].y = opponents_->X10[j];
    tp.oppData[fillIndex].sdx = opponents_->SRXX[j];
    tp.oppData[fillIndex].sdy = opponents_->SRYY[j];
    tp.oppData[fillIndex].sdxy = opponents_->SRXY[j];
    tp.oppData[fillIndex].framesMissed = (frame_info_->frame_id - opponents_->frameLastObserved[j]);
    tp.oppData[fillIndex].filled = true;

    fillIndex++;
    if (fillIndex >= MAX_OPP_MODELS_IN_MEM) break;
  }
  // done with opponent info


  // send info about our role and ball
  // to help figure out roles
  tp.bvrData.role = robot_state_->role_;

  // ball info
  tp.bvrData.ballDistance = ball->distance;
  tp.bvrData.ballSeen = ball->seen;
  tp.bvrData.ballMissed = (frame_info_->frame_id - ball->frameLastSeen);

  // ball bid
  // not just how close, but are we facing it... and are we facing roughly the right direction?
  // turn ~50 deg / sec, walk 192 mm / sec.
  // so every 50 degrees off is anohter 192 mm we could have walked
  // but we only need to turn maybe halfway?
  // so i'll say every 50 degrees is worth 100mm
  tp.bvrData.ballBid = ball->distance;

  float bearingError = fabs(ball->bearing)*RAD_T_DEG - 30.0;
  if (bearingError < 0) bearingError = 0;
  tp.bvrData.ballBid += bearingError * 3.0;

  // and we want to be on right side of ball
  // ramp this error up to a max worth of 800 mm (close to the time required to rotate 180)
  float xError = self->loc.x - ball->loc.x;
  if (xError < 0) xError = 0;
  if (xError > 800) xError = 800;
  tp.bvrData.ballBid += xError;

  // sonar
  float timeSinceSonar = (frame_info_->seconds_since_start - behavior_->sonarObstacleTime);
  if (timeSinceSonar < 0.25){
    // half meter penalty for sonar avoiding robots
    tp.bvrData.ballBid += 500;
  }

  // penalty for keeper
  if (robot_state_->WO_SELF == KEEPER){
    // penalty for keeper?  or maybe an advantage to him?
    // keeper must be 33% closer than other robots to win out and go for it
    tp.bvrData.ballBid *= 1.5;

    // if ball in box... keeper must go
    float boxBuffer = 30; // need some space to rotate around too
    if (ball->loc.x < (-HALF_FIELD_X + PENALTY_X + boxBuffer) && fabs(ball->loc.y) < PENALTY_Y/2.0 + boxBuffer){
      tp.bvrData.ballBid = 1;
    }

    // not if diving
    if (behavior_->keeperDiving != Dive::NONE){
      tp.bvrData.ballBid = 7000;
    }

    // not if ball is coming at us
    if (ball->relVel.x < -30){
      if (ball->relVel.x < -50){
        tp.bvrData.ballBid += 6500;
      } else {
        tp.bvrData.ballBid += 25.0 * -ball->relVel.x;
      }
    }

    // not too far up field
    if (robot_state_->role_ == KEEPER){
      // only in last 2 meters of field and not too far to sideline
      if (ball->loc.x > -1000 || (fabs(ball->loc.y) > 1500)) {
        tp.bvrData.ballBid = 7000;
      }
    }
    // if we're already chasing, chase up to midfield
    else {
      if (ball->loc.x > 0) {
        tp.bvrData.ballBid = 7000;
      }
    }
  }

  // we dont know which way we're going
  //if (localization_->oppositeModels || localization_->fallenModels){
  //tp.bvrData.ballBid += 6000;
  //}

  // state
  tp.bvrData.state = game_state_->state;

  // active getup or there is a fall direction means we've fallen
  tp.bvrData.fallen = (odometry_->getting_up_side_ != Getup::NONE || odometry_->fall_direction_ != Fall::NONE);

  // strategy
  tp.bvrData.strategy = 0;
  tp.bvrData.keeperClearing = behavior_->keeperClearing;
  tp.bvrData.targetX = behavior_->absTargetPt.x;
  tp.bvrData.targetY = behavior_->absTargetPt.y;
  tp.bvrData.useTarget = behavior_->useAbsTarget;

  tp.bvrData.passInfo = behavior_->passInfo;
  tp.bvrData.setPlayInfo = behavior_->setPlayInfo;

  // Now check if we connect and OK to send the data
  //if (!teamUDP.isConnected){
    //// dont print error in sim
    //if (frame_info_->source == MEMORY_ROBOT)
      //std::cout << "not sending pkt, teamUDP not connected!" << std::endl;
    //return;
  //}

  // bad model ... ignore ball
  //if (localization_->bestAlpha < 0.8 || localization_->oppositeModels)
  //tp.bvrData.ballMissed = 60;


  if (frame_info_->frame_id % 6 == 0 && frame_info_->source == MEMORY_ROBOT) {
    bool res = teamUDP.send(tp);
    if (res){
      team_packets_->frameReceived[WO_SELF] = frame_info_->frame_id;
    }
    else
      std::cerr << "Sending team UDP failed" << std::endl;
  } else if (frame_info_->frame_id % 6){
    team_packets_->frameReceived[WO_SELF] = frame_info_->frame_id;
  }

  tp.sentTime = getCurrentTime();
}

void CommunicationModule::sendToolUDP(std::string message) {
    toolUDP.sendToSender(message.c_str(), message.length());
}


void* CommunicationModule::listenToolUDP(void* arg ) {
  // in use  P
  CommunicationModule* module = reinterpret_cast<CommunicationModule*>(arg);
  bool res;
  //char current;
  char buffer[1024];
  while (1) {
    sleep(0.5);
    res = module->toolUDP.recv(buffer,sizeof(buffer));
    if (!res) {
      //std::cerr << "Receiving tool UDP failed" << std::endl;
      continue;
    }

    char option = buffer[0];
    switch (option) {

    case 'S': // State change
      {
        char state_char = buffer[1];
        int prev_state = module->game_state_->state;
        switch (state_char) {
        case 'I':
          module->game_state_->state = INITIAL;
          break;
        case 'R':
          module->game_state_->state = READY;
          break;
        case 'S':
          module->game_state_->state = SET;
          break;
        case 'P':
          module->game_state_->state = PLAYING;
          break;
        case 'X':
          module->game_state_->state = PENALISED;
          break;
        case 'F':
          module->game_state_->state = FINISHED;
          break;
        case 'T':
          module->game_state_->state = TESTING;
          break;
        case 'O':
          {
          module->game_state_->state = TEST_ODOMETRY;
          //char *msg = buffer + 3;
          std::stringstream msg(buffer + 3);
          msg >> module->behavior_->test_odom_fwd;
          msg >> module->behavior_->test_odom_side;
          msg >> module->behavior_->test_odom_turn;
          msg >> module->behavior_->test_odom_walk_time;
          std::cout << "received test walk: " << module->behavior_->test_odom_fwd << " " << module->behavior_->test_odom_side << " " << module->behavior_->test_odom_turn << " " << module->behavior_->test_odom_walk_time << std::endl;
          module->behavior_->test_odom_new = true;
          }
          break;
        case 'C':
          {
            char next_char = buffer[2];
            if (next_char == 'B') {
              module->game_state_->state = BOTTOM_CAM;
            } else {
              module->game_state_->state = TOP_CAM;
            }
          }
          break;
        default:
          break;
        }
        std::cout << "State Changed from " << stateNames[prev_state] << " to " << stateNames[module->game_state_->state] << std::endl;
      }
      break;
    case 'L':
      // logging stuff
      {
        int index = 0, frames = 0;
        double freq = 0;
        char next_char = buffer[++index];
        switch (next_char) {
        case 'S': // select
          module->handleLoggingBlocksMessage(&(buffer[2]));
          break;
        case 'B': // begin
          {
            std::stringstream ss(buffer);
            std::string token;
            std::getline(ss, token, ','); // LB
            std::getline(ss, token, ','); // Frames
            frames = atoi(token.c_str());
            std::getline(ss, token, ','); // Freq
            freq = atof(token.c_str());
            std::cout << "Logging " << frames << " frames, once per " << freq << " second" << (freq == 1 ? "\n" : "s\n");
            module->core_->enableLogging(frames,freq);
          break;
          }
        case 'E': // end
          module->core_->startDisableLogging();
          break;
        default:
          break;
        }
      }
      break;
    case 'X':
      // streaming stuff
      {
        char next_char = buffer[1];
        switch (next_char) {
        case 'B': // begin
          module->startTCP();
          break;
        case 'E': // end
          if (module->tcp_connected_) {
            module->tcp_connected_ = false;
          } else {
            std::cout << "tcp already disconnected" << std::endl;
          }
          break;
        default:
          break;
        }
      }
      break;
    case 'R':
      // restart lua and/or python
      if(lua_restart_requested_) *lua_restart_requested_ = true;
      if(python_restart_requested_) *python_restart_requested_ = true;
      break;
    case 'C':
      // camera parameters
      {
        char next_char = buffer[1];
        switch (next_char) {
        case 'Y': // set top camera params
          module->camera_->set_top_params_ = true;
          module->camera_->comm_module_request_received_ = true;
          module->handleCameraParamsMessage(module->camera_->params_top_camera_, &(buffer[2]));
          cout << "CommunicationModule: Set top camera params" << endl;
          break;
        case 'X': // set bottom camera params
          module->camera_->set_bottom_params_ = true;
          module->camera_->comm_module_request_received_ = true;
          module->handleCameraParamsMessage(module->camera_->params_bottom_camera_, &(buffer[2]));
          cout << "CommmunicationModule: Set bottom camera params" << endl;
          break;
        case 'G': // get camera params - tool should already be streaming
          module->camera_->get_top_params_ = true;
          module->camera_->get_bottom_params_ = true;
          module->camera_->comm_module_request_received_ = true;
          cout << "CommunicationModule: Read camera params" << endl;
          break;
        case 'R': // reset camera
          std::cout << "CommunicationModule: Reset camera " << std::endl;
          module->camera_->reset_bottom_camera_ = true;
          module->camera_->reset_top_camera_ = true;
          module->camera_->comm_module_request_received_ = true;
          break;
        }
      }
      break;

    }
  }

  return NULL;
}

void CommunicationModule::handleCameraParamsMessage(CameraParams &params, char *msg) {

  std::vector<std::string> paramNames;
  std::vector<int> paramValues;

  int i = 0;
  std::string currentToken;

  bool tokenIsParamName = true;

  while (msg[i] != '|') {
    if (msg[i] == ' ') { // current token has ended
      if (tokenIsParamName) {
        paramNames.push_back(currentToken);
        tokenIsParamName = false;
      } else {
        paramValues.push_back(boost::lexical_cast<int>(currentToken));
        tokenIsParamName = true;
      }
      currentToken.clear();
    } else {
      currentToken += msg[i];
    }
    i++;
  }

  for (unsigned i = 0; i < paramNames.size(); i++) {
    if (paramNames[i] == "AutoWhiteBalance") {
      params.kCameraAutoWhiteBalance = paramValues[i];
    } else if (paramNames[i] == "ExposureAuto") {
      params.kCameraExposureAuto = paramValues[i];
    } else if (paramNames[i] == "BacklightCompensation") {
      params.kCameraBacklightCompensation = paramValues[i];
    } else if (paramNames[i] == "Brightness") {
      params.kCameraBrightness = paramValues[i];
    } else if (paramNames[i] == "Contrast") {
      params.kCameraContrast = paramValues[i];
    } else if (paramNames[i] == "Saturation") {
      params.kCameraSaturation = paramValues[i];
    } else if (paramNames[i] == "Hue") {
      params.kCameraHue = paramValues[i];
    } else if (paramNames[i] == "Exposure") {
      params.kCameraExposure = paramValues[i];
    } else if (paramNames[i] == "Gain") {
      params.kCameraGain = paramValues[i];
    } else if (paramNames[i] == "Sharpness") {
      params.kCameraSharpness = paramValues[i];
    }
  }
}

void CommunicationModule::handleLoggingBlocksMessage(char *msg) {
  std::string block_name;
  bool log_block;
  int i = 0;
  while (msg[i] != '|') {
    if (msg[i] == ' ') {
      i++;
      if (msg[i] == '0')
        log_block = false;
      else if (msg[i] == '1')
        log_block = true;
      else {
        std::cout << "bad logging info " << msg[i] << std::endl;
        return;
      }
      if (log_block) {
        std::cout << std::boolalpha << "setting logging of " << block_name << " to " << log_block << std::endl;
      }
      // special case for behavior trace (not a block)
      if (block_name.compare("behavior_trace") == 0){
        if (behavior_ != NULL)
          behavior_->log_behavior_trace_ = log_block;
      } else {
        memory_->setBlockLogging(block_name,log_block);
      }
      block_name.clear();
      i++;
      assert(msg[i] == ',');
    } else
      block_name += msg[i];
    i++;
  }
}

/** This is the callback function used when we receive a UDP packet
    from GameController
    /param void* arg is a pointer to the call. We then call receive gameControllerUDP
*/
void* CommunicationModule::listenGameControllerUDP(void* arg ) {
  CommunicationModule* module = reinterpret_cast<CommunicationModule*>(arg);
  bool res;
  RoboCupGameControlData gc;
  while (1) {
    sleep(0.5);
    res = module->gameControllerUDP.recv(gc);
    if (!res) {
      //std::cerr << "Receiving game controller UDP failed" << std::endl;
      continue;
    }
    
    if (module->robot_state_->ignore_comms_)
      continue;

    // check version of packet
    if (gc.version != GAMECONTROLLER_STRUCT_VERSION){
      cout << endl << "ERROR: expect GameControllerPacket version: "
           << GAMECONTROLLER_STRUCT_VERSION
           << ", received: " << gc.version << endl << endl;
    }

    int teamNum = module->game_state_->gameContTeamNum;
    //cout << "our team num: " << teamNum << endl;

    // First check if this packet is even meant for a robot on our team !
    if (gc.teams[0].teamNumber==teamNum || gc.teams[1].teamNumber==teamNum) {
      int teamIndex=0; // stores the index into the team array
      int oppIndex=1; // stores the index into the team array
      if (teamNum==gc.teams[1].teamNumber) {
        teamIndex=1;
        oppIndex=0;
      }

      TeamInfo t=gc.teams[teamIndex];
      RobotInfo r=t.players[module->robot_state_->WO_SELF-1];       // We index from 1-5, they go 0-4

      // check if our team has changed
      //  if (t.teamColour != commonMem->team){
      // reset world objects for opposite field
      //  commonMem->teamChange = true;
      //}

      // Color info
      if (module->robot_state_->team_ != t.teamColour) {
        std::cout << "Gamecontroller has a team change" << std::endl;
        module->robot_state_->team_changed_ = true;
      }

      module->robot_state_->team_ = t.teamColour;

      int oldState = module->game_state_->state;

      // Do state info
      if (r.penalty>PENALTY_NONE) { // Shit we are penalised !!
        module->game_state_->state=PENALISED;
        module->game_state_->secsTillUnpenalised=r.secsTillUnpenalised;
      } else {    // set the state
        int rState=gc.state;
        if (rState==STATE_INITIAL) module->game_state_->state=INITIAL;
        else if (rState==STATE_READY) module->game_state_->state=READY;
        else if (rState==STATE_SET) module->game_state_->state=SET;
        else if (rState==STATE_PLAYING) module->game_state_->state=PLAYING;
        else if (rState==STATE_FINISHED) module->game_state_->state=FINISHED;
      }

      // check if state changed
      if (oldState != module->game_state_->state){
        module->game_state_->lastStateChangeFromButton = false;
        if (oldState == PENALISED)
          module->game_state_->lastTimeLeftPenalized = (module->frame_info_->seconds_since_start);
        //  module->game_state->stateChange = true;
      }

      // penalty kick
      if (gc.secondaryState == STATE2_PENALTYSHOOT)
        module->game_state_->isPenaltyKick = true;
      else
        module->game_state_->isPenaltyKick = false;

      // random stuff
      module->game_state_->isFirstHalf=gc.firstHalf;            // 1 = game in first half, 0 otherwise
      if (gc.kickOffTeam==t.teamColour) module->game_state_->ourKickOff=true;
      else module->game_state_->ourKickOff=false;
      module->game_state_->lastOutBy = gc.dropInTeam;

      module->game_state_->dropInTime=gc.dropInTime;
      module->game_state_->secsRemaining=gc.secsRemaining;       // estimate of number of seconds remaining in the half
      if (module->game_state_->ourScore<t.score) cout << "WE SCORED !!" << endl << flush;
      module->game_state_->ourScore=t.score;

      if (module->game_state_->opponentScore<gc.teams[oppIndex].score) cout << "CRAP THEY SCORED ???" << endl << flush;
      module->game_state_->opponentScore=gc.teams[oppIndex].score;

      module->game_state_->frameReceived = module->frame_info_->frame_id;
    }
  }

  return NULL;

}

void CommunicationModule::startTCP() {
  tcp_connected_ = false;

  tcp::endpoint endpt(toolUDP.senderAddress(),TOOL_TCP_PORT);

  std::cout << "streaming to " << toolUDP.senderAddress() << ":" << TOOL_TCP_PORT << "\n";

  boost::system::error_code err;
  sock.connect(endpt,err);
  if (err != 0) {
    std::cout << "Error creating tcp connection: " << err << std::endl;
    return;
  }
  tcp_connected_ = true;
  if (stream_msg_ == NULL)
    stream_msg_ = new StreamingMessage();
  if (streaming_logger_ == NULL)
    streaming_logger_ = new StreamLogger();

  pthread_create(&stream_thread_,NULL,&stream,this);
}

void* stream(void *arg) {
  CommunicationModule *module = reinterpret_cast<CommunicationModule*>(arg);
  while (module->tcp_connected_)
    module->sendTCP();
  std::cout << "stream thread exitting" << std::endl;
  return NULL;
}

void CommunicationModule::optionallyStream() {
  if (tcp_connected_) {
    prepareSendTCP();
  } else if (sock.is_open()) {
    std::cout << "disconnecting tcp" << std::endl;
    sock.close();
  }
}

void CommunicationModule::prepareSendTCP() {
  if (stream_lock_->owns())
    return;
  if (stream_lock_->try_lock()) {
    streaming_logger_->writeMemory(*memory_);
    stream_lock_->unlock();
    stream_lock_->notify_one();
  }
}

void CommunicationModule::sendTCP() {
  if (stream_lock_->owns())
    return;
  stream_lock_->lock();
  const StreamBuffer& buffer = streaming_logger_->getBuffer();
  if (buffer.size == 0) {
    stream_lock_->wait();
  }
  if (!stream_msg_->sendMessage(sock, buffer.buffer, buffer.size)) {
    std::cout << "Problem sending tcp, disconnecting" << std::endl;
    tcp_connected_ = false;
  }
  streaming_logger_->clearBuffer();
  stream_lock_->unlock();
}

double CommunicationModule::getCurrentTime() {
  return getSystemTime() - robot_state_->clock_offset_;
}
