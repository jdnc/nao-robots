#ifndef COMMUNICATIOS_MODULE_H
#define COMMUNICATION_MODULE_H

#include <Module.h>
#include <common/RobotInfo.h>

//#include <communications/ThreadedUDPSocket.h>
#include <communications/UDPWrapper.h>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

class VisionCore;
class Logger;
class StreamingMessage;
class Lock;

class RobotStateBlock;
class GameStateBlock;
class LocalizationBlock;
class TeamPacketsBlock;
class WorldObjectBlock;
class FrameInfoBlock;
class OpponentBlock;
class OdometryBlock;
class CameraBlock;
class BehaviorBlock;

void* stream(void *arg);

class CommunicationModule: public Module {
 public:
  CommunicationModule(VisionCore *core);
  ~CommunicationModule();

  void specifyMemoryDependency();
  void specifyMemoryBlocks();
  void initSpecificModule();

  void processFrame();
  void optionallyStream();

  static bool *lua_restart_requested_, *python_restart_requested_;

 private:
  void sendTeamUDP();

  VisionCore *core_;

  // memory blocks
  FrameInfoBlock *frame_info_;
  RobotStateBlock *robot_state_;
  GameStateBlock *game_state_;
  LocalizationBlock *localization_;
  TeamPacketsBlock *team_packets_;
  WorldObjectBlock *world_objects_;
  OpponentBlock *opponents_;
  OdometryBlock *odometry_;
  CameraBlock *camera_;
  BehaviorBlock *behavior_;

  //Udp for robot team communication
  //ThreadedUDPSocket teamUDP;
  UDPWrapper teamUDP;
  static void* listenTeamUDP( void * );

  //Udp for the tools commands to the robot
  //ThreadedUDPSocket toolUDP;
  UDPWrapper toolUDP;
  static void* listenToolUDP( void * );
  void handleCameraParamsMessage(CameraParams &params, char *msg);

  void handleLoggingBlocksMessage(char *);

  //Messages from game controller
  //ThreadedUDPSocket gameControllerUDP;
  UDPWrapper gameControllerUDP;
  static void* listenGameControllerUDP( void * );

  // for streaming to tool
public:
  bool tcp_connected_;
  void sendTCP();
  void sendToolUDP(std::string message);

  double getCurrentTime();
private:
  boost::asio::io_service io_service;
  tcp::socket sock;
  void startTCP();
  void prepareSendTCP();
  Logger *streaming_logger_;
  StreamingMessage *stream_msg_;
  StreamBuffer cbuffer_;
  char *log_buffer_;
  Lock *stream_lock_;
  pthread_t stream_thread_;
};

#endif /* end of include guard: COMMUNICATIONS_MODULE */
