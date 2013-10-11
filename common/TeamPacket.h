#ifndef TEAM_PACKET_H
#define TEAM_PACKET_H

#include <memory/OpponentBlock.h>
#include <common/WorldObject.h>
#include <memory/BehaviorBlock.h>

struct oppStruct {
  oppStruct(){
    filled = false;
    x = 0;
    y = 0;
    sdx = 10000;
    sdy = 10000;
    sdxy = 10000;
    framesMissed = 1000;
  }

  bool filled;
  float x;
  float y;
  float sdx;
  float sdy;
  float sdxy;
  int framesMissed;
};

struct locStruct {
  locStruct(){
    robotX = 0;
    robotY = 0;
    robotSDX = 10;
    robotSDY = 10;
    orient = 0;
    sdOrient = 10;

    ballX = 0;
    ballY = 0;
    ballSDX = 10;
    ballSDY = 10;
    ballSDXY = 10;
  }

  float robotX;
  float robotY;
  float robotSDX;
  float robotSDY;
  float orient;
  float sdOrient;

  float ballX;
  float ballY;
  float ballSDX;
  float ballSDY;
  float ballSDXY;
};

struct bvrStruct {
  bvrStruct(){
    ballBid = 3000;
    ballDistance = 3000;
    ballMissed = 1000;
    ballSeen = false;
    role = 2;
    state = 1;
    strategy = 0;
    fallen = false;
    keeperClearing = false;
    useTarget = false;
  }
  int role;
  float ballBid;
  float ballDistance;
  int ballMissed;
  bool ballSeen;
  int state;
  bool fallen;
  bool keeperClearing;
  float targetX;
  float targetY;
  bool useTarget;

  int strategy;

  PassInfo passInfo;
  SetPlayInfo setPlayInfo;
};

struct TeamPacket {
  TeamPacket():
    robotNumber(-1),
    gcTeam(-1),
    rbTeam(-1)
  {
  }

  int robotNumber;
  int robotIP;

  locStruct locData;
  oppStruct oppData[MAX_OPP_MODELS_IN_MEM];
  bvrStruct bvrData;
  int packetsMissed[WO_TEAM_LAST+1]; // to 6 so we can index teammates 1-5

  int gcTeam;
  int rbTeam;

  double sentTime;
};

#endif
