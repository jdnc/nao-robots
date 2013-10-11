#ifndef GAMESTATE_
#define GAMESTATE_

#include "MemoryBlock.h"
#include <common/States.h>

struct GameStateBlock : public MemoryBlock {
public:
  GameStateBlock():
    isPenaltyKick(false)
  {
    header.version = 2;
    header.size = sizeof(GameStateBlock);

    state = INITIAL;
    ourKickOff = true;
    isFirstHalf = 1;
    gameContTeamNum=1;
    ourScore = 0;
    opponentScore = 0;
    secsTillUnpenalised = 0;
    dropInTime = -1.0;
    lastOutBy = 0;
    secsRemaining = 600;
    frameReceived = 0;
    lastStateChangeFromButton = false;
    lastTimeLeftPenalized = -1;
  }

  // state of the robot (playing, ready, etc)
  int state;

  // What team number are we on
  int gameContTeamNum;

  bool isPenaltyKick;

  // Game controller memory
  bool ourKickOff;
  int secsRemaining;
  int ourScore;
  int opponentScore;
  int secsTillUnpenalised;
  int isFirstHalf;   
  int lastOutBy;           
  int dropInTime;

  int frameReceived;

  bool lastStateChangeFromButton;
  float lastTimeLeftPenalized;
};

#endif 
