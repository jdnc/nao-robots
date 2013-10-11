#ifndef STATES_H
#define STATES_H

#include <string>

enum TEAMS {
 TEAM_BLUE = 0,
 TEAM_RED = 1
};

enum STATES {
  UNDEFINED_STATE = 0,
  INITIAL = 1,
  READY = 2,
  SET = 3,
  PLAYING = 4,
  TESTING = 5,
  PENALISED = 6,
  FINISHED = 7,
  FALLING = 8,
  BOTTOM_CAM = 9,
  TOP_CAM = 10,
  TEST_ODOMETRY = 11
};

const std::string stateNames[] = {
  "undefined",
  "initial",
  "ready",
  "set",
  "playing",
  "testing",
  "penalized",
  "finished",
  "falling",
  "bottom_cam",
  "top_cam",
  "test_odometry"
};


#endif
