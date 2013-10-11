#ifndef BEHAVIORBLOCK_MG1MXV8Q
#define BEHAVIORBLOCK_MG1MXV8Q

#include "MemoryBlock.h"
#include <math/Geometry.h>
#include <common/Field.h>
#include <common/Kicks.h>
#include <memory/BehaviorParamBlock.h>

#define NUM_KICK_REGIONS_X 30
#define NUM_KICK_REGIONS_Y 20

const float KICK_REGION_SIZE_X = (FIELD_X / NUM_KICK_REGIONS_X);
const float KICK_REGION_SIZE_Y = (FIELD_Y / NUM_KICK_REGIONS_Y);

// struct for debug info about kick evaluations
struct KickEvaluation {
  Point2D leftPoint;
  Point2D rightPoint;
  Point2D weakPoint;
  Point2D strongPoint;
  float leftBearing;
  float rightBearing;
  float weakBearing;
  float strongBearing;
  int leftValid;
  int rightValid;
  int weakValid;
  int strongValid;
  int opponentBlock;
  bool evaluated;
  KickEvaluation(){
    evaluated = false;
    leftPoint = Point2D(0,0);
    rightPoint = Point2D(0,0);
    weakPoint = Point2D(0,0);
    strongPoint = Point2D(0,0);
    leftValid = 0;
    rightValid = 0;
    weakValid = 0;
    strongValid = 0;
    opponentBlock = 0;
    leftBearing = 0;
    rightBearing = 0;
    weakBearing = 0;
    strongBearing = 0;
  };
};


class Dive { // class for lua access, otherwise namespace
 public:
  
  enum diveTypes {
    NONE,
    LEFT,
    RIGHT,
    CENTER
  };
};


struct WalkMode { // struct for lua access, otherwise namespace
  enum Mode {
    SLOW,
    MID,
    SPRINT,
    TARGET,
    TARGET_CLOSE,
    KICK,
    SIDE,
    NUM_WALK_MODES
  };

  //#ifndef SWIG
  //static const char* getName(Mode mode) FUNCTION_IS_NOT_USED;
  //#endif
  static const char* getName(Mode mode) {
    switch (mode) {
      case SLOW: return "SLOW";
      case MID: return "MID";
      case SPRINT: return "SPRINT";
      case TARGET: return "TARGET";
      case TARGET_CLOSE: return "TARGET_CLOSE";
      case KICK: return "KICK";
      case SIDE: return "SIDE";
      default: return "UNKNOWN";
    }
  }
};

struct PassInfo {
  bool executingPass;
  int targetPlayer;
  Point2D target;
  float timeUntilPass;
  int kickChoice;

  PassInfo():
    executingPass(false),
    targetPlayer(-1)
  {
  }
};

struct SetPlayInfo {
  int type;
  bool reversed;
  int targetPlayer;

  SetPlayInfo():
    type(SetPlay::none),
    reversed(false),
    targetPlayer(-1)
  {
  }
};

struct BehaviorBlock : public MemoryBlock {
  BehaviorBlock():
    targetPt(0,0), 
    useTarget(false), 
    useTargetBearing(false), 
    lastSearchTurnTime(-1000),
    startBallSearchTime(-1000),
    completeBallSearchTime(-1000),
    completedBallSearch(false),
    layerScanState(-1),
    layerScanTime(100),
    isKickOffShot(false),
    numFramesNotKickOff(0),
    timePlayingStarted(-1),
    avoidSonarTime(0),
    avoidSonarDirIsLeft(true),
    sonarObstacleTime(0),
    keeperClearing(false),
    keeperDiving(Dive::NONE),
    walk_mode_(WalkMode::SLOW),
    log_behavior_trace_(false),
    test_odom_fwd(0),
    test_odom_side(0),
    test_odom_turn(0),
    test_odom_walk_time(0),
    test_odom_new(false),
    isInCluster(false),
    kickTestUsingCluster(false),
    balScanTime(0),
    balSeenGoalFrame(0)
  {
    header.version = 20;
    header.size = sizeof(BehaviorBlock);
  }

  void setValidKickRegion(unsigned int x, unsigned int y, bool valid) {
    validKickRegion[x][y] = valid;
  };

  bool getValidKickRegion(unsigned int x, unsigned int y) {
    return validKickRegion[x][y];
  };

  void setKickEval(int index, KickEvaluation eval){kickEvaluations[index]=eval;};
  void setKickLeftEval(int index, Point2D pt, float bearing, int valid){
    kickEvaluations[index].leftPoint = pt;
    kickEvaluations[index].leftBearing = bearing;
    kickEvaluations[index].leftValid = valid;
  };
  
  void setKickRightEval(int index, Point2D pt, float bearing, int valid){
    kickEvaluations[index].rightPoint = pt;
    kickEvaluations[index].rightBearing = bearing;
    kickEvaluations[index].rightValid = valid;
  };

  void setKickWeakEval(int index, Point2D pt, float bearing, int valid){
    kickEvaluations[index].weakPoint = pt;
    kickEvaluations[index].weakBearing = bearing;
    kickEvaluations[index].weakValid = valid;
  };
  
  void setKickStrongEval(int index, Point2D pt, float bearing, int valid){
    kickEvaluations[index].strongPoint = pt;
    kickEvaluations[index].strongBearing = bearing;
    kickEvaluations[index].strongValid = valid;
  };

  void setKickOpponent(int index, bool opp){
    kickEvaluations[index].opponentBlock = opp;
  };

  void setAllUnevaluated(){
    for (int i = 0; i < NUM_KICKS; i++){
      kickEvaluations[i].evaluated = false;
    }
  };

  void setEvaluated(int index, bool eval){kickEvaluations[index].evaluated=eval;};
  
  Point2D targetPt;
  bool useTarget;
  Point2D absTargetPt;
  bool useAbsTarget;
  float targetBearing;
  bool useTargetBearing;

  // search related
  float lastSearchTurnTime;
  float startBallSearchTime;
  float completeBallSearchTime;
  bool completedBallSearch;
  int layerScanState;
  float layerScanTime;

  bool ballIsInCorner;
  bool robotIsInCorner;
  bool isKickOffShot;

  int numFramesNotKickOff;
  float timePlayingStarted;

  float largeGapHeading;
  float smallGapHeading;
  bool foundKeeper;

  // for hysteris in sonar avoid direction
  float avoidSonarTime;
  bool avoidSonarDirIsLeft;
  float sonarObstacleTime;

  // kick region
  bool validKickRegion[NUM_KICK_REGIONS_X][NUM_KICK_REGIONS_Y];

  // save info about the kick we've selected to display later
  int kickChoice;
  float kickDistance;
  float kickRank;
  float kickHeading;
  bool chooseKick;

  // save info about kicks we evaluated
  KickEvaluation kickEvaluations[NUM_KICKS];

  // keeper is clearing
  bool keeperClearing;
  // keeper is diving
  int keeperDiving;

  Point2D keeperRelBallPos;
  Point2D keeperRelBallVel;

  // walk mode
  WalkMode::Mode walk_mode_;

  // more debug info
  bool log_behavior_trace_;

  // for testing odometry
  float test_odom_fwd;
  float test_odom_side;
  float test_odom_turn;
  float test_odom_walk_time;
  bool test_odom_new;

  // cluster
  bool isInCluster;
  bool kickTestUsingCluster;

  // for BAL
  float balScanTime;
  int balSeenGoalFrame;

  // info about what pass we want to do
  PassInfo passInfo;
  // info about what set play we want to do
  SetPlayInfo setPlayInfo;
};

#endif /* end of include guard: BEHAVIORBLOCK_MG1MXV8Q */
