#ifndef BEHAVIOR_MODULE_H
#define BEHAVIOR_MODULE_H

#include <Module.h>

#include <memory/Memory.h>
#include <math/Geometry.h>

class WorldObjectBlock;
class OpponentBlock;
class BehaviorBlock;
class BehaviorParamBlock;
class RobotStateBlock;
class GameStateBlock;
class FrameInfoBlock;

class BehaviorModule : public Module {

  public:
  void specifyMemoryDependency();
  void specifyMemoryBlocks();

  float calculateSlopeFromPointAngle(Point2D pt, float angle);
  float calculateSlope(Point2D pt1, Point2D pt2);
  int isInKickRegion(float x, float y, float globalKickHeading);
  bool opponentWorldObjectCheck(float kickDist, AngRad kickBearing, Point2D target);
  bool opponentWorldObjectCheckGivenStartLoc(float kickDist, AngRad kickBearing, Point2D target, Point2D startLoc);
  bool haveKickSpaceFromOpponents(float minDist, float openingAngleBehind);
  int checkCornerStrategy(float x, float y);
  float getOpponentDistance(Point2D target);
  float getOpponentBallDistance();
  float getNearestOpponentY();

  bool filterCluster(bool sonarCluster, bool visionCluster);

 private:

  // memory info
  WorldObjectBlock* worldObjects;
  RobotStateBlock* robotState;
  BehaviorBlock* behavior;
  BehaviorParamBlock* behaviorParams;
  OpponentBlock* opponents;
  GameStateBlock* gameState;
  FrameInfoBlock *frameInfo;

  float lastSensedCluster;
  float numSensedCluster;

  static const float CLUSTER_MEMORY_TIME;
  static const float CLUSTER_MIN_SENSINGS;
};

#endif
