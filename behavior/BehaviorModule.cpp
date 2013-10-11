#include "BehaviorModule.h"
#include <iostream>

// memory
#include <memory/WorldObjectBlock.h>
#include <memory/BehaviorBlock.h>
#include <memory/BehaviorParamBlock.h>
#include <memory/OpponentBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/FrameInfoBlock.h>

const float BehaviorModule::CLUSTER_MEMORY_TIME = 2.0;
const float BehaviorModule::CLUSTER_MIN_SENSINGS = 1.0;

void BehaviorModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("behavior");
  requiresMemoryBlock("behavior_params");
  requiresMemoryBlock("opponents");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_frame_info");
}

void BehaviorModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(worldObjects,"world_objects");
  getOrAddMemoryBlock(behavior,"behavior");
  getOrAddMemoryBlock(behaviorParams,"behavior_params");
  getOrAddMemoryBlock(opponents,"opponents");
  getOrAddMemoryBlock(robotState,"robot_state");
  getOrAddMemoryBlock(gameState,"game_state");
  getOrAddMemoryBlock(frameInfo,"vision_frame_info");

  lastSensedCluster = -100;
  numSensedCluster = 0;
}


/** Checks if a given kick is within the valid kick region.
    @param x: The global resting x-position of the ball after the kick
    @param y: The global resting y-position of the ball after the kick
    @param globalKickHeading: The global (absolute) bearing of the ball
    @return 0: invalid,  1: valid,  2: valid&goal */
//#define DEBUG_isInKickRegion
int BehaviorModule::isInKickRegion(float x, float y, float globalKickHeading){
  WorldObject* ball = &(worldObjects->objects_[WO_BALL]);
  const KickStrategy &strategy = behaviorParams->mainStrategy;
  const ClusterKickStrategy &clusterStrategy = behaviorParams->clusterStrategy;

  float openingAngle = ClusterKickStrategy::getValue(behavior->kickTestUsingCluster,strategy.forwardOpeningAngle,clusterStrategy.forwardOpeningAngle);
  float shootOnGoalRadius = ClusterKickStrategy::getValue(behavior->kickTestUsingCluster,strategy.shootOnGoalRadius,clusterStrategy.shootOnGoalRadius);

  // on kickoff, avoid too close to goal
  if (behavior->isKickOffShot){
    // buffer from }line
    float endBuffer = 600;
    if (x > (FIELD_X / 2.0 - endBuffer)) {
      return 0;
    }
  }

  // on penalty kick.. avoid in their penalty box
  if (gameState->isPenaltyKick){
    float buffer = 250;
    if (x > (FIELD_X / 2.0 - PENALTY_X - buffer) && x < (FIELD_X/2.0+buffer)){
#ifdef DEBUG_isInKickRegion
      std::cout << "  on penalty kick, avoid in their penalty box" << std::endl;
#endif
      return 0;
    }
  }

  // front edge of field.... in goal or not
  if (x > (FIELD_X / 2.0)) {
    // get intersection with }line
    float slope = calculateSlope(Point2D(x,y),ball->loc);
    float endY = ball->loc.y + slope * ((FIELD_X / 2.0) - ball->loc.x);
    // inside post by at least postBuffer
    if ((fabs(endY) < (GOAL_Y / 2.0 - strategy.insidePostBuffer))) {
#ifdef DEBUG_isInKickRegion
      std::cout << "  past endline, goal" << std::endl;
#endif
      return 2;
    } else {
#ifdef DEBUG_isInKickRegion
      std::cout << "  past endline, not goal" << std::endl;
#endif
      return 0;
    }
  }

  // if we're in corner region, use corner strategy
  if (behavior->ballIsInCorner) {
#ifdef DEBUG_isInKickRegion
      std::cout << "  using corner strat" << std::endl;
#endif
    return checkCornerStrategy(x,y);
  }

  // check array of valid kick regions
  int xInd, yInd;
  xInd = (int)((x + (FIELD_X/2.0)) / KICK_REGION_SIZE_X);
  yInd = (int)((y + (FIELD_Y/2.0)) / KICK_REGION_SIZE_Y);
  // out of range, not valid
  if (xInd < 0 || yInd < 0 || xInd >= NUM_KICK_REGIONS_X || yInd >= NUM_KICK_REGIONS_Y){
#ifdef DEBUG_isInKickRegion
      std::cout << "  exceeded kick regioun inds" << std::endl;
#endif
    return 0;
  }

  if (!behavior->validKickRegion[xInd][yInd]) {
#ifdef DEBUG_isInKickRegion
      std::cout << "  not in kick region" << std::endl;
#endif
    return 0;
  }

  // angle from myself (i.e. don't kick backwards)
  if (fabs(globalKickHeading) > openingAngle) {
#ifdef DEBUG_isInKickRegion
      std::cout << "  exceeded opening angle: " << fabs(globalKickHeading) << " > " << openingAngle << std::endl;
#endif
    return 0;
  }

  // inside shooting radius, everything else is invalid (other than goals above)
  if ((!behavior->ballIsInCorner) &&
      (!behavior->isKickOffShot) &&
      ball->loc.getDistanceTo(Point2D(FIELD_X/2.0,0)) < shootOnGoalRadius) {
#ifdef DEBUG_isInKickRegion
      std::cout << "  in shooting radius, but not goal" << std::endl;
#endif
    return 0;
  }

#ifdef DEBUG_isInKickRegion
      std::cout << "  checks passed, valid" << std::endl;
#endif
  return 1;

}

float BehaviorModule::calculateSlope(Point2D pt1, Point2D pt2){
  float dx = pt2.x - pt1.x;
  if (fabs(dx) < 0.01) {
    dx = 0.01;
  }
  float slope = (pt2.y - pt1.y) / dx;
  return slope;
}

float BehaviorModule::calculateSlopeFromPointAngle(Point2D pt, float angle){
  Point2D pt2 = pt + Point2D(100,angle,POLAR);
  return calculateSlope(pt,pt2);
}

int BehaviorModule::checkCornerStrategy(float x, float y){
  WorldObject* ball = &(worldObjects->objects_[WO_BALL]);
  CornerKickStrategy cornerStrategy = behaviorParams->cornerStrategy;
  if (x < (FIELD_X/2.0 - cornerStrategy.distFromEndline)) {
    return 0;
  }

  // line between ball and near post
  Point2D post(FIELD_X/2.0, GOAL_Y/2.0);
  if (ball->loc.y < 0)
    post.y = -post.y;

  float m = calculateSlope(post, ball->loc);
  float dx = (y - post.y) / m;
  if (x > post.x + dx) {
    return 0;
  }

  // keep it off the endline
  if (fabs(y) > (GOAL_Y/2.0-100) && x > (FIELD_X/2.0 - cornerStrategy.endlineBuffer))
    return 0;


  // upper corner
  if (ball->loc.y > 0) {
    if (y < ball->loc.y && y > (-GOAL_Y/2.0 + cornerStrategy.distFromFarPost))
      return 1;
    else
      return 0;
  }
  else {
    if (y > ball->loc.y && y < (GOAL_Y/2.0 - cornerStrategy.distFromFarPost))
      return 1;
    else
      return 0;

  }
}

// return true if ball is farther than minDist from opponent
// note, i'm assuming that these kicks are going straight forwards
bool BehaviorModule::haveKickSpaceFromOpponents(float minDist, float openingAngleBehind) {
  WorldObject* ball = &(worldObjects->objects_[WO_BALL]);

  for (int j = 0; j < MAX_OPP_MODELS_IN_MEM; j++){
    Point2D oppLoc = opponents->getLocation(j);
    Point2D oppSD  = opponents->getSD(j);

    // within maximum std
    if (oppSD.x > behaviorParams->mainStrategy.maxOpponentSD ||
        oppSD.y > behaviorParams->mainStrategy.maxOpponentSD ||
        opponents->alpha[j] < 0){
      continue;
    }
    
    // ball is too close to opponent, return false
    if (ball->loc.getDistanceTo(oppLoc) < minDist) {
      WorldObject* me = &(worldObjects->objects_[robotState->WO_SELF]);
      if (fabs(normalizeAngle(me->loc.getBearingTo(oppLoc,me->orientation) + M_PI)) > openingAngleBehind) {
        return false;
      }
    }
    
  }

  // no opponents close to ball, return true. we have clearance
  if (minDist < 7000)
    return true;
  else // there's surely someone within 7m of us
    return false;
}

// Get distance of nearest opponent to ball
float BehaviorModule::getOpponentBallDistance(){
  WorldObject* ball = &(worldObjects->objects_[WO_BALL]);
  return getOpponentDistance(ball->loc);
}

float BehaviorModule::getNearestOpponentY(){
  //int closestOpp = -1;
  int minDist = 100000;
  int robotY = 0;
  WorldObject* robot = &(worldObjects->objects_[robotState->WO_SELF]);

  for (int j = 0; j < MAX_OPP_MODELS_IN_MEM; j++){
    Point2D oppLoc = opponents->getLocation(j);
    Point2D oppSD  = opponents->getSD(j);
    
    // within maximum std
    if (oppSD.x > behaviorParams->mainStrategy.maxOpponentSD ||
        oppSD.y > behaviorParams->mainStrategy.maxOpponentSD ||
        opponents->alpha[j] < 0)
      continue;

    float dist = robot->loc.getDistanceTo(oppLoc);
    
    
    if (dist < minDist){
      minDist = dist;
      //closestOpp = j;
      robotY = oppLoc.y;
    }

  }

  return robotY;
}



// Get distance to nearest valid opponent
float BehaviorModule::getOpponentDistance(Point2D target){

  //int closestOpp = -1;
  int minDist = 100000;

  for (int j = 0; j < MAX_OPP_MODELS_IN_MEM; j++){
    Point2D oppLoc = opponents->getLocation(j);
    Point2D oppSD  = opponents->getSD(j);
    
    // within maximum std
    if (oppSD.x > behaviorParams->mainStrategy.maxOpponentSD ||
        oppSD.y > behaviorParams->mainStrategy.maxOpponentSD ||
        opponents->alpha[j] < 0)
      continue;

    float dist = target.getDistanceTo(oppLoc);
    
    if (dist < minDist){
      minDist = dist;
      //closestOpp = j;
    }

  }

  return minDist;
}

bool BehaviorModule::opponentWorldObjectCheck(float kickDist, AngRad kickBearing, Point2D target) {
  WorldObject* ball = &(worldObjects->objects_[WO_BALL]);
  return opponentWorldObjectCheckGivenStartLoc(kickDist,kickBearing,target,ball->loc);
}

// Check if this intersects an opponent world object detected recently
bool BehaviorModule::opponentWorldObjectCheckGivenStartLoc(float kickDist, AngRad kickBearing, Point2D target, Point2D startLoc)
{
  WorldObject* robot = &(worldObjects->objects_[robotState->WO_SELF]);
  KickStrategy &strat = behaviorParams->mainStrategy;
  float allowOpponentSideDist = ClusterKickStrategy::getValue(behavior->kickTestUsingCluster,strat.allowOpponentSideDist,behaviorParams->clusterStrategy.allowOpponentSideDist);

  for (int j = 0; j < MAX_OPP_MODELS_IN_MEM; j++){
    Point2D oppLoc = opponents->getLocation(j);
    Point2D oppSD  = opponents->getSD(j);

    // within maximum std
    if (oppSD.x > strat.maxOpponentSD ||
        oppSD.y > strat.maxOpponentSD ||
        opponents->alpha[j] < 0 ||
        oppLoc.x < (startLoc.x - 1000))
      continue;

    // for bearing, only deal with opponents within 2m
    float oppDistance = startLoc.getDistanceTo(oppLoc);
    if (oppDistance < 2000) {

      // assume width of oppwidth cm either side
      Point2D topLeftPt(oppLoc.x-strat.opponentWidth, oppLoc.y+strat.opponentWidth);
      Point2D topRightPt(oppLoc.x+strat.opponentWidth, oppLoc.y+strat.opponentWidth);
      Point2D bottomLeftPt(oppLoc.x-strat.opponentWidth, oppLoc.y-strat.opponentWidth);
      Point2D bottomRightPt(oppLoc.x+strat.opponentWidth, oppLoc.y-strat.opponentWidth);
      
      // get bearing from startLoc to opponent
      float oppTopLeftBearing = startLoc.getBearingTo(topLeftPt, robot->orientation);
      float oppTopRightBearing = startLoc.getBearingTo(topRightPt, robot->orientation);
      float oppBottomLeftBearing = startLoc.getBearingTo(bottomLeftPt, robot->orientation);
      float oppBottomRightBearing = startLoc.getBearingTo(bottomRightPt, robot->orientation);
      
      // find max bearing (left)
      float oppLeftBearing = oppTopLeftBearing;
      if (oppTopRightBearing > oppLeftBearing)
        oppLeftBearing = oppTopRightBearing;
      if (oppBottomRightBearing > oppLeftBearing)
        oppLeftBearing = oppBottomRightBearing;
      if (oppBottomLeftBearing > oppLeftBearing)
        oppLeftBearing = oppBottomLeftBearing;
      
      // find min bearing (right)
      float oppRightBearing = oppBottomRightBearing;
      if (oppTopRightBearing < oppRightBearing)
        oppRightBearing = oppTopRightBearing;
      if (oppTopLeftBearing < oppRightBearing)
        oppRightBearing = oppTopLeftBearing;
      if (oppBottomLeftBearing < oppRightBearing)
        oppRightBearing = oppBottomLeftBearing;
      
      // recent opp between us and dest, and within 3 deg of bearing
      if ((kickDist+200) > oppDistance &&
          oppLeftBearing > kickBearing && kickBearing > oppRightBearing){
        
        debugLog((10, BehaviorModuleLog,"Kick at dist %5.3f, bearing %5.3f blocked by opp at (%5.3f, %5.3f) dist %5.3f bear %5.3f",kickDist,RAD_T_DEG*kickBearing,oppLoc.x,oppLoc.y,oppDistance,(oppLeftBearing - oppRightBearing) * 0.5 *RAD_T_DEG));
        return true;
      }
    }

    // or final point is within minOpponentDist of opponent, also bad kick
    if ((target.x < (FIELD_X/2.0 + 200)) && (target.getDistanceTo(oppLoc) < strat.minOpponentDist)) {
      if ((allowOpponentSideDist > 0) && (fabs(oppLoc.y - target.y) > allowOpponentSideDist) && oppLoc.getDistanceTo(startLoc) < 500) { // ignore if it's to the side AND its a robot near the ball. not ok to kick far and land to side of robot
        debugLog((10, BehaviorModuleLog, "Kick to pt (%5.3f, %5.3f) lands only %5.3f from opponent at (%5.3f, %5.3f) - too close, but past the side dist, so allowing, oppBallDist %5.3f", target.x, target.y, target.getDistanceTo(oppLoc), oppLoc.x, oppLoc.y, oppLoc.getDistanceTo(startLoc)));
      } else {
        debugLog((10, BehaviorModuleLog, "Kick to pt (%5.3f, %5.3f) lands only %5.3f from opponent at (%5.3f, %5.3f) - too close, oppBallDist %5.3f", target.x, target.y, target.getDistanceTo(oppLoc), oppLoc.x, oppLoc.y, oppLoc.getDistanceTo(startLoc)));
        return true;
      }
    }
  }
  return false;
}

bool BehaviorModule::filterCluster(bool sonarCluster, bool visionCluster) {
  if (sonarCluster) {
    numSensedCluster += 1.01;
    lastSensedCluster = frameInfo->seconds_since_start;
  }
  if (visionCluster) {
    numSensedCluster += 0.35; // trust vision less because sbarrett wrote it
    lastSensedCluster = frameInfo->seconds_since_start;
  }
  if (frameInfo->seconds_since_start - lastSensedCluster > CLUSTER_MEMORY_TIME)
    numSensedCluster = 0;
  return (numSensedCluster >= CLUSTER_MIN_SENSINGS);
}
