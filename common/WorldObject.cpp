#include "WorldObject.h"

// Create the array of objects
//WorldObject worldObjects[NUM_WORLD_OBJS];

WorldObject::WorldObject() {
  seen = false;

  frameLastSeen = -1;
  type = WO_INVALID;
  visionDistance=0.0;
  visionElevation=0.0;
  visionBearing=0.0;
  distance=0.0;
  elevation=0.0;
  bearing=0.0;
  relVel=Vector2D(0.0, 0.0);
  loc = Point2D(0, 0);
  lineLoc = Line2D(loc, loc);
  orientation=0.0;
  sd= Point2D(0.0, 0.0);
  sdOrientation=0.0;
  absVel= Point2D(0.0, 0.0);
  imageCenterX=0;
  imageCenterY=0;
  fieldLineIndex = -1;
  fromTopCamera = false;
}

WorldObject::~WorldObject() {
}

bool WorldObject::isLine(){
  return isUnknownLine() || isKnownLine();
}

bool WorldObject::isKnownLine(){
  return (type >= LINE_OFFSET && type < (LINE_OFFSET + NUM_LINES));
}

bool WorldObject::isLandmark(){
  return (type >= LANDMARK_OFFSET && type < (LANDMARK_OFFSET + NUM_LANDMARKS));
}

bool WorldObject::isGoal(){
  return (type >= WO_OWN_GOAL && type <= WO_UNKNOWN_GOALPOST);
}

bool WorldObject::isGoalCenter(){
  return (type == WO_OPP_GOAL || type == WO_OWN_GOAL || type == WO_UNKNOWN_GOAL);
}

bool WorldObject::isGoalPost(){
  return (isGoal() && !isGoalCenter());
}

bool WorldObject::isUnknownPost(){
  return (type >= WO_UNKNOWN_LEFT_GOALPOST && type <= WO_UNKNOWN_GOALPOST);
}

bool WorldObject::isUnknownGoal(){
  return (type >= WO_UNKNOWN_GOAL && type <= WO_UNKNOWN_GOALPOST);
}

bool WorldObject::isUnknownGoalCenter(){
  return type == WO_UNKNOWN_GOAL;
}

bool WorldObject::isOwnGoal(){
  return (type == WO_OWN_GOAL || type == WO_OWN_RIGHT_GOALPOST || type == WO_OWN_LEFT_GOALPOST);
}

bool WorldObject::isOppGoal(){
  return (type == WO_OPP_GOAL || type == WO_OPP_RIGHT_GOALPOST || type == WO_OPP_LEFT_GOALPOST);
}

bool WorldObject::isBall(){
  return (type == WO_BALL);
}

bool WorldObject::isIntersection(){
  return isT() || isL();
}

bool WorldObject::isT(){
  return (type >= WO_OPP_PEN_LEFT_T && type <= WO_OWN_PEN_LEFT_T);
}

bool WorldObject::isL(){
  return (type >= WO_OPP_FIELD_LEFT_L && type <= WO_OWN_FIELD_LEFT_L);
}

bool WorldObject::isUnknownIntersection(){
  return (type >= WO_UNKNOWN_L_1 && type <= WO_UNKNOWN_T_2);
}

bool WorldObject::isUnknownT(){
  return (type >= WO_UNKNOWN_T_1 && type <= WO_UNKNOWN_T_2);
}

bool WorldObject::isUnknownL(){
  return (type >= WO_UNKNOWN_L_1 && type <= WO_UNKNOWN_L_2);
}

bool WorldObject::isUnknownLine(){
  return (type >= WO_UNKNOWN_FIELD_LINE_1 && type <= WO_UNKNOWN_FIELD_LINE_4);
}

bool WorldObject::isTeammate(){
  return (type >= WO_TEAM_FIRST && type <= WO_TEAM_LAST);
}

bool WorldObject::isOpponent(){
  return (type >= WO_OPPONENT_FIRST && type <= WO_OPPONENT_LAST);
}

bool WorldObject::isCenterCircle(){
  return type == WO_CENTER_CIRCLE;
}

bool WorldObject::isUnknownPenaltyCross(){
  return type == WO_UNKNOWN_PENALTY_CROSS;
}

bool WorldObject::isKnownPenaltyCross(){
  return (type == WO_OPP_PENALTY_CROSS || type == WO_OWN_PENALTY_CROSS);
}

bool WorldObject::isUnique() {
  return isCenterCircle() || isKnownLine() || isKnownPenaltyCross();  // Center circle and known lines
}

bool WorldObject::isPenaltySideline(){
  return (type >= WO_PENALTY_TOP_OPP && type <= WO_PENALTY_BOTTOM_OWN);
}

bool WorldObject::isAmbiguous() {
  return (isUnknownGoal() || isUnknownIntersection() || isUnknownLine() || isUnknownPenaltyCross());
}

void WorldObject::reset(){
  seen = false;
  fieldLineIndex = -1;
}
