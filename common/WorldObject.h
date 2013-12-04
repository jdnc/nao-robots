#ifndef _WORLD_OBJECT_H
#define _WORLD_OBJECT_H

#include <math/Geometry.h>
#include <common/Enum.h>

ENUM(WorldObjectType,   // Types of objects
  WO_BALL, 

  // Self and Teammates
  WO_TEAM1,
  WO_TEAM2,
  WO_TEAM3,
  WO_TEAM4,
  WO_TEAM5,

  // Opponents?
  WO_OPPONENT1,
  WO_OPPONENT2,
  WO_OPPONENT3,
  WO_OPPONENT4,
  WO_OPPONENT5,

  // center cirlce
  WO_CENTER_CIRCLE,

  // 2 Landmark goals
  WO_OWN_GOAL,
  WO_OPP_GOAL,

  WO_OWN_LEFT_GOALPOST,
  WO_OPP_LEFT_GOALPOST,

  WO_OWN_RIGHT_GOALPOST,
  WO_OPP_RIGHT_GOALPOST,

  WO_BEACON_PINK_YELLOW,
  WO_BEACON_BLUE_YELLOW,
  WO_BEACON_YELLOW_PINK,
  WO_BEACON_PINK_BLUE,
  WO_BEACON_YELLOW_BLUE,
  WO_BEACON_BLUE_PINK,

  // Unknown goals and goal posts
  WO_UNKNOWN_GOAL,
  WO_UNKNOWN_LEFT_GOALPOST,
  WO_UNKNOWN_RIGHT_GOALPOST,
  WO_UNKNOWN_GOALPOST,

  // Field Corners
  WO_UNKNOWN_L_1,
  WO_UNKNOWN_L_2,
  WO_UNKNOWN_T_1,
  WO_UNKNOWN_T_2,

  // L intersections (yellow to blue, top to bottom)
  WO_OPP_FIELD_LEFT_L,
  WO_OPP_FIELD_RIGHT_L,
  WO_OPP_PEN_LEFT_L,
  WO_OPP_PEN_RIGHT_L,
  WO_OWN_PEN_RIGHT_L,
  WO_OWN_PEN_LEFT_L,
  WO_OWN_FIELD_RIGHT_L,
  WO_OWN_FIELD_LEFT_L,

  WO_OPP_BACK_RIGHT_GOAL_L,
  WO_OPP_BACK_LEFT_GOAL_L,
  
  WO_OWN_BACK_RIGHT_GOAL_L,
  WO_OWN_BACK_LEFT_GOAL_L,

  // T intersections (yellow to blue, top to bottom)
  WO_OPP_PEN_LEFT_T,
  WO_OPP_PEN_RIGHT_T,
  WO_CENTER_TOP_T,
  WO_CENTER_BOTTOM_T,
  WO_OWN_PEN_RIGHT_T,
  WO_OWN_PEN_LEFT_T,

  WO_OPP_FRONT_RIGHT_GOAL_T,
  WO_OPP_FRONT_LEFT_GOAL_T,

  WO_OWN_FRONT_RIGHT_GOAL_T,
  WO_OWN_FRONT_LEFT_GOAL_T,

  WO_UNKNOWN_FIELD_LINE_1,
  WO_UNKNOWN_FIELD_LINE_2,
  WO_UNKNOWN_FIELD_LINE_3,
  WO_UNKNOWN_FIELD_LINE_4,


  // Horizontal Lines (right to left)
  WO_OPP_GOAL_LINE,          
  WO_OPP_PENALTY,
  WO_CENTER_LINE,
  WO_OWN_PENALTY,
  WO_OWN_GOAL_LINE,

  WO_OPP_BOTTOM_GOALBAR,
  WO_OWN_BOTTOM_GOALBAR,

  // Vertical Lines (top to bottom)
  WO_TOP_SIDE_LINE,    
  WO_PENALTY_TOP_OPP,   
  WO_PENALTY_TOP_OWN,
  WO_PENALTY_BOTTOM_OPP, 
  WO_PENALTY_BOTTOM_OWN,       
  WO_BOTTOM_SIDE_LINE,   
  
  WO_OPP_LEFT_GOALBAR,
  WO_OPP_RIGHT_GOALBAR,
  WO_OWN_LEFT_GOALBAR,
  WO_OWN_RIGHT_GOALBAR,

  // penalty crosses
  WO_UNKNOWN_PENALTY_CROSS,
  WO_OWN_PENALTY_CROSS,
  WO_OPP_PENALTY_CROSS,

  // cluster of robots
  WO_ROBOT_CLUSTER,

  NUM_WORLD_OBJS,

  WO_INVALID
);


const int WO_FIRST_BEACON = WO_BEACON_PINK_YELLOW;
const int WO_LAST_BEACON = WO_BEACON_BLUE_PINK;
const int WO_OPPONENT_FIRST=WO_OPPONENT1;
const int WO_OPPONENT_LAST=WO_OPPONENT5;

const int WO_TEAM_FIRST=WO_TEAM1;
const int WO_TEAM_FIELD_FIRST=WO_TEAM2;
const int WO_TEAM_LAST=WO_TEAM5;

// some constants to easily see which class each type is
const int WO_GOAL_FIRST = WO_OWN_GOAL;
const int WO_GOAL_LAST = WO_UNKNOWN_GOALPOST;
const int LANDMARK_OFFSET = WO_CENTER_CIRCLE;
const int NUM_LANDMARKS = WO_UNKNOWN_GOAL - LANDMARK_OFFSET + 1;
const int INTERSECTION_OFFSET =  WO_OPP_FIELD_LEFT_L;
const int NUM_INTERSECTIONS = WO_OWN_FRONT_LEFT_GOAL_T - INTERSECTION_OFFSET + 1;
const int LINE_OFFSET =  WO_OPP_GOAL_LINE;
const int NUM_LINES = WO_OWN_RIGHT_GOALBAR - LINE_OFFSET + 1;

const int L_INT_OFFSET = WO_OPP_FIELD_LEFT_L;
const int T_INT_OFFSET = WO_OPP_PEN_LEFT_T;

const int NUM_L_INTS = 8;
const int NUM_T_INTS = 6;

const int CROSS_OFFSET = WO_OWN_PENALTY_CROSS;
const int NUM_CROSSES = 2;

const int NUM_UNKNOWN_L = 2;
const int NUM_UNKNOWN_T = 2;
const int NUM_UNKNOWN_LINES = 4;
const int NUM_UNKNOWN_POSTS = 3;


class WorldObject {
public:

  WorldObject();
  ~WorldObject();

  // quick functions to determine which class of object this is
  bool isLine();
  bool isLandmark();
  bool isGoal();
  bool isGoalCenter();
  bool isGoalPost();
  bool isUnknownPost();
  bool isUnknownGoalCenter();
  bool isUnknownGoal();
  bool isBall();
  bool isIntersection();
  bool isT();
  bool isL();
  bool isTeammate();
  bool isOpponent();
  bool isUnknownIntersection();
  bool isUnknownLine();
  bool isCenterCircle();
  bool isUnknownPenaltyCross();
  bool isKnownPenaltyCross();
  bool isUnique();
  bool isAmbiguous();
  bool isPenaltySideline();
  bool isOwnGoal();
  bool isOppGoal();
  bool isKnownLine();
  bool isUnknownT();
  bool isUnknownL();

  void reset();

  bool seen;
  int frameLastSeen;
  int type;


  // These are relative to the robot and come directly from vision
  float visionDistance;
  float visionElevation; // not used
  AngRad visionBearing;
  float visionConfidence;

  // These are relative to the robot but come from the filtered localization
  float distance;
  float elevation;
  AngRad bearing;  
  //float sdDistance;   // Commented out ones are things we havn't decided on
  //float sdElevation;
  //AngRad sdBearing;

  Vector2D relVel;
  
  //Vector2D sdRelVel;

  // These are the global position after localization (including confidence)
  // or permanent locations for landmarks and unmovable objects
  Point2D loc;
  Point2D endLoc;

  // for lines
  // for unknown lines this is the relative x,y (x in front, y to left) of the end points of the observed line segment
  // for known lines this is the location on the field
  Line2D lineLoc;

  // for seen lines
  Point2D visionPt1;
  Point2D visionPt2;
  Line2D visionLine;

  // heights of beacons/goals/etc in the world
  float lowerHeight; 
  float upperHeight;

  AngRad orientation;
  Point2D sd;
  AngRad sdOrientation;
  Vector2D absVel;

  Point2D relPos;
  AngRad relOrientation;

  // Vision
  int imageCenterX;
  int imageCenterY;
  float radius;
  float width;
  float height;
  int fieldLineIndex;
  int ballBlobIndex;
  float xDisp;
  float yDisp;

  bool fromTopCamera; // true if observed from top cam, false if from bottom
};


#endif
