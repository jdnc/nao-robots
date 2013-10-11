#include "RobotPositions.h"

// just dummies here, real values in lua/robotPositions.lua

Pose2D startingSidelinePoses[] = { Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0) };

Pose2D ourKickoffPosesDesired[NUM_TEAM_POSES] = { Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0) };
Pose2D ourKickoffPosesManual[NUM_TEAM_POSES] = { Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0) };
Pose2D penaltyPoses[NUM_PENALTY_POSES] = { Pose2D(M_PI/2, -PENALTY_CROSS_X, -HALF_FIELD_Y), Pose2D(-M_PI/2, -PENALTY_CROSS_X, HALF_FIELD_Y) };
bool   ourKickoffPosesManualReversible[NUM_TEAM_POSES] = {false,false,false,false,false,false};
Pose2D theirKickoffPosesDesired[NUM_TEAM_POSES] = { Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0) };
Pose2D theirKickoffPosesManual[NUM_TEAM_POSES] = { Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0), Pose2D(0,0,0) };
bool   theirKickoffPosesManualReversible[NUM_TEAM_POSES] = {false,false,false,false,false,false};

/*
#include <common/Field.h>

const float HALF_PENALTY_Y = 0.5 * PENALTY_Y;
const float HALF_GOAL_Y = 0.5 * GOAL_Y;
const float METER = 1000;

// *** REMEMBER POSES ARE ANGLE,X,Y ***

// 0 is default pos
const Pose2D startingSidelinePoses[] = {
  // default
  Pose2D(-M_PI/2.0,-2 * METER,HALF_FIELD_Y),
  // keeper starts on left sideline, 4 meters from midline
  Pose2D(-M_PI/2.0,-4 * METER,HALF_FIELD_Y),
  // player 2 starts on left sideline, 2.5 meters from midline
  Pose2D(-M_PI/2.0,-2.5 * METER,HALF_FIELD_Y),
  // player 3 starts on left side line, 1 meter from midline
  Pose2D(-M_PI/2.0,-1 * METER,HALF_FIELD_Y),
  // player 4 starts on right sideline, 2.5 meters from midline
  Pose2D(M_PI/2.0,-2.5 * METER,-HALF_FIELD_Y),
  // player 5 starts on right side line, 1 meter from midline
  Pose2D(M_PI/2.0,-1 * METER,-HALF_FIELD_Y)
};

const Pose2D ourKickoffPosesManual[] = {
  // default
  Pose2D(0,40,-0.5 * HALF_FIELD_Y),
  // keeper starts in center of goal
  Pose2D(0,-HALF_FIELD_X,0),
  // player 2 starts on corner of penalty box
  Pose2D(0,-1 * (HALF_FIELD_X - PENALTY_X),HALF_PENALTY_Y),
  // player 3 starts on other corner of penalty box
  Pose2D(0,-1 * (HALF_FIELD_X - PENALTY_X),-1 * HALF_PENALTY_Y),
  // player 4 starts even with cross, straight up from post
  Pose2D(0,-1 * PENALTY_CROSS_X, -1 * HALF_GOAL_Y),
  // player 5 starts just behind center circle
  Pose2D(0,0,-1 * CIRCLE_RADIUS - 20)
};


const Pose2D ourKickoffPosesDesired[] = {
  // default
  Pose2D(0,40,-0.5 * HALF_FIELD_Y),
  // keeper:   200 mm in front of goal line
  Pose2D(0,-HALF_FIELD_X + 200, 0),
  // player 2: left defender
  Pose2D(0,-1600,400),
  // player 3: left wing
  Pose2D(0,-300,1400),
  // player 4: right wing
  Pose2D(0,-300,-1400),
  // player 5: near center at a 45% angle
  Pose2D(M_PI / 4.0,-300,-300)
};

const bool ourKickoffPosesManualReversible[] = {
  false,false,false,false,true,false // only player 4 is likely to be reversed
};

const Pose2D theirKickoffPosesDesired[] = {
  // default
  Pose2D(0,40,-0.5 * HALF_FIELD_Y),
  // keeper:   200 mm in front of goal line
  Pose2D(0,-HALF_FIELD_X + 200, 0),
  // player 2: a little in front of the penalty box in front of post
  Pose2D(0,-(HALF_FIELD_X - PENALTY_X + 200),HALF_GOAL_Y),
  // player 3
  Pose2D(0,-900,-800),
  // player 4
  Pose2D(0,-900,800),
  // player 5: 250 mm back from center circle
  Pose2D(0,-CIRCLE_RADIUS - 250,0)
};

const Pose2D theirKickoffPosesManual[] = {
  // default
  Pose2D(0,40,-0.5 * HALF_FIELD_Y),
  // keeper starts in center of goal
  Pose2D(0,-HALF_FIELD_X,0),
  // player 2 starts halfway between corner of penalty box and edge of the field
  Pose2D(0,-1 * (HALF_FIELD_X - PENALTY_X),0.5 * (HALF_PENALTY_Y + HALF_FIELD_Y)),
  // player 3 starts halfway between other corner of penalty box and edge of the field
  Pose2D(0,-1 * (HALF_FIELD_X - PENALTY_X),-0.5 * (HALF_PENALTY_Y + HALF_FIELD_Y)),
  // player 4 starts halfway between corner of penalty box and center of field
  Pose2D(0,-1 * (HALF_FIELD_X - PENALTY_X),0.5 * HALF_PENALTY_Y),
  // player 5 starts halfway between other corner of penalty box and center of field
  Pose2D(0,-1 * (HALF_FIELD_X - PENALTY_X),-0.5 * HALF_PENALTY_Y)
};

const bool theirKickoffPosesManualReversible[] = {
  false,false,false,false,false,false // none are reversible since there's already a model in that spot
};

*/
