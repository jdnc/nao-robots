#ifndef ROBOTPOSITIONS_H_SBZQRKAM
#define ROBOTPOSITIONS_H_SBZQRKAM

#include <math/Pose2D.h>
#include <common/WorldObject.h>
#include <common/Field.h>

const unsigned int NUM_PENALTY_POSES = 2;
const unsigned int NUM_ROBOTS_ON_TEAM = WO_TEAM_LAST-WO_TEAM_FIRST+1;
const unsigned int NUM_TEAM_POSES = NUM_ROBOTS_ON_TEAM + 1; // + 1 for default (placed at ind 0)

extern Pose2D penaltyPoses[NUM_PENALTY_POSES];
// starting poses of robot, before game starts
extern Pose2D startingSidelinePoses[NUM_TEAM_POSES];
// poses during our kickoff
extern Pose2D ourKickoffPosesDesired[NUM_TEAM_POSES];
extern Pose2D ourKickoffPosesManual[NUM_TEAM_POSES];
extern bool   ourKickoffPosesManualReversible[NUM_TEAM_POSES];
// poses during their kickoff
extern Pose2D theirKickoffPosesDesired[NUM_TEAM_POSES];
extern Pose2D theirKickoffPosesManual[NUM_TEAM_POSES];
extern bool   theirKickoffPosesManualReversible[NUM_TEAM_POSES];

#endif /* end of include guard: ROBOTPOSITIONS_H_SBZQRKAM */

