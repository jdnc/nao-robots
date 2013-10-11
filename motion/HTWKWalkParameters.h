#ifndef HTWKWALKPARAMETERS_GWNK1KUR
#define HTWKWALKPARAMETERS_GWNK1KUR

#include <math/Pose2D.h>

#define MAX_NUM_WALK_KICK_SPLINE_PTS 10

struct WalkKickSpline {
  int numPts;
  double times[MAX_NUM_WALK_KICK_SPLINE_PTS];
  double xswing[MAX_NUM_WALK_KICK_SPLINE_PTS];
  double yswing[MAX_NUM_WALK_KICK_SPLINE_PTS];
  double zswing[MAX_NUM_WALK_KICK_SPLINE_PTS];
  double xstance[MAX_NUM_WALK_KICK_SPLINE_PTS];
  double ystance[MAX_NUM_WALK_KICK_SPLINE_PTS];
  double zstance[MAX_NUM_WALK_KICK_SPLINE_PTS];
};

struct HTWKWalkParameters {
  HTWKWalkParameters();
  
  int numFrames; // number of frames for 2 steps
  int frameInc; // frame increment (1 for robot, 2 for sim)
  int stanceSwitch1;
  int stanceSwitch2;

  Pose2D maxVel; // max velocities for fwd, side, and turn
  Pose2D maxVec; // max vec for fwd, side, and turn
  float maxBack; // max back velocity
  Pose2D maxVelAccel; // max acceleration
  Pose2D maxVelDecel; // max deceleration

  float bodyTiltTarget; // desired body tilt
  float bodyTiltTargetFullFwd; // desired body tilt
  float bodyTiltTargetFullBack; // desired body tilt
  float bodyTiltTargetUpdateRate;
  float bodyTiltStop; // body tilt over which to stop at
  float bodyTiltNoAccel; // body tilt over which to not accelerate

  float startLength; // num frames to damp for start

  float balanceKneePitch; // balance factor for knee pitch // formerly balanceGyro1
  float balanceHipPitch; // balance factor for hip pitch // formerly balanceGyro2
  float balanceHipRoll; // balance factor for hip roll // formerly balanceGyro3
  float balanceAnkleRoll; // balance factor for ankle roll  // formerly balanceGyro4

  int sensorDelayProp; // #frames delayed in proprioception
  int sensorDelayFsr; // #frames delayed for fsrs

  bool useAngleLoop;

  Pose2D targetPosErrorAllowed;
  Pose2D targetVelErrorAllowed;

  // ODOMETRY
  Pose2D odometryFactors;

  float bodyTiltNoKick; // body tilt over which to delay the kick
  WalkKickSpline frontKick;
  WalkKickSpline sideKick;

  // hacks
  bool useAnkleTiltHack;
  float ankleTiltHackSwingAmount;
  float ankleTiltHackStanceAmount;
};

#endif /* end of include guard: HTWKWALKPARAMETERS_GWNK1KUR */
