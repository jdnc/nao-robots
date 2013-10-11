#include "HTWKWalkParameters.h"
#include <math/Geometry.h>

float fwd_tilt_gyro_factor = 0.4;

HTWKWalkParameters::HTWKWalkParameters():
  numFrames(44),
  frameInc(1),
  stanceSwitch1(11),
  stanceSwitch2(33),
  maxVel(0.4,360*0.75,90), // rot, fwd, side
  maxVec(0.12,0.16 * 0.75,0.04), // rot, fwd, side
  maxBack(-90),
  maxVelAccel(0.008,1,4),
  maxVelDecel(0.08,72,18),
  bodyTiltTarget(DEG_T_RAD * 2.5),
  bodyTiltTargetFullFwd(DEG_T_RAD * 2.5),
  bodyTiltTargetFullBack(DEG_T_RAD * -2.5),
  bodyTiltTargetUpdateRate(0.01),
  bodyTiltStop(0.14),
  bodyTiltNoAccel(0.07),
  startLength(20),
  balanceKneePitch(fwd_tilt_gyro_factor * -0.5f),
  balanceHipPitch(fwd_tilt_gyro_factor * 0.265f),
  balanceHipRoll(0.78f),
  balanceAnkleRoll(-0.379f),
  sensorDelayProp(7),
  sensorDelayFsr(12),
  useAngleLoop(true),
  targetPosErrorAllowed(DEG_T_RAD * 180,10,10),
  targetVelErrorAllowed(DEG_T_RAD * 180,10,10),
  odometryFactors(1.923,1.9,3.226), // rot, fwd, side
  bodyTiltNoKick(0.07),
  useAnkleTiltHack(false),
  ankleTiltHackSwingAmount(0),
  ankleTiltHackStanceAmount(0)
{
  frontKick.numPts = 0;
  sideKick.numPts = 0;
}
