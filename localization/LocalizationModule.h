#ifndef LOCALIZATION_MODULE_H
#define LOCALIZATION_MODULE_H

#include <Module.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/TeamPacketsBlock.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/JointBlock.h>
#include <memory/BehaviorBlock.h>
#include <memory/ProcessedSonarBlock.h>
#include <memory/DelayedLocalizationBlock.h>
#include <localization/Particle.h>

#define RESAMPLE_FREQ 1
#define RANDOM_WALK_FREQ 1
#define DEGRADE_FACTOR 0.99
#define DELTA_DIST 10
#define DELTA_ANG (DEG_T_RAD * 45)

class LocalizationModule: public Module  {
 public:
  void specifyMemoryDependency();
  void specifyMemoryBlocks();
  void initSpecificModule();
  void processFrame();
 private:
  void updatePose();
  void updateParticlesFromOdometry();
  void resetParticles();
  void setParticleProbabilities(float newProb);
  void randomWalkParticles();
  void copyParticles();
  Particle particles_[NUM_PARTICLES];
  WorldObjectBlock* worldObjects;
  LocalizationBlock* localizationMem;
  TeamPacketsBlock* teamPacketsMem;
  FrameInfoBlock* frameInfo;
  OdometryBlock* odometry;
  RobotStateBlock* robotState;
  GameStateBlock* gameState;
  JointBlock* jointAngles;
  BehaviorBlock* behaviorMem;
  ProcessedSonarBlock* processedSonar;
  DelayedLocalizationBlock* delayedLocalization;
};

#endif
