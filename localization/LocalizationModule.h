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

class LocalizationModule: public Module  {
 public:
  void specifyMemoryDependency();
  void specifyMemoryBlocks();
  void initSpecificModule();
  void processFrame();
 private:
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
