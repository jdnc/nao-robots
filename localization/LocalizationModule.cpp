#include <localization/LocalizationModule.h>

void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("team_packets");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("vision_odometry");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_joint_angles");
  requiresMemoryBlock("behavior");
  requiresMemoryBlock("vision_processed_sonar");
  requiresMemoryBlock("delayed_localization");
}

void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(worldObjects,"world_objects");
  getOrAddMemoryBlock(localizationMem,"localization");
  getOrAddMemoryBlock(teamPacketsMem,"team_packets");
  getOrAddMemoryBlock(frameInfo,"vision_frame_info");
  getOrAddMemoryBlock(odometry,"vision_odometry");
  getOrAddMemoryBlock(robotState,"robot_state");
  getOrAddMemoryBlock(gameState,"game_state");
  getOrAddMemoryBlock(jointAngles,"vision_joint_angles");
  getOrAddMemoryBlock(behaviorMem,"behavior");
  getOrAddMemoryBlock(processedSonar,"vision_processed_sonar");
  getOrAddMemoryBlock(delayedLocalization,"delayed_localization");
}


void LocalizationModule::initSpecificModule(){
}

void LocalizationModule::processFrame() {
}
