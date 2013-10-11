#ifndef INTERFACEINFO_R3BSIHAC
#define INTERFACEINFO_R3BSIHAC

#include <common/RobotInfo.h>

enum WALK_TYPES {
  AL_WALK,
  UT_NAO_DEVILS_WALK,
  HTWK_WALK,
  BHUMAN_WALK
};
extern const bool USE_AL_MOTION;
extern const int WALK_TYPE;

extern const int robot_joint_signs[NUM_JOINTS];
extern const int spark_joint_signs[NUM_JOINTS];

enum CoreType {
  CORE_ROBOT,
  CORE_SIM,
  CORE_TOOL,
  CORE_TOOLSIM,
  CORE_INIT,
  CORE_TOOL_NO_VISION
};

#endif
