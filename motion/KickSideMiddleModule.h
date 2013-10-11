#ifndef KICKSIDEMIDDLE_MODULE_H
#define KICKSIDEMIDDLE_MODULE_H

#include <string>
#include <iostream>
#include <Module.h>
#include <common/RobotInfo.h>
#include <vector>

#include <motion/SpecialMotionModule.h>

using namespace std;

class KickSideMiddleModule: public SpecialMotionModule {
 public:
  KickSideMiddleModule();

  bool isKickingSide();
  void initSideKick(float kick_vel,bool l_r);
  void processSideKick();


 private:
  void prepareKick();
  bool doing_side_kick_;
  float kick_vel;
  bool init_sideKickMotion;
};

#endif /* end of include guard: SIDEKICK_MODULE */
