#ifndef ROBOTSTATE_
#define ROBOTSTATE_

#include "MemoryBlock.h"
#include <common/States.h>
#include <common/Roles.h>

// Man this is a crappy block (but I don't have a great answer)
// ... hopefully we can find somewhere else for it or add something more to the block


struct RobotStateBlock : public MemoryBlock {
public:
  RobotStateBlock():
    WO_SELF(2),
    team_(TEAM_BLUE),
    role_(WO_SELF)
  {
    header.version = 6;
    header.size = sizeof(RobotStateBlock);
    robot_id_ = -1;
    team_changed_ = true;
    ignore_comms_ = false;
  }
  int WO_SELF;

  int team_;
  bool team_changed_;

  int robot_id_;  // Which robot serial number are we, -1 = unknown or sim

  int role_;

  bool ignore_comms_;
  double clock_offset_;
};

#endif 
