#ifndef KICKS_H
#define KICKS_H

#include <string>

// ORDER of kicks does matter, if we're still using strategy.selectFirstValidKick
enum kicks {
  FwdLongLargeGapKick,
  FwdLongSmallGapKick,
  FwdSuperStraightKick,
  FwdLongStraightKick,
  FwdMediumStraightKick,
  FwdPass5Kick,
  FwdPass4Kick,
  FwdPass3Kick,
  FwdPass2Kick,
  FwdShortStraightKick,
  Dribble,
  WalkKickFront,
  WalkKickLeftward,
  WalkKickRightward,
  WalkKickLeftwardSide,
  WalkKickRightwardSide,
  NUM_KICKS
};

const std::string kickNames[] = { 
  "FwdLongLargeGapKick",
  "FwdLongSmallGapKick",
  "FwdSuperStraightKick",
  "FwdLongStraightKick",
  "FwdMediumStraightKick",
  "FwdPass5Kick",
  "FwdPass4Kick",
  "FwdPass3Kick",
  "FwdPass2Kick",
  "FwdShortStraightKick",
  "Dribble",
  "WalkKickFront",
  "WalkKickLeftward",
  "WalkKickRightward",
  "WalkKickLeftwardSide",
  "WalkKickRightwardSide",
};

// speech text is in cfgkick.lua

#define LEFTLEG 0
#define RIGHTLEG 1

#endif
