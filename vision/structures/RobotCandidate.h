#ifndef ROBOT_CANDIDATE_H
#define ROBOT_CANDIDATE_H

#include <vision/structures/Position.h>
#include <vision/structures/Blob.h>
#include <vision/enums/Colors.h>

struct RobotCandidate {
  float centerX;
  float centerY;
  float chestButtonHeight;
  float width, height;
  float widthDistance, heightDistance, kinematicsDistance;
  float jerseyColorPercent, greenWhitePercent, whitePercent, correctPercent;
  float worldHeight;
  int feetY, feetX;
  float confidence;
  bool feetMissing;
  Blob* blob;
  Color color;

  Position relTorso, relFeet;
  Position relPosition;

  RobotCandidate() : feetMissing(false), blob(NULL) { }

  static bool sortPredicate(const RobotCandidate& left, const RobotCandidate& right);
};

#endif
