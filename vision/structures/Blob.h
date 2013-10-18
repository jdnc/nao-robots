#ifndef BLOB_H
#define BLOB_H

#include <constants/VisionConstants.h>
#include <vector>
#include <inttypes.h>

struct Blob {
  uint16_t xi, xf, dx, yi, yf, dy;
  uint16_t lpCount;
  //std::vector<uint32_t> lpIndex;
  float diffStart;
  float diffEnd;
  float doubleDiff;
  uint16_t widthStart;
  uint16_t widthEnd;
  uint16_t avgX;
  uint16_t avgY;
  float avgWidth;
  float correctPixelRatio;
  bool invalid;

  //Blob() : lpIndex(MAX_BLOB_VISIONPOINTS, 0) { };
 //Blob(){ };
};

bool sortBlobAreaPredicate(const Blob& left, const Blob& right);

#endif
