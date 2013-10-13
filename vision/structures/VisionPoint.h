#ifndef VISIONPOINT_H
#define VISIONPOINT_H

#include <inttypes.h>

struct VisionPoint {
  uint16_t xi, xf, dx, yi, yf, dy;
  uint16_t lbIndex;
  bool isValid;
  struct VisionPoint* parent; // poiints to the parent of a region
  struct VisionPoint* next; // points to the next run in its region
};
#endif
