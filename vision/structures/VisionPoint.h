#ifndef VISIONPOINT_H
#define VISIONPOINT_H

#include <inttypes.h>
#include<vision/structures/Blob.h>

struct VisionPoint {
  uint16_t xi, xf, dx, yi, yf, dy;
  uint16_t lbIndex;
  bool isValid;
  uint32_t avgX;
  uint32_t avgY;
  struct VisionPoint* parent; // poiints to the parent of a region
  //struct VisionPoint* next; // points to the next run in its region
  int pixelCount; // stores the number of children having this as parent
  Blob *parentBlob;
};
#endif
