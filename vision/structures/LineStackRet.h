#ifndef LINESTACKRET_H
#define LINESTACKRET_H
#define MAX_BLOBS_PER_LINE 50
struct LineStackRet {
  bool isLine;
  uint16_t pointCount;
  uint16_t lbIndex[MAX_BLOBS_PER_LINE];
  uint16_t lbCount;
};
#endif
