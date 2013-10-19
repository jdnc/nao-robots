#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <math/Point.h>
#include <list>
#include <vector>

#include <memory/TextLogger.h>
#include <constants/ImageConstants.h>
#include <constants/VisionConstants.h>
#include <vision/structures/VisionPoint.h>
#include <vision/structures/VisionParams.h>
#include <vision/structures/HorizonLine.h>
#include <vision/enums/Colors.h>
#include <vision/ColorTableMethods.h>
#include <vision/VisionBlocks.h>
#include <vision/structures/FocusArea.h>
#include <vision/Macros.h>

class Classifier {
 public:
  Classifier(const VisionBlocks& vblocks, const VisionParams& vparams, const ImageParams& iparams, const Camera::Type& camera);
  ~Classifier();
  void init(TextLogger* tl){textlogger = tl;};
  bool classifyImage(unsigned char*);
  void classifyImage(const std::vector<FocusArea>& areas, unsigned char* colorTable);
  void classifyImage(const FocusArea& area, unsigned char* colorTable);
  void constructRuns();
  void connectComponents(uint16_t c);
  inline Color xy2color(int x, int y) {
    return (Color)segImg_[y * iparams_.width + x];
  }
  VisionPoint ***horizontalPoint, ***verticalPoint;
  uint32_t **horizontalPointCount, **verticalPointCount;
  void setStepScale(int hscale, int vscale);
  void getStepSize(int& hstep, int& vstep);
  void getStepScale(int& hscale, int& vscale);
  static uint16_t range_sum(uint16_t x, uint16_t w){ //inline for efficiency
  //returns sum of all integers in interval[x, x+w)
  return (w*(2*x + w-1)/2);
  }
 private:
  bool setImagePointers();
  const VisionBlocks& vblocks_;
  const VisionParams& vparams_;
  const ImageParams& iparams_;
  const Camera::Type& camera_;
  bool initialized_;
  TextLogger* textlogger;

  unsigned char* img_;
  unsigned char* segImg_, *segImgLocal_;
  uint16_t vstep_, hstep_, vscale_, hscale_;
  HorizonLine horizon_;
  unsigned char* colorTable_;
  bool fromLog_;
};
#endif
