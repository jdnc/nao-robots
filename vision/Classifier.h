#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include<opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>




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

  VisionPoint ***horizontalPoint, ***verticalPoint;
  uint32_t **horizontalPointCount, **verticalPointCount;

  bool classifyImage(unsigned char*);

  void constructRuns(int colorFlags = ~0);
  void setHorizon(HorizonLine);
  void preProcessPoints();
  void preProcessGoalPoints();
  bool didHighResBallScan;


  void setStepScale(int,int);
  void getStepSize(int&,int&);
  void getStepScale(int&,int&);
  bool startHighResGoalScan();
  bool startHighResBallScan();
  void completeHighResScan();
  inline Color xy2color(int x, int y) {
    return (Color)segImg_[y * iparams_.width + x];
  }
 private:
  void clearPreviousHighResScans();
  void classifyImage(const std::vector<FocusArea>& areas, unsigned char*);
  void classifyImage(const FocusArea& area, unsigned char*);
  void constructRuns(const std::vector<FocusArea>& areas, int colorFlags);
  void constructRuns(const FocusArea& area, int colorFlags);
 

  bool startHighResScan(Color, int hStepScale = 0, int vStepScale = 0);
  void clearPoints(int colorFlags);
  bool prepareFocusAreas(std::vector<FocusArea>& areas, Color c);

  void completeHorizontalRun(uint8_t &hRunClr, int y, int hStartX, int finX);
  void completeVerticalRun(uint8_t &runClr, int x, int vStartY);

  bool setImagePointers();
  
  const VisionBlocks& vblocks_;
  const VisionParams& vparams_;
  const ImageParams& iparams_;
  const Camera::Type& camera_;
  bool initialized_;
  bool doingHighResScan_;
  bool bodyExclusionAvailable;
  TextLogger* textlogger;

  // For excluding parts of the shoulder in the body
  float bodyExclusionSlope[NUM_BODY_EXCL_POINTS];
  float bodyExclusionOffset[NUM_BODY_EXCL_POINTS];

  unsigned char* img_;
  unsigned char* segImg_, *segImgLocal_;
  uint16_t vstep_, hstep_, vscale_, hscale_;
  HorizonLine horizon_;
  unsigned char* colorTable_;
  bool* pointScanned;
  uint16_t* vGreenPosition;
  uint16_t* hGreenPosition;
  bool fromLog_;
};
#endif
