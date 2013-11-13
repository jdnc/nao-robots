#ifndef LINE_DETECTOR_H
#define LINE_DETECTOR_H

#include <memory/TextLogger.h>
#include <vision/ObjectDetector.h>
#include <common/Field.h>
#include <vision/structures/FieldLine.h>
#include <vision/structures/CornerPoint.h>
#include <vision/structures/LineStackInfo.h>
#include <vision/structures/LineStackRet.h>
#include <vision/structures/Blob.h>
#include <vision/structures/VisionPoint.h>
#include <vision/structures/VisionParams.h>
#include <vision/enums/Colors.h>
#include <vision/enums/VisionBodyPoints.h>
#include <vision/CameraMatrix.h>
#include <constants/VisionConstants.h>
#include <vision/BlobDetector.h>
#include <vision/Classifier.h>
#include <vision/Macros.h>
#include <vision/structures/HorizonLine.h>

class LineDetector : public ObjectDetector {
 public:
  LineDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector);
  void init(TextLogger* tl){textlogger = tl;};

  void HorizontalBlobSort();
  void VerticalBlobSort();
  void FindHorzLines(FieldLine**,unsigned char);
  void FindVertLines(FieldLine**,unsigned char);
  LineStackRet recurseCheckHorizontal(uint16_t,LineStackInfo,unsigned char);
  LineStackRet recurseCheckVertical(uint16_t,LineStackInfo,unsigned char);
  void FormLines(unsigned char);
  void RemoveLinesFromBody();
  void differentiateCurves();
  bool formCircle(float,float,float,float,float,float,FieldLine*,float*,float*,float*,float);
  bool formPenaltyCross(float,float,float,float,int);
  void CalLineDetails(FieldLine*);
  void generateTransformedImage();
  void mergeLines();
  bool mergeOverlap(FieldLine*,FieldLine*);
  bool mergeFieldLines(FieldLine*,FieldLine*);
  void FilterLines();
  bool FilterLine(FieldLine*);
  void DetermineEndPoints();
  void IdentifyLines();
  void DecodeLines();
  void FormCorners();
  void DecodeCorners();
  void setHorizon(HorizonLine);

  FieldLine** fieldLines;
  int FieldLinesCounter;
  int currentLineCounter;

 private:
  float getHeadTilt();
  float getHeadPan();

  Classifier*& classifier_;
  BlobDetector*& blob_detector_;

  TextLogger* textlogger;

  CornerPoint ** cornerPoints;

  int CornerPointCounter;
  int TotalValidLines;

  VisionPoint ***horizontalPoint, ***verticalPoint;
  uint32_t **horizontalPointCount, **verticalPointCount;
  HorizonLine horizon_;
};

#endif
