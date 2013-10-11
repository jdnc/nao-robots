#ifndef LINE_DETECTOR_H
#define LINE_DETECTOR_H

#include <memory/TextLogger.h>
#include <vision/ObjectDetector.h>
#include <vision/BlobDetector.h>
#include <vision/Classifier.h>
#include <vision/structures/FieldLine.h>
#include <vision/structures/LinePoint.h>

class LineDetector : public ObjectDetector {
 public:
  LineDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector);
  void init(TextLogger* tl){textlogger = tl;};
  FieldLine** fieldLines;
  int FieldLinesCounter;
 private:
  TextLogger* textlogger;
  Classifier* classifier_;
  BlobDetector* blob_detector_;
};

#endif
