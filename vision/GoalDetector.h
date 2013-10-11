#ifndef GOALDETECTOR_H
#define GOALDETECTOR_H

#include <memory/TextLogger.h>
#include <vision/ObjectDetector.h>
#include <vision/BlobDetector.h>
#include <vision/LineDetector.h>
#include <vision/Classifier.h>
#include <vision/structures/FieldLine.h>

class GoalDetector : public ObjectDetector {
 public:
  GoalDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector, LineDetector*& line_detector);
  void init(TextLogger* tl){textlogger = tl;};
  FieldLine** yellowPosts; // Goals
  int YellowPostCounter;
 private:
  TextLogger* textlogger;
  Classifier* classifier_;
  BlobDetector* blob_detector_;
  LineDetector* line_detector_;
};

#endif
