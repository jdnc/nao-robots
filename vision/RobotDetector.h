#ifndef JERSEY_ROBOTDETECTOR_H
#define JERSEY_ROBOTDETECTOR_H

#include <memory/TextLogger.h>
#include <vision/ObjectDetector.h>
#include <vision/BlobDetector.h>
#include <vision/Classifier.h>

class RobotDetector : public ObjectDetector {
 public:
  RobotDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector);
  void init(TextLogger* tl){textlogger = tl;};
  std::list<Blob*> getPinkRobots();
  std::list<Blob*> getBlueRobots();
 private:
  TextLogger* textlogger;
  Classifier* classifier_;
  BlobDetector* blob_detector_;
};
#endif
