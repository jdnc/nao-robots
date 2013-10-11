#ifndef BALLDETECTOR_H
#define BALLDETECTOR_H

#include <memory/TextLogger.h>
#include <vision/BlobDetector.h>
#include <vision/ObjectDetector.h>
#include <vision/Classifier.h>
#include <vision/structures/BallCandidate.h>

class BallDetector : public ObjectDetector {
 public:
  BallDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector);
  void init(TextLogger* tl){textlogger = tl;};
  BallCandidate candidates[MAX_BALL_CANDS];
  int candidateCount;
 private:
  TextLogger* textlogger;
  Classifier* classifier_;
  BlobDetector* blob_detector_;
};

#endif
