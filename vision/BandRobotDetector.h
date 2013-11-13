#ifndef ROBOTDETECTOR_H
#define ROBOTDETECTOR_H

#include <vision/ObjectDetector.h>
#include <memory/TextLogger.h>
#include <memory/TextLogger.h>
#include <vision/CameraMatrix.h>
#include <vision/structures/Blob.h>
#include <vision/enums/Colors.h>
#include <vision/Macros.h>
#include <vision/BlobDetector.h>
#include <vision/Classifier.h>

class BandRobotDetector : public ObjectDetector {
 public:
  BandRobotDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector);
  void init(TextLogger* tl){textlogger = tl;};
  void estimateBandBlobData();
  float estimateBandDistanceByWidth(Blob*);
  float estimateBandDistanceByGreenBelow(Blob*);
  float estimateBandDistance(Blob*);
  void sanitizeBandCandidate(Blob*,int);
  void sanitizeBandCandidateOnElevation(Blob*,float);
  void detectBlueRobots();
  void detectPinkRobots();
  void detectRobotCluster();
  std::list<Blob*> getBlueRobots();
  std::list<Blob*> getPinkRobots();
 private:
  Classifier*& classifier_;
  BlobDetector*& blob_detector_;
  TextLogger* textlogger;
  std::list<Blob*> blueRobots_, pinkRobots_;
};
#endif
