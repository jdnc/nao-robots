#ifndef JERSEY_ROBOTDETECTOR_H
#define JERSEY_ROBOTDETECTOR_H

#include <vision/ObjectDetector.h>
#include <memory/TextLogger.h>
#include <memory/TextLogger.h>
#include <vision/CameraMatrix.h>
#include <vision/structures/Blob.h>
#include <vision/enums/Colors.h>
#include <vision/Macros.h>
#include <vision/BlobDetector.h>
#include <vision/Classifier.h>
#include <vision/structures/RobotCandidate.h>
#include <vision/estimators/RobotEstimator.h>

class JerseyRobotDetector : public ObjectDetector {
 public:
  JerseyRobotDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector);
  void init(TextLogger* tl){textlogger = tl;};
  void detectBlueRobots();
  void detectPinkRobots();
  void detectRobotCluster();
  std::list<Blob*> getBlueRobots();
  std::list<Blob*> getPinkRobots();
 private:
  void detectRobots(Color c);
  std::vector<RobotCandidate> formRobotCandidates(std::vector<Blob*>& blobs, Color c);
  void computeCandidateProbabilities(std::vector<RobotCandidate>& candidates);
  void selectRobots(std::vector<RobotCandidate>& candidates);
  float getDistanceByWidth(float width);
  float getDistanceByHeight(float height);
  void fillColorPercents(RobotCandidate& candidate);
  void fillFeet(RobotCandidate& candidate);
  Classifier*& classifier_;
  BlobDetector*& blob_detector_;
  TextLogger* textlogger;
  std::list<Blob*> blueRobots_, pinkRobots_;
  RobotEstimator estimator_;
};
#endif
