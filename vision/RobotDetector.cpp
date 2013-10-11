#include <vision/RobotDetector.h>

using namespace Eigen;

RobotDetector::RobotDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) :
  DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) { 
}

std::list<Blob*> RobotDetector::getPinkRobots() {
  return std::list<Blob*>();
}

std::list<Blob*> RobotDetector::getBlueRobots() {
  return std::list<Blob*>();
}
