#include <vision/GoalDetector.h>

GoalDetector::GoalDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector, LineDetector*& line_detector) : 
  DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector), line_detector_(line_detector) {
}
