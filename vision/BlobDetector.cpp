#include <vision/BlobDetector.h>

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier) : 
  DETECTOR_INITIALIZE, classifier_(classifier) {
    horizontalBlob.resize(NUM_COLORS);
    verticalBlob.resize(NUM_COLORS);
}
