#include "CrossDetector.h"
#define getself() (&vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF])

CrossDetector::CrossDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) :
  DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) {
}
