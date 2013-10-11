#include <vision/LineDetector.h>

LineDetector::LineDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) :
  DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) {
  FieldLinesCounter = 0;
  fieldLines = new FieldLine * [MAX_FIELDLINES];
  for (int i = 0; i < MAX_FIELDLINES; i++) {
    fieldLines[i] = new FieldLine();
    fieldLines[i]->id = i;
    fieldLines[i]->TranPointsArray = new LinePoint * [MAX_POINTS_PER_LINE];
    fieldLines[i]->PointsArray = new LinePoint * [MAX_POINTS_PER_LINE];

    for (int j = 0; j < MAX_POINTS_PER_LINE; j++)
      fieldLines[i]->TranPointsArray[j] = new LinePoint();

    for (int j = 0; j < MAX_POINTS_PER_LINE; j++)
      fieldLines[i]->PointsArray[j] = new LinePoint();
  }
}
