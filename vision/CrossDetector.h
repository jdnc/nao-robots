#ifndef CROSS_DETECTOR_H
#define CROSS_DETECTOR_H

#include <vision/ObjectDetector.h>
#include <vision/Classifier.h>
#include <vision/BlobDetector.h>
#include <memory/TextLogger.h>

class CrossDetector : public ObjectDetector {
  public:
    CrossDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector);
    void init(TextLogger* tl){textlogger = tl;};

  private:
    TextLogger* textlogger;
    Classifier* classifier_;
    BlobDetector* blob_detector_;
};

#endif
