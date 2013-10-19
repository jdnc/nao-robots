#ifndef BLOBDETECTOR_H
#define BLOBDETECTOR_H


#include <memory/TextLogger.h>
#include <constants/VisionConstants.h>
#include <vision/Classifier.h>
#include <vision/structures/Blob.h>
#include <vision/structures/VisionParams.h>
#include <vision/Macros.h>
#include <vision/ObjectDetector.h>
#include <vision/enums/Colors.h>
  
typedef std::vector<Blob> BlobCollection;

struct ProbBeacon{
  Blob * top;
  Blob * bottom;
  Color topColor;
  Color botColor;
  double likely; //probability estimate from aspect ratio
};

class BlobDetector : public ObjectDetector {
 public:
  BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier);
  void init(TextLogger* tl){textlogger = tl;};
  std::vector<BlobCollection> horizontalBlob, verticalBlob;
  void formBlobs(uint16_t c);
  void mergeBlobs(BlobCollection &, uint16_t, uint16_t);
  void findBeacons2();
  void findProbBeacons(BlobCollection &c1Blobs, BlobCollection &c2Blobs, Color c1, Color c2, vector<ProbBeacon>& ProbBeacons);
  void removeOverlapping(vector<ProbBeacon> &ProbBeacons, BlobCollection&, BlobCollection&, BlobCollection&);
  WorldObject * getBeaconFromColors(Color top, Color bottom);
 private:
  Classifier*& classifier_;
  TextLogger* textlogger;
};


#endif
