#ifndef BALLDETECTOR_H
#define BALLDETECTOR_H

#include <memory/TextLogger.h>
#include <constants/VisionConstants.h>
#include <common/Field.h>
#include <vision/BlobDetector.h>
#include <vision/ObjectDetector.h>
#include <vision/structures/BallCandidate.h>
#include <vision/structures/HorizonLine.h>
#include <vision/Macros.h>
#include <vision/estimators/BallEstimator.h>
#include <math/Point.h>

class BallDetector : public ObjectDetector {
 public:
  BallDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector);
  void init(TextLogger* tl){textlogger = tl;};

  uint16_t bestBallCandIndex;
  void findBall();
  float getDirectDistanceByBlobWidth(float,int,int);
  float getDirectDistanceByKinematics(int,int);
  float getDirectDistance(BallCandidate* candidate);
  BallCandidate* candidates_[MAX_BALL_CANDS];
  uint16_t ballCandCount;
  bool bestCandidateFound();
  void setHorizon(HorizonLine);
 private:
  bool checkAndSelectBall();
  BallCandidate* computeCandidateProbabilities(std::vector<BallCandidate*> candidates);
  BallCandidate* sanityCheckBallCands(std::vector<BallCandidate*> candidates);
  float checkPinkPct(int,int,int,int);
  bool checkSurroundingPink(int);
  float checkBelowGreenWhitePct(BallCandidate* candidate);
  float checkColorsInCircleFit(BallCandidate* candidate);
  BallCandidate* formBallCandidate(Blob* blob, int ballCandIndex);
  std::vector<BallCandidate*> formBallCandidates(std::vector<Blob*> blobs);
  void fitCircleToPoints(uint16_t*, uint16_t*, uint16_t, float*, float*, float*, float*);
  void getBlobContour(Blob* blob, uint16_t *xinit, uint16_t *xfinal);
  bool setBestBallCandidate(BallCandidate* candidate);
  
  Classifier*& classifier_;
  BlobDetector*& blob_detector_;
  TextLogger* textlogger;
  HorizonLine horizon_;

  BallEstimator estimator_;
};

#endif
