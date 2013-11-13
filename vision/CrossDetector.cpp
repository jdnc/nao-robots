#include "CrossDetector.h"
#define getself() (&vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF])

CrossDetector::CrossDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) :
  DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) {
  estimator_.setMean(
    1.0, 1.0, 1.0,
    1.0,      1.0,
    1.0, 1.0, 1.0,
    0.0,
    PENALTY_MARK_SIZE, PENALTY_MARK_SIZE
  );
  estimator_.setStdDev(
    0.2, 0.2, 0.2,
    0.2,      0.2,
    0.2, 0.2, 0.2,
    1000.0,
    100.0, 100.0
  );
}

void CrossDetector::detectCrosses() {
  int mergeCount = 100;
  uint16_t mergeIndex[mergeCount];
  BlobCollection blobsUnmerged, blobs;
  blob_detector_->formBlobs(blobs, c_WHITE);
  blob_detector_->calculateBlobData(blobs, c_WHITE);
  int mergeAttempts = 1;
  for(int i = 0; i < mergeAttempts; i++) {
    blobsUnmerged = blobs; blobs.clear();
    blob_detector_->mergeBlobs(blobsUnmerged, mergeIndex, mergeCount);
    for(unsigned int i = 0; i < blobsUnmerged.size(); i++) {
      Blob& blob = blobsUnmerged[i];
      if(mergeIndex[i] == mergeCount) {
        blobs.push_back(blob);
      }
    }
  }
  for(int i = 0; i < NUM_CROSSES; i++) {
    WorldObject* cross = &vblocks_.world_object->objects_[i + CROSS_OFFSET];
    detectCross(blobs, cross);
  }
}

void CrossDetector::detectCross(BlobCollection& blobs, WorldObject* cross) {
  int hstep, vstep;
  classifier_->getStepSize(hstep, vstep);

  Pose2D self(getself()->orientation, getself()->loc.x, getself()->loc.y); // release
  //Pose2D self(0, 250 - HALF_FIELD_X, 0); // Standard goalie position
  for(uint16_t i = 0; i < blobs.size(); i++) {
    Blob& blob = blobs[i];
    int left = blob.xi, right = blob.xf, top = blob.yf, bottom = blob.yi;
    int width = right - left, height = top - bottom;
    int wideXRange = std::max(100, width), narrowXRange = std::max(width / 2, 50);
    int wideYRange = std::max(80, height), narrowYRange = std::max(height / 2, 30);
    //printf("x: [%i,%i,%i,%i], y: [%i,%i,%i,%i]\n", 
        //left - wideXRange, left - narrowXRange, right + narrowXRange, right + wideXRange,
        //bottom - wideYRange, bottom - narrowYRange, top + narrowYRange, top + wideYRange
        //);

    Position 
      lpos = cmatrix_.getWorldPosition(left, (top + bottom) / 2),
      rpos = cmatrix_.getWorldPosition(right, (top + bottom) / 2),
      tpos = cmatrix_.getWorldPosition((right + left) / 2, top),
      bpos = cmatrix_.getWorldPosition((right + left) / 2, bottom),
      relCenter = cmatrix_.getWorldPosition((right + left) / 2, (top + bottom) / 2);

    Pose2D absCenter = Pose2D(relCenter.x, relCenter.y).relativeToGlobal(self);

    double wWidth = (lpos - rpos).abs(), wHeight = (tpos - bpos).abs();
    double pDist = (absCenter.translation - Vector2<float>(cross->loc.x, cross->loc.y)).abs();
    if(vblocks_.game_state->state != PLAYING) pDist = 0; // This shouldn't be used when not in playing, helps with debugging

    double gtopleft = getGreenPercentage(left - wideXRange, left - narrowXRange, top + narrowYRange, top + wideYRange, hstep, vstep);
    double gtopmid = getGreenPercentage(left - narrowXRange, right + narrowXRange, top + narrowYRange, top + wideYRange, hstep, vstep);
    double gtopright = getGreenPercentage(right + narrowXRange, right + wideXRange, top + narrowYRange, top + wideYRange, hstep, vstep);

    double gleft = getGreenPercentage(left - wideXRange, left - narrowXRange, bottom - narrowYRange, top + narrowYRange, hstep, vstep);
    double gright = getGreenPercentage(right + narrowXRange, right + wideXRange, bottom - narrowYRange, top + narrowYRange, hstep, vstep);

    double gbottomleft = getGreenPercentage(left - wideXRange, left - narrowXRange, bottom - wideYRange, bottom - narrowYRange, hstep, vstep);
    double gbottommid = getGreenPercentage(left - narrowXRange, right + narrowXRange, bottom - wideYRange, bottom - narrowYRange, hstep, vstep);
    double gbottomright = getGreenPercentage(right + narrowXRange, right + wideXRange, bottom - wideYRange, bottom - narrowYRange, hstep, vstep);

    visionLog((47, "Blob %i (%i,%i to %i,%i) chosen for cross detection", i, blob.xi, blob.yi, blob.xf, blob.yf));
    visionLog((47, "Self at %2.f,%2.f, candidate at %2.f,%2.f", self.translation.x, self.translation.y, absCenter.translation.x, absCenter.translation.y));

    double prob = estimator_.getLikelihood(
     gtopleft,
     gtopmid,
     gtopright,
     gleft,
     gright,
     gbottomleft,
     gbottommid,
     gbottomright,
     pDist,
     wWidth,
     wHeight
    );
    //estimator_.printLast();
    estimator_.logLast(47, textlogger);

    if(prob > .4) {
      visionLog((47, "Blob %i selected as cross", i));
      setCrossObject(blob, cross);
    }
  }
}

void CrossDetector::correctRanges(int& xmin, int& xmax, int& ymin, int& ymax, int hstep, int vstep) {
  xmin = std::max(0, xmin);
  xmax = std::min(iparams_.width - 1, xmax);
  ymin = std::max(0, ymin);
  ymax = std::min(iparams_.height - 1, ymax);
  xmin = xmin - (xmin % hstep);
  ymin = ymin - (ymin % vstep);
}

float CrossDetector::getGreenPercentage(int xmin, int xmax, int ymin, int ymax, int hstep, int vstep) {
  //printf("Scanning from x:%i->%i, y:%i->%i\n", xmin, xmax, ymin, ymax);
  visionLog((47, "Scanning from x:%i->%i, y:%i->%i", xmin, xmax, ymin, ymax));
  correctRanges(xmin,xmax,ymin,ymax,hstep,vstep);
  int numTotal = 0, numGreen = 0;
  for(int x = xmin; x <= xmax; x += hstep) {
    for(int y = ymin; y <= ymax; y += vstep) {
      Color c = classifier_->xy2color(x,y);
      numTotal++;
      if(c == c_FIELD_GREEN) numGreen++;
      if(c == c_WHITE) return 0.0; // There shouldn't be any white around the cross
    }
  }
  if(numTotal == 0) return 0.0; // No false positives, no regrets
  return (float)numGreen / numTotal;
}

float CrossDetector::getDistanceByBlobWidth(float dx) {
  dx *= 640.0f / iparams_.width; // Tuned with a width of 640
  return .9f * 16765.0f * powf(dx, -0.769f);
}

void CrossDetector::setCrossObject(Blob& blob, WorldObject* cross) {
  int left = blob.xi, right = blob.xf, yval = blob.yi;
  int centerX = left + (right - left) / 2;
  Position p = cmatrix_.getWorldPosition(centerX, yval);
  cross->visionDistance = cmatrix_.groundDistance(p);
  cross->visionBearing = cmatrix_.bearing(p);
  cross->visionElevation = cmatrix_.elevation(p);
  cross->seen = true;
  cross->fromTopCamera = true;
  cross->imageCenterX = centerX;
  cross->imageCenterY = yval;
  cross->visionConfidence = 1.0;
  cross->frameLastSeen = vblocks_.frame_info->frame_id;
  visionLog((47, "Cross identified at %i, %i", centerX, yval));
}
