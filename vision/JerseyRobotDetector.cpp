#include <vision/JerseyRobotDetector.h>
#include <iomanip>

using namespace Eigen;

JerseyRobotDetector::JerseyRobotDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) :
  DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) { 
  estimator_.setMean(1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 0.4, 0.3, 1.0, ROBOT_CHEST_HEIGHT);  
  estimator_.setStdDev(
   0.5, // w/h
   0.75, // h/w
   0.7, // kdist/wdist
   0.7, // kdist/hdist
   0.3, // torso/feet discrepancy
   0.6, // jersey %
   0.3, // green white %
   0.3, // white %
   0.6,  // correct %
   100.0 // chest height
  );
}

void JerseyRobotDetector::detectBlueRobots() {
  blueRobots_.clear();
  detectRobots(c_BLUE);
}

void JerseyRobotDetector::detectPinkRobots() {
  pinkRobots_.clear();
  detectRobots(c_PINK);
}

void JerseyRobotDetector::detectRobots(Color c) {
  blob_detector_->formBlobs(c);
  blob_detector_->calculateBlobData(c);
  uint16_t mergeIndex[MAX_LINE_BLOBS];
  blob_detector_->mergeHorizontalBlobs(c, mergeIndex, MAX_LINE_BLOBS);
  std::vector<Blob>& blobs = blob_detector_->horizontalBlob[c];
  std::vector<Blob*> merged;
  for(unsigned int i = 0; i < blobs.size(); i++) {
    Blob& blob = blobs[i];
    if(mergeIndex[i] == MAX_LINE_BLOBS)
      merged.push_back(&blob);
  }
  std::vector<RobotCandidate> candidates = formRobotCandidates(merged, c);
  computeCandidateProbabilities(candidates);
  selectRobots(candidates);
}

std::vector<RobotCandidate> JerseyRobotDetector::formRobotCandidates(std::vector<Blob*>& blobs, Color c) {
  std::vector<RobotCandidate> candidates;
  for(unsigned int i = 0; i < blobs.size(); i++) {
    Blob* blob = blobs[i];
    RobotCandidate candidate;
    candidate.blob = blob;
    candidate.width = blob->xf - blob->xi + 1;
    candidate.height = blob->yf - blob->yi + 1;
    candidate.centerX = (blob->xi + blob->xf) / 2.0f;
    candidate.centerY = (blob->yi + blob->yf) / 2.0f;
    candidate.color = c;
    fillFeet(candidate);
    candidate.relTorso = cmatrix_.getWorldPosition(candidate.centerX, candidate.centerY, ROBOT_CHEST_HEIGHT);
    candidate.relPosition = (candidate.relFeet + Position(candidate.relTorso.x, candidate.relTorso.y, 0)) / 2;
    candidate.widthDistance = getDistanceByWidth(candidate.width);
    candidate.heightDistance = getDistanceByHeight(candidate.height);
    candidate.kinematicsDistance = cmatrix_.groundDistance(candidate.relPosition);
    //printf("wdist: %2.2f, hdist: %2.2f, kdist: %2.2f\n", candidate.widthDistance, candidate.heightDistance, candidate.kinematicsDistance);
    candidate.worldHeight = cmatrix_.getWorldHeight(Coordinates(candidate.centerX, candidate.centerY), Coordinates(candidate.feetX, candidate.feetY));
    fillColorPercents(candidate);
    candidates.push_back(candidate);
  }
  return candidates;
}

void JerseyRobotDetector::computeCandidateProbabilities(std::vector<RobotCandidate>& candidates) {
  for(unsigned int i = 0; i < candidates.size(); i++) {
    RobotCandidate& candidate = candidates[i];
    double widthHeightRatio = std::min(candidate.width / candidate.height, 1.0f);
    double heightWidthRatio = std::min(candidate.height / candidate.width, 1.0f);
    double kDistOverWDist = candidate.kinematicsDistance / candidate.widthDistance;
    double kDistOverHDist = candidate.kinematicsDistance / candidate.heightDistance;
    double torsoDist = cmatrix_.groundDistance(candidate.relTorso);
    double feetDist = torsoDist;
    if(!candidate.feetMissing) feetDist = cmatrix_.groundDistance(candidate.relFeet);
    double torsoFeetDistDiscrepancy = abs(feetDist - torsoDist) / (feetDist + torsoDist);
    //printf("feet: %2.2f, torso: %2.2f, disc: %2.2f\n", feetDist, torsoDist, torsoFeetDistDiscrepancy);
    double prob = estimator_.getLikelihood(
      widthHeightRatio,
      heightWidthRatio,
      kDistOverWDist,
      kDistOverHDist,
      torsoFeetDistDiscrepancy,
      candidate.jerseyColorPercent,
      candidate.greenWhitePercent,
      candidate.whitePercent,
      candidate.correctPercent,
      candidate.worldHeight
    );
    //estimator_.printLast();
    candidate.confidence = prob;
  }
}

void JerseyRobotDetector::selectRobots(std::vector<RobotCandidate>& candidates) {
  std::sort(candidates.begin(), candidates.end(), RobotCandidate::sortPredicate);
  const unsigned int count = WO_OPPONENT_LAST - WO_OPPONENT_FIRST + 1;
  int oppOffset = 0;
  for(unsigned int i = 0; i < std::min(count, (const unsigned int)candidates.size()); i++) {
    RobotCandidate& candidate = candidates[i];
    if(candidate.confidence < .3) break;
    WorldObject *wo = &vblocks_.world_object->objects_[WO_OPPONENT_FIRST + i + oppOffset];
    while(wo->seen && i + oppOffset < count) {
      oppOffset++;
      wo = &vblocks_.world_object->objects_[WO_OPPONENT_FIRST + i + oppOffset];
    }
    if(i + oppOffset >= count) break;

    wo->visionDistance = cmatrix_.groundDistance(candidate.relPosition);
    wo->visionBearing = cmatrix_.bearing(candidate.relPosition);
    wo->visionElevation = cmatrix_.elevation(candidate.relPosition);
    wo->imageCenterX = candidate.centerX;
    wo->imageCenterY = candidate.centerY;
    wo->fromTopCamera = (camera_ == Camera::TOP);
    wo->seen = true;
    if(candidate.color == c_BLUE)
      blueRobots_.push_back(candidate.blob);
    else
      pinkRobots_.push_back(candidate.blob);
  }
}

std::list<Blob*> JerseyRobotDetector::getBlueRobots() {
  return blueRobots_;
}

std::list<Blob*> JerseyRobotDetector::getPinkRobots() {
  return pinkRobots_;
}

float JerseyRobotDetector::getDistanceByWidth(float width) {
  return cmatrix_.getWorldDistanceByWidth(width, JERSEY_WIDTH);
}

float JerseyRobotDetector::getDistanceByHeight(float height) {
  return cmatrix_.getWorldDistanceByHeight(height, JERSEY_HEIGHT);
}

void JerseyRobotDetector::fillFeet(RobotCandidate& candidate) {
  int hstep, vstep;
  classifier_->getStepSize(hstep, vstep);
  int vstart = ROUND(std::min((int)(candidate.centerY + candidate.height), iparams_.height - 1), vstep);
  int vend = ROUND(std::min(vstart + (int)candidate.height * 6, iparams_.height - 1), vstep);
  int hstart = ROUND(std::min((int)(candidate.centerX - candidate.width / 2), iparams_.width - 1), hstep);
  int hend = ROUND(std::min((int)(candidate.centerX + candidate.width / 2), iparams_.width - 1), hstep);
  int feetX = candidate.centerX;
  int feetY = iparams_.height - vstep;
  for(int y = vstart; y <= vend; y += vstep) {
    int green = 0, other = 0;
    for(int x = hstart; x <= hend; x += hstep) {
      Color c = classifier_->xy2color(x, y);
      if(c == c_FIELD_GREEN) green++;
      else other++;
    }
    float greenPct = (float)green / (other + green);
    if(greenPct > .4) {
      feetY = y; break;
    }
  }
  if(feetY == iparams_.height - vstep) candidate.feetMissing = true;
  Position p = cmatrix_.getWorldPosition(feetX, feetY);
  candidate.feetY = feetY;
  candidate.feetX = candidate.centerX;
  candidate.relFeet = p;
}

void JerseyRobotDetector::fillColorPercents(RobotCandidate& candidate) {
  int hstep, vstep;
  classifier_->getStepSize(hstep, vstep);
  int correct = 0, green = 0, white = 0, rwhite = 0, total = 0;
  for(int y = candidate.blob->yi; y <= candidate.blob->yf; y += vstep) {
    for(int x = candidate.blob->xi; x <= candidate.blob->xf; x += hstep) {
      Color c = classifier_->xy2color(x, y);
      if(c == candidate.color) correct++;
      else if(c == c_WHITE) white++;
      else if(c == c_ROBOT_WHITE) rwhite++;
      else if(c == c_FIELD_GREEN) green++;
      total++;
    }
  }
  candidate.jerseyColorPercent = (float)correct / total;
  for(int y = candidate.blob->yf + vstep; y <= candidate.feetY; y += vstep) {
    for(int x = candidate.blob->xi; x <= candidate.blob->xf; x += hstep) {
      Color c = classifier_->xy2color(x, y);
      if(c == candidate.color) correct++;
      else if(c == c_WHITE) white++;
      else if(c == c_ROBOT_WHITE) rwhite++;
      else if(c == c_FIELD_GREEN) green++;
      total++;
    }
  }

  candidate.greenWhitePercent = (float)(white + rwhite + green) / total;
  candidate.whitePercent = (float)(white + rwhite) / total;
  candidate.correctPercent = (float)(correct + white + rwhite + green) / total;
}

void JerseyRobotDetector::detectRobotCluster() {
}
