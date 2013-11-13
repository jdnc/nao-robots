#include "BandRobotDetector.h"

#define IS_ROBOT_WHITE(c) ((c == c_WHITE) || (c == c_ROBOT_WHITE))

BandRobotDetector::BandRobotDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) :
  DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) { }

float BandRobotDetector::estimateBandDistanceByWidth(Blob * band) {
  // get rid of warning for now`
  band = band;
  return 100000000.0; // hack to ignore robots that don't have green below them
}

float BandRobotDetector::estimateBandDistanceByGreenBelow(Blob * band) {
  int hstep, vstep;
  classifier_->getStepSize(hstep,vstep);

  int yStart = band->avgY;
  yStart = yStart - yStart % vstep;

  int xStart = band->xi + (10 * (band->xf - band->xi)) / 100;
  xStart = xStart + hstep - xStart % hstep;

  int xEnd = band->xf - (10 * (band->xf - band->xi)) / 100;
  xEnd = xEnd - xEnd % hstep;

  for (int y = yStart; y < iparams_.height; y += vstep) {
    int greenCounter = 0;
    int otherCounter = 0;

    for (int x = xStart; x <= xEnd; x += hstep) {

      unsigned char c = getSegPixelValueAt(x, y);
      bool isItGreen = c == c_FIELD_GREEN;
      greenCounter += isItGreen;
      otherCounter += !isItGreen;

    }
    //visionLog((39, "Checking green below at y = %i from xi=%i to xf=%i, Green counter: %i, Other Counter: %i", y, xStart, xEnd, greenCounter, otherCounter));
    if (greenCounter > otherCounter) {
      //FastGetPointProjection(band->avgX, y, &tx, &tz);
      Position p = cmatrix_.getWorldPosition(band->avgX, y);
      return sqrtf(p.x * p.x + p.y * p.y);
    }
  }
  return 100000000.0;
}

float BandRobotDetector::estimateBandDistance(Blob * band) {

  float distance;

  float greenDistance = estimateBandDistanceByGreenBelow(band);
  float widthDistance = estimateBandDistanceByWidth(band);

  distance = (greenDistance < widthDistance) ? greenDistance : widthDistance;

  return distance;

}

void BandRobotDetector::sanitizeBandCandidate(Blob * band, int color) {
  visionLog((39, "sanitizing blob for %s band check, (x,y) = (%i,%i), avgWidth = %2.2f\n", COLOR_NAME(color), band->avgX, band->avgY, band->avgWidth));
  int hstep, vstep;
  classifier_->getStepSize(hstep, vstep);
  int yStart = band->avgY - 1 * band->avgWidth / 2;
  yStart = yStart - yStart % vstep;

  int yEnd = band->avgY + 1 * band->avgWidth / 2;
  yEnd = yEnd + vstep - yEnd % vstep;

  int xStart = band->xi;// - (10 * (band->xf - band->xi)) / 100;
  xStart = xStart - xStart % hstep;

  int xEnd = band->xf;// + (10 * (band->xf - band->xi)) / 100;
  xEnd = xEnd + hstep - xEnd % hstep;

  if (yStart < 0)
    yStart = 0;
  if (yEnd >= iparams_.height)
    yEnd = iparams_.height - 1;

  if (xStart < 0)
    xStart = 0;
  if (xEnd >= iparams_.width)
    xEnd = iparams_.width - 1;

  // Check above
  int whiteCounter = 0;
  int otherCounter = 0;

  int y;
  visionLog((39, "scanning from x = %i to %i, y = %i to max(%i, %i)", xStart, xEnd, yStart, 0, (int)(yStart - 4 * band->avgWidth)));
  // move up from start until you reach a substantial amount of white (1/3 of total pixels in a row)
  for (y = yStart; y >= 0 && y >= yStart - 4 * band->avgWidth && whiteCounter < (xEnd - xStart) / (float)hstep / 3.0; y -= vstep ) {
    whiteCounter = 0;
    for (int x = xStart; x <= xEnd; x += hstep) {
      unsigned char c = getSegPixelValueAt(x, y);
      bool isWhite = IS_ROBOT_WHITE(c);
      whiteCounter += isWhite;
      if(!isWhite && c != color && c != c_UNDEFINED) {
        otherCounter++;
      }
    }
  }

  visionLog((39,  "Robot: Blob at (%i, %i) white at top found at %i, other pixels till then %i, width pixels = %i", band->avgX, band->avgY, y, otherCounter, (xEnd - xStart)));
  if (otherCounter > float(xEnd - xStart) / 6.0) {
    visionLog((39, "  Blob invalid"));
    band->invalid = true;
    return;
  }

  y = y + vstep;
  int yWhiteStart = y;
  whiteCounter = 0;
  otherCounter = 0;
  visionLog((39, "scanning from x = %i to %i, y = %i to max(%i, %i)", xStart, xEnd, y, 0, yWhiteStart));
  for (; y >= 0 && y >= yWhiteStart - 6; y -= vstep) {
    for (int x = xStart; x <= xEnd; x += hstep) {
      unsigned char c = getSegPixelValueAt(x, y);
      bool isItWhite = IS_ROBOT_WHITE(c);
      whiteCounter += isItWhite;
      otherCounter += !isItWhite;
    }
  }

  if (whiteCounter < 3 || 6 * whiteCounter < 2 * otherCounter) {
    visionLog((39,  "Robot: Blob at (%i, %i) thrown out because of too little white above (%i of %i).", band->avgX, band->avgY, whiteCounter, whiteCounter + otherCounter));
    //visionLog((39,  "    (%i, %i, %i).", yWhiteStart, xStart, xEnd));
    band->invalid = true;
    return;
  }

  whiteCounter = 0;
  otherCounter = 0;

  visionLog((39, "scanning from x = %i to %i, y = %i to min(%i, %i)", xStart, xEnd, yEnd, iparams_.height, yEnd + 4 * (int)band->avgWidth));
  // move up from start until you reach a substantial amount of white (1/3 of total pixels in a row)
  for (y = yEnd; y < iparams_.height && y <= yEnd + 4 * band->avgWidth && whiteCounter <= (xEnd - xStart) / (float)hstep / 3.0; y += vstep ) {
    whiteCounter = 0;
    for (int x = xStart; x <= xEnd; x += hstep) {
      unsigned char c = getSegPixelValueAt(x, y);
      bool isWhite = IS_ROBOT_WHITE(c);
      whiteCounter += isWhite;
      if(!isWhite && c != color && c != c_UNDEFINED) {
        otherCounter++;
      }
    }
  }

  y = y - vstep;
  yWhiteStart = y;
  whiteCounter = 0;
  otherCounter = 0;
  visionLog((39, "scanning from x = %i to %i, y = %i to min(%i, %i)", xStart, xEnd, yEnd, iparams_.height, yWhiteStart));
  for (y = yEnd; y < iparams_.height && y <= yWhiteStart; y += vstep) {
    for (int x = xStart; x <= xEnd; x += hstep) {
      unsigned char c = getSegPixelValueAt(x, y);
      bool isItWhite = IS_ROBOT_WHITE(c);
      whiteCounter += isItWhite;
      otherCounter += !isItWhite;
    }
  }

  if (whiteCounter < 3 || 6 * whiteCounter < 2 * otherCounter) {
    visionLog((39,  "Robot: Blob at (%i, %i) thrown out because of too little white below (%i of %i).", band->avgX, band->avgY, whiteCounter, whiteCounter + otherCounter));
    //visionLog((39,  "    (%i, %i, %i).", yWhiteStart, xStart, xEnd));
    band->invalid = true;
    return;
  }

  band->invalid = false;
}

void BandRobotDetector::sanitizeBandCandidateOnElevation(Blob * band, float y) {

  visionLog((39,  "Robot: Blob at (%i, %i) has z height %f", band->avgX, band->avgY, y));
  if (y > 250 || y < -250) {
    visionLog((39,  "Robot: Blob at (%i, %i) thrown out because of bad elevation %f", band->avgX, band->avgY, y));
    band->invalid = true;
    return;
  }

}

void BandRobotDetector::detectBlueRobots() {
  blueRobots_.clear();
  blob_detector_->formBlueBandBlobs();
  std::vector<BlobCollection> verticalBlob = blob_detector_->verticalBlob;
  visionLog((30, "Detecting robots from %i blobs", verticalBlob[c_BLUE].size()));
  int mergeCount = 100;
  uint16_t mergeIndex[mergeCount];
  blob_detector_->mergeVerticalBlobs(c_BLUE,mergeIndex,mergeCount);

  int opp = WO_OPPONENT1;

  // Todd: make sure we start at first opponent not already filled in
  while (vblocks_.world_object->objects_[opp].seen && opp <= WO_OPPONENT4)
    opp++;

  //if (opp > WO_OPPONENT1)
  //  cout << "Camera " << camera_ << " start filling in blue robots at index " << opp << endl;

  if (opp > WO_OPPONENT_LAST){
    // no more opponent slots
    return;
  }


  for (uint32_t i = 0; i < verticalBlob[c_BLUE].size(); i++) {
    Blob * band = &verticalBlob[c_BLUE][i];
    if(mergeIndex[i] == mergeCount) {
      band->invalid = false;
    }
    else {
      band->invalid = true;
      visionLog((39, "Robot: Band %i at x:(%i,%i) ~ %i, y:%i thrown out because it was merged with blob %i", i, band->xi, band->xf, band->avgX, band->avgY, mergeIndex[i]));
    }

    for (uint32_t j = 0; j < i; j++){
      if (verticalBlob[c_BLUE][j].invalid)
        continue;
      float dx = verticalBlob[c_BLUE][j].avgX - band->avgX;
      float dy = verticalBlob[c_BLUE][j].avgY - band->avgY;
      if (dx * dx + dy * dy < 100 * 100)
        {
          band->invalid = true;
          visionLog((39,"Robot: Blob at (%i, %i) thrown out - too close to blob at (%i, %i)",band->avgX,band->avgY,verticalBlob[c_BLUE][j].avgX,verticalBlob[c_BLUE][j].avgY));
          break;
        }
    }
    if (band->invalid) {
      continue;
    }
    visionLog((39, "Robot: Band %i at x:(%i,%i) ~ %i, y:%i selected for band check", i, band->xi, band->xf, band->avgX, band->avgY));
    /*
      TODO: Remove after investigating w/ Piyush. It doesn't look like these were being used at all, since the bluePosts elements
      were never assigned to after initialization. - JM 3/15/2012

      for (int goal = 0; goal < BluePostCounter; goal++) {
      if (!bluePosts[goal]->ValidLine)
      continue;

      float perpendicularSlope = -1 / bluePosts[goal]->Slope;
      float perpendicularOffset = band->avgY - perpendicularSlope * band->avgX;

      int xInt = (perpendicularOffset - bluePosts[goal]->Offset) / (bluePosts[goal]->Slope - perpendicularSlope);
      int yInt = perpendicularSlope * xInt + perpendicularOffset;

      float dist = sqrtf((band->avgX - xInt) * (band->avgX - xInt) + (band->avgY - yInt) * (band->avgY - yInt));

      //visionLog((39,  "Robot: Blob at (%i, %i) xInt: %i, yInt: %i, dist: %f", band->avgX, band->avgY, xInt, yInt, dist));
      if (dist < bluePosts[goal]->width + 10) {
      band->invalid = true;
      break;
      }
      }
      if (band->invalid)
      continue;
    */

    sanitizeBandCandidate(band, c_BLUE);
    if (band->invalid)
      continue;

    // Yay! A Detection. Fill in values;
    float distance = estimateBandDistance(band);

    if (distance < 100 || distance > 6000) { // distance invalid
      visionLog((39, "blob thrown out because of distance %5.3f", distance));
      continue;
    }

    float elevation = cmatrix_.getCameraElevation(band->avgY);

    // Sanitize based on elevation
    sanitizeBandCandidateOnElevation(band, elevation);
    if (band->invalid)
      continue;

    // no more than 4, or we're detecting something crazy
    if (opp == WO_OPPONENT_LAST + 1) {
      //Remove all sightings
      for (int k = WO_OPPONENT_FIRST; k < WO_OPPONENT_LAST; k++)
        vblocks_.world_object->objects_[k].seen = false;
      break;
    }

    WorldObject *wo = &vblocks_.world_object->objects_[opp];
    wo->seen = true;
    Position p = cmatrix_.getWorldPositionByDirectDistance(band->avgX, band->avgY, distance);
    wo->visionDistance = cmatrix_.groundDistance(p);
    wo->visionBearing = cmatrix_.bearing(p);
    wo->visionElevation = cmatrix_.elevation(p);
    wo->imageCenterX = band->avgX;
    wo->imageCenterY = band->avgY;
    wo->fromTopCamera = (camera_ == Camera::TOP);

    opp++;

    blueRobots_.push_back(band);
  }

}

void BandRobotDetector::detectPinkRobots() {
  pinkRobots_.clear();
  blob_detector_->formPinkBandBlobs();
  std::vector<BlobCollection> verticalBlob = blob_detector_->verticalBlob;
  visionLog((30, "Detecting robots from %i blobs", verticalBlob[c_PINK].size()));
  int mergeCount = 100;
  uint16_t mergeIndex[mergeCount];
  blob_detector_->mergeVerticalBlobs(c_PINK,mergeIndex,mergeCount);

  int opp = WO_OPPONENT1;

  // Todd: make sure we start at first opponent not already filled in
  while (vblocks_.world_object->objects_[opp].seen && opp <= WO_OPPONENT4)
    opp++;

  //if (opp > WO_OPPONENT1)
  //  cout << "Camera " << camera_ << " start filling in pink robots at index " << opp << endl;

  if (opp > WO_OPPONENT_LAST){
    // no more opponent slots
    return;
  }

  for (uint32_t i = 0; i < verticalBlob[c_PINK].size(); i++) {
    Blob * band = &verticalBlob[c_PINK][i];
    if(mergeIndex[i] == mergeCount)
      band->invalid = false;
    else band->invalid = true;

    for (uint32_t j = 0; j < i; j++){
      if (verticalBlob[c_PINK][j].invalid)
        continue;
      float dx = verticalBlob[c_PINK][j].avgX - band->avgX;
      float dy = verticalBlob[c_PINK][j].avgY - band->avgY;
      if (dx * dx + dy * dy < 100 * 100)
        {
          band->invalid = true;
          //visionLog((39,"Robot: Blob at (%i, %i) thrown out - too close to blob at (%i, %i)",band->avgX,band->avgY,verticalBlob[c_PINK][j].avgX,verticalBlob[c_PINK][j].avgY));
          break;
        }
    }
    if (band->invalid)
      continue;

    sanitizeBandCandidate(band, c_PINK);
    if (band->invalid)
      continue;

    // Yay! A Detection. Fill in values;
    float distance = estimateBandDistance(band);
    float elevation = cmatrix_.getCameraElevation(band->avgY);

    // Sanitize based on elevation
    sanitizeBandCandidateOnElevation(band, elevation);
    if (band->invalid)
      continue;

    // no more than 4, or we're detecting something crazy
    if (opp == WO_OPPONENT_LAST + 1) {
      //Remove all sightings
      for (int k = WO_OPPONENT_FIRST; k < WO_OPPONENT_LAST; k++)
        vblocks_.world_object->objects_[k].seen = false;
      break;
    }

    WorldObject &wo = vblocks_.world_object->objects_[opp];
    wo.seen = true;
    Position p = cmatrix_.getWorldPositionByDirectDistance(band->avgX, band->avgY, distance);
    wo.visionDistance = cmatrix_.groundDistance(p);
    wo.visionBearing = cmatrix_.bearing(p);
    wo.visionElevation = cmatrix_.elevation(p);
    wo.imageCenterX = band->avgX;
    wo.imageCenterY = band->avgY;
    wo.fromTopCamera = (camera_ == Camera::TOP);

    opp++;

    pinkRobots_.push_back(band);
  }

}

void BandRobotDetector::detectRobotCluster() {
  visionLog((40,"Detection robot clusters"));
  // PARAMETERS for the cluster detection
  const int NUM_COLS = 4;
  const int NUM_ROWS = 4;
  const int MIN_CLUSTER_LENGTH = 2;
  const float MIN_WHITE_FRAC = 0.15;
  const float MIN_GREEN_FRAC = 0.06;
  const int END_GREEN_ROW = 1; // inclusive
  const int MIN_GREEN_COUNT = 2;

  const int COL_WIDTH = iparams_.width / NUM_COLS;
  const int ROW_HEIGHT = iparams_.height / NUM_ROWS;
  const float COL_SIZE = iparams_.height * COL_WIDTH / 4; // /4 because every 4 pixels in seg
  const float CELL_SIZE = COL_SIZE / NUM_ROWS;
  WorldObject &wo = vblocks_.world_object->objects_[WO_ROBOT_CLUSTER];
  int whiteCounts[NUM_COLS];
  int greenCounts[NUM_ROWS][NUM_COLS];

  wo.seen = false; // assume we don't see it
  // count the white and green
  for (int col = 0; col < NUM_COLS; col++) {
    whiteCounts[col] = 0;
    for (int row = 0; row < NUM_ROWS; row++)
      greenCounts[row][col] = 0;
  }

  // check the left and right greens
  for (int x = 0; x < iparams_.width; x += 4) {
    int col = (int)(x / COL_WIDTH);
    for (int y = 0; y < iparams_.height; y++) {
      int row = (int)(y / ROW_HEIGHT);

      unsigned char c = getSegPixelValueAt(x, y);
      whiteCounts[col] += IS_ROBOT_WHITE(c);
      greenCounts[row][col] += (c == c_FIELD_GREEN);
    }
  }

  float whiteFrac;
  bool whiteFound[NUM_COLS];
  float greenFracs[NUM_ROWS][NUM_COLS];
  // count up the white and green
  for (int col = 0; col < NUM_COLS; col++) {
    whiteFrac = whiteCounts[col] / COL_SIZE;
    visionLog((41,"  col %i, white: %f",col,whiteFrac));
    for (int row = 0; row < NUM_ROWS; row++) {
      greenFracs[row][col] = greenCounts[row][col] / CELL_SIZE;
      visionLog((41,"    section %i,%i, green: %f",col,row,greenFracs[row][col]));
    }
    whiteFound[col] = (whiteFrac > MIN_WHITE_FRAC);
  }

  // check for consecutive white sections
  int clusterStart = -1;
  int clusterLength = 0;
  // note: we're only checking for the first cluster of length >= 2
  for (int i = 0; i < NUM_COLS; i++) {
    if (whiteFound[i]) {
      if (clusterStart >= 0) {
        if (clusterStart + clusterLength == i) {
          // add to cluster
          clusterLength++;
        } else if (clusterLength < MIN_CLUSTER_LENGTH) {
          // start a new cluster
          clusterStart = i;
          clusterLength = 1;
        } // else ignore
      } else {
        // start a new cluster
        clusterStart = i;
        clusterLength = 1;
      }
    }
  }
  visionLog((41,"Best cluster: start: %i length: %i",clusterStart,clusterLength));
  if (clusterLength < MIN_CLUSTER_LENGTH) {
    visionLog((40,"Cluster too short: %i",clusterLength));
    return;
  }

  // now check the green
  int greenCount = 0;
  for (int col = 0; col < NUM_COLS; col++) {
    for (int row = 0; row <= END_GREEN_ROW; row++) {
      greenCount += (greenFracs[row][col] > MIN_GREEN_FRAC);
    }
  }
  if (greenCount < MIN_GREEN_COUNT) {
    visionLog((40,"Not enough green for cluster: %i",greenCount));
    return;
  }


  float clusterCenter = (clusterStart + clusterLength / 2.0) * COL_WIDTH;
  float bear = cmatrix_.getCameraBearing(clusterCenter);
  visionLog((40,"Raw cluster: %i %f %f",clusterStart,clusterCenter, bear));
  Position p = cmatrix_.getWorldPositionByDirectDistance(clusterCenter, iparams_.height / 2, 200);
  wo.visionDistance = cmatrix_.groundDistance(p);
  wo.visionBearing = cmatrix_.bearing(p);
  wo.visionElevation = cmatrix_.elevation(p);
 
  if (fabs(wo.visionBearing) > DEG_T_RAD * 20) {
    visionLog((40,"Cluster thrown out for too high of bearing: %f",wo.visionBearing));
    return;
  }
  wo.seen = true;
  wo.fromTopCamera = (camera_ == Camera::TOP);
  visionLog((40,"Cluster found with bearing: %f",wo.visionBearing));
}

std::list<Blob*> BandRobotDetector::getBlueRobots() {
  return blueRobots_;
}

std::list<Blob*> BandRobotDetector::getPinkRobots() {
  return pinkRobots_;
}
