#include "BallDetector.h"

using namespace Eigen;

#define horizontalBlob blob_detector_->horizontalBlob
#define verticalBlob blob_detector_->verticalBlob
#define getball() (&vblocks_.world_object->objects_[WO_BALL])
#define getself() (&vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF])
#define getframe() vblocks_.frame_info->frame_id

BallDetector::BallDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) : DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) {
  for(int i = 0; i < MAX_BALL_CANDS; i++)
    candidates_[i] = new BallCandidate();
  estimator_.setMean( 
    1.0, // orange %
    1.0, // green/white %
    0.0, // circle deviation
    0.0, // height
    0.0, // kinematics / width based distance discrepancy
    0.0, // distance from field
    0.0 // velocity
  );
}

void BallDetector::findBall() {
  bestBallCandIndex = -1;
  
  WorldObject *ball = getball();
  if(ball->seen && !ball->fromTopCamera && camera_ == Camera::TOP) {
    visionLog((34, "Ball was seen already in the bottom camera, bailing out."));
    return;
  }

  blob_detector_->resetOrangeBlobs();
#ifdef TOOL
  vblocks_.robot_vision->doHighResBallScan = true;
#endif

  if(camera_ == Camera::BOTTOM) {
    blob_detector_->formOrangeBlobs();
    blob_detector_->calculateOrangeBlobData();
    checkAndSelectBall();
  }
  else {
    ballCandCount = 0;
    if(classifier_->startHighResBallScan()) {
      visionLog((34, "Started high res scan for balls in the top camera"));
      classifier_->didHighResBallScan = true;
    }
    blob_detector_->formOrangeBlobs();
    blob_detector_->calculateOrangeBlobData();
    checkAndSelectBall();
    classifier_->completeHighResScan();
  }
  vblocks_.robot_vision->doHighResBallScan = false;
}

bool BallDetector::checkAndSelectBall()
{
  ballCandCount = 0;

  if (horizontalBlob[c_ORANGE].size() > MAX_ORANGE_BLOBS) {
    visionLog((34,  "Too many orange blobs in the scene"));
    return true;
  }

  if (horizontalBlob[c_ORANGE].empty()) {
    visionLog((34,  "No orange blobs in the scene"));
    return false;
  }

  // merge orange blobs
  uint16_t mergeIndex[MAX_ORANGE_BLOBS];
  blob_detector_->mergeHorizontalBlobs(c_ORANGE, mergeIndex, MAX_ORANGE_BLOBS);

  std::vector<Blob*> blobs;

  for(unsigned int i = 0; i < horizontalBlob[c_ORANGE].size(); i++) {
    Blob* blob = &horizontalBlob[c_ORANGE][i];
    if(mergeIndex[i] == MAX_ORANGE_BLOBS) {
      blobs.push_back(blob);
      visionLog((34, "merged blob %i: (xi:xf,yi:yf) = (%i:%i,%i:%i)", i, blob->xi, blob->xf, blob->yi, blob->yf));
    }
  }

  // sort orange blobs by blob area size
  //blobs = sortOrangeBlobs(blobs);
  sort(blobs.begin(), blobs.end(), sortBlobAreaPredicate);

  for(unsigned int i = 0; i < blobs.size(); i++) {
    Blob* blob = blobs[i];
    visionLog((34, "sorted blob %i: (xi:xf,yi:yf) = (%i:%i,%i:%i)", i, blob->xi, blob->xf, blob->yi, blob->yf));
  }

  // fit circles to ball candidates_
  std::vector<BallCandidate*> candidates = formBallCandidates(blobs);
  BallCandidate *best;
  //best = sanityCheckBallCands(candidates);
  best = computeCandidateProbabilities(candidates);
  if(!best) return false;
  setBestBallCandidate(best);
  return true;
}

BallCandidate* BallDetector::computeCandidateProbabilities(std::vector<BallCandidate*> candidates) {
  if(candidates.size() == 0) return false;
  Position ballRelPos(getball()->relPos.x, getball()->relPos.y, 0);
  double ballDistance = cmatrix_.groundDistance(ballRelPos);
  bool ballInitialized = (getball()->frameLastSeen > 0);
  const float confThreshold = .3;
  double maxV = 100;
  double maxD = 100;
  // Scale max velocity with expected distance since positional uncertainty increases w/ range
  if(ballInitialized) {
    maxV = std::max(100.0, ballDistance * .1);
    maxD = std::max(100.0, ballDistance * .2);
  }
  estimator_.setStdDev(0.5, 0.4, 0.3, 150.0, 0.4, maxD, maxV);
  for(uint16_t i = 0; i < candidates.size(); i++) {
    BallCandidate* candidate = candidates[i];
    Blob* blob = candidate->blob;
    int bw = blob->xf - blob->xi, bh = blob->yf - blob->yi;
    if(
        // Look at the blobs for these since half circle fitting can alter the height/width calculations.
        bw < 2 ||
        bh < 2 ||
        candidate->blob->correctPixelRatio < .25 // These are all over the place so they don't really fit into the Gaussian model
    ) { 
      visionLog((34, "threw out ball candidate because: xf - xi = %i (min 2), yf - yi = %i (min 2), pixel ratio %2.2f (min .25)", bw, bh, candidate->blob->correctPixelRatio));
      candidate->confidence = 0; continue; 
    }
    double circleOrangePct = checkColorsInCircleFit(candidate);
    double belowGreenWhitePct = camera_ == Camera::TOP ? checkBelowGreenWhitePct(candidate) : 1.0;
    double circleFit = candidate->stddev;
    double height = candidate->relPosition.z;

    Point2D pa = Point2D(abs(candidate->absPosition.x), abs(candidate->absPosition.y));
    float xd = (pa.x > HALF_GRASS_X ? pa.x - HALF_GRASS_X : 0), yd = (pa.y > HALF_GRASS_Y ? pa.y - HALF_GRASS_Y : 0);
    double distanceFromField = sqrt(xd * xd + yd * yd);

    Position diff = ballRelPos - candidate->relPosition;
    int dt = getframe() - getball()->frameLastSeen;
    double dx = abs(diff.x), dy = abs(diff.y);
    double v = sqrt(dx * dx + dy * dy) / dt;
#ifdef TOOL
    v = 0; // Velocity can be problematic when running logs, especially when log frames aren't sequential
#endif
    if(!ballInitialized || dt == 0) v = 0;
    if(vblocks_.game_state->state != PLAYING) distanceFromField = 0; // This shouldn't be used when not in playing, helps with debugging

    double prob = estimator_.getLikelihood(
      circleOrangePct,
      belowGreenWhitePct,
      circleFit,
      height,
      candidate->kwDistanceDiscrepancy,
      distanceFromField,
      v
    );
    //estimator_.printLast();
    visionLog((34, "Checking candidate %i: %i,%i to %i,%i", i, blob->xi, blob->yi, blob->xf, blob->yf));
    estimator_.logLast(34,textlogger);

    candidate->confidence = prob;
  }
  BallCandidate* best = NULL; float bestConf = 0;
  for(uint16_t i = 0; i < candidates.size(); i++) {
    BallCandidate* candidate = candidates[i];
    if(candidate->confidence < confThreshold) continue;
    if(candidate->confidence > bestConf) {
      best = candidate;
      bestConf = candidate->confidence;
    }
  }
  if(!best) return NULL;
  //printf("best position: %2.2f,%2.2f\n", best->absPosition.x, best->absPosition.y);
  return best;
}

BallCandidate* BallDetector::sanityCheckBallCands(std::vector<BallCandidate*> candidates)
{
  visionLog((34, "Sanity checking %i ball candidates_", candidates.size()));
  //WorldObject ballCand[MAX_BALL_CANDS];
  for (uint16_t i = 0; i < candidates.size(); i++) {
    //WorldObject *ball = &ballCand[i];
    BallCandidate *candidate = candidates[i];
    Blob* blob = candidate->blob;

    visionLog((34,   "Ball candidate %i: (cx,cy) = (%f,%f), blob (xi:xf,yi:yf) = (%i:%i,%i:%i)",
               i, candidate->centerX,candidate->centerY, blob->xi,blob->xf,blob->yi,blob->yf));

    // Size Check
    int minXSize = 7, minYSize = 7;
    // allow smaller balls if we did high res scan
    if (classifier_->didHighResBallScan) {
      minXSize = 4;
      minYSize = 2;
    }
    if (blob->dx < minXSize || blob->dy < minYSize) {
      visionLog((34,  "bad minSize orange-blob[%i] dx %i, dy %i, minXSize %i minYSize %i",
                 i, blob->dx, blob->dy, minXSize,minYSize));
      //ball->visionConfidence = -1;
      continue;
    }

    /*
    // Ratio check
    float ratio = blob->dy / (float)blob->dx;
    if (ratio < .5 || ratio > 2) {
    ball->visionConfidence = -1;
    continue;
    }
    */

    // pixel ratio check
    if (blob->correctPixelRatio < 0.25f ) {
      visionLog((34,  "bad pixel ratio for orange-blob[%i]=%f", i,blob->correctPixelRatio));
      //ball->visionConfidence = -1;
      continue;
    }

    // Check circle fit results
    bool ballCenterXInBlob =
      (candidate->centerX > blob->xi && candidate->centerX < blob->xf);
    bool ballCenterYInBlob =
      (candidate->centerY > blob->yi && candidate->centerY < blob->yf);
    bool ballCenterInBlob = (ballCenterXInBlob && ballCenterYInBlob);
    bool decentCircleFit = (candidate->stddev < .5);
    //float dist;
    if (ballCenterInBlob ||
        (decentCircleFit && ( // ball is most probably at the edge
                             // (ballCenterXInBlob && ballCenterYOffScreen) ||
                             // (ballCenterYInBlob && ballCenterXOffScreen)))) {
                             ballCenterXInBlob || ballCenterYInBlob))) {
                               //ball->visionConfidence -= candidate->stddev;
      visionLog((34,   "Circle deviation[%i] = %f", i,candidate->stddev));
      

    } else { // ball center is not in blob
      //ball->visionConfidence = -1;
      visionLog((34,   "Ball center for candidate %i is not in blob", i));
      continue;
    }

    // check ball radius compared to blob width and height
    /* if (candidate->radius < 0.33f * max(blob->dx,blob->dy)) {
       visionLog((20,  "Bad circle radius compared to blob size for orange-blob[%i], %f, %f",sortIndex[i], candidate->radius, max(blob->dx,blob->dy)));
       continue;
       } */



    // Horizon check because the Z height check doesn't work very well

    if(horizon_.exists && !horizon_.isAbovePoint(candidate->centerX, candidate->centerY + 10)) {
      visionLog((34, "Ball at %i is above horizon, throwing out.", i ));
      continue;
    }

    // Height Check

    //float ballZ = ball->visionDistance * tan(ball->visionElevation);
    //if (ballZ > 500 || ballZ < -450) {
      //ball->visionConfidence = -1;
      //visionLog((34,  "Bad Z height %5.5f, distance = %f, elevation = %f", ballZ, ball->visionDistance, ball->visionElevation * RAD_T_DEG));
      //continue;
    //}

    // Elevation Check

    //if (ball->visionElevation > 20 * DEG_T_RAD || ball->visionDistance > 8000) {
      //ball->visionConfidence = -1;
      //visionLog((34,  "Bad elevation %5.5f or distance %5.5f", ball->visionElevation, ball->visionDistance));
      //continue;
    //}

    // check if ball is in pink band
    if (!checkSurroundingPink(i)){
      //ball->visionConfidence = -1;
      visionLog((34, "Too much pink in ball, throw out"));
      continue;
    }

    visionLog((34,"ball dx %i, dy %i", blob->dx, blob->dy));
    // check for green/white below ball
    // Todd: good balls always had at least 0.26 pct green/white
    // bad balls had as high as 0.125 pct
    // so being very conservative
    // also, this check will return that 1.0 if check was cutoff
    // so we'll accept all balls near bottom of image
    // Todd: only do this with small balls.. big ones have large shadows

    // Disabling this temporarily, need to retune everything anyway - JM 05/22/13
    if (camera_ == Camera::TOP || (blob->dx < 55 && blob->dy < 55)) {
      float belowGreenWhitePct = checkBelowGreenWhitePct(candidate);
      float minPct = .115;
      if (belowGreenWhitePct < minPct){
        visionLog((34, "Ball had %5.3f pct green/white (minimum is %5.3f)", belowGreenWhitePct, minPct));
        continue;
      }
    }

    //if (camera_ == Camera::TOP && (blob->dx > 85 || blob->dy > 85)) {
      //ball->visionConfidence = -1;
      //visionLog((34, "Ball is too big"));
      //continue;
    //}

    // Check percent of orange in fitted circle
    float circleOrangePct = checkColorsInCircleFit(candidate);
    // Todd: Even 0.1 will throw out most of the bad ones in legs/pink
    // But a few bands with a lot of orange in them were up around .45 and .57
    // And a few valid balls were as low as .45
    // I'm thinking we want to keep those valid ones, and just not let that
    // much pink get classified as orange.  But we could set this higher
    // (like to 0.6)
    if (circleOrangePct < 0.33f){
      //ball->visionConfidence = -1;
      visionLog((34, "Bad orange pct in fitted circle: %5.5f", circleOrangePct));
      continue;
    }

    // if we get here, we have a ball!!!
    visionLog((34, "Candidate %i selected as ball", i));
    bestBallCandIndex = i;
    candidate->confidence = 1;
    //printf("dist: %2.4f\n", candidate->groundDistance);
    return candidate;
  }
  return NULL;
}


/** checks the percent of pink between a given set of points */
float BallDetector::checkPinkPct(int x1, int x2, int y1, int y2){
  if (x1 < 0) x1 = 0;
  if (y1 < 0) y1 = 0;
  if (x2 >= iparams_.width) x2 = iparams_.width-1;
  if (y2 >= iparams_.height) y2 = iparams_.height-1;

  int numTotal = 0;
  int numPink = 0;
  for (int x = x1; x <= x2; x+=4){
    for (int y = y1; y <= y2; y+=2){
      numTotal++;
      int c = getSegPixelValueAt(x, y);
      if (c == c_PINK) numPink++;
    }
  }

  //visionLog((43, "check from x: %i, %i, and y: %i, %i, total: %i, pink: %i",
  //    x1, x2, y1, y2, numTotal, numPink));

  if (numTotal == 0)
    return 0.0;

  return (float)numPink/(float)numTotal;

}


/** Todd: this checks along 4 vectors from the ball for pink. */
bool BallDetector::checkSurroundingPink(int i){

  int roundX = 4;
  int roundY = 2;
  int pixToCheck = 10;

  int midY = ((int)(candidates_[i]->centerY/roundY))*roundY;
  int midX = ((int)(candidates_[i]->centerX/roundX))*roundX;

  int endX1 = ((int)((candidates_[i]->centerX - candidates_[i]->radius)/roundX))*roundX;
  int startX1 = endX1 - roundX * pixToCheck;

  float check1 = checkPinkPct(startX1, endX1, midY-roundY*2, midY+roundY*2);

  int startX2 = ((int)((candidates_[i]->centerX + candidates_[i]->radius)/roundX))*roundX;
  int endX2 = startX2 + roundX * pixToCheck;

  float check2 = checkPinkPct(startX2, endX2, midY-roundY*2, midY+roundY*2);

  int endY1 = ((int)((candidates_[i]->centerY - candidates_[i]->radius)/roundY))*roundY;
  int startY1 = endY1 - roundY * pixToCheck;

  float check3 = checkPinkPct(midX - roundX*2, midX+roundX*2, startY1, endY1);

  int startY2 = ((int)((candidates_[i]->centerY + candidates_[i]->radius)/roundY))*roundY;
  int endY2 = startY2 + roundY * pixToCheck;

  float check4 = checkPinkPct(midX - roundX*2, midX+roundX*2, startY2, endY2);

  visionLog((43, "Nearby pink %f, %f, %f, %f", check1, check2, check3, check4));

  // Todd: for good balls, these never got over 0.09
  // for orange in pink bands, there was always one over 0.16
  // going to set threshold at 0.15
  if (check1 > 0.15 || check2 > 0.15 || check3 > 0.15 || check4 > 0.15){
    return false;
  }

  return true;

}

/** Check the percantage of green/white below ball. */
float BallDetector::checkBelowGreenWhitePct(BallCandidate* candidate) {
  // We use the default step sizes because the area under the ball shouldn't be high-res scanned
  int hstep = 1 << iparams_.defaultHorizontalStepScale, vstep = 1 << iparams_.defaultVerticalStepScale;

  int ymin = candidate->blob->yf, ymax = candidate->blob->yf + candidate->radius * 2;
  ymax += vstep - ymax % vstep;
  int xmin = candidate->blob->xi, xmax = candidate->blob->xf;
  ymin -= ymin % vstep;
  ymax += vstep - ymax % vstep;
  xmin -= xmin % hstep;
  xmax += hstep - xmax % hstep;

  xmin = std::max(xmin, 0);
  xmax = std::min(xmax, iparams_.width - 1);
  ymin = std::max(ymin, 0);
  ymax = std::min(ymax, iparams_.height - 1);

  int green = 0, white = 0, total = 0;
  int orange = 0, black = 0;
  for (int x = xmin; x <= xmax; x += hstep){
    for (int y = ymin; y <= ymax; y += vstep){
      int c = getSegPixelValueAt(x, y);
      total++;
      green += (c == c_FIELD_GREEN);
      white += (c == c_WHITE || c == c_ROBOT_WHITE);
      orange += (c == c_ORANGE);
      black += (c == c_UNDEFINED);
    }
  }

  float pct = (float)(green + white) / total;

  //printf("h:%i,v:%i,x:[%i,%i], y:[%i,%i], green: %i, white: %i, orange: %i, black: %i, total: %i, pct: %2.2f\n",
      //hstep, vstep, xmin, xmax, ymin, ymax, green, white, orange, black, total, pct);

  if(!total && ymin >= iparams_.height)
  // We can't see below the ball due to the height, so we assume 1. There
  // is no reason the robot should see anything else that's orange and that
  // extends vertically below the lower boundary of the image.
    pct = 1;

  visionLog((43, "check from x: %i, %i, and y: %i, %i, hstep: %i, vstep: %i, total: %i, greenwhite: %i, pct: %5.3f", xmin, xmax, ymin, ymax, hstep, vstep, total, green + white, pct));

  return pct;
}


/** This checks the percentage of orange inside in the fitted ball circle
    (or, reality, inside the biggest square that can fit inside that circle.
    It helps prevent false balls when we fit orange to a small sliver of orange. */
float BallDetector::checkColorsInCircleFit(BallCandidate* candidate){
  // check pixels inside circle radius
  // whats the length of one side of the largest square inside the circle
  float squareWidth = sqrtf((4.0*candidate->radius*candidate->radius)/2.0);

  int hstep, vstep;
  classifier_->getStepSize(hstep,vstep);
  int roundX = hstep;
  int roundY = vstep;

  int startX = ((int)((candidate->centerX - squareWidth/2.0)/roundX))*roundX;
  int startY = ((int)((candidate->centerY - squareWidth/2.0)/roundY))*roundY;
  int endX = candidate->centerX + squareWidth/2.0;
  int endY = candidate->centerY + squareWidth/2.0;
  if (startX < 0) startX = 0;
  if (startY < 0) startY = 0;
  if (endX >= iparams_.width) endX = iparams_.width-1;
  if (endY >= iparams_.height) endY = iparams_.height-1;

  //visionLog((43, "ball radius scan from x: %i, %i, and y: %i, %i, radius %f, sqWidth %f",
  //         startX, endX, startY, endY, candidate->radius, squareWidth));

  int totalPixelsChecked = 0;
  int numOrange = 0;
  //int numUndefined = 0;
  //int numPink = 0;
  for (int x = startX; x < endX; x+=hstep){
    for (int y = startY; y < endY; y+=vstep){
      totalPixelsChecked++;
      int c = getSegPixelValueAt(x, y);
      if (c == c_ORANGE) numOrange++;
      // if (c == c_PINK) numPink++;
      //if (c == c_UNDEFINED) numUndefined++;
    }
  }

  //visionLog((43, "ball radius scan total %i, orange %i, undef %i, pink %i",
  //          totalPixelsChecked, numOrange, numUndefined, numPink));
  if (totalPixelsChecked == 0) {
    return 0;
  }

  float pctOrange = (float)numOrange / (float)totalPixelsChecked;
  //visionLog((43, "pctOrange in circle fit %f", pctOrange));

  return pctOrange;
}


void BallDetector::fitCircleToPoints(
                                     uint16_t *x, uint16_t *y, uint16_t n,               // in
                                     float *cx, float *cy, float *radius, float *stddev) // out
{
  *cx = 0; *cy = 0; *radius = 0; *stddev = 1e6;
  if (n < 3) return; // ERROR: fewer than three points

  // computing centroids
  float centroidX = 0, centroidY = 0;
  for (uint16_t i = 0; i < n; i++) {
    centroidX += x[i]; centroidY += y[i]; }
  centroidX /= n; centroidY /= n;

  // computing moments
  float m_xx = 0, m_xy = 0, m_xz = 0, m_yy = 0, m_yz = 0, m_zz = 0;
  for (uint16_t i = 0; i < n; i++) {
    float xi = x[i] - centroidX, yi = y[i] - centroidY;
    float zi = xi * xi + yi * yi;
    m_xx += xi * xi; m_xy += xi * yi; m_xz += xi * zi;
    m_yy += yi * yi; m_yz += yi * zi; m_zz += zi * zi;
  }
  m_xx /= n; m_xy /= n; m_xz /= n; m_yy /= n; m_yz /= n; m_zz /= n;

  // computing the coefficients of the characteristic polynomial
  float m_z = m_xx + m_yy;
  float cov_xy = m_xx * m_yy - m_xy * m_xy;
  float a3 = 4 * m_z;
  float a2 = -3 * m_z * m_z - m_zz;
  float a1 = m_zz * m_z + 4 * cov_xy * m_z;
  a1 -= m_xz * m_xz + m_yz * m_yz + m_z * m_z * m_z;
  float a0 = m_xz * m_xz * m_yy + m_yz * m_yz * m_xx;
  a0 -= m_zz * cov_xy + 2 * m_xz * m_yz * m_xy - m_z * m_z * cov_xy;
  float a22 = a2 + a2;
  float a33 = a3 + a3 + a3;
  float newX = 0;
  float newY = 1e20;
  const float eps = 1e-12;
  const size_t MAX_ITER = 20;

  // newton's method starting at x = 0
  for (size_t i = 0; i < MAX_ITER; i++) {
    float oldY = newY;
    newY = a0 + newX * (a1 + newX * (a2 + newX * a3));
    if (fabs(newY) > fabs(oldY))
      return; // ERROR: newton-taubin going wrong dir.
    float dy = a1 + newX * (a22 + newX * a33);
    float oldX = newX;
    newX = oldX - newY / dy;
    if (fabs((newX - oldX) / newX) < eps)
      break; // converged!
    if (newX < 0)
      newX = 0; // ERROR: newton-taubin having neg. root
  }

  // computing the circle parameters
  float det = 2 * (newX * newX - newX * m_z + cov_xy);
  if (fabs(det) < 1e-3) return; // ERROR: zero determinant

  *cx = (m_xz * (m_yy - newX) - m_yz * m_xy) / det;
  *cy = (m_yz * (m_xx - newX) - m_xz * m_xy) / det;
  *radius = sqrtf(*cx * *cx + *cy * *cy + m_z);
  *cx += centroidX; *cy += centroidY;

  // compute standard deviation with a normalized circle (r = 1)
  float m = 0, v = 0;
  for (uint16_t i = 0; i < n; i++) {
    float dx = x[i] - *cx, dy = y[i] - *cy;
    float d = sqrtf(dx * dx + dy * dy) / *radius;
    v += d * d; m += d;
  }
  m /= n; v /= n;
  *stddev = sqrtf(v - m * m);
}

void BallDetector::getBlobContour(Blob* blob, uint16_t *xinit, uint16_t *xfinal) {
  int hscale, vscale, hstep, vstep;
  classifier_->getStepScale(hscale,vscale);
  classifier_->getStepSize(hstep,vstep);
  // initialize output
  for (uint16_t y = 0; y < iparams_.height; y += vstep) {
    xinit[y >> vscale] = (uint16_t)-1; xfinal[y >> vscale] = 0; }

  // insert first blob's contour
  for (uint16_t i = 0; i < blob->lpCount; i++) {
    uint16_t index = blob->lpIndex[i] & 0xffff;
    uint16_t y = blob->lpIndex[i] >> 16; // index
    VisionPoint *lp = &classifier_->horizontalPoint[c_ORANGE][y][index];
    xinit[y >> vscale] = std::min(xinit[y >> vscale], lp->xi);    // coord
    xfinal[y >> vscale] = std::max(xfinal[y >> vscale], lp->xf);
    //std::cout << "set xinit[" << (y >> vscale) << "] to " << xinit[y >> vscale] << ", xfinal[" << (y >> vscale) << "] to " << xfinal[y >> vscale] << "\n";

    }  // coord

  // insert merged blobs' contours
  //for (uint16_t i = 0; i < horizontalBlob[c_ORANGE].size(); i++) {
    //if (mergeIndex[i] != blobIndex)
      //continue;
    //Blob *blj = &horizontalBlob[c_ORANGE][i];
    //for (uint16_t j = 0; j < blj->lpCount; j++) {
      //uint16_t index = blj->lpIndex[j] & 0xffff;
      //uint16_t y = blj->lpIndex[j] >> 16; // index
      //VisionPoint *lp = &classifier_->horizontalPoint[c_ORANGE][y][index];
      //xinit[y >> vscale] = std::min(xinit[y >> vscale], lp->xi);    // coord
      //xfinal[y >> vscale] = std::max(xfinal[y >> vscale], lp->xf); }} // coord
}

BallCandidate* BallDetector::formBallCandidate(Blob* blob, int ballCandIndex) {
  int hscale, vscale;
  classifier_->getStepScale(hscale,vscale);
  int size = iparams_.height >> vscale;
  // get ball candidate's contour
  uint16_t xi[size], xf[size];
  getBlobContour(blob, xi, xf);

  // format full, left, right contour for circle fitting
  uint16_t contFX[iparams_.height     ], contFY[iparams_.height     ]; // full
  uint16_t contFCount = 0;
  uint16_t contLX[size], contLY[size]; // left
  uint16_t contLCount = 0;
  uint16_t contRX[size], contRY[size]; // rite
  uint16_t contRCount = 0;
  const Blob *bl = blob;
  for (uint16_t y = 0; y < size; y++) {
    if (xi[y] == (uint16_t)-1 && xf[y] == 0) continue;
    if (bl->xi > 8) {
      contFX[contFCount] = contLX[contLCount] = xi[y];
      contFY[contFCount] = contLY[contLCount] = y << vscale;
      contFCount++; contLCount++; 
    }
    if (bl->xf < iparams_.width - 8) {
      contFX[contFCount] = contRX[contRCount] = xf[y];
      contFY[contFCount] = contRY[contRCount] = y << vscale;
      contFCount++; contRCount++; 
    }
  }

  // only consider upper half of the contour if too close to body
  if (bl->dx > 40 && bl->yi > size) {
    contFCount = 3 * contFCount / 5;
    contLCount = 3 * contLCount / 5;
    contRCount = 3 * contRCount / 5; }

  // fit circle
  float cxF, cyF, rF, sdF;
  for(int i = 0; i < contFCount; i++) {
    //std::cout << "contfx,contfy: (" << contFX[i] << "," << contFY[i] << ")\n";
  }                              // full
  fitCircleToPoints(contFX, contFY, contFCount, &cxF, &cyF, &rF, &sdF);
  BallCandidate* candidate = candidates_[ballCandIndex];
  //std::cout << "fit points 1: (cx,cy), r, sd: " << "(" << cxF << "," << cyF << "), " << rF << ", " << sdF << "\n";
  if (2 * bl->dy > 3 * bl->dx) {
    float cxL, cyL, rL, sdL;                                       // left
    fitCircleToPoints(contLX, contLY, contLCount, &cxL, &cyL, &rL, &sdL);
    float cxR, cyR, rR, sdR;                                       // rite
    fitCircleToPoints(contRX, contRY, contRCount, &cxR, &cyR, &rR, &sdR);

    // set final ball candidate circle fit values
    //const uint16_t cxBlob = (bl->xi + bl->xf) >> 1;
    if (cxR > bl->xf) {
      visionLog((15,  "<<< LEFT contour chosen !! >>>"));
      candidate->centerX = cxL;
      candidate->centerY = cyL;
      candidate->radius = rL;
      candidate->width = 2 * rL;
      candidate->stddev = sdL;
    } else if (cxL < bl->xi) {
      visionLog((15,   "<<< RITE contour chosen !! >>>"));
      candidate->centerX = cxR;
      candidate->centerY = cyR;
      candidate->radius = rR;
      candidate->width = 2 * rR;
      candidate->stddev = sdR;
    }
  } else {
    visionLog((15,  "<<< FULL contour chosen: (cx,cy) = (%f,%f), rad = %f, stddev = %f !! >>>", cxF, cyF, rF, sdF));
    candidate->centerX = cxF;
    candidate->centerY = cyF;
    candidate->radius = rF;
    candidate->stddev = sdF;
    candidate->width = blob->dx;
  }
  candidate->index = ballCandIndex;
  candidate->blob = blob;
  candidate->height = blob->dy;
  float directDistance = getDirectDistance(candidate);
  candidate->relPosition = cmatrix_.getWorldPositionByDirectDistance(candidate->centerX, candidate->centerY, directDistance);
  candidate->groundDistance = cmatrix_.groundDistance(candidate->relPosition);
  //printf("dd: %2.2f, rel pos: %2.2f,%2.2f,%2.2f, gd: %2.2f\n", 
      //directDistance, candidate->relPosition.x, candidate->relPosition.y, candidate->relPosition.z, candidate->groundDistance);
  candidate->relPosition.z -= BALL_RADIUS;
  candidate->valid = true;
  Pose2D self(getself()->orientation, getself()->loc.x, getself()->loc.y); // release
  //Pose2D self(M_PI / 2, 0, -3000); // midfield
  //Pose2D self(M_PI / 2, 0, 0); // center facing right
  //Pose2D self(3 * M_PI / 4, 4000, -3000); // corner w/ near goal to the right
  Pose2D cand = Pose2D(candidate->relPosition.x, candidate->relPosition.y).relativeToGlobal(self);
  candidate->absPosition = Position(cand.translation.x, cand.translation.y, candidate->relPosition.z);
  return candidate;
}

std::vector<BallCandidate*> BallDetector::formBallCandidates(std::vector<Blob*> blobs) {
  std::vector<BallCandidate*> candidates;
  for(uint16_t i = 0; i < blobs.size() && i < MAX_BALL_CANDS; i++) {
    BallCandidate* candidate = formBallCandidate(blobs[i], i);
    candidates.push_back(candidate);
  }
  return candidates;
}

//bool BallDetector::setBestBallCandAsBall(uint16_t *sortIndex, WorldObject *ballCand) {
  //// Find
  //uint16_t maxConfIndex = (uint16_t)-1;
  //float maxConf = -1e6;
  //for (uint16_t i = 0; i < ballCandCount; i++) {
    //float conf = ballCand[i].visionConfidence;
    //if (conf > maxConf) {
      //maxConf = conf;
      //maxConfIndex = i;
    //}
  //}
  //return setBestBallCandidate(sortIndex, ballCand, maxConfIndex);
//}


bool BallDetector::setBestBallCandidate(BallCandidate* candidate){

  //if(ball->seen && !ball->fromTopCamera && camera_ == Camera::TOP) {
    //visionLog((34, "Ball was already selected by the bottom camera, throwing out."));
    //return false; // Always take balls from the bottom camera when available
  //}
  //ball->reset();

  // check if ball is way off field (like a kid in an orange shirt at the US open)
  //WorldObject *bestBallCand = &ballCand[index];
  //if (bestBallCand->visionDistance > 2000){
    //WorldObject* self = &(vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF]);
    //Point2D relBall = Point2D(bestBallCand->visionDistance,bestBallCand->visionBearing, POLAR);
    //Point2D absBall = relBall.relativeToGlobal(self->loc, self->orientation);
    //if (fabs(absBall.x) > ((FIELD_X / 2.0) + 2000) ||
        //fabs(absBall.y) > ((FIELD_Y / 2.0) + 2000)){
      //visionLog((34, " Ball thrown out for being too far off field, %f, %f",
                 //absBall.x, absBall.y));
      //return false;
    //}
  //}
  // Set
  WorldObject *ball = getball();
  if (candidate->confidence > 0) {
    ball->reset();
    ball->radius = candidate->radius;
    ball->imageCenterX = candidate->centerX;
    ball->imageCenterY = candidate->centerY;
    ball->visionDistance = candidate->groundDistance;
    ball->visionBearing = cmatrix_.bearing(candidate->relPosition);
    ball->visionElevation = cmatrix_.elevation(candidate->relPosition);
    ball->seen = true;
    ball->visionConfidence = candidate->confidence;
    ball->frameLastSeen = getframe();
    ball->fromTopCamera = (camera_ == Camera::TOP);
    bestBallCandIndex = candidate->index;
    //printf("selected ball at %3.0f,%3.0f dist=%2.2f, bear=%2.2f, elev=%2.2f\n",
        //candidate->absPosition.x, candidate->absPosition.y,
        //ball->visionDistance, ball->visionBearing * RAD_T_DEG, ball->visionElevation * RAD_T_DEG
        //);
    visionLog((34,"selected ball at %3.0f,%3.0f dist=%2.2f, bear=%2.2f, elev=%2.2f",
        candidate->absPosition.x, candidate->absPosition.y,
        ball->visionDistance, ball->visionBearing * RAD_T_DEG, ball->visionElevation * RAD_T_DEG
        ));

    return true;
  }
  else
    return false;
}

//std::vector<Blob*> BallDetector::sortOrangeBlobs(uint16_t *mergeIndex) {
  //std::vector<Blob*> sorted;
  //ballCandCount = 0;
  //for (uint16_t i = 0; i < horizontalBlob[c_ORANGE].size(); i++)
    //if (mergeIndex[i] == MAX_ORANGE_BLOBS)
      //sortIndex[ballCandCount++] = i;
  //for (uint16_t i = 0; i < ballCandCount - 1; i++) {
    //Blob *bli = &horizontalBlob[c_ORANGE][sortIndex[i]];
    //uint32_t areai = bli->dx * bli->dy;
    //for (uint16_t j = i + 1; j < ballCandCount; j++) {
      //Blob *blj = &horizontalBlob[c_ORANGE][sortIndex[j]];
      //uint32_t areaj = blj->dx * blj->dy;
      //if (areaj > areai) {
        //uint16_t temp = sortIndex[i];
        //sortIndex[i] = sortIndex[j];
        //sortIndex[j] = temp;
        //areai = areaj;
      //}
    //}
  //}
  //ballCandCount = std::min(ballCandCount, (uint16_t)MAX_BALL_CANDS);
//}

float BallDetector::getDirectDistanceByBlobWidth(float dx, int /*imageX*/, int /*imageY*/) {
  dx *= 640.0f / iparams_.width; // Normalize because this was tuned with a width of 640
  //float dist = 24269.169211 * powf(dx,-0.904299);  // tuned for exploreUT13
  float dist = 28699.871512 * powf(dx,-0.923806); // tuned on 6/14/13 by sbarrett and katie
  //printf("\tBall distance for diameter(%0.3f): %0.3f\n",dx,dist);
  return dist;
}

float BallDetector::getDirectDistanceByKinematics(int x, int y) {
  Position p = cmatrix_.getWorldPosition(x, y, BALL_RADIUS);
  float dist = cmatrix_.directDistance(p);
  return dist;
}

float BallDetector::getDirectDistance(BallCandidate* candidate) {
  float dist;
  float wdist = getDirectDistanceByBlobWidth(2.0f*candidate->radius, candidate->centerX, candidate->centerY);
  float kdist = getDirectDistanceByKinematics(candidate->centerX, candidate->centerY);

  // Kdist is better up close, wdist is better farther away. We scale the ratio of each so that we
  // don't get large discrepancies at the cutoff points
  float minKdist = 1000, maxKdist = 10000;
  float wdistRatio = pow((kdist - minKdist) / (maxKdist - minKdist), 2);
  if(kdist > maxKdist) dist = wdist;
  else if (kdist > minKdist) dist = kdist * (1 - wdistRatio) + wdist * wdistRatio;
  else dist = kdist;

  candidate->kwDistanceDiscrepancy = fabs(kdist - wdist) / (kdist + wdist);

  visionLog((34,"ball candidate (%i) width dist: %3.0f  kin dist: %3.0f  wdistRatio: %0.3f",candidate->index,wdist,kdist,wdistRatio));
  return dist;
}

bool BallDetector::bestCandidateFound() {
    return (bestBallCandIndex != ((uint16_t)-1));
}

void BallDetector::setHorizon(HorizonLine horizon) {
    horizon_ = horizon;
}

// DO NOT REMOVE and leave at bottom
// vim: expandtab:noai:sts=2:sw=2:ts=2
