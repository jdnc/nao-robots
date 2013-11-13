#include "GoalDetector.h"

GoalDetector::GoalDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector, LineDetector*& line_detector) : 
  DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector), line_detector_(line_detector), color(c_YELLOW) {
  
  YellowPostCounter = 0;

  yellowPosts = new FieldLine * [MAX_FIELDLINES];
  for (int i = 0; i < MAX_FIELDLINES; i++) {
    yellowPosts[i] = new FieldLine();
    yellowPosts[i]->id = i;
    yellowPosts[i]->PointsArray =
      new LinePoint * [MAX_POINTS_PER_LINE];

    for (int j = 0; j < MAX_POINTS_PER_LINE; j++)
      yellowPosts[i]->PointsArray[j] = new LinePoint();

  }
}

void GoalDetector::sanitizeGoalBlobs() {

  for (uint32_t j = 0; j < blob_detector_->horizontalBlob[color].size(); j++) {
    Blob * lb = &blob_detector_->horizontalBlob[color][j];
    float xDiff = (int16_t) lb->xf - (int16_t) lb->xi;
    float yDiff = (int16_t) lb->yf - (int16_t) lb->yi;
    float slope = yDiff / xDiff;
    lb->invalid = slope < 1 && slope > -1;
  }

}

float GoalDetector::estimateGoalDistanceByPosts(Position left, Position right) {
  float kdist = (cmatrix_.groundDistance(left) + cmatrix_.groundDistance(right)) / 2.0f;
  float apparentWidth = (left - right).abs();
  
  // When we assume the goal is farther away, we over-estimate its world width.
  float cdist = kdist * GOAL_WIDTH / apparentWidth;
  visionLog((35, "Both posts found, computing distance from apparent width: %2.f ( / %2.f = %2.2f), kdist: %2.f, corrected dist: %2.f", apparentWidth, GOAL_WIDTH, GOAL_WIDTH / apparentWidth, cdist));
  return cdist;
}

float GoalDetector::estimateGoalDistance(FieldLine * goal) {
  float y1 = goal->tL.y;
  float x1 = (int) ((y1 - goal->Offset) / goal->Slope);
  float y2 = goal->bR.y;
  float x2 = (int) ((y2 - goal->Offset) / goal->Slope);
  float x = x1 - x2, y = y1 - y2;

  float height = sqrtf(x * x + y * y);
  float width = fabs(goal->preciseWidth * sinf(goal->Angle));

  float wdist = cmatrix_.getWorldDistanceByWidth(width, GOAL_POST_WIDTH);
  float kdist = estimateGoalDistanceByKinematics(goal);

  // Width dist is bad at long distances due to the fact that goal posts are relatively thin.
  float wmin = 10.0f, wmax = 40.0f;
  float wratio = std::max(0.0f, std::min(1.0f, (width - wmin) / (wmax - wmin)));
  float kratio = 1.0f - wratio;
  float avg = (wdist * wratio + kdist * kratio) / (wratio + kratio);

  //printf("gw pct : %2.2f, width dist(%2.2f): %2.2f, height dist(%2.2f): %2.2f, kinematics dist(%2.2f,%2.2f): %2.2f, avg: %2.2f\n", greenWhite, width, wdist, height, hdist, goal->bR.x, goal->bR.y, kdist, avg);
  visionLog((35,"width dist(%2.2f): %2.2f, height dist(%2.2f): %2.2f, kinematics dist(%2.2f,%2.2f): %2.2f, avg: %2.2f", width, wdist, height, 0.0, goal->bR.x, goal->bR.y, kdist, avg));

  return avg;
}

float GoalDetector::estimateGoalDistanceByKinematics(FieldLine * goal) {
  Position p = cmatrix_.getWorldPosition(goal->bR.x, goal->bR.y);
  float dist = cmatrix_.groundDistance(p);
  return dist;
}

float GoalDetector::estimateGoalDistanceByHeight(float height) {
  height *= 480.0f / iparams_.height; // Normalize because this was tuned with a height of 480
  // piyushk 6/14/12
  //return 1243031.764766 * powf(height,-1.235303);
  // sbarrett 6/17/12
  return 863523.709600 * powf(height,-1.183317);
}

float GoalDetector::estimateGoalDistanceByWidth(float width) {
  width *= 640.0f / iparams_.width; // Normalize because this was tuned with a width of 640
  // piyushk 6/14/12
  //return 38438.155815 * powf(width,-0.905797);
  // sbarrett 6/17/13
  //return 21203.039034 * powf(width,-0.755975);
  // sbarrett 6/17/13 take 2
  return 47611.197494 * powf(width,-0.978952);
}

void GoalDetector::sanitizeGoalCandidate(FieldLine *goal) {

  if (!goal->ValidLine) {
      visionLog((35, "    Reject goal %i (ValidLine is false)", goal->id));
      return;
  }

  goal->confidence = 1.0;

  // Remove if not too vertical
  if (goal->Slope < 2 && goal->Slope > -2) {
    visionLog((35, "    Reject goal %i on slope %f (not vertical enough)", goal->id, goal->Slope));
    goal->ValidLine = false;
    totalValidGoals--;
    return;
  }

  // sanitize goals based on left-right edge
  visionLog((35,"    Goal avgX: %i width: %i",goal->avgX,goal->width));
  float left  = goal->avgX - goal->width * 0.5;
  float right = goal->avgX + goal->width * 0.5;
  if (left < 10 || right > iparams_.width - 10) {
    visionLog((35,  "    Reject Goal %i close to left-right edge %i", goal->id,goal->avgX));
    goal->ValidLine = false;
    totalValidGoals--;
    return;
  }

  // sanitize based on height to width
  float height = (float)goal->length;
  float width = fabs(goal->width * sinf(goal->Angle));
  float htw = height / width;

  float minhtw = .3 * 7.5, maxhtw = 4 * 7.5;
  if (htw > maxhtw || htw < minhtw) {
    visionLog((35,  "    Reject Goal %i htw %f = %2.2f/%2.2f outside [%2.2f,%2.2f] ", goal->id,htw, height, width, minhtw, maxhtw));
    goal->ValidLine = false;
    totalValidGoals--;
    return;
  }

  // Sanitize based on elevation of lower most point

  if ((goal->bR.y < iparams_.height - 5) &&
      (goal->avgX > 10 && goal->avgY < iparams_.width - 10)) {
    float distance = estimateGoalDistance(goal);
    Position p = cmatrix_.getWorldPositionByDirectDistance(goal->bR.x, goal->bR.y, distance);
    visionLog((35, "    Goal %i: vision (%5.0f, %5.0f), transform(%f, %f, %f), distance %f", goal->id, goal->bR.x, goal->bR.y, p.x, p.y, p.z, distance));
    if (p.z > 100) {
      visionLog((35, "      Reject goal %i on height of bottom point %f", goal->id, p.z));
      goal->ValidLine = false;
      totalValidGoals--;
      return;
    }
  }

  float greenWhitePercent = greenWhitePercentBelow(goal);
  float minPercent = .1;
  if(greenWhitePercent < minPercent) {
    visionLog((35,  "    Reject Goal %i due to green/white percent %5.3f (minimum %5.3f)", goal->id, greenWhitePercent, minPercent));
    goal->ValidLine = false;
    totalValidGoals--;
    return;
  }
}

void GoalDetector::setGoalObject(int goalIndex, float distance, float bearing, float elevation, int centerX, int centerY, float confidence, int lineIndex) {
  vblocks_.world_object->objects_[goalIndex].seen = true;
  vblocks_.world_object->objects_[goalIndex].visionBearing = bearing;
  vblocks_.world_object->objects_[goalIndex].visionElevation = elevation;
  vblocks_.world_object->objects_[goalIndex].visionDistance = distance;
  vblocks_.world_object->objects_[goalIndex].imageCenterX = centerX;
  vblocks_.world_object->objects_[goalIndex].imageCenterY = centerY;
  vblocks_.world_object->objects_[goalIndex].frameLastSeen = vblocks_.frame_info->frame_id;
  vblocks_.world_object->objects_[goalIndex].visionConfidence = confidence;
  vblocks_.world_object->objects_[goalIndex].fieldLineIndex = lineIndex;
  vblocks_.world_object->objects_[goalIndex].fromTopCamera = (camera_ == Camera::TOP);

}

void GoalDetector::FormGoal() {
  // We saw too many valid goal posts, this looks suspicious, or we saw none at all
  visionLog((31, "FormGoal"));
  if (totalValidGoals > 2 || totalValidGoals < 1) {
    return;
  }

  int index1 = -1, index2 = -1;
  for (int goal = 0; goal < line_detector_->currentLineCounter; goal++) {
    if (currentLine[goal]->ValidLine) {
      if (index1 == -1) {
        index1 = goal;
        if (totalValidGoals != 2) {
          break;
        }
      } else {
        index2 = goal;
        break;
      }
    }
  }

  if(index1 < 0 || (index2 < 0 && totalValidGoals > 1)) {
    visionLog((32, " invalid goal indexes - THIS SHOULD NOT HAPPEN WHAT DID YOU DO"));
    return;
  }

  if (totalValidGoals == 1) { // Unknown Goal Post
    visionLog((32, "    Single Goal Post Detected:-"));
    FieldLine* post = currentLine[index1];

    // The distance estimate also gives us some confidence on its value
    // Values relative to camera
    float confidence = post->confidence;
    float headDistance = estimateGoalDistance(post);
    int headX = post->avgX;
    int headY = post->avgY;
    //Position p = cmatrix_.getWorldPosition(post->avgX, post->avgY);
    Position p = cmatrix_.getWorldPositionByGroundDistance(post->bR.x, post->bR.y, headDistance);
    // Values relative to body

    // Attempt to find which goal post it based on intersections
    bool canCalculate = false;
    bool isLeft = false;
    for (int corner = 0; corner < cornerPointCounter; corner++) {
      if (cornerPoints[corner]->CornerType) {
        canCalculate = true;
        isLeft = cornerPoints[corner]->PosX < post->avgX;
      }
    }
    int postIndex = WO_UNKNOWN_GOALPOST;
    if (canCalculate) {
      visionLog((32,  "        Post Location detected using T intersection next to goal post as (left = 1, right = 0): %i"));
      postIndex = (isLeft) ? WO_UNKNOWN_LEFT_GOALPOST : WO_UNKNOWN_RIGHT_GOALPOST;
    }

    // Set the unknown goal post
    setGoalObject(postIndex, cmatrix_.groundDistance(p), cmatrix_.bearing(p), cmatrix_.elevation(p),
                  headX, headY,
                  confidence, post->id);

  } else { // Both Goal Posts seen, Yay!

    // Information for post 1
    visionLog((32, "     Both Goal Posts Detected:-"));

    visionLog((32, "     Post 1:-"));
    float confidence1 = currentLine[index1]->confidence;
    float headDistance1 = estimateGoalDistance(currentLine[index1]);
    Position p1 = cmatrix_.getWorldPosition(currentLine[index1]->bR.x, currentLine[index1]->bR.y);
    
    visionLog((32, "     Post 2:-"));
    float confidence2 = currentLine[index2]->confidence;
    float headDistance2 = estimateGoalDistance(currentLine[index2]);
    Position p2 = cmatrix_.getWorldPosition(currentLine[index2]->bR.x, currentLine[index2]->bR.y);
    
    float confidence = confidence1 * confidence2;
    float d1 = cmatrix_.groundDistance(p1), d2 = cmatrix_.groundDistance(p2);
    float b1 = cmatrix_.bearing(p1), b2 = cmatrix_.bearing(p2);
    float e1 = cmatrix_.elevation(p1), e2 = cmatrix_.elevation(p2);
    float bearing = (b1 + b2) / 2, elevation = (e1 + e2) / 2;

    // Decide which post is left and which post is right
    Position left = p1, right = p2;
    int post1 = WO_UNKNOWN_LEFT_GOALPOST;
    int post2 = WO_UNKNOWN_RIGHT_GOALPOST;
    if (currentLine[index2]->avgX < currentLine[index1]->avgX) {
      post1 = WO_UNKNOWN_RIGHT_GOALPOST;
      post2 = WO_UNKNOWN_LEFT_GOALPOST;
      left = p2;
      right = p1;
    }
    float distancePosts = estimateGoalDistanceByPosts(left, right);
    float distanceAvg = (headDistance1 + headDistance2) / 2;
    float distance = (distancePosts + distanceAvg) / 2;
    distance *= .9; // measured through extensive analysis

    // Set Goals and Goal Posts
    setGoalObject(WO_UNKNOWN_GOAL, distance, bearing, elevation,
                  (currentLine[index1]->avgX + currentLine[index2]->avgX) / 2,
                  (currentLine[index1]->avgY + currentLine[index2]->avgY) / 2,
                  confidence, -1);
    setGoalObject(post1, d1, b1, e1,
                  currentLine[index1]->avgX, currentLine[index1]->avgY,
                  confidence1, currentLine[index1]->id);
    setGoalObject(post2, d2, b2, e2,
                  currentLine[index2]->avgX, currentLine[index2]->avgY,
                  confidence2, currentLine[index2]->id);
  }
}

void GoalDetector::resetYellowGoal() {
  visionLog((32, "resetYellowGoal()"));
  YellowPostCounter = 0;
  totalValidGoals = 0;
}

void GoalDetector::FormYellowGoal() {
  visionLog((32, "FormYellowGoal()"));

  currentLine = yellowPosts;
  color = c_YELLOW;
  line_detector_->currentLineCounter = 0;

  blob_detector_->horizontalBlobSort(color);
  blob_detector_->calculateHorizontalBlobData(color);
  visionLog((32, "%i yellow blobs for goal formation\n", blob_detector_->horizontalBlob[c_YELLOW].size()));
  sanitizeGoalBlobs();
  line_detector_->FindVertLines(currentLine, color);

  visionLog((32, "     %i initial detections", line_detector_->currentLineCounter));

  YellowPostCounter = line_detector_->currentLineCounter;
  totalValidGoals = line_detector_->currentLineCounter;

  // Calculate the details for the lines comprising of the goal
  for (short line = 0; line < YellowPostCounter; line++) {
    line_detector_->CalLineDetails(yellowPosts[line]);
  }
  for (short lineTo = 0; lineTo < YellowPostCounter; lineTo++) {
    if (!yellowPosts[lineTo]->ValidLine)
      continue;
    for (short lineFrom = lineTo + 1; lineFrom < YellowPostCounter; lineFrom++) {
      if (!yellowPosts[lineFrom]->ValidLine)
        continue;
      mergeLinesVertical(yellowPosts[lineTo], yellowPosts[lineFrom]);
    }
  }
  for (short line = 0; line < YellowPostCounter; line++) {
    sanitizeGoalCandidate(yellowPosts[line]);
  }

  FormGoal();

}


bool GoalDetector::mergeLinesVertical(FieldLine * lineTo, FieldLine * lineFrom) {

  if (!lineTo->ValidLine || !lineFrom->ValidLine) {
    return false;
  }

  // This is an extra sanitary check. If multiple detections are made,
  // the goals should always have similar widths or something is crazy
  if ((lineTo->width / lineFrom->width) > 6.0){
    lineFrom->ValidLine = false;
    totalValidGoals--;
    return false;
  } else if ((lineFrom->width / lineTo->width) > 6.0){
    lineTo->ValidLine = false;
    totalValidGoals--;
    return false;
  }

  float angleDiff = abs(lineTo->Angle - lineFrom->Angle);
  if (angleDiff < (M_PI / 6) || angleDiff > (M_PI - M_PI / 6)) {

    // Calculate Point of intersection
    float xInt = (lineTo->Offset - lineFrom->Offset) / (lineFrom->Slope - lineTo->Slope);
    float yInt = xInt * lineTo->Slope + lineTo->Offset;
    uint16_t avgDiff = abs(lineTo->avgX - lineFrom->avgX);
    float avgWidth = (lineTo->width + lineFrom->width) / 2.0;

    float yOffsetDiff = lineTo->Offset - lineFrom->Offset;
    yOffsetDiff *= cosf(lineTo->Angle);
    yOffsetDiff = fabs(yOffsetDiff);
    float xOffsetDiff = lineTo->Offset / lineTo->Slope - lineFrom->Offset / lineFrom->Slope;
    xOffsetDiff *= sinf(lineTo->Angle);
    xOffsetDiff = fabs(xOffsetDiff);

    if ((yInt > -iparams_.height / 10 && yInt < 11 * iparams_.height / 10) || avgDiff < avgWidth || xOffsetDiff < 64) {

      float posX[MAX_POINTS_PER_LINE];
      float posY[MAX_POINTS_PER_LINE];
      float width[MAX_POINTS_PER_LINE];

      uint16_t counter, counter1 = 0, counter2 = 0;
      for (counter = 0; counter < lineTo->Points + lineFrom->Points && counter < MAX_POINTS_PER_LINE; counter++) {
        if (counter2 == lineFrom->Points || (lineTo->PointsArray[counter1]->PosY <= lineFrom->PointsArray[counter2]->PosY && counter1 < lineTo->Points)) {
          posX[counter] = lineTo->PointsArray[counter1]->PosX;
          posY[counter] = lineTo->PointsArray[counter1]->PosY;
          width[counter] = lineTo->PointsArray[counter1]->Width;
          counter1++;
        } else {
          posX[counter] = lineFrom->PointsArray[counter2]->PosX;
          posY[counter] = lineFrom->PointsArray[counter2]->PosY;
          width[counter] = lineFrom->PointsArray[counter2]->Width;
          counter2++;
        }
      }

      for (counter = 0; counter < lineTo->Points + lineFrom->Points && counter < MAX_POINTS_PER_LINE; counter++) {
        lineTo->PointsArray[counter]->PosX = posX[counter];
        lineTo->PointsArray[counter]->PosY = posY[counter];
        lineTo->PointsArray[counter]->Width = width[counter];
      }

      lineTo->Points = lineTo->Points + lineFrom->Points;
      if (lineTo->Points > MAX_POINTS_PER_LINE)
        lineTo->Points = MAX_POINTS_PER_LINE;

      line_detector_->CalLineDetails(lineTo);
      lineFrom->ValidLine = false;
      totalValidGoals--;

      return true;

    }
  }

  return false;

}

void GoalDetector::setCornerPoints(CornerPoint** points){
  cornerPoints = new CornerPoint * [MAX_CORNERPOINTS];
  for (int i = 0; i < MAX_CORNERPOINTS; i++)
    cornerPoints[i] = points[i];
}


void GoalDetector::setCornerPointCounter(int value) {
  cornerPointCounter = value;
}

float GoalDetector::greenWhitePercentBelow(FieldLine* post){
  int hstep, vstep;
  classifier_->getStepSize(hstep,vstep);

  int
    xmin = post->avgX - post->width / 2.0,
    xmax = post->avgX + post->width / 2.0,
    ymin = post->avgY + post->length / 2.0 + post->length * .02, // rounding on the post increases the length by about 2%
    ymax = post->avgY + post->length / 2.0 + post->length * .2; // this worked in testing, could be increased or decreased

  xmin = std::max(xmin, 0) - (xmin % hstep);
  xmax = std::min(xmax, iparams_.width - 1) - (xmax % hstep);
  ymin = std::max(ymin, 0) - (ymin % vstep);
  ymax = std::min(ymax, iparams_.height - 1) - (ymax % vstep);

  int numTotal = 0;
  int numGreen = 0;
  for (int x = xmin; x <= xmax; x += hstep){
    for (int y = ymin; y <= ymax; y += vstep){
      numTotal++;
      Color c = classifier_->xy2color(x,y);
      if (c == c_FIELD_GREEN || c == c_WHITE) numGreen++;
    }
  }

  float pct = (float)numGreen/(float)numTotal;

  if(!numTotal && ymin >= iparams_.height)
  // We can't see below the post due to the height, so we assume 1. There
  // is no reason the robot should see anything else that's yellow and that
  // extends vertically below the lower boundary of the image.
    pct = 1;

  visionLog((43, "check from x: %i, %i, and y: %i, %i, total: %i, green: %i, pct: %5.3f", xmin, xmax, ymin, ymax, numTotal, numGreen, pct));
  return pct;
}
