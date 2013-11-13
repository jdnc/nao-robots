#include "LineDetector.h"

#define horizontalBlob blob_detector_->horizontalBlob
#define verticalBlob blob_detector_->verticalBlob

using namespace std;

LineDetector::LineDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) :
  DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) {

  FieldLinesCounter = 0;
  CornerPointCounter = 0;

  cornerPoints = new CornerPoint * [MAX_CORNERPOINTS];
  for (int i = 0; i < MAX_CORNERPOINTS; i++){
    cornerPoints[i] = new CornerPoint();
    cornerPoints[i]->Valid = false;
  }

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

  horizontalPoint = classifier_->horizontalPoint;
  verticalPoint = classifier_->verticalPoint;
  horizontalPointCount = classifier_->horizontalPointCount;
  verticalPointCount = classifier_->verticalPointCount;
}

void LineDetector::FindHorzLines(FieldLine** currentLine, unsigned char color) {

  // Compute Required Line Dat
  for (uint16_t i = 0; i < verticalBlob[color].size(); i++) {
    if (currentLineCounter == MAX_FIELDLINES) {
      visionLog((45, "Horizontal lines at max count of %i", MAX_FIELDLINES));
      break;
    }

    Blob *lbNew = &verticalBlob[color][i];
    //std::cout << "checking blob " << i << "\n";
    if (lbNew->invalid) continue;
    //std::cout << "1\n";

    LineStackInfo lineStack;
    lineStack.doubleDiff = lbNew->doubleDiff;
    lineStack.pointCount = lbNew->lpCount;
    lineStack.diff = lbNew->diffEnd;
    lineStack.width = lbNew->widthEnd;
    lineStack.diffStart = lbNew->diffStart;
    lineStack.posStart = lbNew->xi;

    // If line is found translate to fieldLines Data Structure
    LineStackRet retVal = recurseCheckHorizontal(i, lineStack, color);
    if (retVal.isLine) {
        //std::cout << "2\n";
      currentLine[currentLineCounter]->ValidLine = true;
      currentLine[currentLineCounter]->Points = 0;
      currentLine[currentLineCounter]->isVertical = false;
      currentLine[currentLineCounter]->isHorizontal = true;
      for (int16_t j = retVal.lbCount - 1; j >= 0; j--) {
        Blob *lb = &verticalBlob[color][retVal.lbIndex[j]];
        for (uint16_t k = 0; k < lb->lpCount; k++) {
          VisionPoint *lp = &verticalPoint[color][lb->lpIndex[k] >> 16][lb->lpIndex[k] & 0xfffful];
          currentLine[currentLineCounter]->PointsArray[currentLine[currentLineCounter]->Points]->PosX = lp->xi;
          currentLine[currentLineCounter]->PointsArray[currentLine[currentLineCounter]->Points]->PosY = (lp->yi + lp->yf) / 2;
          currentLine[currentLineCounter]->PointsArray[currentLine[currentLineCounter]->Points]->Width = lp->yf - lp->yi;
          currentLine[currentLineCounter]->Points++;
          //std::cout << "(" << lp->xi << "," << lp->yi << ");";
        }
        lb->invalid = true;
      }
      currentLineCounter++;
    }
  }
  //std::cout << "\n";
}

void LineDetector::FindVertLines(FieldLine** currentLine, unsigned char color) {

  // Compute Required Line Data
  for (uint16_t i = 0; i < horizontalBlob[color].size(); i++) {
    if (currentLineCounter == MAX_FIELDLINES) {
      visionLog((45, "Vertical lines at max count of %i", MAX_FIELDLINES));
      break;
    }
    //printf ("Blob Index from main loop: %i\n", i);

    Blob *lbNew = &horizontalBlob[color][i];
    if (lbNew->invalid) continue;

    LineStackInfo lineStack;
    lineStack.doubleDiff = lbNew->doubleDiff;
    lineStack.pointCount = lbNew->lpCount;
    lineStack.diff = lbNew->diffEnd;
    lineStack.width = lbNew->widthEnd;
    lineStack.diffStart = lbNew->diffStart;
    lineStack.posStart = lbNew->yi;

    // If line is found translate to fieldLines Data Structure
    LineStackRet retVal = recurseCheckVertical(i, lineStack, color);
    if (retVal.isLine) {
      currentLine[currentLineCounter]->ValidLine = true;
      currentLine[currentLineCounter]->Points = 0;
      currentLine[currentLineCounter]->isVertical = true;
      currentLine[currentLineCounter]->isHorizontal = false;
      for (int16_t j = retVal.lbCount - 1; j >= 0; j--) {
        Blob *lb = &horizontalBlob[color][retVal.lbIndex[j]];
        for (uint16_t k = 0; k < lb->lpCount; k++) {
          VisionPoint *lp = &horizontalPoint[color][lb->lpIndex[k] >> 16][lb->lpIndex[k] & 0xfffful];
          currentLine[currentLineCounter]->PointsArray[currentLine[currentLineCounter]->Points]->PosY = lp->yi;
          currentLine[currentLineCounter]->PointsArray[currentLine[currentLineCounter]->Points]->PosX = (lp->xi + lp->xf) / 2;
          currentLine[currentLineCounter]->PointsArray[currentLine[currentLineCounter]->Points]->Width = lp->xf - lp->xi;
          currentLine[currentLineCounter]->Points++;
        }
        lb->invalid = true;
      }
      currentLineCounter++;
    }

  }
}

LineStackRet LineDetector::recurseCheckHorizontal (uint16_t startIndex, LineStackInfo lineStack, unsigned char color) {

  Blob *lbOrig = &verticalBlob[color][startIndex];
  VisionPoint *lpOrigEnd = &verticalPoint[color][lbOrig->lpIndex[lbOrig->lpCount - 1] >> 16][lbOrig->lpIndex[lbOrig->lpCount - 1] & 0xfffful];
  int16_t xOrig = lpOrigEnd->xi, yOrig = (lpOrigEnd->yi + lpOrigEnd->yf) / 2;

  // Equation of the curve will be of the form y = a(x^2) + b(x) + c;
  // Reconstructing from xorig, yorig & y' & y''
  float aVal = lineStack.doubleDiff / 2.0;
  float bVal = lineStack.diff - lineStack.doubleDiff * xOrig;
  float cVal = yOrig - aVal * xOrig * xOrig - bVal * xOrig;

  LineStackRet bestRetVal;
  bestRetVal.lbCount = 0;
  bestRetVal.pointCount = 0;
  bestRetVal.isLine = false;

  for (uint16_t j = startIndex + 1; j < verticalBlob[color].size(); j++) {

    Blob *lbNew = &verticalBlob[color][j];
    VisionPoint *lpNewStart = &verticalPoint[color][lbNew->lpIndex[0] >> 16][lbNew->lpIndex[0] & 0xfffful];

    if (lbNew->invalid || lbNew->lpIndex[0] >> 16 <= (unsigned int) xOrig) {
      continue;
    }

    if ((lbNew->lpIndex[0] >> 16) > (unsigned int) xOrig + (iparams_.width >> vparams_.horzSeparation[color])) {
      break;
    }

    uint16_t xNew = lpNewStart->xi, yNew = (lpNewStart->yi + lpNewStart->yf) / 2;
    int16_t yExpected = aVal * xNew * xNew + bVal * xNew + cVal;
    float slopeExpected = 2 * aVal * xNew + bVal;
    float angleExpected = atanf(slopeExpected);

    // Calculate Distance factor
    int16_t yDiff = yExpected - yNew;
    yDiff *= cosf(angleExpected);
    bool posOk = !vparams_.useHorzPos[color] || (yDiff <= vparams_.horzPosLimit[color] && yDiff >= -vparams_.horzPosLimit[color]);

    // Calculate Slope Factor
    float angleDiff = atanf(lbNew->diffStart) - angleExpected;  // Slope should never be pi / 2, so this is fine
    angleDiff *= !(lbNew->lpCount == 2);
    bool angleOk = !vparams_.useHorzAngle[color] || (angleDiff <= vparams_.horzAngleLimit[color] && angleDiff >= -vparams_.horzAngleLimit[color]);

    // Calculate Width Factor
    int16_t width1 = lbNew->widthStart, width2 = lineStack.width;
    int16_t widthDiff = width1 - width2;
    widthDiff *= 2 * (width1 > width2) - 1; // make positive
    widthDiff -= 2;                         // avoid low differences
    widthDiff *= (widthDiff > 0);           // make non-negative
    int16_t widthMax = width1 + (width1 < width2) * (width2 - width1); // get max width
    float widthFactor = ((float) widthDiff) / widthMax;
    bool widthOk = !vparams_.useHorzWidth[color] || (widthFactor <= vparams_.horzWidthLimit[color] && widthFactor >= -vparams_.horzWidthLimit[color]);

    visionLog((35,  "%i -> %i: (%i, %i, %i) for (%i, %f, %f)", startIndex, j, posOk, angleOk, widthOk, yDiff, angleDiff, widthFactor));

    if (widthOk && angleOk && posOk) {

      visionLog((35,  "%i -> %i", startIndex, j));

      if (lbNew->lpCount == 2) {
        lineStack.diff = yNew - yOrig;
        lineStack.diff /= xNew - xOrig;
      } else {
        lineStack.diff = lbNew->diffEnd;
      }
      lineStack.pointCount += lbNew->lpCount;
      lineStack.doubleDiff = (lineStack.diff - lineStack.diffStart);
      lineStack.doubleDiff /= (lbNew->yf - lineStack.posStart);
      lineStack.width = lbNew->widthEnd;
      LineStackRet retVal = recurseCheckHorizontal(j, lineStack, color);

      if (retVal.isLine) {
        if (retVal.lbCount < MAX_BLOBS_PER_LINE) {
          retVal.lbIndex[retVal.lbCount++] = startIndex;
        } else {
          visionLog((45, "This one is pretty horrible.... MAX_BLOBS_PER_LINE exceeded"));
        }
        retVal.pointCount += lbOrig->lpCount;

        return retVal;
      } else if (vparams_.horzValA[color] * retVal.pointCount + vparams_.horzValB[color] * retVal.lbCount > vparams_.horzValA[color] * bestRetVal.pointCount + vparams_.horzValB[color] * bestRetVal.lbCount) {
        bestRetVal = retVal;
      }

    }
  }

  // Add data to the best return value
  if (bestRetVal.lbCount < MAX_BLOBS_PER_LINE) {
    bestRetVal.lbIndex[bestRetVal.lbCount++] = startIndex;
  } else {
    visionLog((45, "This one is pretty horrible.... MAX_BLOBS_PER_LINE exceeded"));
  }
  bestRetVal.pointCount += lbOrig->lpCount;

  // Decide whether its a line or not
  if (vparams_.horzValA[color] * bestRetVal.pointCount + vparams_.horzValB[color] * bestRetVal.lbCount > vparams_.horzValThreshold[color]) {
    bestRetVal.isLine = true;
  }

  visionLog((35, "Returning from %i, isLine: %i, numPoints: %i, numBlobs: %i, A: %i, B: %i, Threshold: %i", startIndex, bestRetVal.isLine, bestRetVal.pointCount, bestRetVal.lbCount, vparams_.horzValA[color], vparams_.horzValB[color], vparams_.horzValThreshold[color]));
  return bestRetVal;
}

LineStackRet LineDetector::recurseCheckVertical (uint16_t startIndex, LineStackInfo lineStack, unsigned char color) {

  Blob *lbOrig = &horizontalBlob[color][startIndex];
  VisionPoint *lpOrigEnd = &horizontalPoint[color][lbOrig->lpIndex[lbOrig->lpCount - 1] >> 16][lbOrig->lpIndex[lbOrig->lpCount - 1] & 0xfffful];
  int16_t xOrig = (lpOrigEnd->xi + lpOrigEnd->xf) / 2, yOrig = lbOrig->lpIndex[lbOrig->lpCount - 1] >> 16;

  // Equation of the curve will be of the form y = a(x^2) + b(x) + c;
  // Reconstructing from xorig, yorig & y' & y''
  float aVal = lineStack.doubleDiff / 2.0;
  float bVal = lbOrig->diffEnd - lineStack.doubleDiff * yOrig;
  float cVal = xOrig - aVal * yOrig * yOrig - bVal * yOrig;

  LineStackRet bestRetVal;
  bestRetVal.lbCount = 0;
  bestRetVal.pointCount = 0;
  bestRetVal.isLine = false;

  for (uint16_t j = startIndex + 1; j < horizontalBlob[color].size(); j++) {

    Blob *lbNew = &horizontalBlob[color][j];
    VisionPoint *lpNewStart = &horizontalPoint[color][lbNew->lpIndex[0] >> 16][lbNew->lpIndex[0] & 0xfffful];

    if (lbNew->invalid || lbNew->lpIndex[0] >> 16 <= (unsigned int)yOrig) {
      continue;
    }

    if ((lbNew->lpIndex[0] >> 16) > (unsigned int)yOrig + (iparams_.height >> vparams_.vertSeparation[color])) {
      break;
    }

    int16_t xNew = (lpNewStart->xi + lpNewStart->xf) >> 1, yNew = lbNew->lpIndex[0] >> 16;
    int16_t xExpected = aVal * yNew * yNew + bVal * yNew + cVal;
    float slopeExpected = 2 * aVal * yNew + bVal;
    float angleExpected = atanf(slopeExpected);

    // Calculate Distance factor
    int16_t xDiff = xExpected - xNew;
    xDiff *= cosf(angleExpected);
    bool posOk = !vparams_.useVertPos[color] || (xDiff <= vparams_.vertPosLimit[color] && xDiff >= -vparams_.vertPosLimit[color]);

    // Calculate Slope Factor
    float angleDiff = atan(lbNew->diffStart) - angleExpected;  // slope shoulde never be pi / 2, so this is ok
    angleDiff *= !(lbNew->lpCount == 2);                             // 2 point blob contain too little slope information
    bool angleOk = !vparams_.useVertAngle[color] || (angleDiff <= vparams_.vertAngleLimit[color] && angleDiff >= -vparams_.vertAngleLimit[color]);

    // Calculate Width Factor
    int16_t width1 = lbNew->widthStart, width2 = lineStack.width;
    int16_t widthDiff = width1 - width2;
    widthDiff *= 2 * (width1 > width2) - 1; // make positive
    widthDiff -= 4;                         // avoid low differences
    widthDiff *= (widthDiff > 0);           // make non-negative
    int16_t widthMax = width1 + (width1 < width2) * (width2 - width1); // get max width
    float widthFactor = ((float) widthDiff) / widthMax;
    bool widthOk = !vparams_.useVertWidth[color] || (widthFactor <= vparams_.vertWidthLimit[color] && widthFactor >= -vparams_.vertWidthLimit[color]);

    if (widthOk && angleOk && posOk) {

      // Calculate the new values in the line stack
      if (lbNew->lpCount == 2) {
        lineStack.diff = xNew - xOrig;
        lineStack.diff /= yNew - yOrig;
      } else {
        lineStack.diff = lbNew->diffEnd;
      }
      lineStack.pointCount += lbNew->lpCount;
      lineStack.doubleDiff = (lineStack.diff - lineStack.diffStart);
      lineStack.doubleDiff /= (lbNew->xf - lineStack.posStart);
      lineStack.width = lbNew->widthEnd;

      LineStackRet retVal = recurseCheckVertical(j, lineStack, color);
      if (retVal.isLine) {
        if (retVal.lbCount < MAX_BLOBS_PER_LINE) {
          retVal.lbIndex[retVal.lbCount++] = startIndex;
        } else {
          visionLog((45, "This one is pretty horrible.... MAX_BLOBS_PER_LINE exceeded"));
        }
        retVal.pointCount += lbOrig->lpCount;
        return retVal;
      } else if (vparams_.vertValA[color] * retVal.pointCount + vparams_.vertValB[color] * retVal.lbCount > vparams_.vertValA[color] * bestRetVal.pointCount + vparams_.vertValB[color] * bestRetVal.lbCount) {
        bestRetVal = retVal;
      }

    }
  }

  // Add data to the best return value
  if (bestRetVal.lbCount < MAX_BLOBS_PER_LINE) {
    bestRetVal.lbIndex[bestRetVal.lbCount++] = startIndex;
  } else {
    visionLog((45, "This one is pretty horrible.... MAX_BLOBS_PER_LINE exceeded"));
  }
  bestRetVal.pointCount += lbOrig->lpCount;

  // Decide whether its a line or not
  if (vparams_.vertValA[color] * bestRetVal.pointCount + vparams_.vertValB[color] * bestRetVal.lbCount > vparams_.vertValThreshold[color]) {
    bestRetVal.isLine = true;
  }
  return bestRetVal;
}

void LineDetector::FormLines(unsigned char color) {
  visionLog((32, "FormLines()"));

  CornerPointCounter = 0;
  currentLineCounter = 0;

  blob_detector_->verticalBlobSort(color);
  //blob_detector_->CalHorzBlobData(color); // shouldn't be necessary anymore since CalHorzBlobData is done in formWhiteLineBlobs
  FindHorzLines(fieldLines, color);

  blob_detector_->horizontalBlobSort(color);
  //blob_detector_->CalVertBlobData(color); // shouldn't be necessary anymore since CalVertBlobData is done in formWhiteLineBlobs
  FindVertLines(fieldLines, color);

  FieldLinesCounter = currentLineCounter;
  TotalValidLines = FieldLinesCounter;

  if (TotalValidLines > 12) {
    for (short line = 0; line < FieldLinesCounter; line++) {
      fieldLines[line]->ValidLine = false;
      TotalValidLines--;
    }
  } else {
    for (short line = 0; line < FieldLinesCounter; line++) {
      CalLineDetails(fieldLines[line]);
    }
  }

  visionLog((30, "After forming, num valid lines are %i", TotalValidLines));

}

void LineDetector::RemoveLinesFromBody() {
  if (camera_ == Camera::TOP) return;
  for (int i = 0; i < FieldLinesCounter; i++) {
    FieldLine* line = fieldLines[i];
    if (!line->ValidLine) {
      continue;
    }
    if (((getHeadTilt()>DEG_T_RAD*10) && (line->MaxY>iparams_.height/2.0)) || (getHeadTilt()>DEG_T_RAD*20 && fabs(getHeadPan())>DEG_T_RAD*30) ) {
      line->ValidLine=false;
      TotalValidLines--;
    }
  }
  visionLog((30, "After remove lines from body, num valid lines are %i", TotalValidLines));
}

void LineDetector::differentiateCurves() {

  uint16_t stride;

  bool foundCircle = false;
  float cx, cy, radius;

  for (int line = 0; line < FieldLinesCounter; line++) {

    stride = fieldLines[line]->Points / 2 - 1;

    if (!fieldLines[line]->ValidLine)
      continue;

    float prevtx, prevty, currtx, currty, prevSlope, currSlope, avgRateSlope = 0;

    // for potential circle
    float tx1, ty1, tx2, ty2, tx3, ty3;

    Position p = cmatrix_.getWorldPosition(fieldLines[line]->PointsArray[0]->PosX, fieldLines[line]->PointsArray[0]->PosY);
    fieldLines[line]->TranPointsArray[0]->PosX = p.x;
    fieldLines[line]->TranPointsArray[0]->PosY = p.y;

    prevtx = p.x;
    prevty = p.y;

    p = cmatrix_.getWorldPosition(fieldLines[line]->PointsArray[stride]->PosX, fieldLines[line]->PointsArray[stride]->PosY);
    fieldLines[line]->TranPointsArray[stride]->PosX = p.x;
    fieldLines[line]->TranPointsArray[stride]->PosY = p.y;

    currtx = p.x;
    currty = p.y;

    // save these for later circle fit

    tx1 = prevtx;
    ty1 = prevty;
    tx2 = currtx;
    ty2 = currty;
    tx3 = 0; // suppress warning
    ty3 = 0; // suppress warning

    prevSlope = (currty - prevty) / (currtx - prevtx);
    prevtx = currtx, prevty = currty;

    uint16_t numPoints = 0;

    for (unsigned int point = 2 * stride; point < fieldLines[line]->Points; point += stride) {
      Position p = cmatrix_.getWorldPosition(fieldLines[line]->PointsArray[point]->PosX, fieldLines[line]->PointsArray[point]->PosY);

      fieldLines[line]->TranPointsArray[point]->PosX = p.x;
      fieldLines[line]->TranPointsArray[point]->PosY = p.y;

      currtx = p.x;
      currty = p.y;

      // save these for later circle fit
      tx3 = currtx;
      ty3 = currty;

      currSlope = (currty - prevty) / (currtx - prevtx);
      avgRateSlope += fabs(atanf(currSlope) - atanf(prevSlope)) * 180.0 / M_PI;
      prevtx = currtx, prevty = currty;
      prevSlope = currSlope;

      numPoints++;

    }

    fieldLines[line]->onCircle = false;
    fieldLines[line]->isCircle = false;
    fieldLines[line]->isCurve = false;
    fieldLines[line]->isCross = false;

    // lets calculate distance here
    float dist = sqrtf((tx1-tx3)*(tx1-tx3)+(ty1-ty3)*(ty1-ty3));

    if (dist < 200) {
      visionLog((32, "threw out line for being too short. distance: %5.3f", dist));
      fieldLines[line]->ValidLine = false;
      TotalValidLines--;
      continue;
    }

    avgRateSlope /= numPoints;
    fieldLines[line]->rateSlope = avgRateSlope;

    visionLog((32, "line dist %5.3f, avg rate slope %5.3f", dist, avgRateSlope));

    // Todd: I think 6 should throw out 99% of curves
    // as opposed to 15 that you were using
    // when in doubt, throw it out!
    if (avgRateSlope > 6 && avgRateSlope < 180 - 6 && !fieldLines[line]->isCross){
      fieldLines[line]->isCircle = false;
      if (!foundCircle)
        foundCircle = formCircle(tx1, ty1, tx2, ty2, tx3, ty3, fieldLines[line], &cx, &cy, &radius, dist);

      fieldLines[line]->isCurve = true;
      TotalValidLines--;
    }

    // Todd: stricter curve check on short lines
    else if (dist < 300 && avgRateSlope > 1.6 && avgRateSlope < (180-1.6)){
      visionLog((32, "Line %i, bad slope %f for short line %f", line, avgRateSlope, dist));
      fieldLines[line]->isCurve = true;
      TotalValidLines--;
    }

  }

  // if we found a circle, go back through and throw out lines that are on it
  if (foundCircle){
    for (int line = 0; line < FieldLinesCounter; line++) {

      if (!fieldLines[line]->ValidLine || fieldLines[line]->isCircle || fieldLines[line]->isCurve)
        continue;

      stride = fieldLines[line]->Points / 2 - 1;
      for (unsigned int pt = 0; pt < fieldLines[line]->Points; pt += stride) {

        //calculate distance from center
        float x1 = fieldLines[line]->TranPointsArray[pt]->PosX;
        float y1 = fieldLines[line]->TranPointsArray[pt]->PosY;
        float dist = sqrtf((cx-x1)*(cx-x1)+(cy-y1)*(cy-y1));

        // if we're off from the radius, then break out, this line is ok
        if (fabs(dist - radius) > 250)
          break;

        // if we're on the last point, then this line is on the circle
        if (pt == (unsigned int)2 * stride){
          fieldLines[line]->onCircle = true;
          TotalValidLines--;
        }

      }
    }
  }
  visionLog((30, "After diff curves, num valid lines are %i", TotalValidLines));
}

bool LineDetector::formCircle(float x1, float y1, float x2, float y2, float x3, float y3, FieldLine* Line, float *cx, float *cy, float *radius, float dist){


  WorldObject* wo=&(vblocks_.world_object->objects_[WO_CENTER_CIRCLE]);
  // dont do this if we've already detected it
  if (wo->seen){
    return false;
  }

  float minx = x1;
  float maxx = x1;
  if (x2 < minx)
    minx=x2;
  if (x2 > maxx)
    maxx = x2;
  if (x3 < minx)
    minx = x3;
  if (x3 > maxx)
    maxx = x3;

  // distance in y (side to side)
  if (fabs(y1-y3) < 200) {
    visionLog((32,  "points too close side2side %5.5f", (y3-y1)));
    return false;
  }

  // distance in x (forward)
  if ((maxx-minx) < 120) {
    visionLog((32,  "points too close in xdist %5.5f", (maxx-minx)));
    return false;
  }

  // and full distance
  if (dist < 300){
    visionLog((32,  "points too close in fulldist %5.5f", dist));
    return false;
  }

  visionLog((32,  "Circle xdist %5.5f, ydist %5.5f, fulldist %5.5f", (maxx-minx), (y3-y1), dist));

  if (maxx > 4000) {
    visionLog((32,  "points too far away %5.5f", maxx));
    return false;
  }


  Point2D circleCenter;

  float temp = x2*x2+y2*y2;
  float bc = (x1*x1 + y1*y1 - temp)/2.0;
  float cd = (temp - x3*x3 - y3*y3)/2.0;
  float det = (x1-x2)*(y2-y3)-(x2-x3)*(y1-y2);
  if (fabs(det) < 1.0e-6) {
    visionLog((32,  "circle fail %5.5f", det));
    return false;
  }
  det = 1/det;
  circleCenter.x = (bc*(y2-y3)-cd*(y1-y2))*det;
  circleCenter.y = ((x1-x2)*cd-(x2-x3)*bc)*det;
  x2 = circleCenter.x;
  y2 = circleCenter.y;
  *radius = sqrtf((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));

  *cx = circleCenter.x;
  *cy = circleCenter.y;

  visionLog((32,  "circle radius: %5.5f", *radius));

  visionLog((32,  "circle: (%5.5f, %5.5f)", circleCenter.x, circleCenter.y));

  if (fabs(*radius - CIRCLE_RADIUS) > 180){
    visionLog((32,  "circle radius wrong: %f, exp: %f", *radius, CIRCLE_RADIUS));
    return false;
  }

  // how many points
  visionLog((32, "Circle numPoints %i", Line->Points));

  // lets sample more points and see if they're on the circle
  // and see what the slope is between them
  int numPoints = 6;
  float lastx = 0, lasty = 0;
  float lastAngle = 0;
  for (int i = 0; i < numPoints; i++){
    int index = i * Line->Points / (numPoints+1) - 1;
    if (index < 0) index = 0;
    if (i == (numPoints-1)) index = Line->Points-1;
    //FastGetPointProjection(Line->PointsArray[index]->PosX, Line->PointsArray[index]->PosY, &y1, &x1);
    Position p = cmatrix_.getWorldPosition(Line->PointsArray[index]->PosX, Line->PointsArray[index]->PosY);
    x1 = p.x; y1 = p.y;

    // check distance from circle
    float dist = sqrtf((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    visionLog((32, "Sampled pt %i is %5.5f from circle radius: %5.5f", i, dist, *radius));

    if (fabs(dist - *radius) > 40) {
      visionLog((32, "  distance too high, throwing circle out", i, dist, *radius));
      return false;
    }

    // check slope, severe change means probably a 90 degree intersection
    if (i > 0){
      float angle = atan2f(y1 - lasty, x1 - lastx);

      visionLog((32, "  Angle from previous sample point: %5.5f", angle));

      // compare with last
      if (i > 1){
        float change = fabs(angle-lastAngle);
        visionLog((32, "  Slope change: %f", change));
        float slopeChangeLimit = 0.7;//M_PI / 8;
        if (change > slopeChangeLimit && change < 2 * M_PI - slopeChangeLimit){
          visionLog((32, "Slope change is too high %f > %f", change, slopeChangeLimit));
          return false;
        }
      }
      lastAngle = angle;
    }

    lastx = x1;
    lasty = y1;
  }

  if (!FilterLine(Line)) {
    visionLog((32, "Filter line failed"));
    return false;
  }


  wo->visionPt1 = circleCenter;
  wo->visionDistance = circleCenter.getMagnitude();
  wo->visionBearing = circleCenter.getDirection();
  wo->frameLastSeen = vblocks_.frame_info->frame_id;
  wo->seen = true;
  wo->visionConfidence = 1.0;
  wo->fieldLineIndex = Line->id;
  Line->isCircle=true;
  wo->fromTopCamera = (camera_ == Camera::TOP);

  visionLog((32, "Circle Found !!!!!"));
  return true;
}

bool LineDetector::formPenaltyCross(float x1, float y1, float x2, float y2, int line) {

  float xDiff = x2 - x1;
  float yDiff = x2 - x1;

  int minX = (x1 < x2) ? (int) x1 : (int) x2;
  int minY = (y1 < y2) ? (int) y1 : (int) y2;
  int maxX = (x1 > x2) ? (int) x1 : (int) x2;
  int maxY = (y1 > y2) ? (int) y1 : (int) y2;

  int left = minX - (int) xDiff / 2;
  left = left - left % 4;
  int right = maxX + (int) xDiff / 2;
  right = right - right % 4;
  int top = minY - (int) yDiff / 2;
  top = top - top % 2;
  int bottom = maxY + (int) yDiff / 2;
  bottom = bottom - bottom % 4;

  if (left < 0 || right >= iparams_.width || top < 0 || bottom >= iparams_.height) {
    return false;
  }

  visionLog((32,  "line %i penalty cross bounding box (%i, %i) to (%i, %i)", line, left, top, right, bottom));

  int totalCounter = 0;
  int greenCounter = 0, undefinedCounter = 0, otherCounter = 0;
  char c;

  for (int i = left; i <= right; i += 8) {

    c = getSegPixelValueAt(i, top);

    switch(c) {
    case c_FIELD_GREEN: greenCounter++; break;
    case c_UNDEFINED: undefinedCounter++; break;
    default: otherCounter++; break;
    }

    c = getSegPixelValueAt(i, bottom);

    switch(c) {
    case c_FIELD_GREEN: greenCounter++; break;
    case c_UNDEFINED: undefinedCounter++; break;
    default: otherCounter++; break;
    }

    if (otherCounter != 0)
      return false;

    totalCounter += 2;

  }

  for (int i = top; i <= bottom; i += 8) {

    c = getSegPixelValueAt(left, i);

    switch(c) {
    case c_FIELD_GREEN: greenCounter++; break;
    case c_UNDEFINED: undefinedCounter++; break;
    default: otherCounter++; break;
    }

    c = getSegPixelValueAt(right, i);

    switch(c) {
    case c_FIELD_GREEN: greenCounter++; break;
    case c_UNDEFINED: undefinedCounter++; break;
    default: otherCounter++; break;
    }

    if (otherCounter != 0)
      return false;

    totalCounter += 2;

  }

  visionLog((32, "line %i penalty cross green %i vs undefinedCounter %i in total %i", line, greenCounter, undefinedCounter, totalCounter));

  if (greenCounter > 4 * undefinedCounter) {
    visionLog((32, "Line %i detected as penalty cross", line));
    Position p = cmatrix_.getWorldPosition((x1 + x2) / 2, (y1 + y2) / 2);
    WorldObject* wo=&(vblocks_.world_object->objects_[WO_UNKNOWN_PENALTY_CROSS]);
    wo->seen = true;
    wo->frameLastSeen = vblocks_.frame_info->frame_id;
    wo->visionDistance = cmatrix_.groundDistance(p);
    wo->visionBearing = cmatrix_.bearing(p);
    wo->visionElevation = cmatrix_.elevation(p);
    wo->visionConfidence = 1.0;
    wo->imageCenterX = (x1 + x2) / 2;
    wo->imageCenterY = (y1 + y2) / 2;
    wo->fieldLineIndex = line;
    wo->fromTopCamera = (camera_ == Camera::TOP);
    return true;
  }

  return false;
}
void LineDetector::CalLineDetails(FieldLine* line) {

  // Set the 2 end points
  // tL is either the top or left point,
  // bR is either the bottom or right point
  // these points have no meaning in curves/circles

  line->tL.x = line->PointsArray[0]->PosX;
  line->tL.y = line->PointsArray[0]->PosY;

  line->bR.x = line->PointsArray[line->Points - 1]->PosX;
  line->bR.y = line->PointsArray[line->Points - 1]->PosY;

  // The line parameters using least squares
  // and the width
  line->width = 0;
  line->avgX = 0;
  line->avgY = 0;

  float sumx = 0, sumy = 0, sumx2 = 0, sumy2 = 0, sumxy = 0;
  for (unsigned int pt = 0; pt < line->Points; pt++) {

    int xVal = line->PointsArray[pt]->PosX;
    int yVal = line->PointsArray[pt]->PosY;
    line->width += line->PointsArray[pt]->Width;
    line->avgX += xVal;
    line->avgY += yVal;

    sumx += xVal;
    sumy += yVal;
    sumx2 += (xVal * xVal);
    sumy2 += (yVal * yVal);
    sumxy += (xVal * yVal);

  }

  line->preciseWidth = ((float) line->width) / line->Points;

  line->width /= line->Points;
  line->avgX /= line->Points;
  line->avgY /= line->Points;

  float x = line->tL.x - line->bR.x;
  float y = line->tL.y - line->bR.y;
  line->length = (int) sqrtf(x * x + y * y);

  line->sumX = sumx;
  line->sumY = sumy;
  line->sumX2 = sumx2;
  line->sumY2 = sumy2;
  line->sumXY = sumxy;

  float sxx = sumx2 - sumx*sumx/(line->Points);
  float syy = sumy2 - sumy*sumy/(line->Points);
  float sxy = sumxy - sumx*sumy/(line->Points);
  float sigma =
    (sxx + syy - sqrtf((sxx - syy) * (sxx - syy) + 4 * sxy * sxy)) / 2;
  float aVal, bVal, cVal;

  if (sxx > syy) {
    aVal = -sxy;
    bVal = (sxx - sigma);
    cVal = (sumx * sxy - (sxx - sigma) * sumy) / line->Points;
  } else {
    aVal = (syy - sigma);
    bVal = -sxy;
    cVal = (sumy * sxy - (syy - sigma) * sumx) / line->Points;
  }

  if (fabs(bVal) <= 0.0001) {
    bVal = 0.0001;
  }

  line->Slope = -aVal / bVal;
  if (fabs(line->Slope) <= 0.0001) {
    line->Slope = 0.0001;
  }

  line->Angle = atan(line->Slope);
  line->Offset = -cVal / bVal;

  // Ideally this hsould not be required
  if (line->tL.x < line->bR.x){
    line->MinX = line->tL.x;
    line->MaxX = line->bR.x;
  } else {
    line->MaxX = line->tL.x;
    line->MinX = line->bR.x;
  }

  if (line->tL.y < line->bR.y){
    line->MinY = line->tL.y;
    line->MaxY = line->bR.y;
  } else {
    line->MaxY = line->tL.y;
    line->MinY = line->bR.y;
  }

  // add half width on
  line->MinX -= line->width / 2.0;
  line->MinY -= line->width / 2.0;
  line->MaxX += line->width / 2.0;
  line->MaxY += line->width / 2.0;

}

void LineDetector::generateTransformedImage() {

  // This is run only to generate the full transformed image in the tool, its not required otherwise.
#ifndef TOOL
  return;
#endif

  for (int line = 0; line < FieldLinesCounter; line++) {
    for (uint16_t point = 0; point < fieldLines[line]->Points; point++) {

      Position p = cmatrix_.getWorldPosition(fieldLines[line]->PointsArray[point]->PosX, fieldLines[line]->PointsArray[point]->PosY);
      fieldLines[line]->TranPointsArray[point]->PosX = p.x;
      fieldLines[line]->TranPointsArray[point]->PosY = p.y;
    }
  }

}

void LineDetector::mergeLines() {
  for (int line1 = 0; line1 < FieldLinesCounter; line1++) {

    if (!fieldLines[line1]->ValidLine)
      continue;

    for(int line2 = line1+1; line2 < FieldLinesCounter; line2++) {

      if (!fieldLines[line2]->ValidLine)
        continue;

      bool merged = false;

      merged = mergeFieldLines(fieldLines[line1], fieldLines[line2]);
      if (merged) break;

    }
  }
  visionLog((30, "After merge lines, num valid lines are %i", TotalValidLines));
}

bool LineDetector::mergeOverlap(FieldLine * line1, FieldLine * line2) {

  // Merge only if line has some horizontal and vertical component. Typically this
  // should only be used for arc merging
  /*
    bool mergeOK =
    (line1->isVertical && line2->isHorizontal) ||
    (line1->isHorizontal && line2->isVertical);

    if (!mergeOK) {
    return false;
    }
  */
  FieldLine * finalLine = fieldLines[FieldLinesCounter];

  unsigned int line2Counter = 0, mergeConfidence = 0, finalCounter = 0;
  bool goForMerge = false;

  for (unsigned int line1Counter = 0;
       line1Counter < line1->Points && line2Counter < line2->Points && finalCounter < 395;
       line1Counter++) {

    float distX = line2->PointsArray[line2Counter]->PosX - line1->PointsArray[line1Counter]->PosX;
    float distY = line2->PointsArray[line2Counter]->PosY - line1->PointsArray[line1Counter]->PosY;
    float minDist = (abs(distX) + abs(distY)) / 2, tempDist;
    tempDist = minDist;
    bool cond = false;
    do {
      finalLine->PointsArray[finalCounter]->PosX = line2->PointsArray[line2Counter]->PosX;
      finalLine->PointsArray[finalCounter]->PosY = line2->PointsArray[line2Counter]->PosY;
      finalLine->PointsArray[finalCounter]->Width = line2->PointsArray[line2Counter]->Width;
      finalCounter++;
      minDist = tempDist;
      line2Counter++;
      distX = line2->PointsArray[line2Counter]->PosX - line1->PointsArray[line1Counter]->PosX;
      distY = line2->PointsArray[line2Counter]->PosY - line1->PointsArray[line1Counter]->PosY;
      tempDist = (abs(distX) + abs(distY)) / 2;
      if (line1->avgY > line2->avgY)
        cond = tempDist > 20;
    } while ((tempDist < minDist || cond) && (line2Counter + 1) < line2->Points && finalCounter < 395);

    line2Counter--;
    finalCounter--;

    mergeConfidence += (minDist < 6);
    goForMerge = mergeConfidence > 6;

    finalLine->PointsArray[finalCounter]->PosX = line1->PointsArray[line1Counter]->PosX;
    finalLine->PointsArray[finalCounter]->PosY = line1->PointsArray[line1Counter]->PosY;
    finalLine->PointsArray[finalCounter]->Width = line1->PointsArray[line1Counter]->Width;
    finalCounter++;
  }

  while(line2Counter < line2->Points - 2&& finalCounter < 395) {
    finalLine->PointsArray[finalCounter]->PosX = line2->PointsArray[line2Counter]->PosX;
    finalLine->PointsArray[finalCounter]->PosY = line2->PointsArray[line2Counter]->PosY;
    finalLine->PointsArray[finalCounter]->Width = line2->PointsArray[line2Counter]->Width;
    finalCounter++;
    line2Counter++;
  }

  finalLine->Points = finalCounter;
  if (goForMerge && FieldLinesCounter < 20) {

    visionLog((35, "        Merge 1 Called"));
    line1->ValidLine = line2->ValidLine = false;
    finalLine->ValidLine = true;
    CalLineDetails(finalLine);
    FieldLinesCounter++;
    TotalValidLines++;


    return true;
  }

  line2Counter = 0, mergeConfidence = 0, finalCounter = 0;
  goForMerge = false;

  for (int line1Counter = line1->Points - 1;
       line1Counter >= 0 && line2Counter < line2->Points && finalCounter < 395;
       line1Counter--) {

    float distX = line2->PointsArray[line2Counter]->PosX - line1->PointsArray[line1Counter]->PosX;
    float distY = line2->PointsArray[line2Counter]->PosY - line1->PointsArray[line1Counter]->PosY;
    float minDist = sqrtf(distX * distX + distY * distY), tempDist;
    tempDist = minDist;
    bool cond = false;

    do {
      finalLine->PointsArray[finalCounter]->PosX = line2->PointsArray[line2Counter]->PosX;
      finalLine->PointsArray[finalCounter]->PosY = line2->PointsArray[line2Counter]->PosY;
      finalLine->PointsArray[finalCounter]->Width = 2;//line2->PointsArray[line2Counter]->Width;
      finalCounter++;
      minDist = tempDist;
      line2Counter++;
      distX = line2->PointsArray[line2Counter]->PosX - line1->PointsArray[line1Counter]->PosX;
      distY = line2->PointsArray[line2Counter]->PosY - line1->PointsArray[line1Counter]->PosY;
      tempDist = sqrtf(distX * distX + distY * distY);
      if (line1->avgY > line2->avgY)
        cond = tempDist > 20;
    } while ((tempDist < minDist || cond) && (line2Counter + 1) < line2->Points && finalCounter < 395);

    line2Counter--;
    finalCounter--;

    mergeConfidence += (minDist < 8);
    goForMerge = mergeConfidence > 4;

    finalLine->PointsArray[finalCounter]->PosX = line1->PointsArray[line1Counter]->PosX;
    finalLine->PointsArray[finalCounter]->PosY = line1->PointsArray[line1Counter]->PosY;
    finalLine->PointsArray[finalCounter]->Width = 1;//line1->PointsArray[line1Counter]->Width;
    finalCounter++;
  }

  while(line2Counter < line2->Points - 2 && finalCounter < 395) {
    finalLine->PointsArray[finalCounter]->PosX = line2->PointsArray[line2Counter]->PosX;
    finalLine->PointsArray[finalCounter]->PosY = line2->PointsArray[line2Counter]->PosY;
    finalLine->PointsArray[finalCounter]->Width = 2;//line2->PointsArray[line2Counter]->Width;
    finalCounter++;
    line2Counter++;
  }
  finalLine->Points = finalCounter;

  if (goForMerge && FieldLinesCounter < 20) {
    line1->ValidLine = line2->ValidLine = false;
    finalLine->ValidLine = true;
    CalLineDetails(finalLine);
    FieldLinesCounter++;
    TotalValidLines++;
    return true;
  }

  return false;

}


bool LineDetector::mergeFieldLines(FieldLine * line1, FieldLine * line2) {

  float angleDiff = fabs(line1->Angle - line2->Angle) * 180 / M_PI;

  if (angleDiff < 7 || angleDiff > 180 - 7) {
    float offsetYDiff = fabs(line1->Offset - line2->Offset);
    float line1XOffset = (-line1->Offset) / line1->Slope;
    float line2XOffset = (-line2->Offset) / line2->Slope;
    float offsetXDiff = fabs(line1XOffset - line2XOffset);

    if ((offsetYDiff < 10 && line1->Offset < 2 * iparams_.height && line1->Offset > - iparams_.height) ||
        (offsetXDiff < 12 && line1XOffset < 2 * iparams_.width && line1XOffset > - iparams_.width)) {

      float posX[MAX_POINTS_PER_LINE];
      float posY[MAX_POINTS_PER_LINE];
      float width[MAX_POINTS_PER_LINE];

      bool horizontalCheck = false, verticalCheck = false;
      unsigned int counter1Start = 0, counter2Start = 0, counter1Inc = 1, counter2Inc = 1,
        counter1End = line1->Points, counter2End = line2->Points;
      float width1Mult = 1.0, width2Mult = 1.0;

      if (line1->isHorizontal && line2->isHorizontal) {
        horizontalCheck = true;
      } else if (line1->isVertical && line2->isVertical) {
        verticalCheck = true;
      } else if (line1->isHorizontal && line2->isVertical) {

        if (line1->Slope < 1 && line1->Slope > -1) {
          horizontalCheck = true;
          width2Mult = ((float)line1->width) / line2->width;
          if (line1->Slope < 0) {
            counter2Start = line2->Points - 1;
            counter2Inc = -1;
            counter2End = -1;
          }
        } else {
          verticalCheck = true;
          width1Mult = ((float)line2->width) / line1->width;
          if (line1->Slope < 0) {
            counter1Start = line1->Points - 1;
            counter1Inc = -1;
            counter1End = -1;
          }
        }
      } else if (line1->isVertical && line2->isHorizontal) {
        if (line1->Slope < 1 && line1->Slope > -1) {
          horizontalCheck = true;
          width1Mult = ((float)line2->width) / line1->width;
          if (line1->Slope < 0) {
            counter1Start = line1->Points - 1;
            counter1Inc = -1;
            counter1End = -1;
          }
        } else {
          verticalCheck = true;
          width2Mult = ((float)line1->width) / line2->width;
          if (line1->Slope < 0) {
            counter2Start = line2->Points - 1;
            counter2Inc = -1;
            counter2End = -1;
          }
        }
      }

      visionLog((38, "horizontal: %i, %i vs %i, with slope %f", horizontalCheck, counter1Start, counter2Start, line1->Slope));

      unsigned int counter, counter1 = counter1Start, counter2 = counter2Start;
      for (counter = 0; counter < line1->Points + line2->Points && counter < MAX_POINTS_PER_LINE; counter++) {
        //printf("%i, %i\n", counter1, counter2);
        //fflush(stdout);

        bool goForFirst = counter2 == counter2End;
        if (!goForFirst) {
          goForFirst = !(counter1 == counter1End);
          if (goForFirst) {
            goForFirst =
              (horizontalCheck && line1->PointsArray[counter1]->PosX <= line2->PointsArray[counter2]->PosX) ||
              (verticalCheck && line1->PointsArray[counter1]->PosY <= line2->PointsArray[counter2]->PosY);
          }
        }

        if (goForFirst) {
          posX[counter] = line1->PointsArray[counter1]->PosX;
          posY[counter] = line1->PointsArray[counter1]->PosY;
          width[counter] = line1->PointsArray[counter1]->Width * width1Mult;
          counter1 += counter1Inc;
        } else {
          posX[counter] = line2->PointsArray[counter2]->PosX;
          posY[counter] = line2->PointsArray[counter2]->PosY;
          width[counter] = line2->PointsArray[counter2]->Width * width2Mult;
          counter2 += counter2Inc;
        }
      }

      for (counter = 0; counter < line1->Points + line2->Points && counter < MAX_POINTS_PER_LINE; counter++) {
        fieldLines[FieldLinesCounter]->PointsArray[counter]->PosX = posX[counter];
        fieldLines[FieldLinesCounter]->PointsArray[counter]->PosY = posY[counter];
        fieldLines[FieldLinesCounter]->PointsArray[counter]->Width = width[counter];
      }

      // set # of points
      fieldLines[FieldLinesCounter]->ValidLine = true;
      fieldLines[FieldLinesCounter]->isHorizontal = horizontalCheck;
      fieldLines[FieldLinesCounter]->isVertical = verticalCheck;
      fieldLines[FieldLinesCounter]->Points = line1->Points + line2->Points;
      if (fieldLines[FieldLinesCounter]->Points == MAX_POINTS_PER_LINE) {
        fieldLines[FieldLinesCounter]->Points = MAX_POINTS_PER_LINE;
      }

      CalLineDetails(fieldLines[FieldLinesCounter]); // TODO(piyushk): Replace this to a more efficient call
      line2->ValidLine = false;
      line1->ValidLine = false;
      TotalValidLines--;
      FieldLinesCounter++;


      return true;

    }
  }
  return false;

}

/**
 * Filters out lines based on a thickness estimate on the ground,
 * and also based on the length of the line in the vision frame.
 */
void LineDetector::FilterLines() {
  visionLog((39, "FilterLines()"));
  for (uint16_t i = 0; i < FieldLinesCounter; i++) {

    FieldLine *line = fieldLines[i];
    if (!line->ValidLine || line->isCurve || line->onCircle || line->isCross)
      continue;

    FilterLine(line);
  }
  visionLog((30, "After filter lines, num valid lines are %i", TotalValidLines));
}

bool LineDetector::FilterLine(FieldLine* line){

  // Filter line based on length in the image

  if (line->length < 110) {
    line->ValidLine = false;
    TotalValidLines--;
    return false;
  }

  // (piyushk): This formulation actually does not give you the
  // correct points, but its the best one i have got till now.

  // Correct width based on angle
  float angle = line->Angle;
  if (line->isVertical) {
    angle += M_PI / 2;
  }
  float multiplier = cosf(angle);
  float actualWidth = multiplier * line->width;

  // Get x mid point
  float xi = line->PointsArray[line->Points / 2]->PosX;
  float yi = line->PointsArray[line->Points / 2]->PosY;

  int xf1 = xi - 0.5 * actualWidth * sinf(line->Angle);
  int yf1 = yi + 0.5 * actualWidth * cosf(line->Angle);

  int xf2 = xi + 0.5 * actualWidth * sinf(line->Angle);
  int yf2 = yi - 0.5 * actualWidth * cosf(line->Angle);

  Position p1 = cmatrix_.getWorldPosition(xf1, yf1);
  Position p2 = cmatrix_.getWorldPosition(xf2, yf2);

  float distanceSqr = pow(p1.x - p2.x,2) + pow(p1.y - p2.y, 2);
  float dist = sqrtf(distanceSqr);

  if (dist > 160 || distanceSqr < 16) {
    line->ValidLine = false;
    TotalValidLines--;
    visionLog((35, "        line thrown on dist>160 || distanceSqr<16"));
    return false;
  }

  return line->ValidLine;
}

void LineDetector::DetermineEndPoints() {

  visionLog((38, "DetermineEndPoints()"));
  if (TotalValidLines < 1) return;

  for (int i = 0; i < FieldLinesCounter; i++){

    FieldLine* line = fieldLines[i];

    if (!line->ValidLine || line->isCurve || line->onCircle || line->isCross) {
      visionLog((39, "    Line %i invalid",i));
      continue;
    }
    float x1 = 0, y1 = 0, x2 = 0, y2 = 0;

    // This formulation is done to minimize error
    // Get the end points in vision
    if (line->Slope < 1 && line->Slope > -1) {
      x1 = line->tL.x;
      y1 = line->Slope * x1 + line->Offset;
      x2 = line->bR.x;
      y2 = line->Slope * x2 + line->Offset;
    } else {
      y1 = line->tL.y;
      x1 = (y1 - line->Offset) / line->Slope;
      y2 = line->bR.y;
      x2 = (y2 - line->Offset) / line->Slope;
    }

    // Get corresponding points projected on the ground for the end points of the line
    Position p1 = cmatrix_.getWorldPosition(x1, y1);
    Position p2 = cmatrix_.getWorldPosition(x2, y2);

    // Get relative field positions of the x, y of the 2 ends of the line
    // Vision and localization use different coordinate systems, this is counter-intuitive.
    Point2D sP(p1.x, p1.y);
    Point2D eP(p2.x, p2.y);

    line->pt1 = sP;
    line->pt2 = eP;

  }
}

/**
 * Tries to identify which line based on vision and transformations alone.
 * Currently Identifies a goal line based on the presence of the goal post.
 *
 * TODO(piyushk): Do something about this, if the line is identified correctly
 */
void LineDetector::IdentifyLines() {

  visionLog((38, "IdentifyLines()"));

  if (TotalValidLines < 1) return;

  for (int i = 0; i < FieldLinesCounter; i++) {
    fieldLines[i]->identifiedIndex = -1;
  }

  bool goalLineSeen = false;
  bool goalSeen = false;
  bool isGoalBlue = false;
  bool centerCircleSeen = false;
  bool centerLineSeen = false;

  int object;
  int finalLineIndex;

  float posX = 0, posY = 0;

  WorldObject *wo = vblocks_.world_object->objects_;

  if (wo[WO_CENTER_CIRCLE].seen) {
    object = WO_CENTER_CIRCLE;
    centerCircleSeen = true;
  }

  if (goalSeen) {
    return;
    // TODO fix this problem later on
    // Goals are ambiguous now TH 2/27/12
    /*
      int index = wo[object].fieldLineIndex;
      if (isGoalBlue) {
      //FastGetPointProjection(bluePosts[index]->bR.x, bluePosts[index]->bR.y, &posY, &posX);
      transform_.getPointProjection(bluePosts[index]->bR.x, bluePosts[index]->bR.y, posX, posY);
      finalLineIndex = WO_BLUE_GOAL_LINE;
      } else {
      //FastGetPointProjection(yellowPosts[index]->bR.x, yellowPosts[index]->bR.y, &posY, &posX);
      transform_.getPointProjection(yellowPosts[index]->bR.x, yellowPosts[index]->bR.y, posX, posY);
      finalLineIndex = WO_YELLOW_GOAL_LINE;
      }
    */
  } else if (centerCircleSeen) {
    posX = wo[object].visionPt1.x;
    posY = wo[object].visionPt1.y;
    finalLineIndex = WO_CENTER_LINE;
  } else {
    return;
  }

  int goalLineId;

  for (int i = 0; i < FieldLinesCounter; i++) {

    FieldLine* line = fieldLines[i];

    if (!line->ValidLine || line->isCurve || line->onCircle || line->isCross) {
      visionLog((38, "    Line %i is not a line",i));
      continue;
    }

    if ((!goalLineSeen && goalSeen)|| (!centerLineSeen && centerCircleSeen)) {

      // Form the corresponding line on the ground (relative to robot)
      Line2D lineEq(line->pt1, line->pt2);

      Point2D goalBase;
      goalBase.x = posX;
      goalBase.y = posY;
      Point2D closestPoint = lineEq.getPointOnLineClosestTo(goalBase);
      float distance = goalBase.getDistanceTo(closestPoint);

      if (distance < 150) {
        goalLineSeen = goalSeen;
        centerLineSeen = centerCircleSeen;
        goalLineId = i;
        line->identifiedIndex = finalLineIndex;
        visionLog((38, "    Line %i seen attached to object. Add to object %i of (BGL = %i, YGL = %i, CL = %i)", i, finalLineIndex, WO_OPP_GOAL_LINE, WO_OWN_GOAL_LINE, WO_CENTER_LINE));
        continue;
      }

    }
  }

  if (true) return;

  if (!goalLineSeen) {
    return;
  } else {
    if (isGoalBlue) {
      //finalLineIndex = WO_BLUE_PENALTY;
    } else {
      //finalLineIndex = WO_YELLOW_PENALTY;
    }
  }

  FieldLine * goalLine = fieldLines[goalLineId];

  // Get corresponding points projected on the ground for the end points of the line
  float goalStartX = goalLine->pt1.x, goalStartY = goalLine->pt1.y;
  float goalEndX = goalLine->pt2.x, goalEndY = goalLine->pt2.y;

  float goalLineAngle = atanf((goalEndY - goalStartY)/(goalEndX - goalStartX)) * 180 / M_PI;

  // Try to detect other lines based on the goal line
  for (int i = 0; i < FieldLinesCounter; i++) {

    FieldLine* line = fieldLines[i];

    if (!line->ValidLine || line->isCurve || line->onCircle || line->isCross) {
      visionLog((38, "    Line %i is not a line",i));
      continue;
    }

    // Don't check if this is the goal line
    if (i == goalLineId) {
      continue;
    }

    // Get corresponding points projected on the ground for the end points of the line
    float startX = line->pt1.x, startY = line->pt1.y;
    float endX = line->pt2.x, endY = line->pt2.y;
    float lineAngle = atanf((endY - startY) / (endX - startX)) * 180 / M_PI;
    float angleDiff = fabs(lineAngle - goalLineAngle);

    // Calculate Point on line which has the shortest perpendicular distance to the bottom of the goal
    // Form the corresponding line on the ground (relative to robot)
    Line2D lineEq(line->pt1, line->pt2);

    Point2D goalBase;
    goalBase.x = posX;
    goalBase.y = posY;
    Point2D closestPoint = lineEq.getPointOnLineClosestTo(goalBase);
    float distance = goalBase.getDistanceTo(closestPoint);

    visionLog((38, "    Line %i angle %f vs %f. Dist %f", i, lineAngle, goalLineAngle, distance));

    // Try parallel penalty box line
    if (angleDiff < 10 || angleDiff > 180 - 10) {
      if (distance < 700 && distance > 400) {
        line->identifiedIndex = finalLineIndex;
        visionLog((38, "    Line %i is the penalty line", i));
        break;
      }
    }

    // Try penalty box side line
    /*
      if (angleDiff < 100 && angleDiff > 80) {
      if (distance < 450 && distance > 220) {
      visionLog((38, "    Line %i is the penalty side line", i));
      }
      }
    */
  }

}

/**
 * Fills the line objects so that localization can make use of it
 */
void LineDetector::DecodeLines() {

  int nextLine = WO_UNKNOWN_FIELD_LINE_1;
  int lastLine = WO_UNKNOWN_FIELD_LINE_4;

  // Todd: make sure we start at first line not already filled in
  while (vblocks_.world_object->objects_[nextLine].seen && nextLine <= lastLine)
    nextLine++;

  //if (nextLine > WO_UNKNOWN_FIELD_LINE_1)
  //  cout << "Camera " << camera_ << " start filling in lines at index " << nextLine << endl;

  if (nextLine > lastLine){
    visionLog((39, "    No Lines Slots left to be filled in)"));
    return;
  }

  visionLog((39, "DecodeLines()"));

  if (TotalValidLines < 1) {
    visionLog((39, "    No Lines Present"));
    return;
  }

  for (int i = 0; i < FieldLinesCounter; i++){

    FieldLine* line = fieldLines[i];

    if (!line->ValidLine || line->isCurve || line->onCircle || line->isCross) {
      visionLog((39, "    Line %i invalid",i));
      continue;
    }

    if (line->identifiedIndex == -1 && nextLine > lastLine) {
      visionLog((39,"    Ran out of WOs to fill. %i / %i lines remaining", TotalValidLines, FieldLinesCounter));
      continue;
    }

    // Get relative field positions of the x, y of the 2 ends of the line
    // Vision and localization use different coordinate systems, this is counter-intuitive.
    Point2D sP(line->pt1.x, line->pt1.y);
    Point2D eP(line->pt2.x, line->pt2.y);

    // Form the corresponding line on the ground (relative to robot)
    Line2D lineEq(sP, eP);

    // Find closest point and the distance and angle to it.
    Point2D robot = Point2D(0, 0);
    Point2D closestPoint = lineEq.getPointOnLineClosestTo(robot);
    float distance = robot.getDistanceTo(closestPoint);
    float angle = robot.getAngleTo(closestPoint);

    // if we think we're right on line, angle could be bogus
    if (distance < 5.0){
      line->ValidLine = false;
      continue;
    }

    WorldObject *wo = NULL;
    if (line->identifiedIndex == -1) {
      visionLog((39,"    Unidentified Line %i seen, added to worldObject %i", i, nextLine));
      wo= &(vblocks_.world_object->objects_[nextLine]);
      nextLine++;
    } else {
      visionLog((39,"    Indentified Line %i seen, added to worldObject %i", i, line->identifiedIndex));
      wo= &(vblocks_.world_object->objects_[line->identifiedIndex]);
    }

    wo->seen = true;
    wo->frameLastSeen = vblocks_.frame_info->frame_id;
    wo->visionDistance = distance;
    wo->visionBearing = angle;
    //(piyushk): we dont use this info, and no longer calculate it
    wo->visionElevation = 0;//(eElev + sElev) / 2.0; // Hack probably shouldn't use this anyway since the distance is already on the horizontal

    // visionPt1 and visionPt2 for relative endpoints of line
    wo->visionPt1 = sP;
    wo->visionLine = lineEq;
    wo->visionPt2 = eP;
    wo->fieldLineIndex = line->id;
    wo->fromTopCamera = (camera_ == Camera::TOP);
  }
}
/**
 * Forms corners based on the lines detected. It also tries to figure
 * out whether its an L intersection or a T intersection
 */
void LineDetector::FormCorners() {

  visionLog((36, "FormCorners()"));

  if (TotalValidLines < 2) {
    visionLog((36,  "    Too few lines found (%i). Adieu!", TotalValidLines));
    return;
  }

  int commonX, commonY;
  int lineIDStart;

  for (lineIDStart = 0; lineIDStart < FieldLinesCounter - 1; lineIDStart++) {

    if (!fieldLines[lineIDStart]->ValidLine || fieldLines[lineIDStart]->isCurve ||
        fieldLines[lineIDStart]->onCircle || fieldLines[lineIDStart]->isCross) {
      visionLog((36, "  Line %i is not a valid line", lineIDStart));
      continue;
    }

    for (int lineIDCheck = lineIDStart + 1; lineIDCheck < FieldLinesCounter; lineIDCheck++) {

      if (!fieldLines[lineIDCheck]->ValidLine || fieldLines[lineIDCheck]->isCurve ||
          fieldLines[lineIDCheck]->onCircle || fieldLines[lineIDCheck]->isCross) {
        continue;
      }

      bool angleOK = fabs(fieldLines[lineIDStart]->Angle - fieldLines[lineIDCheck]->Angle) >= .08;

      commonX = (int) ((fieldLines[lineIDStart]->Offset - fieldLines[lineIDCheck]->Offset) /
                       (fieldLines[lineIDCheck]->Slope - fieldLines[lineIDStart]->Slope));
      bool xOK = commonX > 0 && commonX < iparams_.width;

      commonY = (int)((fieldLines[lineIDStart]->Slope * commonX) + fieldLines[lineIDStart]->Offset);
      bool yOK = commonY > 0 && commonY < iparams_.height;

      bool horizonOK = !horizon_.exists || (commonY > (horizon_.gradient * commonX + horizon_.offset));

      if (angleOK && xOK && yOK && horizonOK) {

        if (CornerPointCounter >= MAX_CORNERPOINTS) {
          visionLog((36, "    Ran out of cornerPoints to fill. Increase MAX_CORNERPOINTS!!!"));
          return;
        }

        cornerPoints[CornerPointCounter]->PosX = commonX;
        cornerPoints[CornerPointCounter]->PosY = commonY;
        cornerPoints[CornerPointCounter]->Line[0] = fieldLines[lineIDStart];
        cornerPoints[CornerPointCounter]->Line[1] = fieldLines[lineIDCheck];

        cornerPoints[CornerPointCounter]->Direction = 0;
        cornerPoints[CornerPointCounter]->Orientation = 0;
        cornerPoints[CornerPointCounter]->CornerType = 0;
        cornerPoints[CornerPointCounter]->FieldObjectID = 0;

        // Logic for detecting corner type
        bool startMinLeftOfX = fieldLines[lineIDStart]->tL.x < commonX - 20;
        bool startMaxRightOfX = fieldLines[lineIDStart]->bR.x > commonX + 20;
        bool checkMinLeftOfX = fieldLines[lineIDCheck]->tL.x < commonX - 20;
        bool checkMaxRightOfX = fieldLines[lineIDCheck]->bR.x > commonX + 20;

        bool startMinAboveY = fieldLines[lineIDStart]->tL.y < commonY - 20;
        bool startMaxBelowY = fieldLines[lineIDStart]->bR.y > commonY + 20;
        bool checkMinAboveY = fieldLines[lineIDCheck]->tL.y < commonY - 20;
        bool checkMaxBelowY = fieldLines[lineIDCheck]->bR.y > commonY + 20;

        cornerPoints[CornerPointCounter]->CornerType =
          (startMinLeftOfX && startMaxRightOfX) || (checkMinLeftOfX && checkMaxRightOfX) ||
          (startMinAboveY && startMaxBelowY) || (checkMinAboveY && checkMaxBelowY);
        // TODO(piyushk): I don't know what this is
        cornerPoints[CornerPointCounter]->Orientation = 1;

        visionLog((36, "    Detected corner between %i and %i of type %i (1 = T, 0 = L)", lineIDStart, lineIDCheck, cornerPoints[CornerPointCounter]->CornerType));

        CornerPointCounter++;

      } else {
        visionLog((36, "  Corner between %i and %i failed check (ang = %i, pos = %i, horizon = %i)", angleOK, xOK && yOK, horizonOK));
      }
    }
  }
}

/**
 * Puts the detected corners into the corresponding WOs
 */
void LineDetector::DecodeCorners() {

  visionLog((37, "DecodeCorners()"));

  if (CornerPointCounter > 4 || CornerPointCounter < 1) {
    visionLog((37, "    Too many or zero corner points found (%i). Not Setting WOs", CornerPointCounter));
    return;
  }

  int tCount = 0, lCount = 0;
  int tempLocation;
  int x;

  // Todd: check how many corners were already filled in by other camera
  if (vblocks_.world_object->objects_[WO_UNKNOWN_L_1].seen) lCount++;
  if (vblocks_.world_object->objects_[WO_UNKNOWN_L_2].seen) lCount++;
  if (vblocks_.world_object->objects_[WO_UNKNOWN_T_1].seen) tCount++;
  if (vblocks_.world_object->objects_[WO_UNKNOWN_T_2].seen) tCount++;

  //cout << "Camera " << camera_ << " start lCount at " << lCount << " and tCount at " << tCount << endl;


  for (x = 0; x < CornerPointCounter; x++) {

    tempLocation = 0;

    if (cornerPoints[x]->CornerType == 0) {
      if (lCount < 2) {
        tempLocation = WO_UNKNOWN_L_1 + lCount;
        lCount++;
        visionLog((37, "  Filling L corner %i in WO %i", x, tempLocation));
      } else {
        visionLog((37, "  Ran out of L corners to fill"));
        continue;
      }

    } else {
      if (tCount < 2) {
        tempLocation = WO_UNKNOWN_T_1 + tCount;
        tCount++;
        visionLog((37, "  Filling T corner %i in WO %i", x, tempLocation));
      } else {
        visionLog((37, "  Ran out of T corners to fill"));
        continue;
      }
    }


    Position p = cmatrix_.getWorldPosition(cornerPoints[x]->PosX, cornerPoints[x]->PosY);
    WorldObject* wo= &(vblocks_.world_object->objects_[tempLocation]);
    wo->seen = true;
    wo->frameLastSeen = vblocks_.frame_info->frame_id;
    wo->visionDistance = cmatrix_.groundDistance(p);
    wo->visionBearing = cmatrix_.bearing(p);
    wo->visionElevation = cmatrix_.elevation(p);
    wo->imageCenterX = cornerPoints[x]->PosX;
    wo->imageCenterY = cornerPoints[x]->PosY;
    wo->visionConfidence = 1.0;
    wo->fromTopCamera = (camera_ == Camera::TOP);

    cornerPoints[x]->FieldObjectID = tempLocation;
  }
}

void LineDetector::setHorizon(HorizonLine horizon) {
  horizon_ = horizon;
}


float LineDetector::getHeadTilt() {
  return -vblocks_.joint->values_[HeadTilt];
}

float LineDetector::getHeadPan() {
  return vblocks_.joint->values_[HeadPan];
}

