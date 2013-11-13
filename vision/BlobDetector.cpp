#include "BlobDetector.h"

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier) :
  DETECTOR_INITIALIZE, classifier_(classifier) {
  verticalPoint = classifier_->verticalPoint;
  verticalPointCount = classifier_->verticalPointCount;
  horizontalPoint = classifier_->horizontalPoint;
  horizontalPointCount = classifier_->horizontalPointCount;

  //Blob horizontalBlob[NUM_COLORS][MAX_LINE_BLOBS];
  for(int i = 0; i < NUM_COLORS; i++)
    horizontalBlob.push_back(std::vector<Blob>());

  //Blob verticalBlob[NUM_COLORS][MAX_LINE_BLOBS];
  for(int i = 0; i < NUM_COLORS; i++)
    verticalBlob.push_back(std::vector<Blob>());
}

void BlobDetector::resetOrangeBlobs() {
  horizontalBlob[c_ORANGE].clear();
}

void BlobDetector::resetYellowBlobs() {
  horizontalBlob[c_YELLOW].clear();
}

void BlobDetector::clearPointBlobReferences(Color color) {
  int hstep, vstep;
  classifier_->getStepSize(hstep,vstep);
  for (int y = 0; y < iparams_.height - vstep; y += vstep)
    for (uint16_t i = 0; i < horizontalPointCount[color][y]; i++)
      horizontalPoint[color][y][i].lbIndex = (uint16_t)-1;
  for (int x = 0; x < iparams_.width - hstep; x += hstep)
    for (uint16_t i = 0; i < verticalPointCount[color][x]; i++)
      verticalPoint[color][x][i].lbIndex = (uint16_t)-1;
}

void BlobDetector::formWhiteLineBlobs() {
  unsigned char horzColors[] = { c_WHITE };
  formBlobsWithHorzLineSegs(horzColors, 1);
  calculateHorizontalBlobData(c_WHITE);

  unsigned char vertColors[] = { c_WHITE };
  formBlobsWithVertLineSegs(vertColors, 1);
  calculateVerticalBlobData(c_WHITE);
  clearPointBlobReferences(c_WHITE);
}

void BlobDetector::formBlueBandBlobs() {
  unsigned char vertColors[] = { c_BLUE };
  formBlobsWithVertLineSegs(vertColors, 1);
  calculateBandBlobData(c_BLUE);
}

void BlobDetector::formPinkBandBlobs() {
  unsigned char vertColors[] = { c_PINK };
  formBlobsWithVertLineSegs(vertColors, 1);
  calculateBandBlobData(c_PINK);
}

void BlobDetector::formYellowBlobs() {

  unsigned char horzColors[] = {
    c_YELLOW
  };

  formBlobsWithHorzLineSegs(horzColors, 1);
}

void BlobDetector::formBlobsWithHorzLineSegs(unsigned char * horzColors, int numColors) {
  int hstep, vstep;
  classifier_->getStepSize(hstep,vstep);
  for (int c = 0; c < numColors; c++) {
    unsigned char color = horzColors[c];
    horizontalBlob[color].clear();
    for (int y = 0; y < iparams_.height - vstep; y += vstep) {
      for (uint16_t i = 0; i < horizontalPointCount[color][y]; i++) {
        if (!horizontalPoint[color][y][i].isValid)
          continue;
        // Check for overlapping line segments with similar width
        uint16_t j;
        for (j = 0; j < horizontalPointCount[color][y + vstep]; j++) {
          if (!horizontalPoint[color][y + vstep][j].isValid)
            continue;
          bool overlap =
            (horizontalPoint[color][y][i].xi < horizontalPoint[color][y + vstep][j].xf &&
             horizontalPoint[color][y][i].xf > horizontalPoint[color][y + vstep][j].xi);
          bool similarWidth = (
                               (horizontalPoint[color][y][i].dx >= horizontalPoint[color][y + vstep][j].dx &&
                                ((vparams_.blobWidthRatio[color] * horizontalPoint[color][y][i].dx <=
                                  8 * horizontalPoint[color][y + vstep][j].dx) ||
                                 (horizontalPoint[color][y][i].dx - horizontalPoint[color][y + vstep][j].dx <= vparams_.blobWidthAbs[color]))) ||
                               (horizontalPoint[color][y][i].dx <= horizontalPoint[color][y + vstep][j].dx &&
                                ((8 * horizontalPoint[color][y][i].dx >=
                                  vparams_.blobWidthRatio[color] * horizontalPoint[color][y + vstep][j].dx) ||
                                 (horizontalPoint[color][y + vstep][j].dx - horizontalPoint[color][y][i].dx <= vparams_.blobWidthAbs[color]))));
          if (overlap && (!vparams_.blobUseWidthRatioHorz[color] || similarWidth))
            break;
        }

        // If no overlapping line segments, continue
        if (j == horizontalPointCount[color][y + vstep])
          continue;

        // If line segment belongs to an existing field line blob
        if (y > 0 && horizontalPoint[color][y][i].lbIndex != (uint16_t)-1) {
          Blob *bl = &horizontalBlob[color][horizontalPoint[color][y][i].lbIndex];
          bl->lpIndex[bl->lpCount++] = (y + vstep) << 16 | j;
          horizontalPoint[color][y + vstep][j].lbIndex = horizontalPoint[color][y][i].lbIndex;

          // Otherwise, create a new field line blob
        } else if (horizontalBlob[color].size() < MAX_LINE_BLOBS - 1) {
          const Blob b;
          horizontalBlob[color].push_back(b);
          Blob *bl = &horizontalBlob[color].back();
          bl->lpCount = 0;
          bl->lpIndex[bl->lpCount++] =  y      << 16 | i;
          bl->lpIndex[bl->lpCount++] = (y + vstep) << 16 | j;
          horizontalPoint[color][y    ][i].lbIndex =
            horizontalPoint[color][y + vstep][j].lbIndex = horizontalBlob[color].size() - 1;
          // bound check
        } else {
          visionLog((30, "****MAX_LINE_BLOBS exceeded while forming blobs. Please increase in Vision.h"));
        }

      }
    }
  }
}

void BlobDetector::formBlobsWithVertLineSegs(unsigned char * vertColors, int numColors) {
  int hstep, vstep;
  classifier_->getStepSize(hstep, vstep);
  for (uint16_t c = 0; c < numColors; c++) {

    unsigned char color = vertColors[c];
    verticalBlob[color].clear();
    // Form the actual blob
    for (int x = 0; x < iparams_.width - hstep; x += hstep) {
        //std::cout << "-- x:" << x / hstep << ";";
      for (uint16_t i = 0; i < verticalPointCount[color][x]; i++) {
        if (!verticalPoint[color][x][i].isValid)
          continue;
        uint16_t j = 0;
        // Check for overlapping line segments with similar width
        for (; j < verticalPointCount[color][x + hstep]; j++) {
            //std::cout << "1;";
          if (!verticalPoint[color][x + hstep][j].isValid)
            continue;
            //std::cout << "2;";
            //if(color == c_WHITE) std::cout << "(" << x / hstep << "," << i << ":" << verticalPoint[color][x][i].yi << "," << verticalPoint[color][x][i].yf << "):(" << (x / hstep + 1) << "," << j << ":" << verticalPoint[color][x + hstep][j].yi << "," << verticalPoint[color][x + hstep][j].yf << ");";
          bool overlap =
            (verticalPoint[color][x][i].yi < verticalPoint[color][x + hstep][j].yf &&
             verticalPoint[color][x][i].yf > verticalPoint[color][x + hstep][j].yi);
          bool similarWidth = (
                               (verticalPoint[color][x][i].dy >= verticalPoint[color][x + hstep][j].dy &&
                                ((vparams_.blobWidthRatio[color] * verticalPoint[color][x][i].dy <=
                                  8 * verticalPoint[color][x + hstep][j].dy) ||
                                 (verticalPoint[color][x][i].dy - verticalPoint[color][x + hstep][j].dy <= vparams_.blobWidthAbs[color]))) ||
                               (verticalPoint[color][x][i].dy <= verticalPoint[color][x + hstep][j].dy &&
                                ((8 * verticalPoint[color][x][i].dy >=
                                  vparams_.blobWidthRatio[color] * verticalPoint[color][x + hstep][j].dy) ||
                                 (verticalPoint[color][x + hstep][j].dy - verticalPoint[color][x][i].dy <= vparams_.blobWidthAbs[color]))));
                                 //std::cout << "sim: " << similarWidth << ";over: " << overlap << ";";
          if (overlap && (!vparams_.blobUseWidthRatioVert[color] || similarWidth))
            break;
        }
        // If no overlapping line segments, continue
        if (j == verticalPointCount[color][x + hstep]) {
          j = 0;
          continue;
        }
        //std::cout << "3;";
        // If line segment belongs to an existing field line blob
        if (x > 0 && verticalPoint[color][x][i].lbIndex != (uint16_t)-1) {
          Blob *lb =
            &verticalBlob[color][verticalPoint[color][x][i].lbIndex];
          lb->lpIndex[lb->lpCount++] = (x + hstep) << 16 | j;
          verticalPoint[color][x + hstep][j].lbIndex =
            verticalPoint[color][x][i].lbIndex;
            //std::cout << "4;";
          // Otherwise, create a new field line blob
        } else if (verticalBlob[color].size() < MAX_LINE_BLOBS - 1){
            //std::cout << "st:(" << x / hstep << "," << i << ") ,, ";
          const Blob b;
          verticalBlob[color].push_back(b);
          Blob *lb = &verticalBlob[color].back();
          lb->lpCount = 0;
          lb->lpIndex[lb->lpCount++] = x << 16 | i;
          lb->lpIndex[lb->lpCount++] = (x + hstep) << 16 | j;
          verticalPoint[color][x][i].lbIndex =
            verticalPoint[color][x + hstep][j].lbIndex = verticalBlob[color].size() - 1;
        } else {
          visionLog((30, "****MAX_LINE_BLOBS exceeded while forming blobs. Please increase in Vision.h"));
        }
      }
    }
  }
  //std::cout << "\n";
}


void BlobDetector::calculateVerticalBlobData (unsigned char color) {

  // Calculating Data for each blob
  for (uint16_t i = 0; i < verticalBlob[color].size(); i++) {
    Blob *lbi = &verticalBlob[color][i];
    //if(color == c_WHITE) std::cout << i << ": " << (lbi->lpIndex[0] >> 18) << ",";
    lbi->invalid = false;

    // To Calculate the start and end slopes, we need to avergae out a few values to get correct results.
    uint16_t amountAverage = (2 * lbi->lpCount) / 3;
    amountAverage = (amountAverage > 5) ? 5 : amountAverage;

    // Start Slope Initial Point
    VisionPoint *lp0 = &verticalPoint[color][lbi->lpIndex[0] >> 16][lbi->lpIndex[0] & 0xfffful];
    uint16_t y0 = (lp0->yi + lp0->yf) / 2, x0 = lp0->xi;

    visionLog((45,  "Blob %i: (%i, %i)", i, x0, y0));

    // Start Slope Finish Point
    VisionPoint *lp1 = &verticalPoint[color][lbi->lpIndex[amountAverage] >> 16][lbi->lpIndex[amountAverage] & 0xfffful];
    uint16_t y1 = (lp1->yi + lp1->yf) / 2, x1 = lp1->xi;

    // End Slope Start Point
    VisionPoint *lp2 = &verticalPoint[color][lbi->lpIndex[lbi->lpCount - 1 - amountAverage] >> 16][lbi->lpIndex[lbi->lpCount - 1 - amountAverage] & 0xfffful];
    uint16_t y2 = (lp2->yi + lp2->yf) / 2, x2 = lp2->xi;

    // End Slope Finish Point
    VisionPoint *lp3 = &verticalPoint[color][lbi->lpIndex[lbi->lpCount - 1] >> 16][lbi->lpIndex[lbi->lpCount - 1] & 0xfffful];
    uint16_t y3 = (lp3->yi + lp3->yf) / 2, x3 = lp3->xi;

    // Calculate Final Values;
    lbi->diffStart = ((float) (y1 - y0)) / (x1 - x0);
    lbi->diffEnd = ((float) (y3 - y2)) / (x3 - x2);

    if (amountAverage == lbi->lpCount - 1) {
      lbi->doubleDiff = 0;
    } else {
      lbi->doubleDiff = (lbi->diffEnd - lbi->diffStart) / (x2 - x0);
    }

    // Calculate width at the end
    amountAverage = (lbi->lpCount > 5) ? 5 : lbi->lpCount - 1;
    uint16_t width = 0;
    for (uint16_t point = 0; point <= amountAverage; point++) {
      uint16_t yi = verticalPoint[color][lbi->lpIndex[point] >> 16][lbi->lpIndex[point] & 0xfffful].yi;
      uint16_t yf = verticalPoint[color][lbi->lpIndex[point] >> 16][lbi->lpIndex[point] & 0xfffful].yf;
      width += yf - yi;
    }
    width /= amountAverage + 1;
    lbi->widthStart = width;

    // Calculate width at the end
    width = 0;
    for (uint16_t point = lbi->lpCount - amountAverage - 1; point < lbi->lpCount; point++) {
      uint16_t yi = verticalPoint[color][lbi->lpIndex[point] >> 16][lbi->lpIndex[point] & 0xfffful].yi;
      uint16_t yf = verticalPoint[color][lbi->lpIndex[point] >> 16][lbi->lpIndex[point] & 0xfffful].yf;
      width += yf - yi;
    }
    width /= amountAverage + 1;
    lbi->widthEnd = width;

    lbi->xi = x0;
    lbi->yi = y0;
    lbi->xf = x3;
    lbi->yf = y3;

    visionLog((45,  "Blob %i:- Starts at (%i, %i), Ends at (%i, %i), Num Line Points: %i, Calculate Values = (%f, %f, %f), width = (%i, %i)", i, x0, y0, x3, y3, lbi->lpCount, lbi->diffStart, lbi->diffEnd, lbi->doubleDiff, lbi->widthStart, lbi->widthEnd));
  }
  //std::cout << "\n";
}

void BlobDetector::calculateHorizontalBlobData(unsigned char color) {

  // Calculating Data for each blob
  for (uint16_t i = 0; i < horizontalBlob[color].size(); i++) {

    Blob *lbi = &horizontalBlob[color][i];
    lbi->invalid = false;

    // To Calculate the start and end slopes, we need to avergae out a few values to get correct results.
    uint16_t amountAverage = (2 * lbi->lpCount) / 3;
    amountAverage = (amountAverage > 5) ? 5 : amountAverage;

    // Start Slope Initial Point
    VisionPoint *lp0 = &horizontalPoint[color][lbi->lpIndex[0] >> 16][lbi->lpIndex[0] & 0xfffful];
    uint16_t y0 = lp0->yi, x0 = (lp0->xi + lp0->xf) / 2;

    // Start Slope Finish Point
    VisionPoint *lp1 = &horizontalPoint[color][lbi->lpIndex[amountAverage] >> 16][lbi->lpIndex[amountAverage] & 0xfffful];
    uint16_t y1 = lp1->yi, x1 = (lp1->xi + lp1->xf) / 2;

    // End Slope Start Point
    VisionPoint *lp2 = &horizontalPoint[color][lbi->lpIndex[lbi->lpCount - 1 - amountAverage] >> 16][lbi->lpIndex[lbi->lpCount - 1 - amountAverage] & 0xfffful];
    uint16_t y2 = lp2->yi, x2 = (lp2->xi + lp2->xf) / 2;

    // End Slope Finish Point
    VisionPoint *lp3 = &horizontalPoint[color][lbi->lpIndex[lbi->lpCount - 1] >> 16][lbi->lpIndex[lbi->lpCount - 1] & 0xfffful];
    uint16_t y3 = lp3->yi, x3 = (lp3->xi + lp3->xf) / 2;

    // Calculate Final Values;
    lbi->diffStart = ((float) (x1 - x0)) / (y1 - y0);
    lbi->diffEnd = ((float) (x3 - x2)) / (y3 - y2);

    if (amountAverage == lbi->lpCount - 1) {
      lbi->doubleDiff = 0;
    } else {
      lbi->doubleDiff = (lbi->diffEnd - lbi->diffStart) / (y2 - y1);
    }

    // Calculate width at the end
    amountAverage = (lbi->lpCount > 5) ? 5 : lbi->lpCount - 1;
    uint16_t width = 0;
    for (uint16_t point = 0; point <= amountAverage; point++) {
      uint16_t xi = horizontalPoint[color][lbi->lpIndex[point] >> 16][lbi->lpIndex[point] & 0xfffful].xi;
      uint16_t xf = horizontalPoint[color][lbi->lpIndex[point] >> 16][lbi->lpIndex[point] & 0xfffful].xf;
      width += xf - xi;
    }
    width /= amountAverage + 1;
    lbi->widthStart = width;

    // Calculate width at the end
    width = 0;
    for (uint16_t point = lbi->lpCount - amountAverage - 1; point < lbi->lpCount; point++) {
      uint16_t xi = horizontalPoint[color][lbi->lpIndex[point] >> 16][lbi->lpIndex[point] & 0xfffful].xi;
      uint16_t xf = horizontalPoint[color][lbi->lpIndex[point] >> 16][lbi->lpIndex[point] & 0xfffful].xf;
      width += xf - xi;
    }
    width /= amountAverage + 1;
    lbi->widthEnd = width;

    lbi->xi = x0;
    lbi->yi = y0;
    lbi->xf = x3;
    lbi->yf = y3;

    visionLog((45,  "Blob %i: (%i, %i) - %f, %f, %f, %i, %i", i, x0, y0, lbi->diffStart, lbi->diffEnd, lbi->doubleDiff, lbi->widthStart, lbi->widthEnd));
  }
}


void BlobDetector::verticalBlobSort(unsigned char color) {
  // Selection Sort
  for (uint16_t i = 0; i < verticalBlob[color].size(); i++) {
    Blob *minIndex = &verticalBlob[color][i];
    VisionPoint *lp = &verticalPoint[color][minIndex->lpIndex[0] >> 16][minIndex->lpIndex[0] & 0xfffful];
    uint32_t minVal = lp->xi * iparams_.height + ((lp->yi + lp->yf) / 2);
    bool isOrigValue = true;
    for (uint16_t j = i + 1; j < verticalBlob[color].size(); j++) {
      Blob *tempIndex = &verticalBlob[color][j];
      lp = &verticalPoint[color][tempIndex->lpIndex[0] >> 16][tempIndex->lpIndex[0] & 0xfffful];
      uint32_t tempVal = lp->xi * iparams_.height + ((lp->yi + lp->yf) / 2);
      if (tempVal < minVal) {
        minIndex = tempIndex;
        minVal = tempVal;
        isOrigValue = false;
      }
    }
    if (!isOrigValue) {
      Blob temp = *minIndex;
      *minIndex = verticalBlob[color][i];
      verticalBlob[color][i] = temp;
    }
  }
}

void BlobDetector::horizontalBlobSort(unsigned char color) {
  // Selection Sort
  for (uint16_t i = 0; i < horizontalBlob[color].size(); i++) {
    Blob *minIndex = &horizontalBlob[color][i];
    VisionPoint *lp = &horizontalPoint[color][minIndex->lpIndex[0] >> 16][minIndex->lpIndex[0] & 0xfffful];
    uint32_t minVal = ((lp->xi + lp->xf) / 2) + (lp->yi) * iparams_.width;
    bool isOrigValue = true;
    for (uint16_t j = i + 1; j < horizontalBlob[color].size(); j++) {
      Blob *tempIndex = &horizontalBlob[color][j];
      lp = &horizontalPoint[color][tempIndex->lpIndex[0] >> 16][tempIndex->lpIndex[0] & 0xfffful];
      uint32_t tempVal = ((lp->xi + lp->xf) / 2) + (lp->yi) * iparams_.width;
      if (tempVal < minVal) {
        minIndex = tempIndex;
        minVal = tempVal;
        isOrigValue = false;
      }
    }
    if (!isOrigValue) {
      Blob temp = *minIndex;
      *minIndex = horizontalBlob[color][i];
      horizontalBlob[color][i] = temp;
    }
  }
}


void BlobDetector::formOrangeBlobs() {
  formBlobs(c_ORANGE);
}

void BlobDetector::formBlobs(Color color) {
  horizontalBlob[color].clear();
  formBlobs(horizontalBlob[color], color);
}

void BlobDetector::formBlobs(BlobCollection& blobs, Color color) {
  int hstep, vstep;
  classifier_->getStepSize(hstep,vstep);

  for (uint16_t y = 0; y < iparams_.height - vstep; y += vstep){
    for (uint16_t i = 0; i < horizontalPointCount[color][y]; i++) {
      VisionPoint *lpi = &horizontalPoint[color][y][i];
      //std::cout << lpi->xi << "," << lpi->xf << ",";
      for (uint16_t j = 0; j < horizontalPointCount[color][y + vstep]; j++) {
        VisionPoint *lpj = &horizontalPoint[color][y + vstep][j];

        // if line segments overlap
        if (lpi->xi < lpj->xf && lpi->xf > lpj->xi) {

          // if line segment belongs to an existing blob
          if (y > 0 && lpi->lbIndex != (uint16_t)-1) {
            Blob *bl = &blobs[lpi->lbIndex];
            if(bl->lpCount >= MAX_BLOB_VISIONPOINTS) {
                visionLog((30, "****Max line points exceeded while forming %s blobs. Please increase in Blob.h", COLOR_NAME(color)));
            }
            else {
                bl->lpIndex[bl->lpCount++] = (y + vstep) << 16 | j;
                lpj->lbIndex = lpi->lbIndex;
            }

            // otherwise, create a new blob
          } else if (blobs.size() < MAX_LINE_BLOBS - 1) {
            const Blob b;
            blobs.push_back(b);
            Blob *bl = &blobs.back();
            bl->lpCount = 0;
            bl->lpIndex[bl->lpCount++] =  y      << 16 | i;
            bl->lpIndex[bl->lpCount++] = (y + vstep) << 16 | j;
            lpi->lbIndex = lpj->lbIndex = blobs.size() - 1;
            visionLog((30, "Created %s blob", COLOR_NAME(color)));
            // bound check
          } else {
            visionLog((30, "****MAX_LINE_BLOBS exceeded while forming %s blobs. Please increase in Vision.h", COLOR_NAME(color)));
          }
        }
      }
    }
  }
}

void BlobDetector::calculateOrangeBlobData() {
  calculateBlobData(c_ORANGE);
}

void BlobDetector::calculateBlobData(Color color) {
  calculateBlobData(horizontalBlob[color], color);
}

void BlobDetector::calculateBlobData(BlobCollection& blobs, Color color) {
  // Compute each blob's xi, xf, dx, yi, yf, dy
  for (unsigned int i = 0; i < blobs.size(); i++) {
    Blob *bl = &blobs[i];
    int y = bl->lpIndex[0] >> 16;
    int k = bl->lpIndex[0] & 0xffff;
    bl->xi = horizontalPoint[color][y][k].xi;
    bl->xf = horizontalPoint[color][y][k].xf;
    bl->correctPixelRatio = horizontalPoint[color][y][k].dx;
    for (int j = 1; j < bl->lpCount; j++) {
      int y = bl->lpIndex[j] >> 16;
      int k = bl->lpIndex[j] & 0xffff;
      VisionPoint *lp = &horizontalPoint[color][y][k];
      bl->xi = bl->xi > lp->xi ? lp->xi : bl->xi;
      bl->xf = bl->xf < lp->xf ? lp->xf : bl->xf;
      bl->correctPixelRatio += lp->dx;
    }
    bl->dx = bl->xf - bl->xi + 1;
    bl->yi = bl->lpIndex[0] >> 16;
    bl->yf = bl->lpIndex[bl->lpCount - 1] >> 16;
    bl->dy = bl->yf - bl->yi;
    bl->avgX = (bl->xi + bl->xf) / 2;
    bl->avgY = (bl->yi + bl->yf) / 2;
    visionLog((30, "%s blob %i calculated: (xi:xf,yi:yf) = (%i:%i,%i:%i)", COLOR_NAME(color), i, bl->xi, bl->xf, bl->yi, bl->yf));
    bl->correctPixelRatio /= 0.5f * bl->dx * bl->dy;
    visionLog((20,   "correctpixel[%i] = %f",i,bl->correctPixelRatio));
  }
}


void BlobDetector::calculateBandBlobData(Color color) {
  for (uint32_t i = 0; i < verticalBlob[color].size(); i++) {

    Blob * blob = &verticalBlob[color][i];

    int j = 0;
    VisionPoint *lp = &verticalPoint[color][blob->lpIndex[j] >> 16][blob->lpIndex[j] & 0xfffful];
    blob->xi = lp->xi;

    uint32_t sumY = (lp->yi + lp->yf) >> 1 ;
    float sumWidth = lp->dy;

    uint16_t minY = iparams_.height - 1, maxY = 0;

    for (j = 1; j < blob->lpCount; j++) {
      lp = &verticalPoint[color][blob->lpIndex[j] >> 16][blob->lpIndex[j] & 0xfffful];
      sumY += (lp->yi + lp->yf) >> 1 ;
      sumWidth += lp->dy;
      minY = std::min(lp->yi, minY);
      maxY = std::max(lp->yf, maxY);
    }
    blob->yi = minY;
    blob->yf = maxY;
    blob->xf = lp->xi;


    blob->avgX = (blob->xf + blob->xi) >> 1;
    blob->avgY = sumY / blob->lpCount;
    blob->avgWidth = sumWidth / blob->lpCount;
  }
}


uint16_t BlobDetector::mergeHorizontalBlobs(Color color, uint16_t* mergeIndex, int mergeCount) {
  return mergeBlobs(color, horizontalBlob, mergeIndex, mergeCount);
}

uint16_t BlobDetector::mergeVerticalBlobs(Color color, uint16_t* mergeIndex, int mergeCount) {
  return mergeBlobs(color, verticalBlob, mergeIndex, mergeCount);
}

uint16_t BlobDetector::mergeBlobs(Color color, std::vector<BlobCollection>& blobs, uint16_t *mergeIndex, int mergeCount) {
  return mergeBlobs(blobs[color], mergeIndex, mergeCount);
}
// This function merges blobs by setting a merge index for blobs that get merged down. The
// merge index of a blob is the index of the earlier blob it was merged with. So the blobs
// that should actually be used are the ones whose merge indexes point to themselves.
// i.e. use blobs[i] iff mergeIndex[i] == i
uint16_t BlobDetector::mergeBlobs(BlobCollection& blobs, uint16_t *mergeIndex, int mergeCount) {
  // Initialize
  for (uint16_t i = 0; i < blobs.size(); i++)
    mergeIndex[i] = mergeCount;

  // For each (blob[i], blob[j]) tuple
  for (uint16_t i = 0; i < blobs.size(); i++) {
    Blob *bli = &blobs[i];
    for (uint16_t j = i + 1; j < blobs.size(); j++) {
      Blob *blj = &blobs[j];
      //visionLog((39, "comparing (%i,%i),(%i,%i) with (%i,%i),(%i,%i)", bli->xi, bli->xf, bli->yi, bli->yf, blj->xi, blj->xf, blj->yi, blj->yf));

      // Merge close and overlapping blobs
      bool closeX = (abs((int)bli->xf - (int)blj->xi) < 30);
      bool closeY = (abs((int)bli->yf - (int)blj->yi) < 30);
      bool overlapX = (bli->xi <= blj->xf && bli->xf >= blj->xi);
      bool overlapY = (bli->yi <= blj->yf && bli->yf >= blj->yi);
      //printf("trying to merge blob %i,%i to %i,%i with %i,%i to %i,%i: closex, %i, closey, %i, overx, %i, overy, %i\n",
          //bli->xi, bli->xf, bli->yi, bli->yf,
          //blj->xi, blj->xf, blj->yi, blj->yf,
          //closeX, closeY, overlapX, overlapY);
      if ((overlapX && closeY) ||
          (closeX && overlapY) ||
          (overlapX && overlapY)) {
            //visionLog((39,"blob %i overlaps with %i: closeX: %i closeY: %i overlapX: %i overlapY: %i", i, j, closeX, closeY, overlapX, overlapY));
        bli->xi = std::min(bli->xi, blj->xi);
        bli->xf = std::max(bli->xf, blj->xf);
        bli->yi = std::min(bli->yi, blj->yi);
        bli->yf = std::max(bli->yf, blj->yf);
        bli->correctPixelRatio = (bli->dx * bli->dy * bli->correctPixelRatio + blj->dy * blj->dx * blj->correctPixelRatio);
        // Update dx, dy
        bli->dx = bli->xf - bli->xi;
        bli->dy = bli->yf - bli->yi;
        bli->correctPixelRatio /= 0.5f * bli->dx * bli->dy;

        for (uint16_t k = 0; k < blj->lpCount && bli->lpCount < MAX_BLOB_VISIONPOINTS; k++) {
          bli->lpIndex[bli->lpCount++] = blj->lpIndex[k];
        }

        uint16_t minBlobIndex = std::min(i, j);
        uint16_t maxBlobIndex = std::max(i, j);
        if(mergeIndex[minBlobIndex] == mergeCount) // if the minimum one hasn't been merged
          mergeIndex[maxBlobIndex] = minBlobIndex; // this is the first merging for both blobs, so put them together
        else
          mergeIndex[maxBlobIndex] = mergeIndex[minBlobIndex]; // otherwise, merge with the earlier blob's merged blob
        for (uint16_t k = 0; k < blobs.size(); k++)
          if (mergeIndex[k] == maxBlobIndex)
            mergeIndex[k] = mergeIndex[maxBlobIndex];
      }
    }

    bli->avgX = (bli->xi + bli->xf) / 2;
    bli->avgY = (bli->yi + bli->yf) / 2;

    // TODO:  This loop is incomplete.  After a blob merge, you have
    // to restart the entire loop which requires a runtime of O(n^4)
  }
  uint16_t sum = 0;
  for (uint16_t i = 0; i < blobs.size(); i++)
    sum += (mergeIndex[i] == mergeCount);
  return sum;
}
