#include "Classifier.h"
#include <iostream>

Classifier::Classifier(const VisionBlocks& vblocks, const VisionParams& vparams, const ImageParams& iparams, const Camera::Type& camera) :
    vblocks_(vblocks), vparams_(vparams), iparams_(iparams), camera_(camera), initialized_(false), doingHighResScan_(false) {

  segImg_ = new unsigned char[iparams.size];
  segImgLocal_ = segImg_;
  vGreenPosition = new uint16_t[iparams_.width];
  hGreenPosition = new uint16_t[iparams_.height];

  horizontalPoint = new VisionPoint**[NUM_COLORS];
  for(int i=0;i<NUM_COLORS;i++) {
    horizontalPoint[i] = new VisionPoint*[iparams_.height];
    for(int j = 0; j < iparams_.height; j++)
      horizontalPoint[i][j] = new VisionPoint[iparams_.width];
  }
  verticalPoint = new VisionPoint**[NUM_COLORS];
  for(int i=0;i<NUM_COLORS;i++) {
    verticalPoint[i] = new VisionPoint*[iparams_.width];
    for(int j = 0; j < iparams_.width; j++)
      verticalPoint[i][j] = new VisionPoint[iparams_.height];
  }
  horizontalPointCount = new uint32_t*[NUM_COLORS];
  for(int i=0;i<NUM_COLORS;i++) {
    horizontalPointCount[i] = new uint32_t[iparams_.height];
  }
  verticalPointCount = new uint32_t*[NUM_COLORS];
  for(int i=0;i<NUM_COLORS;i++)
    verticalPointCount[i] = new uint32_t[iparams_.width];

  bodyExclusionAvailable = false;

  setStepScale(iparams_.defaultHorizontalStepScale, iparams_.defaultVerticalStepScale);
}

Classifier::~Classifier() {
  delete [] vGreenPosition;
  delete [] hGreenPosition;

  for(int i=0;i<NUM_COLORS;i++) {
    for(int j = 0; j < iparams_.height; j++)
      delete [] horizontalPoint[i][j];
    delete [] horizontalPoint[i];
  }
  delete [] horizontalPoint;

  for(int i=0;i<NUM_COLORS;i++) {
    for(int j = 0; j < iparams_.width; j++)
      delete [] verticalPoint[i][j];
    delete [] verticalPoint[i];
  }
  delete [] verticalPoint;
  
  for(int i=0;i<NUM_COLORS;i++)
    delete [] horizontalPointCount[i];
  delete [] horizontalPointCount;
  
  for(int i=0;i<NUM_COLORS;i++)
    delete [] verticalPointCount[i];
  delete [] verticalPointCount;
  delete [] segImgLocal_;
}

bool Classifier::setImagePointers() {
  bool imageLoaded = vblocks_.image->loaded_;
  if(vblocks_.image == NULL) {
    printf("No image block loaded! Classification failed.\n");
    return false;
  }
  if(vblocks_.robot_vision == NULL) {
    printf("No vision block loaded! Classification failed.\n");
    return false;
  }
  if(camera_ == Camera::TOP) {
    #ifdef TOOL
    if(imageLoaded) {
    #endif
      vblocks_.robot_vision->setSegImgTop(segImg_);
      img_ = vblocks_.image->getImgTop();
      if(!img_) return false;
    #ifdef TOOL
    } else if(vblocks_.robot_vision->loaded_) {
      segImg_ = vblocks_.robot_vision->getSegImgTop();
    }
    #endif
  }
  else {
    #ifdef TOOL
    if(imageLoaded) {
    #endif
      vblocks_.robot_vision->setSegImgBottom(segImg_);
      img_ = vblocks_.image->getImgBottom();
      if(!img_) return false;
    #ifdef TOOL
    } else if(vblocks_.robot_vision->loaded_) {
      segImg_ = vblocks_.robot_vision->getSegImgBottom();
    }
    #endif
  }
  if(!initialized_) {
    #ifdef TOOL
    if(imageLoaded)
    #endif
    memset(segImg_, c_UNDEFINED, sizeof(unsigned char) * iparams_.size);
    initialized_ = true;
  }
  return true;
}

bool Classifier::classifyImage(unsigned char *colorTable) {
  if(!setImagePointers()) return false;
  clearPreviousHighResScans();
  FocusArea area(0, 0, iparams_.width - 1, iparams_.height - 1);
  classifyImage(area, colorTable);
  return true;
}

void Classifier::classifyImage(const std::vector<FocusArea>& areas, unsigned char *colorTable) {
  if(!setImagePointers()) return;
  for(unsigned int i = 0; i < areas.size(); i++)
    classifyImage(areas[i], colorTable);
}


void Classifier::classifyImage(const FocusArea& area, unsigned char* colorTable){
  bool imageLoaded = vblocks_.image->loaded_;
  if(!imageLoaded) {
    visionLog((20, "Classifying with no raw image"));
  }
  colorTable_ = colorTable;
  visionLog((28, "Classifying on area %i,%i to %i,%i with horizon %2.f,%2.f", area.x1, area.y1, area.x2, area.y2, horizon_.gradient, horizon_.offset));
  for (int y = area.y1; y <= area.y2; y += vstep_) {
    for(int x = area.x1; x <= area.x2; x += hstep_) {
      Color c;
#ifdef TOOL
      if (imageLoaded) // if a raw image is available
#endif
      {
        c = ColorTableMethods::xy2color(img_, colorTable, x, y, iparams_.width);
        if(c == c_ORANGE && !horizon_.isAbovePoint(x, y)) continue;
        if(c == c_WHITE && !horizon_.isAbovePoint(x,y)) c = c_ROBOT_WHITE; // We shouldn't be handling lines above the horizon
        segImg_[iparams_.width * y + x] = c;
      }
    }
  }
}

bool Classifier::startHighResScan(Color c, int hStepScale, int vStepScale) {
  std::vector<FocusArea> areas;
  if(!prepareFocusAreas(areas, c)) {
    visionLog((28, "Threw out %i focus areas", areas.size()));
    return false;
  }
  int totalArea = 0;
  int hstep = 1 << hStepScale, vstep = 1 << vStepScale;
  for(unsigned int i = 0; i < areas.size(); i++) {
    const FocusArea& area = areas[i];
    totalArea += area.area / hstep / vstep;
  }
  if((areas.size() > MAX_FOCUS_AREA_COUNT && totalArea > MAX_FOCUS_AREA) || totalArea > 10000) {
    visionLog((28, "focus area: %i (max %i), count: %i (max %i)", totalArea, MAX_FOCUS_AREA, areas.size(), MAX_FOCUS_AREA_COUNT));
    return false;
  }
  // If there are too many focus areas it will slow things down a lot,
  // and this indicates that we're close to the scan object anyway.
  doingHighResScan_ = true;
  setStepScale(hStepScale, vStepScale);
  visionLog((28, "Preparing high res scan on %i focus areas for %s", areas.size(), COLOR_NAME(c)));
  classifyImage(areas, colorTable_);
  constructRuns(areas, 1 << c);
  return true;
}

bool Classifier::startHighResBallScan() {
  didHighResBallScan = true;
  return startHighResScan(c_ORANGE, 0, 0);
}

bool Classifier::startHighResGoalScan() {
  return startHighResScan(c_YELLOW, 1, 2);
}

void Classifier::completeHighResScan() {
  setStepScale(iparams_.defaultHorizontalStepScale, iparams_.defaultVerticalStepScale);
  doingHighResScan_ = false;
}

void Classifier::setHorizon(HorizonLine horizon) {
  horizon_ = horizon;
}

void Classifier::clearPoints(int colorFlags) {
  // Reset Vertical Point Counts
  for (int z = 0; z < NUM_COLORS; z++) {
    if(!isInFlags(z, colorFlags)) continue;
    memset(verticalPointCount[z], 0, sizeof(uint32_t) * iparams_.width);
  }

  // Reset Horizontal Point Counts
  for (int z = 0; z < NUM_COLORS; z++) {
    if(!isInFlags(z, colorFlags)) continue;
    memset(horizontalPointCount[z], 0, sizeof(uint32_t) * iparams_.height);
  }
}

void Classifier::constructRuns(int colorFlags) {
  clearPoints(colorFlags);
  FocusArea area(0, 0, iparams_.width - 1, iparams_.height - 1);
  constructRuns(area, colorFlags);
}

void Classifier::constructRuns(const std::vector<FocusArea>& areas, int colorFlags) {
  clearPoints(colorFlags);
  for(unsigned int i = 0; i < areas.size(); i++)
    constructRuns(areas[i], colorFlags);
}

void Classifier::constructRuns(const FocusArea& area, int colorFlags) {
  int x = 0, y = 0;

  // Reset stuff used to track vertical line points in horizontal scan
  uint8_t vRecent[iparams_.width];
  memset(vRecent, c_UNDEFINED, sizeof(uint8_t) * iparams_.width);
  uint8_t vRunClr[iparams_.width]; // Running line region color
  memset(vRunClr, c_UNDEFINED, sizeof(uint8_t) * iparams_.width);
  uint16_t vStartY[iparams_.width]; // Start of running line region
  memset(vRunClr, -1, sizeof(uint16_t) * iparams_.width);

  for (y = area.y1; y <= area.y2; y += vstep_) {

    hGreenPosition[y] = (uint16_t)-1;
    uint8_t hRecent = c_UNDEFINED;
    uint8_t hRunClr = c_UNDEFINED;
    uint16_t hStartX = 0;

    for (x = area.x1; x <= area.x2; x += hstep_) {
      uint8_t c = segImg_[iparams_.width * y + x];

      // HORIZONTAL

      // Segment Finishes - when 2 consecutive pixels are different
      // Allow runs of length 1 for the ball
      if (hRunClr != c && (hRunClr != hRecent || hRunClr == c_ORANGE || c == c_ORANGE) && isInFlags(hRunClr, colorFlags)) {
        if (hRunClr == c_FIELD_GREEN) {
          hGreenPosition[y] = hStartX;
        } else if (hRunClr == c_BLUE || hRunClr == c_PINK || hRunClr == c_YELLOW || hRunClr == c_ORANGE ||
                   (hRunClr == c_WHITE && hGreenPosition[y] != (uint16_t) -1)) {

          uint16_t valXI = hStartX;
          uint16_t valXF = (x - 2 * hstep_);

          // Allow runs of length 1 for the ball
          if(hRunClr == c_ORANGE)
            valXF = (x - hstep_);

          switch(hRunClr) {

          case c_WHITE: {
            bool aboveHorizon = horizon_.exists && (horizon_.gradient * valXI + horizon_.offset > y);
            bool widthTooHigh = valXF - valXI > 75 * iparams_.origFactor;
            bool onShoulder = bodyExclusionAvailable;
            uint16_t valAvgX = (valXI + valXF) / 2;
            uint16_t xIndex = ((NUM_BODY_EXCL_POINTS - 1) * valAvgX) / iparams_.width;
            onShoulder &=
              bodyExclusionSlope[xIndex] * valAvgX + bodyExclusionOffset[xIndex] < y;
            if (aboveHorizon || widthTooHigh || onShoulder) {
              break;
            }
          }

          default: {
            // Save line segment
            VisionPoint *lp =
              &horizontalPoint[hRunClr][y][horizontalPointCount[hRunClr][y]++];
            lp->xi = valXI;
            lp->xf = valXF;
            lp->dx = lp->xf - lp->xi + 1;
            lp->yi = lp->yf = y;
            lp->dy = 1;
            lp->isValid = true;
            lp->lbIndex = (uint16_t)-1; // Will get this information later
          }

          }
        }

        hRunClr = c_UNDEFINED;
      }
      // Segment Starts - when 2 consecutive pixels are same
      // Allow runs of length 1 for the ball
      if (c != hRunClr && (c == c_ORANGE || c == hRecent)) {
        hRunClr = c;
        if(x > hstep_)
          hStartX = x - hstep_;
        else
          hStartX = 0;
      }
      hRecent = c;

      // VERTICAL

      uint8_t runClr = vRunClr[x];
      uint8_t recent = vRecent[x];

      // Vision Point Segment finishes
      if (runClr != recent && runClr != c && isInFlags(runClr, colorFlags)) {
        if (runClr == c_FIELD_GREEN) {
          vGreenPosition[x] = vStartY[x];
        } else if (runClr == c_BLUE || runClr == c_PINK ||
                   (vGreenPosition[x] != (uint16_t)-1 && runClr == c_WHITE)) {

          uint16_t valX = x;
          uint16_t valYI = vStartY[x];
          uint16_t valYF = y - 4;

          // Introduce some special checks here
          switch(runClr) {

          case c_WHITE: {
            bool aboveHorizon = horizon_.exists && (horizon_.gradient * valX + horizon_.offset > valYI);
            bool widthTooHigh = valYF - valYI > 125 * iparams_.origFactor;
            bool onShoulder = bodyExclusionAvailable;
            uint16_t xIndex = ((NUM_BODY_EXCL_POINTS - 1) * valX) / iparams_.width;
            onShoulder &=
              bodyExclusionSlope[xIndex] * valX + bodyExclusionOffset[xIndex] < valYF;
            if (aboveHorizon || widthTooHigh || onShoulder) {
              break;
            }
          }

          default: {
            // Save Line Segment
            VisionPoint *lp =
              &verticalPoint[runClr][x][verticalPointCount[runClr][x]++];
            lp->xi = lp->xf = valX;
            lp->dx = 1;
            lp->yi = valYI;
            lp->yf = valYF;
            lp->dy = lp->yf - lp->yi + 1;
            lp->isValid = true;
            lp->lbIndex = (uint16_t)-1;
          }

          }
        }
        vRunClr[x] = c_UNDEFINED;
      }

      // Vision Point Segment Starts
      if (c != runClr && c == recent) {
        vRunClr[x] = c;
        vStartY[x] = y - 2;
      }
      vRecent[x] = c;
    }

    // Last Horizontal Segment
    // Don't save the white ones
    if(!isInFlags(hRunClr, colorFlags)) continue;
    if (hRunClr == c_FIELD_GREEN) {
      hGreenPosition[y] = hStartX;
    } else if (hRunClr == c_BLUE || hRunClr == c_YELLOW || hRunClr == c_ORANGE) {
      // Save line segment
      completeHorizontalRun(hRunClr, y, hStartX, area.x2);
    }
  }

  // Last Vertical Segment
  // Don't save the white ones
  for (x = 0; x < iparams_.width; x++) {
    uint8_t runClr = vRunClr[x];
    if(!isInFlags(runClr, colorFlags)) continue;
    if (runClr == c_FIELD_GREEN) {
      vGreenPosition[x] = vStartY[x];
    } else if (runClr == c_BLUE || runClr == c_PINK) {
      // Save line segment
      completeVerticalRun(runClr, x, vStartY[x]);
      //TODO: Fix for high res vertical scans
    }
  }
}

void Classifier::preProcessPoints() {

  // White Points without green at the end are removed
  for (uint16_t x = 0; x < iparams_.width; x++) {
    for (uint16_t point = 0; point < verticalPointCount[c_WHITE][x]; point++) {
      if (verticalPoint[c_WHITE][x][point].yi > vGreenPosition[x]) {
        verticalPointCount[c_WHITE][x] = point;
        break;
      }
    }
  }

  // Horizontal white points without green at the end are also removed
  for (uint16_t y = 0; y < iparams_.height; y++) {
    for (uint16_t point = 0; point < horizontalPointCount[c_WHITE][y]; point++) {
      if (horizontalPoint[c_WHITE][y][point].xi > hGreenPosition[y]) {
        horizontalPointCount[c_WHITE][y] = point;
        break;
      }
    }
  }

  // This is done for the colored bands
  // Join 2 linepoints together which are in the same vertical line and very close.
  if (vparams_.ALLOW_BAND_MERGE_POINTS) {
    unsigned char bandColors[] = {
      c_BLUE,
      c_PINK
    };

    for (uint16_t c = 0; c < 2; c++) {
      unsigned char color = bandColors[c];
      for (uint16_t x = 0; x < iparams_.width; x++) {
        uint16_t skip = 1;
        for (int16_t i = 0; i < ((int16_t)verticalPointCount[color][x]) - 1; i += skip) {
          skip = 1;
          for (uint16_t j = i + 1; j < verticalPointCount[color][x]; j++) {
            if (verticalPoint[color][x][j].yi - verticalPoint[color][x][i].yf <= vparams_.BAND_MERGE_POINT_DIST) {
              verticalPoint[color][x][i].yf = verticalPoint[color][x][j].yf;
              verticalPoint[color][x][i].dy = verticalPoint[color][x][i].yf - verticalPoint[color][x][i].yi + 1;
              verticalPoint[color][x][j].isValid = false;
              skip++;
            }
          }
        }
      }
    }
  }

}

void Classifier::preProcessGoalPoints() {

  // This is done for the goals
  // Join 2 linepoints together which are in the same vertical line and very close.
  if (vparams_.ALLOW_GOAL_MERGE_POINTS) {
    unsigned char goalColors[] = {
      c_YELLOW
    };

    for (uint16_t c = 0; c < 1; c++) {
      unsigned char color = goalColors[c];
      for (uint16_t x = 0; x < iparams_.height; x++) {
        uint16_t skip = 1;
        for (int16_t i = 0; i < ((int16_t)horizontalPointCount[color][x]) - 1; i += skip) {
          skip = 1;
          for (uint16_t j = i + 1; j < horizontalPointCount[color][x]; j++) {
            if (horizontalPoint[color][x][j].xi - horizontalPoint[color][x][i].xf <= vparams_.GOAL_MERGE_POINT_DIST) {
              horizontalPoint[color][x][i].xf = horizontalPoint[color][x][j].xf;
              horizontalPoint[color][x][i].dx = horizontalPoint[color][x][i].xf - horizontalPoint[color][x][i].xi + 1;
              horizontalPoint[color][x][j].isValid = false;
              skip++;
            }
          }
        }
      }
    }
  }

}

void Classifier::completeHorizontalRun(uint8_t &hRunClr, int y, int hStartX, int finX) {
  VisionPoint *lp = &horizontalPoint[hRunClr][y][horizontalPointCount[hRunClr][y]++];
  lp->xi = hStartX;
  lp->xf = ((finX >> hscale_) - 1) << hscale_;
  lp->dx = lp->xf - lp->xi + 1;
  lp->yi = lp->yf = y;
  lp->dy = 1;
  lp->isValid = true;
  lp->lbIndex = (uint16_t)-1;
  hRunClr = c_UNDEFINED;
}

void Classifier::completeVerticalRun(uint8_t &runClr, int x, int vStartY) {
  VisionPoint *lp =
    &verticalPoint[runClr][x][verticalPointCount[runClr][x]++];
  lp->xi = lp->xf = x;
  lp->dx = 1;
  lp->yi = vStartY;
  lp->yf = iparams_.height - (1 << vscale_);
  lp->dy = lp->yf - lp->yi + 1;
  lp->isValid = true;
  lp->lbIndex = (uint16_t)-1;
  runClr = c_UNDEFINED;
}

void Classifier::setStepScale(int h, int v){
    hstep_ = (1 << h);
    vstep_ = (1 << v);
    hscale_ = h;
    vscale_ = v;
}

void Classifier::getStepSize(int& h, int& v){
    h = hstep_;
    v = vstep_;
}

void Classifier::getStepScale(int& h, int& v){
    h = hscale_;
    v = vscale_;
}

bool Classifier::prepareFocusAreas(std::vector<FocusArea>& areas, Color c) {
  int hrange = FOCUS_RANGE_HORIZONTAL(c), vrange = FOCUS_RANGE_VERTICAL(c);
  for(int y = 0; y < iparams_.height; y += vstep_) {
    int ymin = std::max(0, y - vrange), ymax = std::min(iparams_.height - 1, y + vrange);
    for(unsigned int i = 0; i < horizontalPointCount[c][y]; i++) {
      VisionPoint* point = &horizontalPoint[c][y][i];
      int xmin = std::max(0, point->xi - hrange), xmax = std::min(iparams_.width - 1, point->xf + hrange);
      FocusArea area(xmin, ymin, xmax, ymax);
      areas.push_back(area);
    }
  }
  visionLog((28, "%i initial focus areas found", areas.size()));
  std::vector<FocusArea> final;
  switch(c) {
    case c_ORANGE: areas = FocusArea::merge(areas); break;
    case c_YELLOW: 
       areas = FocusArea::mergeVertical(areas, iparams_);
       for(unsigned int i = 0; i < areas.size(); i++)
         if(areas[i].height >= 100)
           final.push_back(areas[i]);
       areas = final;
       break;
    default: break;
  }
  visionLog((28, "%i final focus areas found", areas.size()));
  return areas.size() > 0;
}

void Classifier::clearPreviousHighResScans() {
#ifdef TOOL
  bool imageLoaded = vblocks_.image->loaded_;
  if (imageLoaded) // if a raw image is available
#endif
  {
    if (segImg_) {
      memset(segImg_, c_UNDEFINED, iparams_.size);
      visionLog((20, "clearing seg image"));
    }
  }
}

void Classifier::opticalFlow(){
  const unsigned char * prevImage;
  const unsigned char * curImage;
  bool  imageLoaded = vblocks_.image->loaded_;
  if (imageLoaded){
     prevImage = img_;
     cv::Mat cvPrevImage = color::rawToMat(prevImage, iparams_);
     cv::Mat cvCurImage = color::rawToMat(curImage, iparams_);
     cv::Mat newmat;
     // trial run
     cv::calcOpticalFlowSF(cvPrevImage, cvCurImage, newmat, 3, 2, 4);
     std::cout<< "hare Krishna";
  }
}
