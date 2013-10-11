#include "Classifier.h"
#include <iostream>

Classifier::Classifier(const VisionBlocks& vblocks, const VisionParams& vparams, const ImageParams& iparams, const Camera::Type& camera) :
    vblocks_(vblocks), vparams_(vparams), iparams_(iparams), camera_(camera), initialized_(false) {
  segImg_ = new unsigned char[iparams.size];
  segImgLocal_ = segImg_;
  setStepScale(iparams_.defaultHorizontalStepScale, iparams_.defaultVerticalStepScale);
  
  horizontalPointCount = new uint32_t*[NUM_COLORS];
  for(int i=0;i<NUM_COLORS;i++) {
    horizontalPointCount[i] = new uint32_t[iparams_.height];
    memset(horizontalPointCount[i], 0, iparams_.height);
  }
  verticalPointCount = new uint32_t*[NUM_COLORS];
  for(int i=0;i<NUM_COLORS;i++) {
    verticalPointCount[i] = new uint32_t[iparams_.width];
    memset(verticalPointCount[i], 0, iparams_.width);
  }
  horizontalPoint = new VisionPoint**[NUM_COLORS];
  for (int i=0; i<NUM_COLORS; i++){
      horizontalPoint[i] = new VisionPoint*[iparams_.height];
      memset(horizontalPoint[i], 0, iparams_.height);
  }
  verticalPoint = new VisionPoint**[NUM_COLORS];
  for (int i=0; i<NUM_COLORS; i++){
      verticalPoint[i] = new VisionPoint*[iparams_.width];
      memset(verticalPoint[i], 0, iparams_.width);
  }
}

Classifier::~Classifier() {
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
        if(horizon_.exists && c == c_ORANGE && !horizon_.isAbovePoint(x, y)) continue;
        if(horizon_.exists && c == c_WHITE && !horizon_.isAbovePoint(x,y)) c = c_ROBOT_WHITE; // We shouldn't be handling lines above the horizon
        segImg_[iparams_.width * y + x] = c;
      }
    }
  }
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

void Classifier::constructRuns(){
    //construct vertical runs
    printf("Hare Krishna. In construct Runs!\n");
    for(int i=0; i<iparams_.width; i++){
        uint16_t xi, xf;
        xi = xf = i;
	for(int j=1; j<iparams_.height;){
            //see how far the current run goes
            uint16_t yi = j-1;
            while (getSegPixelValueAt(i, j) == getSegPixelValueAt(i, j-1)){
		j++;
	    }
            //initialize a new visionPoint struct for the current run
            unsigned char runColor = getSegPixelValueAt(i, yi);
            //verticalPoint[runColor][i][yi] = new VisionPoint;
            VisionPoint *v = &verticalPoint[runColor][i][yi];
            v->xi = xi;
            v->xf = xf;
            v->yi = yi;
            v->yf = j-1;
            v->dx = 0;
            v->dy = v->yf - v->yi;
            //increment run count for current color and line
            verticalPointCount[runColor][i]++;
            //increment j so we don't repeat a check
            j++;
	}
    }


    //construct horizontal runs
    for(int j=0; j<iparams_.height; j++){
        uint16_t yi, yf;
        yi = yf = j;
	for(int i=1; i<iparams_.width; i++){
            //see how far the current run goes
            uint16_t xi = i-1;
            while (getSegPixelValueAt(i, j) == getSegPixelValueAt(i-1, j)){
		i++;
	    }
            //initialize a new visionPoint struct for the current run
            unsigned char runColor = getSegPixelValueAt(xi, j);
            VisionPoint *v = &horizontalPoint[runColor][j][xi];
            v->xi = xi;
            v->xf = i-1;
            v->yi = yi;
            v->yf = yf;
            v->dy = 0;
            v->dx = v->xf - v->xi;
            //increment run count for current color and line
            horizontalPointCount[runColor][i]++;
            //increment i so we don't repeat a check
            i++;
	}
    }
}

