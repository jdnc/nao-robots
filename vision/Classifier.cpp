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
      for (int j=0; j<iparams_.height; j++)
          horizontalPoint[i][j] = new VisionPoint[iparams_.width];
  }
  verticalPoint = new VisionPoint**[NUM_COLORS];
  for (int i=0; i<NUM_COLORS; i++){
      verticalPoint[i] = new VisionPoint*[iparams_.width];
      for(int j=0; j<iparams_.width; j++)
          verticalPoint[i][j] = new VisionPoint[iparams_.height];
  }
}

Classifier::~Classifier() {
  delete [] segImgLocal_;
  // don't delete these
  /*
  for(int i=0; i<NUM_COLORS; i++){
  	delete[] horizontalPointCount[i];
	delete[] verticalPointCount[i];
  }
  delete[] horizontalPointCount;
  delete[] verticalPointCount;
  for(int i=0; i<NUM_COLORS; i++){
    for(int j=0; j<iparams_.width; j++)
      delete[] verticalPoint[i][j];
    delete[] verticalPoint[i];
    for(int k=0; k<iparams_.height; k++)
      delete[] horizontalPoint[i][k];
    delete[] horizontalPoint[i];
  }
  delete[] horizontalPoint;
  delete[] verticalPoint;*/
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
    printf("Hare Krishna\n");
    int unique = 0;
    uint16_t colorIndex;
    //construct horizontal runs
    printf("height : %d", iparams_.height);
    printf("width : %d", iparams_.width);
    for(int j=0; j<iparams_.height; j++){
        uint16_t yi, yf;
        yi = yf = j;
	for(int i=1; i<iparams_.width;){
            //see how far the current run goes
            uint16_t xi = i-1;
            while (getSegPixelValueAt(i, j) == getSegPixelValueAt((i-1), j) && i<iparams_.width){
		i++;
	    }
            //initialize a new visionPoint struct for the current run
            unsigned char runColor = getSegPixelValueAt(xi, j);
	    //printf("coloIndex : %d", colorIndex); //debug
	    //printf("runColor %d", runColor); //debug
	    //printf("NUM_COLORS:%d", NUM_COLORS); //debug
            // only if color is not undefined
            if (runColor){
	    colorIndex = horizontalPointCount[runColor][j];
            VisionPoint *v = &horizontalPoint[runColor][j][colorIndex];
            v->xi = xi;
            v->xf = i-1;
            v->yi = yi;
            v->yf = yf;
            v->dy = 0;
            v->dx = v->xf - v->xi;
            v->lbIndex = unique;
            v->next = NULL;
            v->parent = v; 
            unique++;
            //printf("Struct stats\n"); //DEBUG
            //printf("%u %u", v->xi, v->yi); //DEBUG
            //increment run count for current color and line
            horizontalPointCount[runColor][j]++;
            }
            //increment i so we don't repeat a check
            i++;
	}
    }
}

// construct only horizontal runs
//construct only for given color
void connectComponents(Color rkcolor){
    // do it for each row
    //find the parent
    VisionPoint **rle_map = horizontalPoint[rkcolor];
    uint32_t *count_map = horizontalPointCount[rkcolor];
    uint32_t lim1, lim2, l1, l2;
    l1 = l2 = 0;
    VisionPoint *n, *p;
    //find the max runs for this color
    int max_run = 0;
    for(int i=0; i<iparams_.height; i++) max_run += count_map[i];
    VisionPoint *s = &rle_map[1][0];
    VisionPoint *r2 = &rle_map[0][0];
    VisionPoint *r1 = &rle_map[1][0];
    
    for (int i=0; i<iparams_.height; i++){
    lim2 = count_map[i];
    lim1 = count_map[i+1];
    while(l1 < lim1 && l2 < lim2){
    VisionPoint *r2 = &rle_map[i][l2]; // the previous run
    VisionPoint *r1 = &rle_map[i+1][l1]; // the current run
    if(r1 && r2 && rkcolor){
    //they are not zero - so just check four connectedness
    if((r1->xi >= r2->xi && r1->xi < r2->xf) || (r2->xi >= r1->xi && r2->xi < r1->xf)) {
      if(s != r1){
        s->parent = r1->parent = r2->parent; // moving the one below to the next
        s = r2;
      }
      else{ //moving the one above to the next, need to adjust parents
        n = r1->parent;
        while(n->parent != n) n = n->parent;
        p = r2->parent;
        while(p->parent != p) p = p->parent;
        //find the smaller one 
        if (n->yi < p->yi){
          p->parent = n;
        }
        else if(n->xi < p->xi){
          p->parent = n;
        }
        else{
          n->parent = p;
        }
      }
    }
    }
    else if (r1 && r2){ //if not overlapping
      if (r1->xf < r2->xf) l1++;
      else l2++;
    }
      
    }
    }
    // compress paths
    for(int i=0; i<iparams_.height; i++){
      for(int lim=0; lim<count_map[i]; lim++){
        n = &rle_map[i][lim];
        p = n->parent;
        if (p->yi > n->yi || (p->yi == n->yi && p->xi > n->xi)){
          while(p != p->parent) p = p->parent;
          n->parent = p;
        }
      }
    }
}


