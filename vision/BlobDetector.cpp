#include <vision/BlobDetector.h>
#include<vision/bconstraints.h>
#include<algorithm>
#include<string>

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier) : 
  DETECTOR_INITIALIZE, classifier_(classifier) {
    horizontalBlob.resize(NUM_COLORS);
    verticalBlob.resize(NUM_COLORS);
}
void BlobDetector::formBlobs(uint16_t c){
   // remove spurious and small blobs
   vector<VisionPoint*> parentList;
   parentList.clear();
   horizontalBlob[c].clear();
   VisionPoint *p, *n;
   int i, j, lim; //changed from uint32_t to int
   VisionPoint **rle_map = classifier_->horizontalPoint[c];
   for(i=0; i<iparams_.height; i++){
    lim = classifier_->horizontalPointCount[c][i];
    for(int j=0; j<lim; j++){
      n = &rle_map[i][j];
      if(n->parent == n){
        //this is a root
        parentList.push_back(n);
      }
      else{	
	// this is child, update appropriate statistics in parent.
        p = n->parent;
        p->xi = min(p->xi, n->xi);
	p->yi = min(p->yi, n->yi);
	p->xf = max(p->xf, n->xf);
	p->yf = max(p->yf, n->yf); 
        p->avgX += Classifier::range_sum(n->xi, n->dx);
        p->avgY += Classifier::range_sum(n->yi, n->dy);
        p->pixelCount += n->pixelCount;       
      }
    } 
  }
  // remove all small regions
  vector<VisionPoint *> :: iterator it;
  for(it=parentList.begin(); it!=parentList.end();){
    int dx = (*it)->xf - (*it)->xi;
    int dy = (*it)->yf - (*it)->yi;
    if ((*it)->pixelCount < 15 ||  dx < 4 || dy < 4) 
      it = parentList.erase(it);
    else 
      it++; 
  }
  // fill up horizontal blobs
  for(it=parentList.begin(); it!=parentList.end(); it++){
      Blob b;
      n = *it;
      b.xi = n->xi;
      b.xf = n->xf;
      b.yi = n->yi;
      b.yf = n->yf;
      b.invalid = false;
      b.dx = n->xf - n->xi;
      b.dy = n->yf - n->yi;
      b.avgX = n->avgX / n->pixelCount;
      b.avgY = n->avgY / n->pixelCount;
      horizontalBlob[c].push_back(b);
  }  
  //sorted in horizontalBlob
  std::sort(horizontalBlob[c].begin(), horizontalBlob[c].end(), sortBlobAreaPredicate); 
}

void BlobDetector::mergeBlobs(BlobCollection& blobs, uint16_t x, uint16_t y){
}

void BlobDetector::findBeacons2(){
  BlobCollection probPinks;
  BlobCollection probBlues;
  BlobCollection probYellows;
  WorldObject *wo;
  vector<ProbBeacon> ProbBeacons;
  vector<ProbBeacon>::iterator it;
  std::remove_copy_if(horizontalBlob[c_PINK].begin(), horizontalBlob[c_PINK].end(),std::back_inserter(probPinks), areaOutOfRangePredicate);
  std::remove_copy_if(horizontalBlob[c_BLUE].begin(), horizontalBlob[c_BLUE].end(),std::back_inserter(probBlues), areaOutOfRangePredicate);
  std::remove_copy_if(horizontalBlob[c_YELLOW].begin(), horizontalBlob[c_YELLOW].end(),std::back_inserter(probYellows), areaOutOfRangePredicate);
  findProbBeacons(probPinks, probBlues, c_PINK, c_BLUE, ProbBeacons);
  findProbBeacons(probPinks, probYellows, c_PINK, c_YELLOW, ProbBeacons);
  findProbBeacons(probYellows, probBlues, c_YELLOW, c_BLUE, ProbBeacons);
  // draw 
  for(it=ProbBeacons.begin(); it!=ProbBeacons.end(); it++){
    int centerX, centerY;
    centerX = it->top->xi + it->top->dx/2;
    centerY = it->top->yi + it->top->dy/2 +it->bottom->dy/2;
    //("Top color = %s Bott Color=%s centerX=%d centerY=%d", COLOR_NAME(it->topColor), COLOR_NAME(it->botColor), avgX, avgY); //debug
    wo = getBeaconFromColors(it->topColor, it->botColor);
    // set parameters
    wo->seen = true;
    wo->width = max(it->top->dx, it->bottom->dx);
    wo->height = it->top->dy + it->bottom->dy;
    wo->imageCenterX = centerX;
    wo->imageCenterY = centerY;
    wo->fromTopCamera = true;
  }
}

void BlobDetector::findProbBeacons(BlobCollection &c1Blobs, BlobCollection &c2Blobs, Color c1, Color c2, vector<ProbBeacon>& ProbBeacons){
BlobCollection::iterator b1, b2;
ProbBeacon pb;
for(b1=c1Blobs.begin(); b1!=c1Blobs.end(); b1++){
    for(b2=c2Blobs.begin(); b2!=c2Blobs.end(); b2++){
      if(centroidcc(*b1, *b2)) {
	if (b1->yi < b2->yi){
	   pb.top = &(*b1);
	   pb.bottom = &(*b2);
	   pb.topColor = c1;
	   pb.botColor = c2;
	}
	else{
	   pb.top = &(*b2);
	   pb.bottom = &(*b1);
	   pb.topColor = c2;
	   pb.botColor = c1;
       }
       pb.likely = max(b1->dx, b2->dx)/ (b1->dy + b2->dy); //aspect ratio
       ProbBeacons.push_back(pb);
       }
     }  
   }
return;
}
 
WorldObject * BlobDetector::getBeaconFromColors(Color top, Color bottom){
  if(top==c_PINK && bottom==c_BLUE)
     return &vblocks_.world_object->objects_[WO_BEACON_PINK_BLUE];

  if(top==c_PINK && bottom==c_YELLOW)
     return &vblocks_.world_object->objects_[WO_BEACON_PINK_YELLOW];

  if(top==c_BLUE && bottom==c_PINK)
    return &vblocks_.world_object->objects_[WO_BEACON_BLUE_PINK];

  if(top==c_BLUE && bottom==c_YELLOW)
    return &vblocks_.world_object->objects_[WO_BEACON_BLUE_YELLOW];

  if(top==c_YELLOW && bottom==c_BLUE)
    return &vblocks_.world_object->objects_[WO_BEACON_YELLOW_BLUE];
   
  if(top==c_YELLOW && bottom==c_PINK)
    return &vblocks_.world_object->objects_[WO_BEACON_YELLOW_PINK];
  
  return NULL;
}
