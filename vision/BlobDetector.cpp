#include <vision/BlobDetector.h>
#include<vision/bconstraints.h>
#include<algorithm>
#include<string>

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier) : 
  DETECTOR_INITIALIZE, classifier_(classifier) {
    horizontalBlob.resize(NUM_COLORS);
    verticalBlob.resize(NUM_COLORS);
}
void BlobDetector::preProcess(uint16_t c){
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

void BlobDetector::formBlobs(uint16_t c){
  VisionPoint *p, *n;
  bool badFlag = false;
  vector<Blob>::iterator it;
  //horizontalBlob[c].erase(horizontalBlob[c].begin(), horizontalBlob[c].end());
  horizontalBlob[c].clear();
  //Blob *b;
  int i, j, lim; //changed from uint32_t to int
  VisionPoint **rle_map = classifier_->horizontalPoint[c];
  //uint32_t *count_map = classifier_->horizontalPointCount[c];
  //for(int i=0; i<iparams_.height; i++) //debug
  // if (classifier_->horizontalPointCount[c_ORANGE][i] > 0) //debug
   //   printf("orange runs %d - %d", i,classifier_->horizontalPointCount[c_ORANGE][i]); //debug
  for(i=0; i<iparams_.height; i++){
    lim = classifier_->horizontalPointCount[c][i];
    for(int j=0; j<lim; j++){
      n = &rle_map[i][j];
      if(n->parent == n){
        // remove all small insignificant pixels
        /*if (n->childCount <=2){
          n->isValid = false;
          continue;
           // don't add this
        }*/
        printf("I am in parent\n");
        //this is a root
        Blob b;
        //printf("MAX=%d",MAX_BLOB_VISIONPOINTS); //debug
	//printf("MAX=%d",MAX_BLOB_VISIONPOINTS); //debug   
        //b->lpCount = 1;
        //uint32_t strange = n->yi << 16 | n->xi;
        //b->lpIndex.push_back(strange);
        b.xi = n->xi;
        b.xf = n->xf;
        b.yi = n->yi;
        b.yf = n->yf;
        b.invalid = false;
        horizontalBlob[c].push_back(b);
	//n->parentBlob = &horizontalBlob[c][horizontalBlob[c].size()-1];
        n->parentBlob = &(horizontalBlob[c].back());
      }
      else{
	// this is a child, find parent
        /*if(!(n->parent->isValid)){ // if not valid parent make this invalid and remove it too
           n->isValid = false;
           badFlag=true;
           continue;
        }*/
	Blob *pBlob = n->parent->parentBlob;
        //n->parentBlob = pBlob; // just in case
	//pBlob->lpIndex[pBlob->lpCount] = n->yi << 16 | n->xi;
	//pBlob->lpCount += 1;
        pBlob->xi = min(pBlob->xi, n->xi);
	pBlob->yi = min(pBlob->yi, n->yi);
	pBlob->xf = max(pBlob->xf, n->xf);
	pBlob->yf = max(pBlob->yf, n->yf);        
      }
    }
    
  }
 // calculate other statistics 
 for(it=horizontalBlob[c].begin(); it!=horizontalBlob[c].end(); it++){
   //calculate  dx, dy
   it->dx = it->xf - it->xi;
   it->dy = it->yf - it->yi;
   it->avgX = (it->xi + it->xf)/2;
   it->avgY = (it->yi + it->yf)/2;
   //calculate the averages - leaving this for now
 }
 // sort blobs using predicate
 std::sort(horizontalBlob[c].begin(), horizontalBlob[c].end(), sortBlobAreaPredicate); 
}

void BlobDetector::mergeBlobs(BlobCollection& blobs, uint16_t x, uint16_t y){
}

void BlobDetector::findBeacons(){
  double low_t = 350;
  double up_t = 17000;
  // find pink blobs first
  BlobCollection& pinks = horizontalBlob[c_PINK];
  BlobCollection& blues = horizontalBlob[c_BLUE];
  BlobCollection& yellows = horizontalBlob[c_YELLOW];
  // already sorted
  // remove extra large and extra small
  BlobCollection::iterator ip, ib, iy;
  for(ip = pinks.begin(); ip != pinks.end();){
      double p_area = ip->dx * ip->dy;
      if (p_area >= up_t || p_area <=low_t) ip = pinks.erase(ip);
      else ++ip;
  }
  for(ip = yellows.begin(); ip != yellows.end();){
      double p_area = ip->dx * ip->dy;
      if (p_area >= up_t || p_area <=low_t) ip = yellows.erase(ip);
      else ++ip;
  }
  for(ip = blues.begin(); ip != blues.end();){
      double p_area = ip->dx * ip->dy;
      if (p_area >= up_t || p_area <=low_t) ip = blues.erase(ip);
      else ++ip;
  }
  WorldObject *bpy = &vblocks_.world_object->objects_[WO_BEACON_PINK_YELLOW];
  WorldObject *bpb = (&vblocks_.world_object->objects_[WO_BEACON_PINK_BLUE]);
  WorldObject *bby = (&vblocks_.world_object->objects_[WO_BEACON_BLUE_YELLOW]);
  WorldObject *byb = (&vblocks_.world_object->objects_[WO_BEACON_YELLOW_BLUE]);
  WorldObject *byp = (&vblocks_.world_object->objects_[WO_BEACON_YELLOW_PINK]);
  WorldObject *bbp = (&vblocks_.world_object->objects_[WO_BEACON_BLUE_PINK]);
  // yellow and pink
  for(ip = pinks.begin(); ip != pinks.end(); ip++){
    for(iy = yellows.begin(); iy != yellows.end(); iy++){
      if(centroidcc(*ip, *iy) && ratiocc(*ip, *iy) && rangecc(*ip, *iy)){
         if(ip->yi < iy->yi) { // pink over yellow
            if (bpy->seen == false){
                printf("yes its true\n");
                bpy->seen = true;
		bpy->width = max(ip->xf-ip->xi, iy->xf-iy->xi);
		bpy->height = ip->yi - iy->yi;
	        bpy->imageCenterX = bpy->width/2;
                bpy->imageCenterY = bpy->height/2;
                printf("BPY centerX: %d, centerY:%d", bpy->imageCenterX, bpy->imageCenterY);
                bpy->fromTopCamera = true;
            }
         }
        else if (iy->yi < ip->yi) { // yellow over pink
            if (byp->seen == false){
                 printf("yes its true\n");
                byp->seen = true;
		byp->width = max(ip->xf-ip->xi, iy->xf-iy->xi);
		byp->height = iy->yi - ip->yi;
	        byp->imageCenterX = bpy->width/2;
                byp->imageCenterY = bpy->height/2;
                byp->fromTopCamera = true;
            }
         }
      }
    }     
  }

  // pink and blue
  for(ip = pinks.begin(); ip != pinks.end(); ip++){
    for(iy = blues.begin(); iy != blues.end(); iy++){
      if(centroidcc(*ip, *iy) && ratiocc(*ip, *iy) && rangecc(*ip, *iy)){
         if(ip->yi < iy->yi) { // pink over blue
            if (bpy->seen == false){
                 printf("yes its true\n");
                bpy->seen = true;
		bpy->width = max(ip->xf-ip->xi, iy->xf-iy->xi);
		bpy->height = ip->yi - iy->yi;
	        bpy->imageCenterX = bpy->width/2;
                bpy->imageCenterY = bpy->height/2;
                bpy->fromTopCamera = true;
            }
         }
        else if (iy->yi < ip->yi) { // blue over pink
            if (byp->seen == false){
                byp->seen = true;
                 printf("yes its true\n");
		byp->width = max(ip->xf-ip->xi, iy->xf-iy->xi);
		byp->height = iy->yi - ip->yi;
	        byp->imageCenterX = bpy->width/2;
                byp->imageCenterY = bpy->height/2;
                byp->fromTopCamera = true;
            }
         }
      }
    }     
  }

  // yellow and blue
  for(ip = blues.begin(); ip != blues.end(); ip++){
    for(iy = yellows.begin(); iy != yellows.end(); iy++){
      if(centroidcc(*ip, *iy) && ratiocc(*ip, *iy) && rangecc(*ip, *iy)){
         if(ip->yi < iy->yi) { // blue over yellow
            if (bpy->seen == false){
                bpy->seen = true;
                 printf("yes its true\n");
		
            }
         }
        else if (iy->yi < ip->yi) { // yellow over blue
            if (byp->seen == false){
                byp->seen = true;
                 printf("yes its true\n");
		byp->width = max(ip->xf-ip->xi, iy->xf-iy->xi);
		byp->height = iy->yi - ip->yi;
	        byp->imageCenterX = bpy->width/2;
                byp->imageCenterY = bpy->height/2;
                byp->fromTopCamera = true;
            }
         }
      }
    }     
  }

 
  
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
