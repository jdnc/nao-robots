#include <vision/BlobDetector.h>
#include<vision/bconstraints.h>
#include<algorithm>

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier) : 
  DETECTOR_INITIALIZE, classifier_(classifier) {
    horizontalBlob.resize(NUM_COLORS);
    verticalBlob.resize(NUM_COLORS);
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
  double low_t = 20;
  double up_t = 6000;
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
                bpy->seen = true;
		bpy->width = max(ip->xf-ip->xi, iy->xf-iy->xi);
		bpy->height = ip->yi - iy->yi;
	        bpy->imageCenterX = bpy->width/2;
                bpy->imageCenterY = bpy->height/2;
                bpy->fromTopCamera = true;
            }
         }
        else if (iy->yi < ip->yi) { // yellow over pink
            if (byp->seen == false){
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
		bpy->width = max(ip->xf-ip->xi, iy->xf-iy->xi);
		bpy->height = ip->yi - iy->yi;
	        bpy->imageCenterX = bpy->width/2;
                bpy->imageCenterY = bpy->height/2;
                bpy->fromTopCamera = true;
            }
         }
        else if (iy->yi < ip->yi) { // yellow over blue
            if (byp->seen == false){
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

 // check if beacons are 
  
 }
void BlobDetector::searchMatch(WorldObject *b, Color top, Color bot){
  
}

