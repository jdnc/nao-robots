#include <vision/BlobDetector.h>

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier) : 
  DETECTOR_INITIALIZE, classifier_(classifier) {
    horizontalBlob.resize(NUM_COLORS);
    verticalBlob.resize(NUM_COLORS);
}
void BlobDetector::formBlobs(uint16_t c){
  classifier_->connectComponents(c);
  VisionPoint *p, *n;
  vector<Blob>::iterator it;
  uint32_t i, j, lim;
  VisionPoint **rle_map = classifier_->horizontalPoint[c];
  uint32_t *count_map = classifier_->horizontalPointCount[c];
  for(int i=0; i<iparams_.height; i++) //debug
  printf("orange runs %d - %d", i,classifier_->horizontalPointCount[c_ORANGE][i]); //debug
  for(i=0; i<=iparams_.height; i++){
    lim = count_map[i];
    for(int j=0; j<lim; j++){
      n = &rle_map[i][j];
      if(n->parent == n){
        //this is a root
        Blob b;
        b.lpCount = 1;
        b.lpIndex[0] = n->yi << 16 | n->xi;
        b.xi = n->xi;
        b.xf = n->xf;
        b.yi = n->yi;
        b.yf = n->yf;
        b.invalid = false;
        horizontalBlob[c].push_back(b);
	n->parentBlob = &b;
      }
      else{
	// this is a child, find parent
	Blob *pBlob = n->parent->parentBlob;
        n->parentBlob = pBlob; // just in case
	pBlob->lpIndex[pBlob->lpCount] = n->yi << 16 | n->xi;
	pBlob->lpCount += 1;
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
   //calculate the averages - leaving this for now
 }
 // sort blobs using predicate
 std::sort(horizontalBlob[c].begin(), horizontalBlob[c].end(), sortBlobAreaPredicate); 
}
