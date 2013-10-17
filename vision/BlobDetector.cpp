#include <vision/BlobDetector.h>

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier) : 
  DETECTOR_INITIALIZE, classifier_(classifier) {
    horizontalBlob.resize(NUM_COLORS);
    verticalBlob.resize(NUM_COLORS);
}

void BlobDetector::formBlobs(Color c){
  classifier_->connectComponents(c);
  VisionPoint *p, *n;
  uint32_t i, j, lim;
  VisionPoint **rle_map = horizontalPoint[c];
  uint32_t *count_map = count_map[c];
  for(i=0; i<=iparams_.height; i++){
    lim = count_map[i];
    for(int j=0; j<lim; j++){
      n = &rle_map[i][j];
      if(n->parent == n){
        //this is a root
        Blob b = new Blob;
        b.lpCount = 1;
        b.lpIndex[0] = n->yi << 16 | n->xi;
        b.xi = n->xi;
        b.xf = n->xf;
        b.yi = n->yi;
        b.yf = n->yf;
        b.invalid = false;
        horizontalBlob[c].push_back(b);
      }
      else{
        
      }
    }
  }
}
