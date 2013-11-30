#include "vision/VOdometer.h"
#define getImageTop() vblocks_.image->getImgTop()
#define getImageBottom() vblocks_.image->getImgBottom()
#define LOOK_BACK 7
#define MAX_POINTS 400
#define IMG_SIZE 640 * 480
#define THRESHOLD 
using namespace cv;
using namespace std;


VOdometer::VOdometer(DETECTOR_DECLARE_ARGS, Classifier*& classifier) : DETECTOR_INITIALIZE, classifier_(classifier) {
  //TODO make it use the expr below
  //IMG_SIZE = iparams_.height * iparams_.width;
  // initialize prevImage to whatever its looking at now
  lastImageIndex = 0;
  cumlTurn = 0;
  validFeatures = 0;
}

void VOdometer::calcOpticalFlow(){
     cout << "hare Krishna"<< "lastImageIndex" << lastImageIndex<<endl;
     vector<uchar> status;
     vector<float> angles;
     vector<float> err;
     float median_turn;
     TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.3);
     Size winSize = Size(3,3);
     Mat prevGray, curGray;
     getImage(curImage);        
     cvtColor(curImage, curGray, CV_BGR2GRAY);     
     if(lastImageIndex == LOOK_BACK){
       trackedFeatures.clear();
       trackedImages.clear();
       lastImageIndex = 0;
     }
     if(lastImageIndex == 0){
       // just do initialization -  no need for actual processing
       //cout << "Now here" << endl;
       prevGray =  curGray;
       findFeaturesToTrack(prevGray, corners);
       lastImageIndex++;
       trackedFeatures.push_back(corners);
       trackedImages.push_back(curGray);
       return;
       }
       if(lastImageIndex >= 1){
         corners = trackedFeatures.back();
         prevGray = trackedImages.back();
         calcOpticalFlowPyrLK(prevGray, curGray, corners, outCorners,
                            status, err, winSize, 4, termcrit);
         int in = 0, out = 0;
         validFeatures = 0;
         for(int i=0; i<outCorners.size(); i++){
         //cout << "out"<< out<<"in"<<in<<endl;
           out++;
           if (status[i] == 0){
            in++;
            corners[i] = outCorners[i] = Point2f(-1.0f, -1.0f);
           }
           else{
           validFeatures++;
           visionLog((8,"Index is %d start (%f , %f)end (%f, %f)",i, corners[i].x,corners[i].y,outCorners[i].x, outCorners[i].y));
           float ax = outCorners[i].x;
           float ay = outCorners[i].y;
           float bx = corners[i].x;
           float by = corners[i].y;
           //some confusion in signs
           float dx =  ax - bx;
           float dy = ay - by;
           visionLog((8,"dx %0.2f dy %0.2f ",dx, dy));
           float angle  =  abs(dx/iparams_.width * FOVx);
           angles.push_back(angle);
           }
        } 
        if (angles.size() > 0)  {

              sort( angles.begin(), angles.end());
              for(int i = 0; i< angles.size(); i++){
                visionLog((9, "index %d angle is %f deg\n",i, angles[i] * RAD_T_DEG ));
              }
              median_turn = angles[(int)(angles.size()/2)];   
              }
        trackedImages.push_back(curGray);
        trackedFeatures.push_back(outCorners);
        lastImageIndex++;
        visionLog((7, "angle is %f deg and total angle is %f deg lastImageindex %d",median_turn*180/3.14159, cumlTurn, lastImageIndex -1 ));
	cout <<"angle"<<median_turn*180/3.14159<<"deg"<<endl;
	cumlTurn += median_turn*180/3.14159;
	cout << "total angle change" << cumlTurn << endl;
       } 
     
}


/* find the features to track */
void VOdometer::findFeaturesToTrack(Mat &grayImage, vector<Point2f>& foundFeatures, bool filter){
  TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.3);
  Size winSize = Size(3,3);
  goodFeaturesToTrack(grayImage, foundFeatures, MAX_POINTS, 0.005, 20);
  cornerSubPix(grayImage, foundFeatures, winSize, Size(-1,-1), termcrit);
  return;
}

/* determine if need o reset the found points*/
bool VOdometer::needReset(){
  if (validFeatures < 50) {
    return true;
  }
  return false;
}

/* TODO Change this very bad results */ 
float VOdometer::getIncAngle(){
  float ax, ay, bx, by, dx, dy;
  vector<float> angles;
  if (trackedFeatures.size() > 2){
    outCorners = trackedFeatures[trackedFeatures.size() - 1];
    corners = trackedFeatures[trackedFeatures.size() - 2]; 
    for(int i = 0; i < outCorners.size(); i++){
      if(corners[i] == Point2f(-1.0f, -1.0f) || outCorners[i] == Point2f(-1.0f, -1.0f)){
        continue;
      }
      ax = outCorners[i].x;
      ay = outCorners[i].y;
      bx = corners[i].x;
      by = corners[i].y;
      //some confusion in signs
      dx =  ax - bx;
      dy = ay - by;
      //visionLog((8,"dx %0.2f dy%0.2f ",i, dx, dy));
      float angle  =  dx/640 * FOVx;
      angles.push_back(angle);
    }
    if (angles.size() > 0) {
      for(int i = 0; i< angles.size(); i++){
        //visionLog((9, "index %d angle is %f deg\n",i, angles[i] * RAD_T_DEG ));
      }
      sort( angles.begin(), angles.end() );
      float median_turn = angles[(int)(angles.size()/2)];
      return median_turn;
    }
    // print out all the angels found DEBUGGING only
    
  }
  else
   return 0.0f;
}

/* fetches the images -  no changes in this required */
void VOdometer::getImage(Mat& image){
  if (vblocks_.image->loaded_)
  {
    unsigned char * rawImage;
    rawImage = new unsigned char[iparams_.rawSize];
    if(camera_ == Camera::TOP){
    memcpy(rawImage, vblocks_.image->getImgTop(), iparams_.rawSize);
    }
    else{
    memcpy(rawImage, vblocks_.image->getImgBottom(), iparams_.rawSize);
    }
    image = color::rawToMat(rawImage, iparams_);
    delete [] rawImage;
  }
  return;
}
