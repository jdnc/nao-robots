#include "vision/VOdometer.h"
#define getImageTop() vblocks_.image->getImgTop()
#define getImageBottom() vblocks_.image->getImgBottom()
#define LOOK_BACK 7
#define MAX_POINTS 400
#define IMG_SIZE 640 * 480
using namespace cv;
using namespace std;


VOdometer::VOdometer(DETECTOR_DECLARE_ARGS, Classifier*& classifier) : DETECTOR_INITIALIZE, classifier_(classifier) {
  //TODO make it use the expr below
  //IMG_SIZE = iparams_.height * iparams_.width;
  // initialize prevImage to whatever its looking at now
  lastImageIndex = 0;
  cumlTurn = 0;
  foundFeatures = false;
}

void VOdometer::calcOpticalFlow(){
     cout << "hare Krishna"<< "lastImageIndex" << lastImageIndex<<endl;
     //cout << "fovx" << FOVx<<endl;
     vector<uchar> status;
     vector<float> err;
     TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.3);
     Size winSize = Size(3,3);
     // the prevImage is the image last pushed into the vector
     // the current image is the image being seen in the current frame
     // keep track of number of images seen so far
     // resize vector when required.
     Mat prevGray, curGray;
     getImage(curImage);        
     //convert both images from BGR to grayscale
     //cvtColor(prevImage, prevGray, CV_BGR2GRAY);
     cvtColor(curImage, curGray, CV_BGR2GRAY);     
     //find which would be good points to track (uses shi and tomasi)
     //declare the vector of 2D float points 
     //vector<Point2f> corners;
     //TODO finetune on 0.01 and 10
     if(lastImageIndex == LOOK_BACK){
       trackedFeatures.clear();
       trackedImages.clear();
       lastImageIndex = 0;
     }
     if(lastImageIndex == 0){
       //cout << "Now here" << endl;
       prevGray =  curGray;
       goodFeaturesToTrack(curGray, corners, MAX_POINTS, 0.005, 20);     
       //get for subpixel level for more accuracy
       //Set the window size centered around the found pixel       
       //Set the termination criteria TODO finetune
       //TODO check if it really improves accuracy
       cornerSubPix(curGray, corners, winSize, Size(-1,-1), termcrit);
       lastImageIndex++;
       trackedFeatures.push_back(corners);
       trackedImages.push_back(curGray);
       }
       if(lastImageIndex >= 1){
       corners = trackedFeatures.back();
       prevGray = trackedImages.back();
       //finally find the optical flow uses Lucas Kanade
       //output vector of 2D float points for current image
       //vector<Point2f> outCorners; should be returned
       //unsigned char status array
       //error corresponding to each vector TODO finetune     
       calcOpticalFlowPyrLK(prevGray, curGray, corners, outCorners,
                            status, err, winSize, 4, termcrit);
       
        //cout << "prevCorners" << corners.size() << endl;
        //cout << "curCorners" << outCorners.size()<< endl;
       //only retain points that were found corresponding to the previous image
       //otherwise set them to -1
       int in = 0, out = 0;
       for(int i=0; i<outCorners.size(); i++){
       //cout << "out"<< out<<"in"<<in<<endl;
       out++;
       if (status[i] == 0){
        in++;
        corners[i] = outCorners[i] = Point2f(-1.0f, -1.0f);
       }
       else{
       visionLog((8,"start %f %f end %f %f", corners[i].x,corners[i].y,outCorners[i].x, outCorners[i].y));
       }
       }
        trackedImages.push_back(curGray);
        trackedFeatures.push_back(outCorners);
        lastImageIndex++;
      }
     visionLog((7, "angle is %f deg and total angle is %f deg lastImageindex %d",getIncAngle()*180/3.14159, cumlTurn, lastImageIndex -1 ));
     cout <<"angle"<<getIncAngle()*180/3.14159<<"deg"<<endl;
     cumlTurn += getIncAngle()*180/3.14159;
     cout << "total angle change" << cumlTurn << endl;
}

void VOdometer::calcOdometry(){

}

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
      float angle  =  dx/iparams_.width * FOVx;
      angles.push_back(angle);
    }
    if (angles.size() > 0) {
      sort( angles.begin(), angles.end() );
      float median_turn = angles[(int)(angles.size()/2)];
      return median_turn;
    }
  }
  else
   return 0.0f;
}
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
