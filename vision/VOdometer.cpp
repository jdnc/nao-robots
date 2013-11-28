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
  foundFeatures = false;
}

void VOdometer::calcOpticalFlow(){
     cout << "hare Krishna"<< "lastImageIndex" << lastImageIndex<<endl;
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
       cout << "Now here" << endl;
       prevGray =  curGray;
       goodFeaturesToTrack(curGray, corners, MAX_POINTS, 0.01, 10);     
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
       
        cout << "prevCorners" << corners.size() << endl;
        cout << "curCorners" << outCorners.size()<< endl;
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
       }
        trackedImages.push_back(curGray);
        trackedFeatures.push_back(outCorners);
        lastImageIndex++;
      }
     
}

void VOdometer::calcOdometry(){

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
