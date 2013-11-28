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
  vector<Mat> trackedImages;
  //just initialize the vector and previus image with whatever its seeing right now
  // this is in the opencv accepted Mat form
  //getImage(prevImage);
  //Always remember to convert to grayscale when pushing out of the vector
  //trackedImages.push_back(prevImage);
  lastImageIndex = 1;
  foundFeatures = false;
}

void VOdometer::calcOpticalFlow(){
     // the prevImage is the image last pushed into the vector
     // the current image is the image being seen in the current frame
     // keep track of number of images seen so far
     // resize vector when required.
     Mat prevGray, curGray;

     //prevImage = trackedImages.back();
     getImage(prevImage);
     getImage(curImage);
     
     //convert both images from BGR to grayscale
     cvtColor(prevImage, prevGray, CV_BGR2GRAY);
     cvtColor(curImage, curGray, CV_BGR2GRAY);
     
     //find which would be good points to track (uses shi and tomasi)
     //declare the vector of 2D float points 
     //vector<Point2f> corners;
     //TODO finetune on 0.01 and 10
     goodFeaturesToTrack(prevGray, corners, MAX_POINTS, 0.01, 10);
     
     //get for subpixel level for more accuracy
     //Set the window size centered around the found pixel
     Size winSize = Size(3,3);
     //Set the termination criteria TODO finetune
     TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.3);
     //TODO check if it really improves accuracy
     cornerSubPix(prevGray, corners, winSize, Size(-1,-1), termcrit);
     //finally find the optical flow uses Lucas Kanade
     //output vector of 2D float points for current image
     //vector<Point2f> outCorners; should be returned
     //unsigned char status array
     vector<uchar> status;
     //error corresponding to each vector TODO finetune
     vector<float> err;
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

     trackedImages.push_back(curImage);
     lastImageIndex++;
     //erase all images if more than lookback images are stored
     if(lastImageIndex > LOOK_BACK){
       trackedImages.erase(trackedImages.begin(), trackedImages.begin() + LOOK_BACK);     
     }
     
     return;
     /* OLD CODE
     TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
     Size subPixWinSize(10,10), winSize(31,31);
     vector<uchar> status;
     vector<float> err;
     vector<Point2f> points[2];
     // get the current and previous images
     Mat cvPrevImage = color::rawToMat(prevImage, iparams_);
     Mat cvCurImage = color::rawToMat(curImage, iparams_);
     // convert them to grayscale
     // Mat cvPrevGray = cvPrevImage.clone();
     Mat cvCurGray, cvPrevGray; 
     cvtColor(cvPrevImage, cvPrevGray, CV_RGB2GRAY);
     cvtColor(cvCurImage, cvCurGray, CV_RGB2GRAY);
     goodFeaturesToTrack(cvCurGray, points[1], 500, 0.01, 10, Mat(), 3, 0, 0.04);
     cornerSubPix(cvCurGray, points[1], subPixWinSize, Size(-1,-1), termcrit);
     Mat flow;
     //cv::calcOpticalFlowSF(cvPrevImage, cvCurImage, flow, 3, 2, 4);
     //cv::calcOpticalFlowSF(cvPrevImage, cvCurImage, newmat, 3, 2, 4);
     calcOpticalFlowPyrLK(cvPrevGray, cvCurGray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
     std::swap(points[1], points[0]);
     cv::swap(cvPrevGray, cvCurGray);
     */
 
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
