#include "vision/VOdometer.h"
#define getImageTop() vblocks_.image->getImgTop()
#define getImageBottom() vblocks_.image->getImgBottom()
using namespace cv;
using namespace std;
VOdometer::VOdometer(DETECTOR_DECLARE_ARGS, Classifier*& classifier) : DETECTOR_INITIALIZE, classifier_(classifier) {
  // initialize prevImage to whatever its looking at now
  if (camera_ == Camera::TOP){
    prevImage = getImageTop();
  }
  else{
    prevImage = getImageBottom();
  }
}

void VOdometer::calcOpticalFlow(){
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
 
}

void VOdometer::calcOdometry(){

}
