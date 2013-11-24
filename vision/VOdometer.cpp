#include "vision/VOdometer.h"
#define getImageTop() vblocks_.image->getImgTop()
#define getImageBottom() vblocks_.image->getImgBottom()
#define MAX_CORNERS 400
using namespace cv;
using namespace std;
VOdometer::VOdometer(DETECTOR_DECLARE_ARGS, Classifier*& classifier) : DETECTOR_INITIALIZE, classifier_(classifier) {
  // initialize prevImage to whatever its looking at now
   
  prevImage = new unsigned char[2 * iparams_.height * iparams_.width];
  curImage =  new unsigned char[2 * iparams_.height * iparams_.width];
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
     vector<Point2f> prevCorners;
     vector<Point2f> curCorners;
     //CvPoint2D32f* prevCorners = new CvPoint2D32f[ MAX_CORNERS ];
     //CvPoint2D32f* curCorners = new CvPoint2D32f[ MAX_CORNERS ];
     // get the current and previous images
     Mat cvPrevImage = color::rawToMat(prevImage, iparams_);
     Mat cvCurImage = color::rawToMat(curImage, iparams_);
     // convert them to grayscale
     // Mat cvPrevGray = cvPrevImage.clone();
     Mat cvCurGray, cvPrevGray; 
     //prevCorners.convertTo(prevCorners, CV_32F);
     //curCorners.convertTo(curCorners, CV_32F);
     cvtColor(cvPrevImage, cvPrevGray, CV_BGR2GRAY);
     cvtColor(cvCurImage, cvCurGray, CV_BGR2GRAY);
     goodFeaturesToTrack(cvCurGray, prevCorners, MAX_CORNERS, 0.01, 10, Mat(), 3, 0, 0.04);
     //can't get this to work
     cornerSubPix(cvCurGray, prevCorners, subPixWinSize, Size(-1,-1), termcrit);
     Mat flow;
     //cv::calcOpticalFlowSF(cvPrevImage, cvCurImage, flow, 3, 2, 4);
     //cv::calcOpticalFlowSF(cvPrevImage, cvCurImage, newmat, 3, 2, 4);
     //calcOpticalFlowPyrLK(cvPrevGray, cvCurGray, prevCorners, curCorners, status, err, winSize, 3, termcrit, 0, 0.001);
     std::swap(curCorners, prevCorners);
     //namedWindow("Hare Krishna");
     //imshow("Hare Krishna", cvCurGray);
     cv::swap(cvPrevGray, cvCurGray);
 
}

void VOdometer::calcOdometry(){

}
