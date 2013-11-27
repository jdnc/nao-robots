#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

// The maximum points that should be tracked from frame to frame TODO finetune
#define MAX_POINTS 400

using namespace std;
using namespace cv;

void getFlowVectors(Mat& prevImage, Mat& curImage, vector<Point2f> &corners, vector<Point2f> &outCorners);
void drawFlowImage(vector<Point2f> startPoints, vector<Point2f> endPoints, Mat& prevGray, Mat& curGray);


int main(int argc, char *argv[]){
  Mat prevImage = imread("nao-logs/walks/walk1/00top.bmp");
  Mat curImage = imread("nao-logs/walks/walk1/02top.bmp");
  vector<Point2f> startPoints;
  vector<Point2f> endPoints;
  getFlowVectors(prevImage, curImage, startPoints, endPoints);
  waitKey(0);
  return 0;
}

void getFlowVectors(Mat& prevImage, Mat& curImage, vector<Point2f> &corners, vector<Point2f> &outCorners){
  // prevImage - 8 bit 3 channel BGR previous image
  // curImage - 8 bit 3 channel BGR current image
  // flowVectors - outputs the flowVectors as a mat
  Mat prevGray, curGray;
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
  //just for debugging draw the flow vectors and show
  drawFlowImage(corners, outCorners, prevGray, curGray);  
  return; 
  
}
  
void drawFlowImage(vector<Point2f> startPoints, vector<Point2f> endPoints, Mat& prevGray, Mat& curGray){
  // for now just draw the points and see
  namedWindow("Krishna1", CV_WINDOW_NORMAL);
  namedWindow("Krishna2", CV_WINDOW_NORMAL);
  for(int i = 0; i<endPoints.size(); i++){
    if(startPoints[i] == Point2f(-1.0f, -1.0f) || endPoints[i] == Point2f(-1.0f, -1.0f)) continue;
    circle(prevGray, startPoints[i], 3, Scalar(0,255,0), -1, 8);
    circle(curGray, endPoints[i], 3, Scalar(0,255,0), -1, 8);
    cout<<"start"<<startPoints[i]<<"end"<<endPoints[i]<<endl;
  }  
  imshow("Krishna1", prevGray);
  imshow("Krishna2", curGray);
  
}

//TODO Make this private?
void getCoordinates(float& x, float& y, vector<Point2f> startPoints, vector<Point2f> endPoints){


}

//TODO make this private?
void getTheta(float &theta, vector<Point2f> startPoints, vector<Point2f> endPoints){

}


