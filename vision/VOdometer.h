#ifndef VOdometer_H
#define VOdometer_H

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <list>
#include<math.h>
#include <vector>

#include <constants/ImageConstants.h>
#include <memory/TextLogger.h>
#include <constants/VisionConstants.h>
#include <vision/Macros.h>
#include <common/Field.h>
#include <vision/BlobDetector.h>
#include<vision/CameraMatrix.h>
#include <math/Point.h>
#include <vision/BlobDetector.h>
#include <vision/ObjectDetector.h>
#include<vision/structures/Position.h>
#include <math/Point.h>
#include<common/Profiling.h>

class VOdometer : public ObjectDetector{
  public:
    int validFeatures; // keep track of number of valid features found
    int lastImageIndexTop; // useful if keeping history of frames
    int lastImageIndexBottom; //similar for bottom camera
    float cumlTurn; // the net angular displacement for all the frames from start to current
    float curTurn;
    float cumDispX; // sum of total x displacements from the start frame
    float cumDispY; // sum of total y displacements from the start frame
    float noRotX;  // similar but with rotation effect removed
    float noRotY;
    VOdometer(DETECTOR_DECLARE_ARGS, Classifier*& classifier);
    void init(TextLogger* tl){textlogger = tl;}; 
    void calcOpticalFlow();
    void findFeaturesToTrack(cv::Mat &, vector<cv::Point2f> &, bool filter=false);
    float getIncAngle();
    bool needReset();
   
  private:
    TextLogger* textlogger;
    Classifier*& classifier_;
    cv::Mat prevImage;
    cv::Mat curImage;
    vector<cv::Point2f> corners;
    vector<cv::Point2f> outCorners;
    vector<cv::Mat>trackedTopImages;
    vector< vector<cv::Point2f> > trackedTopFeatures;
    vector<cv::Mat>trackedBottomImages;
    vector< vector<cv::Point2f> > trackedBottomFeatures;  
    void getImage(cv::Mat &);
    float getMedian(vector<float> &);
    
};

#endif
