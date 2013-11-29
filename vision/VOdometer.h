#ifndef VOdometer_H
#define VOdometer_H

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math/Point.h>
#include <list>
#include <vector>

#include <constants/ImageConstants.h>
#include <memory/TextLogger.h>
#include <constants/VisionConstants.h>
#include <vision/Macros.h>
#include <common/Field.h>
#include <vision/BlobDetector.h>
#include <math/Point.h>
#include <vision/BlobDetector.h>
#include <vision/ObjectDetector.h>
#include <math/Point.h>

class VOdometer : public ObjectDetector{
  public:
    void init(TextLogger* tl){textlogger = tl;};
    //static int IMG_SIZE;
    int lastImageIndex;
    VOdometer(DETECTOR_DECLARE_ARGS, Classifier*& classifier);
    void calcOpticalFlow();
    void calcOdometry();
    float getIncAngle();
    float cumlTurn;
 
  private:
    TextLogger* textlogger;
    Classifier*& classifier_;
    cv::Mat prevImage;
    cv::Mat curImage;
    vector<cv::Point2f> corners;
    vector<cv::Point2f> outCorners;
    vector<cv::Mat>trackedImages;
    vector< vector<cv::Point2f> > trackedFeatures;
    bool foundFeatures;   
    void getImage(cv::Mat &);
    
};

#endif
