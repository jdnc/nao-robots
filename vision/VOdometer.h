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
    //static int IMG_SIZE;
    VOdometer(DETECTOR_DECLARE_ARGS, Classifier*& classifier);
    void calcOpticalFlow();
    void calcOdometry();
 
  private:
    Classifier*& classifier_;
    cv::Mat prevImage;
    cv::Mat curImage;
    vector<cv::Point2f> corners;
    vector<cv::Point2f> outCorners;
    vector<cv::Mat>trackedImages;
    bool foundFeatures;
    int lastImageIndex;
    void getImage(cv::Mat &);
  

};

#endif
