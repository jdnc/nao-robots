#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <kinematics/ForwardKinematics.h>
#include <common/RobotDimensions.h>
#include <memory/TextLogger.h>
#include <vision/CameraMatrix.h>
#include <vision/VisionBlocks.h>
#include <common/RobotInfo.h>
#include <vision/LineDetector.h>
#include <vision/GoalDetector.h>
#include <vision/BallDetector.h>
#include <vision/BandRobotDetector.h>
#include <vision/JerseyRobotDetector.h>
#include <vision/CrossDetector.h>
#include <vision/Classifier.h>
#include <vision/RobotCalibration.h>
#include <vision/structures/BallCandidate.h>
#include <math/Pose3D.h>

class ImageProcessor {
  public:
    typedef JerseyRobotDetector RobotDetector;
    /*typedef BandRobotDetector RobotDetector;*/
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera);
    void processFrame();
    void init(TextLogger*);
    void SetColorTable(unsigned char*);
    LineDetector* line_detector_;
    GoalDetector* goal_detector_;
    BallDetector* ball_detector_;
    BlobDetector* blob_detector_;
    RobotDetector* robot_detector_;
    CrossDetector* cross_detector_;
    Classifier* classifier_;
    unsigned char* getImg();
    unsigned char* getSegImg();
    unsigned char* getColorTable();
    bool isRawImageLoaded();
    int getImageHeight();
    int getImageWidth();
    const ImageParams& getImageParams() const { return iparams_; }
    const CameraMatrix& getCameraMatrix();
    void setCalibration(RobotCalibration);
    void enableCalibration(bool value);
    void updateTransform();
    std::vector<BallCandidate*> getBallCandidates();
    BallCandidate* getBestBallCandidate();
    bool isImageLoaded();
  private:
    int getTeamColor();
    double getCurrentTime();

    VisionBlocks& vblocks_;
    const ImageParams& iparams_;
    Camera::Type camera_;
    CameraMatrix cmatrix_;
    
    VisionParams vparams_;
    unsigned char* color_table_;
    TextLogger* textlogger;

    float getHeadPan() const;
    float getHeadTilt() const;
    float getHeadChange() const;

    RobotCalibration* calibration_;
    bool enableCalibration_;
};

#endif
