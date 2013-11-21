#include "ImageProcessor.h"
#include <iostream>

ImageProcessor::ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera) :
  vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(iparams_, camera), calibration_(NULL)
{
  enableCalibration_ = false;
  classifier_ = new Classifier(vblocks_, vparams_, iparams_, camera_);
  classifier_->didHighResBallScan = false;
  blob_detector_ = new BlobDetector(DETECTOR_PASS_ARGS, classifier_);
  line_detector_ = new LineDetector(DETECTOR_PASS_ARGS, classifier_, blob_detector_);
  goal_detector_ = new GoalDetector(DETECTOR_PASS_ARGS, classifier_, blob_detector_, line_detector_);
  ball_detector_ = new BallDetector(DETECTOR_PASS_ARGS, classifier_, blob_detector_);
  robot_detector_ = new RobotDetector(DETECTOR_PASS_ARGS, classifier_, blob_detector_);
  cross_detector_ = new CrossDetector(DETECTOR_PASS_ARGS, classifier_, blob_detector_);
}

void ImageProcessor::init(TextLogger* tl){
  textlogger = tl;
  vparams_.init();
  classifier_->init(tl);
  blob_detector_->init(tl);
  line_detector_->init(tl);
  goal_detector_->init(tl);
  ball_detector_->init(tl);
  robot_detector_->init(tl);
  cross_detector_->init(tl);
}

unsigned char* ImageProcessor::getImg() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->getImgTop();
  return vblocks_.image->getImgBottom();
}

unsigned char* ImageProcessor::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

unsigned char* ImageProcessor::getColorTable(){
  return color_table_;
}

const CameraMatrix& ImageProcessor::getCameraMatrix(){
  return cmatrix_;
}

void ImageProcessor::updateTransform(){
  BodyPart::Part camera;
  if(camera_ == Camera::TOP)
    camera = BodyPart::top_camera;
  else
    camera = BodyPart::bottom_camera;
  if(enableCalibration_) {
    float joints[NUM_JOINTS], sensors[NUM_SENSORS], dimensions[RobotDimensions::NUM_DIMENSIONS];
    memcpy(joints, vblocks_.joint->values_, NUM_JOINTS * sizeof(float));
    memcpy(sensors, vblocks_.sensor->values_, NUM_SENSORS * sizeof(float));
    memcpy(dimensions, vblocks_.robot_info->dimensions_.values_, RobotDimensions::NUM_DIMENSIONS * sizeof(float));
    Pose3D rel_parts[BodyPart::NUM_PARTS], abs_parts[BodyPart::NUM_PARTS];
    calibration_->applyJoints(joints);
    calibration_->applySensors(sensors);
    calibration_->applyDimensions(dimensions);
    ForwardKinematics::calculateRelativePose(joints, rel_parts, dimensions);
#ifdef TOOL
    Pose3D base = ForwardKinematics::calculateVirtualBase(calibration_->useLeft, rel_parts);
    ForwardKinematics::calculateAbsolutePose(base, rel_parts, abs_parts);
#else
    ForwardKinematics::calculateAbsolutePose(sensors, rel_parts, abs_parts);
#endif
    cmatrix_.setCalibration(*calibration_);
    cmatrix_.updateCameraPose(abs_parts[camera]);
  }
  else {
    cmatrix_.updateCameraPose(vblocks_.body_model->abs_parts_[camera]);
  }
}

bool ImageProcessor::isRawImageLoaded() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->img_top_;
  return vblocks_.image->img_bottom_;
}

int ImageProcessor::getImageHeight() {
  return iparams_.height;
}

int ImageProcessor::getImageWidth() {
  return iparams_.width;
}

double ImageProcessor::getCurrentTime() {
  return vblocks_.frame_info->seconds_since_start;
}

void ImageProcessor::setCalibration(RobotCalibration calibration){
  if(calibration_) delete calibration_;
  calibration_ = new RobotCalibration(calibration);
}

void ImageProcessor::processFrame(){
  //trial
  classifier_->opticalFlow();
  visionLog((30, "Process Frame camera %i", camera_));

  visionLog((30, "Calculating horizon line"));
  updateTransform();
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 30000);
  vblocks_.robot_vision->horizon = horizon;

  visionLog((30, "Classifying Image", camera_));
  classifier_->setHorizon(horizon);
  if(!classifier_->classifyImage(color_table_)) return;
  classifier_->constructRuns();

  visionLog((30, "Preprocessing line points and forming blobs"));
  classifier_->preProcessPoints();
  blob_detector_->formWhiteLineBlobs();

  visionLog((30, "Detecting balls"));
  ball_detector_->setHorizon(horizon);
  ball_detector_->findBall();

  visionLog((30, "Detecting White Lines"));
  line_detector_->setHorizon(horizon);
  line_detector_->FormLines(c_WHITE);
  line_detector_->mergeLines();
  line_detector_->generateTransformedImage();
  line_detector_->differentiateCurves();

  line_detector_->FilterLines();

  line_detector_->FormCorners();
  line_detector_->DecodeCorners();


  // TODO: set corners for goal detector
  // only do goals on top camera
  if (camera_ == Camera::TOP){
    goal_detector_->resetYellowGoal();
    blob_detector_->resetYellowBlobs();
    if (fabs(getHeadChange()) > DEG_T_RAD * .5) {
      vblocks_.robot_vision->reported_head_moving = true;
      visionLog((30,"Skipping yellow goal formation because of head movement:%0.3f ",getHeadChange()));
    } else {
      if (vblocks_.robot_vision->reported_head_moving) {
        visionLog((30,"Head STOP reported"));
        vblocks_.robot_vision->reported_head_stop_time = getCurrentTime();
        vblocks_.robot_vision->reported_head_moving = false;
      }
      if (getCurrentTime() - vblocks_.robot_vision->reported_head_stop_time > HEAD_STOP_THRESHOLD) {
        visionLog((30, "Enough time has passed since head stop (%f), forming yellow goals", getCurrentTime() - vblocks_.robot_vision->reported_head_stop_time));
        classifier_->startHighResGoalScan();
        classifier_->preProcessGoalPoints();
        blob_detector_->formYellowBlobs();
        goal_detector_->FormYellowGoal();
        classifier_->completeHighResScan();
      } else {
        visionLog((30, "Too soon since head movement stop, skipping yellow goal formation (%f)", getCurrentTime() - vblocks_.robot_vision->reported_head_stop_time));
      }
    }
  }

  line_detector_->DetermineEndPoints();
  line_detector_->IdentifyLines();
  line_detector_->DecodeLines();

  // only do robots on top camera
  if (camera_ == Camera::TOP){
    visionLog((30, "Detecting robots - calculating band data"));
    //TODO: enable these ifndefs when we get to robocup
#ifndef TOOL
    if (getTeamColor() == TEAM_BLUE)
#endif
    {
      visionLog((30, "Detecting Pink Robots"));
      robot_detector_->detectPinkRobots();
#ifndef TOOL
    } else {
#endif
      visionLog((30, "Detecting Blue Robots"));
      robot_detector_->detectBlueRobots();
    }
    //Disabled this because it runs really slowly with the high res images, needs to be redone anyway - JM 05/22/13
    //robot_detector_->detectRobotCluster();
  }
  vblocks_.robot_vision->lookForCross = true; // Always enabled for now
  if(camera_ == Camera::TOP && vblocks_.robot_vision->lookForCross) {
    visionLog((30, "Detecting crosses"));
    cross_detector_->detectCrosses();
  }
  else visionLog((30, "Skipping cross detection"));

  visionLog((21, "Vision frame process complete"));
}


int ImageProcessor::getTeamColor() {
  return vblocks_.robot_state->team_;
}

void ImageProcessor::SetColorTable(unsigned char* table) {
  color_table_ = table;
}

float ImageProcessor::getHeadChange() const {
  if (vblocks_.joint == NULL)
    return 0;
  return vblocks_.joint->changes_[HeadPan];
}

std::vector<BallCandidate*> ImageProcessor::getBallCandidates() {
    std::vector<BallCandidate*> candidates;
    for(int i = 0; i < ball_detector_->ballCandCount; i++) {
        candidates.push_back(ball_detector_->candidates_[i]);
    }
    return candidates;
}

BallCandidate* ImageProcessor::getBestBallCandidate() {
    if(ball_detector_->bestCandidateFound()) {
      BallCandidate* best = ball_detector_->candidates_[ball_detector_->bestBallCandIndex];
        return best;
    }
    return 0;
}

void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->loaded_;
}
