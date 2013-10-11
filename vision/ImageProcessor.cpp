#include "ImageProcessor.h"
#include <iostream>

ImageProcessor::ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera) :
  vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(iparams_, camera), calibration_(NULL)
{
  enableCalibration_ = false;
  classifier_ = new Classifier(vblocks_, vparams_, iparams_, camera_);
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

void ImageProcessor::setCalibration(RobotCalibration calibration){
  if(calibration_) delete calibration_;
  calibration_ = new RobotCalibration(calibration);
}

void ImageProcessor::processFrame(){
  updateTransform();
  classifier_->classifyImage(color_table_);
  classifier_->constructRuns();
}

void ImageProcessor::SetColorTable(unsigned char* table) {
  color_table_ = table;
}

std::vector<BallCandidate*> ImageProcessor::getBallCandidates() {
  std::vector<BallCandidate*> candidates;
  return candidates;
}

BallCandidate* ImageProcessor::getBestBallCandidate() {
  return NULL;
}

void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->loaded_;
}
