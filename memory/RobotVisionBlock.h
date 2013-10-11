#ifndef ROBOTVISIONBLOCK_
#define ROBOTVISIONBLOCK_

#include "MemoryBlock.h"
#include <common/RobotInfo.h>
#include <constants/ImageConstants.h>
#include <math/Geometry.h>
#include <vision/structures/HorizonLine.h>
#include <boost/interprocess/offset_ptr.hpp>

#define MAX_VISION_OPPS 10

struct RobotVisionBlock : public MemoryBlock {

  enum VisionBodyPoints {
    LEFT_SHOULDER,
    RIGHT_SHOULDER,
    LEFT_FOOT_TL,
    LEFT_FOOT_TR,
    LEFT_FOOT_BL,
    LEFT_FOOT_BR,
    RIGHT_FOOT_TL,
    RIGHT_FOOT_TR,
    RIGHT_FOOT_BL,
    RIGHT_FOOT_BR,
    NUM_VISION_BODY_POINTS
  };

private:
  unsigned char *segImgTopLocal, *segImgBottomLocal;
  boost::interprocess::offset_ptr<unsigned char> segImgTop;
  boost::interprocess::offset_ptr<unsigned char> segImgBottom;

public:
  ImageParams top_params_, bottom_params_;
  bool loaded_;
  RobotVisionBlock() : top_params_(Camera::TOP), bottom_params_(Camera::BOTTOM) {
    header.version = 5;
    header.size = sizeof(RobotVisionBlock);
    reported_head_stop_time = 0.0;
    reported_head_moving = false;
    segImgBottomLocal = new unsigned char[bottom_params_.size];
    segImgTopLocal = new unsigned char[top_params_.size];
    segImgTop = NULL;
    segImgBottom = NULL;
    loaded_ = false;
  }

  ~RobotVisionBlock() {
    delete [] segImgTopLocal;
    delete [] segImgBottomLocal;
  }

  inline const unsigned char* getSegImgTop() const { return segImgTop.get(); }
  inline const unsigned char* getSegImgBottom() const { return segImgBottom.get(); }
  inline unsigned char* getSegImgTop() { return segImgTop.get(); }
  inline unsigned char* getSegImgBottom() { return segImgBottom.get(); }
  inline void setSegImgTop(unsigned char* img) { segImgTop = img; }
  inline void setSegImgBottom(unsigned char* img) { segImgBottom = img; }


  RobotVisionBlock& operator=(const RobotVisionBlock& other) {
    header = other.header;
    if(top_params_.rawSize != other.top_params_.rawSize) {
      delete [] segImgTopLocal;
      top_params_ = other.top_params_;
      segImgTopLocal = new unsigned char[top_params_.rawSize];
    }
    if(bottom_params_.rawSize != other.bottom_params_.rawSize) {
      delete [] segImgBottomLocal;
      bottom_params_ = other.bottom_params_;
      segImgBottomLocal = new unsigned char[bottom_params_.rawSize];
    }
    
    memcpy(segImgTopLocal, other.getSegImgTop(), top_params_.size);
    memcpy(segImgBottomLocal, other.getSegImgBottom(), bottom_params_.size);
    segImgTop = segImgTopLocal;
    segImgBottom = segImgBottomLocal;
    horizon = other.horizon;
    loaded_ = true;
    return *this;
  }

  void serialize(StreamBuffer& buffer, std::string);
  bool deserialize(const StreamBuffer& buffer, std::string);

  bool doHighResBallScan;
  bool lookForCross;

  double reported_head_stop_time;
  double reported_head_moving;

  Point2D bodyPointsImage[NUM_VISION_BODY_POINTS];
  HorizonLine horizon;

};

#endif
