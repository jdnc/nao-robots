#ifndef IMAGEBLOCK_95BEQPL8
#define IMAGEBLOCK_95BEQPL8

#include "MemoryBlock.h"
#include <common/RobotInfo.h>
#include <boost/interprocess/offset_ptr.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <common/ColorConversion.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

struct ImageBlock : public MemoryBlock {
private:
  // private arrays used for logging and for sim
  unsigned char *img_top_local_, *img_bottom_local_;
public:
  ImageParams top_params_, bottom_params_;
  bool loaded_;
  ImageBlock() : top_params_(Camera::TOP), bottom_params_(Camera::BOTTOM) {
    header.version = 3;
    header.size = sizeof(ImageBlock);
    img_top_local_ = new unsigned char[top_params_.rawSize];
    img_bottom_local_ = new unsigned char[bottom_params_.rawSize];
    img_top_ = NULL;
    img_bottom_ = NULL;
    loaded_ = false;
  }

  ~ImageBlock() {
    delete [] img_top_local_;
    delete [] img_bottom_local_;
  }

  ImageBlock& operator=(const ImageBlock& other) {
    header = other.header;
    if(top_params_.rawSize != other.top_params_.rawSize) {
      delete [] img_top_local_;
      top_params_ = other.top_params_;
      img_top_local_ = new unsigned char[top_params_.rawSize];
    }
    if(bottom_params_.rawSize != other.bottom_params_.rawSize) {
      delete [] img_bottom_local_;
      bottom_params_ = other.bottom_params_;
      img_bottom_local_ = new unsigned char[bottom_params_.rawSize];
    }

    memcpy(img_top_local_, other.img_top_local_, top_params_.rawSize);
    memcpy(img_bottom_local_, other.img_bottom_local_, bottom_params_.rawSize);
    img_top_ = img_top_local_;
    img_bottom_ = img_bottom_local_;
    loaded_ = other.loaded_;
    return *this;
  }

  inline unsigned char* getImgTop() { return img_top_.get(); }
  inline unsigned char* getImgBottom() { return img_bottom_.get(); }
  inline void setImgTop(unsigned char* img) { img_top_ = img; }
  inline void setImgBottom(unsigned char* img) { img_bottom_ = img; }

  bool isFromLog() {
    bool isTopFromLog = img_top_ == boost::interprocess::offset_ptr<unsigned char>(&(img_top_local_[0]));
    bool isBottomFromLog = img_bottom_ == boost::interprocess::offset_ptr<unsigned char>(&(img_bottom_local_[0]));
    return isTopFromLog && isBottomFromLog;
  }

  void serialize(StreamBuffer& buffer, std::string data_dir);
  bool deserialize(const StreamBuffer& buffer, std::string data_dir);

  void writeImage(const unsigned char* imgraw, std::string path, const ImageParams& iparams) {
    cv::Mat cvimage = color::rawToMat(imgraw, iparams);
    cv::imwrite(path, cvimage);
  }
  void readImage(unsigned char* imgraw, std::string path, const ImageParams& iparams) {
    cv::Mat cvimage = cv::imread(path);
    if(cvimage.data)
      color::matToRaw(cvimage, imgraw, iparams);
    else
      loaded_ = false;
  }

  boost::interprocess::offset_ptr<unsigned char> img_top_;
  boost::interprocess::offset_ptr<unsigned char> img_bottom_;
};

#endif /* end of include guard: IMAGEBLOCK_95BEQPL8 */
