#include <common/ColorConversion.h>

cv::Mat color::rawToMat(const unsigned char* imgraw, const ImageParams& params) {
  cv::Mat cvimage(params.height, params.width, CV_8UC3);
  for(int r = 0; r < params.height; r++) {
    for(int c = 0; c < params.width; c += 2) {
      color::Yuv422 yuyv;
      yuyv.y0 = (int) (*(imgraw++));
      yuyv.u = (int) (*(imgraw++));
      yuyv.y1 = (int) (*(imgraw++));
      yuyv.v = (int) (*(imgraw++));

      color::Rgb rgb1, rgb2;
      color::yuv422ToRgb(yuyv, rgb1, rgb2);
      cv::Vec3b color1(rgb1.b, rgb1.g, rgb1.r), color2(rgb2.b, rgb2.g, rgb2.r);
      cvimage.at<cv::Vec3b>(r, c) = color1;
      cvimage.at<cv::Vec3b>(r, c + 1) = color2;
    }
  }
  return cvimage;
}

void color::matToRaw(const cv::Mat& mat, unsigned char* imgraw, const ImageParams& iparams) {
  for(int r = 0; r < iparams.height; r++) {
    for(int c = 0; c < iparams.width; c += 2) {
      color::Rgb rgb1, rgb2;
      cv::Vec3b p1 = mat.at<cv::Vec3b>(r, c), p2 = mat.at<cv::Vec3b>(r, c + 1);
      rgb1.r = p1[2]; rgb1.g = p1[1]; rgb1.b = p1[0];
      rgb2.r = p2[2]; rgb2.g = p2[1]; rgb2.b = p2[0];
      color::Yuv422 yuv = color::rgbtoToYuv422(rgb1, rgb2);
      *(imgraw++) = yuv.y0;
      *(imgraw++) = yuv.u;
      *(imgraw++) = yuv.y1;
      *(imgraw++) = yuv.v;
    }
  }
}
