#ifndef _COLOUR_SPACES_H
#define _COLOUR_SPACES_H

// Color conversions

struct RGB {
  int r;
  int g;
  int b;
};

struct YUV444 {
  int y;
  int u;
  int v;
};

struct YUV422 {
  int u;
  int y0;
  int v;
  int y1;
};

inline int clip255(int x) {
  if (x>255) return 255;
  if (x<0) return 0;
  else return x;
}

inline RGB TORGB(int r, int g, int b) {
  RGB col;
  col.r=r;  
  col.g=g;
  col.b=b;
  return col;
}

inline YUV444 RGB_TO_YUV444 (int r, int g, int b) {
  YUV444 yuv;
  yuv.y= ( ( (66 * r + 129 * g +  25 * b + 128) >> 8) +  16);
  yuv.u= ( ( ( -38 * r -  74 * g + 112 * b + 128) >> 8) + 128);
  yuv.v= ( ( ( 112 * r -  94 * g -  18 * b + 128) >> 8) + 128);
    
  return yuv;
}

inline YUV444 RGB_TO_YUV444 (RGB rgb) {
  return RGB_TO_YUV444(rgb.r,rgb.g,rgb.b);
}

inline YUV422 RGB_TO_YUV422 (RGB rgb1, RGB rgb2) {
  YUV444 yuv1 = RGB_TO_YUV444(rgb1);
  YUV444 yuv2 = RGB_TO_YUV444(rgb2);
  YUV422 uyvy;
  uyvy.u=(int)((yuv1.u+yuv2.u)/2.0);
  uyvy.y0=yuv1.y;
  uyvy.v=(int)((yuv1.v+yuv2.v)/2.0);
  uyvy.y1=yuv2.y;
  return uyvy;
}

inline RGB YUV444_TO_RGB (int y, int u, int v) {
  RGB rgb;

  int C = y - 16;
  int D = u - 128;
  int E = v - 128;

  rgb.r = clip255(( 298 * C           + 409 * E + 128) >> 8);
  rgb.g = clip255(( 298 * C - 100 * D - 208 * E + 128) >> 8);
  rgb.b = clip255(( 298 * C + 516 * D           + 128) >> 8);
    
  return rgb;
}

inline RGB YUV444_TO_RGB (YUV444 yuv) {
  return YUV444_TO_RGB(yuv.y,yuv.u,yuv.v);
}

inline void YUV422_TO_RGB(YUV422 uyvy, RGB* rgb1, RGB* rgb2) {
  *rgb1 = YUV444_TO_RGB(uyvy.y0,uyvy.u,uyvy.v);
  *rgb2 = YUV444_TO_RGB(uyvy.y1,uyvy.u,uyvy.v);
}
#endif
