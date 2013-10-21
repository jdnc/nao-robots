#ifndef BCONSTRAINTS_H
#define BCONSTRAINTS_H
#include <vision/BlobDetector.h>
#include<stdlib.h>

// if constraint true then good
bool centroidcc(Blob &b1, Blob &b2){
  return (b1.avgX <= b2.xf && b1.avgX >=b2.xi) && (b2.avgX <= b1.xf && b2.avgX >=b1.xi);
}

bool rangecc(Blob &b1, Blob &b2){
  bool c1 = (b1.xf - b1.xi + 1) < 2 * (b2.xf - b2.xi + 1);
  bool c2 = (b2.xf - b2.xi + 1) < 2 * (b1.xf - b1.xi + 1);
  bool c3 = (b1.yf - b1.yi + 1) < 2 * (b2.yf - b2.yi + 1);
  bool c4 = (b2.yf - b2.yi + 1) < 2 * (b1.yf - b1.yi + 1);
  return c1 && c2 && c3 && c4;
}

bool ratiocc(Blob &b1, Blob &b2){ 
  double a1 = b1.dx * b1.dy;
  double a2 = b2.dx * b2.dy;
  bool c1 = (a1 >= 0.5 * a2);
  bool c2 = (a2 >= 0.5 * a1);
  return c1 && c2;
}

// floating constraint
bool floatcc(ProbBeacon &pb){
  return (pb.bottom->yi - pb.top->yf) <= 30; 
}
 
bool floatcc(Blob& b1, Blob& b2){
  return(min(abs(b1.yf-b2.yi), abs(b1.yi-b2.yf)) <= 20); //increasing threshold to be safe
}
#endif
