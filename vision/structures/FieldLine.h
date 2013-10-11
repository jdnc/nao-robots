#ifndef FIELDLINE_H
#define FIELDLINE_H

#include <vision/structures/LinePoint.h>

#define MAX_FIELDLINES 25

struct FieldLine {

  LinePoint** PointsArray;
  LinePoint** TranPointsArray;
  unsigned int Points;

  float Offset;
  float Slope, Angle;
  bool ValidLine;

  int MinX, MaxX, MinY, MaxY;

  // least square values saved for faster merging
  float sumX, sumY, sumX2, sumY2, sumXY;

  Point2D tL;
  Point2D bR;

  bool isCurve;
  bool isCircle;
  bool onCircle;
  bool isCross;

  int length;
  int width;
  int avgX;
  int avgY;
  int id;
  float rateSlope;

  bool isVertical;
  bool isHorizontal;

  int identifiedIndex;

  float confidence;

  float preciseWidth;

  Point2D pt1;
  Point2D pt2;
};

#endif
