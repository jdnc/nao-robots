#ifndef GOALDETECTOR_H
#define GOALDETECTOR_H

#include <memory/TextLogger.h>
#include <vision/ObjectDetector.h>
#include <vision/structures/GoalPoint.h>
#include <vision/structures/FieldLine.h>
#include <vision/structures/CornerPoint.h>
#include <vision/enums/Colors.h>
#include <vision/BlobDetector.h>
#include <vision/LineDetector.h>
#include <vision/Classifier.h>

class GoalDetector : public ObjectDetector {
 public:
  GoalDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector, LineDetector*& line_detector);
  void init(TextLogger* tl){textlogger = tl;};

  float estimateGoalDistance(FieldLine * goal);
  void sanitizeGoalBlobs();
  float estimateGoalDistanceByKinematics(FieldLine* goal);
  float estimateGoalDistanceByHeight(float height);
  float estimateGoalDistanceByWidth(float width);
  float estimateGoalDistanceByPosts(Position left, Position right);
  void sanitizeGoalCandidate(FieldLine* goal);
  void setCornerPoints(CornerPoint**);
  void FormGoal();
  void FormYellowGoal();
  void resetYellowGoal();
  void setGoalObject(int goalIndex, float distance, float bearing, float elevation, int centerX, int centerY, float confidence, int lineIndex);
  void setCornerPointCounter(int value);
  bool mergeLinesVertical(FieldLine * lineTo, FieldLine * lineFrom);
  FieldLine** yellowPosts; // Goals
  int YellowPostCounter;
 private:
  
  Classifier*& classifier_;
  BlobDetector*& blob_detector_;
  LineDetector*& line_detector_;
  
  unsigned char color;
  TextLogger* textlogger;

  int cornerPointCounter;
  CornerPoint** cornerPoints;
  int totalValidGoals;
  FieldLine** currentLine;

  float greenWhitePercentBelow(FieldLine*);
};

#endif
