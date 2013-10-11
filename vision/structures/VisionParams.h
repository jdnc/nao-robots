#ifndef VISIONPARAMS_H
#define VISIONPARAMS_H

#include <vision/enums/Colors.h>
#include <stdint.h>

struct VisionParams {
    int WHITE_WIDTH_RATIO_BLOB;// = 7;
    int WHITE_WIDTH_ABS_BLOB;// = 2;

    int COLOR_WIDTH_RATIO_BLOB;// = 6;
    int COLOR_WIDTH_ABS_BLOB;// = 0;

    bool ALLOW_BAND_MERGE_POINTS;// = true;
    int BAND_MERGE_POINT_DIST;// = 30;

    bool ALLOW_GOAL_MERGE_POINTS;// = true;
    int GOAL_MERGE_POINT_DIST;// = 5;

    // Horizontal field line (white) formation parameters

    bool ALLOW_WHITE_HORZ_MERGE_POS;// = true;
    int WHITE_HORZ_MERGE_POS_LIMIT;// = 7;

    bool ALLOW_WHITE_HORZ_MERGE_ANGLE;// = true;
    float WHITE_HORZ_MERGE_ANGLE_LIMIT;// = 0.3;

    bool ALLOW_WHITE_HORZ_MERGE_WIDTH;// = true;
    float WHITE_HORZ_MERGE_WIDTH_LIMIT;// = 0.3;

    int WHITE_HORZ_VAL_FUNCTION_PARAM_A;// = 1;
    int WHITE_HORZ_VAL_FUNCTION_PARAM_B;// = 1;
    int WHITE_HORZ_VAL_FUNCTION_THRESHOLD;// = 12;

    int WHITE_HORZ_SEPARATION; // = 3;

    // Vertical Goal formation parameters

    bool ALLOW_GOAL_VERT_MERGE_POS;// = true;
    int GOAL_VERT_MERGE_POS_LIMIT;// = 0.7;

    bool ALLOW_GOAL_VERT_MERGE_ANGLE;// = true;
    float GOAL_VERT_MERGE_ANGLE_LIMIT;// = 0.3;

    bool ALLOW_GOAL_VERT_MERGE_WIDTH;// = true;
    float GOAL_VERT_MERGE_WIDTH_LIMIT;// = 0.3;

    int GOAL_VERT_VAL_FUNCTION_PARAM_A;// = 1;
    int GOAL_VERT_VAL_FUNCTION_PARAM_B;// = 0;
    int GOAL_VERT_VAL_FUNCTION_THRESHOLD;// = 12;

    int GOAL_VERT_SEPARATION; // = 2;

    // Vert line formation parameters

    bool ALLOW_WHITE_VERT_MERGE_POS;// = true;
    int WHITE_VERT_MERGE_POS_LIMIT;// = 7;

    bool ALLOW_WHITE_VERT_MERGE_ANGLE;// = true;
    float WHITE_VERT_MERGE_ANGLE_LIMIT;// = 0.3;

    bool ALLOW_WHITE_VERT_MERGE_WIDTH;// = true;
    float WHITE_VERT_MERGE_WIDTH_LIMIT;// = 0.3;

    int WHITE_VERT_VAL_FUNCTION_PARAM_A;// = 1;
    int WHITE_VERT_VAL_FUNCTION_PARAM_B;// = 1;
    int WHITE_VERT_VAL_FUNCTION_THRESHOLD;// = 12;

    int WHITE_VERT_SEPARATION; // = 3;
    bool useHorzPos[NUM_COLORS];
    bool useHorzAngle[NUM_COLORS];
    bool useHorzWidth[NUM_COLORS];
    int horzPosLimit[NUM_COLORS];
    float horzAngleLimit[NUM_COLORS];
    float horzWidthLimit[NUM_COLORS];
    int horzSeparation[NUM_COLORS];

    int horzValA[NUM_COLORS];
    int horzValB[NUM_COLORS];
    int horzValThreshold[NUM_COLORS];

    bool useVertPos[NUM_COLORS];
    bool useVertAngle[NUM_COLORS];
    bool useVertWidth[NUM_COLORS];
    int vertPosLimit[NUM_COLORS];
    float vertAngleLimit[NUM_COLORS];
    float vertWidthLimit[NUM_COLORS];
    int vertSeparation[NUM_COLORS];

    int vertValA[NUM_COLORS];
    int vertValB[NUM_COLORS];
    int vertValThreshold[NUM_COLORS];

    uint16_t blobWidthAbs[NUM_COLORS];
    bool blobUseWidthRatioVert[NUM_COLORS];
    bool blobUseWidthRatioHorz[NUM_COLORS];
    uint16_t blobWidthRatio[NUM_COLORS];

    VisionParams() {
        WHITE_WIDTH_RATIO_BLOB = 7;
        WHITE_WIDTH_ABS_BLOB = 2;

        COLOR_WIDTH_RATIO_BLOB = 6;
        COLOR_WIDTH_ABS_BLOB = 0;

        ALLOW_BAND_MERGE_POINTS = true;
        BAND_MERGE_POINT_DIST = 30;

        ALLOW_GOAL_MERGE_POINTS = true;
        GOAL_MERGE_POINT_DIST = 20;

        // Horizontal field line (white) formation parameters

        ALLOW_WHITE_HORZ_MERGE_POS = true;
        WHITE_HORZ_MERGE_POS_LIMIT = 7;

        ALLOW_WHITE_HORZ_MERGE_ANGLE = true;
        WHITE_HORZ_MERGE_ANGLE_LIMIT = 0.3;

        ALLOW_WHITE_HORZ_MERGE_WIDTH = true;
        WHITE_HORZ_MERGE_WIDTH_LIMIT = 0.3;

        WHITE_HORZ_VAL_FUNCTION_PARAM_A = 1;
        WHITE_HORZ_VAL_FUNCTION_PARAM_B = 1;
        WHITE_HORZ_VAL_FUNCTION_THRESHOLD = 12;

        WHITE_HORZ_SEPARATION = 3;

        // Vertical Goal formation parameters

        ALLOW_GOAL_VERT_MERGE_POS = true;
        GOAL_VERT_MERGE_POS_LIMIT = 15;

        ALLOW_GOAL_VERT_MERGE_ANGLE = true;
        GOAL_VERT_MERGE_ANGLE_LIMIT = 0.5;

        ALLOW_GOAL_VERT_MERGE_WIDTH = false;
        GOAL_VERT_MERGE_WIDTH_LIMIT = 0;

        GOAL_VERT_VAL_FUNCTION_PARAM_A = 1;
        GOAL_VERT_VAL_FUNCTION_PARAM_B = 0;
        GOAL_VERT_VAL_FUNCTION_THRESHOLD = 12;

        GOAL_VERT_SEPARATION = 2;

        // Vert line formation parameters

        ALLOW_WHITE_VERT_MERGE_POS = true;
        WHITE_VERT_MERGE_POS_LIMIT = 7;

        ALLOW_WHITE_VERT_MERGE_ANGLE = true;
        WHITE_VERT_MERGE_ANGLE_LIMIT = 0.3;

        ALLOW_WHITE_HORZ_MERGE_WIDTH = true;
        WHITE_VERT_MERGE_WIDTH_LIMIT = 0.3;

        WHITE_VERT_VAL_FUNCTION_PARAM_A = 1;
        WHITE_VERT_VAL_FUNCTION_PARAM_B = 1;
        WHITE_VERT_VAL_FUNCTION_THRESHOLD = 12;

        WHITE_VERT_SEPARATION = 3;
    }

    void init() {
      for (int i = 0; i < NUM_COLORS; i++) {
        if (i != c_WHITE) {

          blobWidthRatio[i] = COLOR_WIDTH_RATIO_BLOB;
          blobWidthAbs[i] = COLOR_WIDTH_ABS_BLOB;

          useHorzPos[i] = false;
          useHorzAngle[i] = false;
          useHorzWidth[i] = false;
          horzPosLimit[i] = 0;
          horzAngleLimit[i] = 0;
          horzWidthLimit[i] = 0;
          horzSeparation[i] = 0;

          horzValA[i] = 0;
          horzValB[i] = 0;
          horzValThreshold[i] = 0;

          useVertPos[i] = ALLOW_GOAL_VERT_MERGE_POS;
          useVertAngle[i] = ALLOW_GOAL_VERT_MERGE_ANGLE;
          useVertWidth[i] = ALLOW_GOAL_VERT_MERGE_WIDTH;
          vertPosLimit[i] = GOAL_VERT_MERGE_POS_LIMIT;
          vertAngleLimit[i] = GOAL_VERT_MERGE_ANGLE_LIMIT;
          vertWidthLimit[i] = GOAL_VERT_MERGE_WIDTH_LIMIT;
          vertSeparation[i] = GOAL_VERT_SEPARATION;

          vertValA[i] = GOAL_VERT_VAL_FUNCTION_PARAM_A;
          vertValB[i] = GOAL_VERT_VAL_FUNCTION_PARAM_B;
          vertValThreshold[i] = GOAL_VERT_VAL_FUNCTION_THRESHOLD;

        } else {

          blobWidthRatio[i] = WHITE_WIDTH_RATIO_BLOB;
          blobWidthAbs[i] = WHITE_WIDTH_ABS_BLOB;

          useHorzPos[i] = ALLOW_WHITE_HORZ_MERGE_POS;
          useHorzAngle[i] = ALLOW_WHITE_HORZ_MERGE_ANGLE;
          useHorzWidth[i] = ALLOW_WHITE_HORZ_MERGE_WIDTH;
          horzPosLimit[i] = WHITE_HORZ_MERGE_POS_LIMIT;
          horzAngleLimit[i] = WHITE_HORZ_MERGE_ANGLE_LIMIT;
          horzWidthLimit[i] = WHITE_HORZ_MERGE_WIDTH_LIMIT;
          horzSeparation[i] = WHITE_HORZ_SEPARATION;

          horzValA[i] = WHITE_HORZ_VAL_FUNCTION_PARAM_A;
          horzValB[i] = WHITE_HORZ_VAL_FUNCTION_PARAM_B;
          horzValThreshold[i] = WHITE_HORZ_VAL_FUNCTION_THRESHOLD;

          useVertPos[i] = ALLOW_WHITE_VERT_MERGE_POS;
          useVertAngle[i] = ALLOW_WHITE_VERT_MERGE_ANGLE;
          useVertWidth[i] = ALLOW_WHITE_VERT_MERGE_WIDTH;
          vertPosLimit[i] = WHITE_VERT_MERGE_POS_LIMIT;
          vertAngleLimit[i] = WHITE_VERT_MERGE_ANGLE_LIMIT;
          vertWidthLimit[i] = WHITE_VERT_MERGE_WIDTH_LIMIT;
          vertSeparation[i] = WHITE_VERT_SEPARATION;

          vertValA[i] = WHITE_VERT_VAL_FUNCTION_PARAM_A;
          vertValB[i] = WHITE_VERT_VAL_FUNCTION_PARAM_B;
          vertValThreshold[i] = WHITE_VERT_VAL_FUNCTION_THRESHOLD;

        }
        if (i != c_PINK && i != c_BLUE) {
          blobUseWidthRatioVert[i] = true;
        } else {
          blobUseWidthRatioVert[i] = false;
        }
        if (i != c_ORANGE) {
          blobUseWidthRatioHorz[i] = true;
        } else {
          blobUseWidthRatioHorz[i] = false;
        }
      }
    }
};
#endif
