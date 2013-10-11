#ifndef _FIELD_H
#define _FIELD_H

#include "WorldObject.h"
#include <stdio.h>

//#ifdef interface_H
//cout << "Field_h";
//#endif


// setup constants for field size, landmark locations, etc
// New Field

const float BALL_RADIUS = 31;

const float FIELD_Y = 6000; // regulation is 6000
const float FIELD_X = 8000; // TODO: regulation is 9000
const float GRASS_Y = 6600; //
const float GRASS_X = 8600; // TODO: regulation is 10400

const float HALF_FIELD_Y = FIELD_Y/2.0;
const float HALF_FIELD_X = FIELD_X/2.0;
const float HALF_GRASS_Y = GRASS_Y/2.0;
const float HALF_GRASS_X = GRASS_X/2.0;

const float GOAL_Y = 1500;// TODO: regulation is 1500
const float GOAL_X = 500;
const float HALF_GOAL_Y = GOAL_Y / 2.0;
const float BORDER_OFFSET = 150;
const float GOAL_OFFSET   = 50;
const float PENALTY_Y = 2200; // regulation is 2200;
const float PENALTY_X =  600;
const float CIRCLE_RADIUS =  750; // regulation size
const float LINE_WIDTH = 50;

const float FIELD_LINE_WIDTH =     50; //25;
const float BEACON_HEIGHT       = 200;
const float BEACON_LOWER_HEIGHT = 200;
const float BEACON_UPPER_HEIGHT = 400;
const float GOAL_HEIGHT =    800; //400;
const float BALL_HEIGHT =          65;

const float ROBOT_HEIGHT =   520; //   570;

const float FIELD_CENTER_X = 0;
const float FIELD_CENTER_Y = 0;

const float PENALTY_CROSS_X = HALF_FIELD_X - 1800;
const float PENALTY_MARK_SIZE = 106; // regulation is 100

const float CIRCLE_HEX_LENGTH = 2.0*(CIRCLE_RADIUS*sinf(DEG_T_RAD*30.0));

// Some rectangles

const Rectangle FIELD =
Rectangle( Point2D( -FIELD_X / 2,  FIELD_Y / 2 ),
	   Point2D(  FIELD_X / 2, -FIELD_Y / 2 ) );

const Rectangle GRASS =
Rectangle( Point2D( -GRASS_X / 2,  GRASS_Y / 2 ),
	   Point2D(  GRASS_X / 2, -GRASS_Y / 2 ) );

// circle and cross points
const Point2D circleLocation = Point2D(0, 0);
const Point2D oppCrossLocation = Point2D(PENALTY_CROSS_X, 0);
const Point2D ownCrossLocation = Point2D(-PENALTY_CROSS_X, 0);


// Landmark locations
const Point2D landmarkLocation[] = {
  Point2D(0, 0),  // center circle

  Point2D( -FIELD_X / 2, 0),                 // WO_OWN_GOAL = 7,
  Point2D( FIELD_X / 2, 0 ),                 // WO_OPP_GOAL = 10,

  Point2D( -FIELD_X / 2, -GOAL_Y / 2),       // WO_OWN_LEFT_GOALPOST = 8,
  Point2D( FIELD_X / 2, GOAL_Y / 2 ),        // WO_OPP_LEFT_GOALPOST = 11,

  Point2D( -FIELD_X / 2, GOAL_Y / 2),        // WO_OWN_RIGHT_GOALPOST = 9,
  Point2D( FIELD_X / 2, -GOAL_Y / 2)         // WO_OPP_RIGHT_GOALPOST = 12,
};


// Line intersection locations
const Point2D intersectionLocation[] = {
  // L
  Point2D( FIELD_X / 2, FIELD_Y / 2),                 // 0 WO_OPP_FIELD_LEFT_L
  Point2D( FIELD_X / 2, -FIELD_Y / 2),                //   WO_OPP_FIELD_RIGHT_L
  Point2D( FIELD_X / 2 - PENALTY_X, PENALTY_Y / 2),   // 2 WO_OPP_PEN_LEFT_L
  Point2D( FIELD_X / 2 - PENALTY_X, -PENALTY_Y / 2),  //   WO_OPP_PEN_RIGHT_L
  Point2D( -FIELD_X / 2 + PENALTY_X, PENALTY_Y / 2),  // 4 WO_OWN_PEN_RIGHT_L
  Point2D( -FIELD_X / 2 + PENALTY_X, -PENALTY_Y / 2), //   WO_OWN_PEN_LEFT_L
  Point2D( -FIELD_X / 2, FIELD_Y / 2),                // 6 WO_OWN_FIELD_RIGHT_L
  Point2D( -FIELD_X / 2, -FIELD_Y / 2),               // 7 WO_OWN_FIELD_LEFT_L

  // T
  Point2D( FIELD_X / 2, PENALTY_Y / 2),               // 8  WO_OPP_PEN_LEFT_T
  Point2D( FIELD_X / 2, -PENALTY_Y / 2),              //    WO_OPP_PEN_RIGHT_T
  Point2D( 0, FIELD_Y / 2),                           // 10 WO_CENTER_TOP_T
  Point2D( 0, -FIELD_Y / 2),                          //    WO_CENTER_BOTTOM_T,
  Point2D( -FIELD_X / 2, PENALTY_Y / 2),              // 12 WO_OWN_PEN_RIGHT_T
  Point2D( -FIELD_X / 2, -PENALTY_Y / 2)             // 13 WO_OWN_PEN_LEFT_T

};


// Line location
const Point2D lineLocationStarts[] = {

  // HORIZONTAL LINES
  intersectionLocation[0],    // WO_OPP_GOAL_LINE
  intersectionLocation[2],    // WO_OPP_PENALTY
  intersectionLocation[10],  // WO_CENTER_LINE
  intersectionLocation[4],  // WO_OWN_PENALTY
  intersectionLocation[6],    // WO_OWN_GOAL_LINE

  // VERTICAL LINES
  intersectionLocation[0],    // WO_TOP_SIDE_LINE
  intersectionLocation[2],    // WO_PENALTY_TOP_OPP
  intersectionLocation[4],    // WO_PENALTY_TOP_OWN
  intersectionLocation[3],    // WO_PENALTY_BOTTOM_OPP
  intersectionLocation[5],    // WO_PENALTY_BOTTOM_OWN
  intersectionLocation[1]     // WO_BOTTOM_SIDE_LINE

};

// Line location
const Point2D lineLocationEnds[] = {

  // HORIZONTAL LINES
  intersectionLocation[1],    // WO_OPP_GOAL_LINE
  intersectionLocation[3],    // WO_OPP_PENALTY
  intersectionLocation[11],  // WO_CENTER_LINE
  intersectionLocation[5],  // WO_OWN_PENALTY
  intersectionLocation[7],    // WO_OWN_GOAL_LINE

  // VERTICAL LINES
  intersectionLocation[6],    // WO_TOP_SIDE_LINE
  intersectionLocation[8],    // WO_PENALTY_TOP_OPP
  intersectionLocation[12],    // WO_PENALTY_TOP_OWN
  intersectionLocation[9],    // WO_PENALTY_BOTTOM_OPP
  intersectionLocation[13],    // WO_PENALTY_BOTTOM_OWN
  intersectionLocation[7]     // WO_BOTTOM_SIDE_LINE

};


#endif
