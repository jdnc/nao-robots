/**
* @file Representations/MotionControl/HeadJointRequest.h
* This file declares a class that represents the requested head joint angles.
* @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
*/

#pragma once

#include "Tools/Streams/Streamable.h"

/**
* @class HeadJointRequest
* A class that represents the requested head joint angles.
*/
class HeadJointRequest : public Streamable
{
public:
  float pan, /**< Head pan target angle in radians. */
        tilt; /**< Head tilt target angle in radians. */
  bool reachable; /**< Whether the head motion request points on a reachable position. */
  bool moving; /**< Whether the head is currently in motion or not. */

  /**
  * Default constructor.
  */
  HeadJointRequest() : pan(0), tilt(0), reachable(true) {}

private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(pan);
    STREAM(tilt);
    STREAM(reachable);
    STREAM(moving);
    STREAM_REGISTER_FINISH();
  }
};
