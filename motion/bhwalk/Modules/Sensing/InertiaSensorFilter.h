/**
* @file InertiaSensorFilter.h
* Declaration of module InertiaSensorFilter.
* @author Colin Graf
*/

#pragma once

//#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Tools/Math/Pose3D.h"
#include "Tools/Math/Matrix3x3.h"

//MODULE(InertiaSensorFilter)
//  REQUIRES(FrameInfo)
//  REQUIRES(InertiaSensorData)
//  REQUIRES(RobotModel)
//  REQUIRES(SensorData)
//  USES(MotionInfo)
//  USES(WalkingEngineOutput)
//  PROVIDES_WITH_MODIFY(OrientationData)
//END_MODULE

/**
* @class InertiaSensorFilter
* A module for estimating velocity and orientation of the torso.
*/
class InertiaSensorFilter //: public InertiaSensorFilterBase
{
public:
  /** Default constructor. */
  InertiaSensorFilter();

private:
  /**
  * A collection of parameters for this module.
  */
  class Parameters : public Streamable
  {
  public:
    Vector2BH<> processNoise; /**< The standard deviation of the process. */
    Vector3BH<> accNoise; /**< The standard deviation of the inertia sensor. */
    Vector2BH<> calculatedAccLimit; /**< Use a calculated angle up to this angle (in rad). (We use the acceleration sensors otherwise.) */

    Matrix2x2BH<> processCov; /**< The covariance matrix for process noise. */
    Matrix3x3BH<> sensorCov; /**< The covariance matrix for sensor noise. */

    /** Default constructor. */
    Parameters() {};

    /**
    * Calculates parameter dependent constants used to speed up some calculations.
    */
    void calculateConstants();

  private:
    /**
    * The method makes the object streamable.
    * @param in The stream from which the object is read.
    * @param out The stream to which the object is written.
    */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(processNoise);
      STREAM(accNoise);
      STREAM(calculatedAccLimit);
      STREAM_REGISTER_FINISH();

      if(in)
        calculateConstants();
    }
  };

  /**
  * Represents the state to be estimated.
  */
  template <class V = float> class State
  {
  public:
    RotationMatrixBH rotation; /** The rotation of the torso. */

    /** Default constructor. */
    State() {}

    /**
    * Adds some world rotation given as angle axis.
    * @param value The flat vector to add.
    * @return A new object after the calculation.
    */
    State operator+(const Vector2BH<V>& value) const;

    /**
    * Adds some world rotation given as angle axis.
    * @param value The flat vector to add.
    * @return A reference this object after the calculation.
    */
    State& operator+=(const Vector2BH<V>& value);

    /**
    * Calculates a flat difference vector of two states.
    * @return The difference.
    */
    Vector2BH<V> operator-(const State& other) const;
  };

  Parameters p; /**< The parameters of this module. */

  Pose3DBH lastLeftFoot; /**< The pose of the left foot of the previous iteration. */
  Pose3DBH lastRightFoot; /**< The pose of the right foot of the previous iteration. */
  unsigned int lastTime; /**< The frame time of the previous iteration. */
  Vector2BH<> safeRawAngle; /**< The last not corrupted angle from aldebarans angle estimation algorithm. */

  State<> x; /**< The estimate */
  Matrix2x2BH<> cov; /**< The covariance of the estimate. */
  Matrix2x2BH<> l; /**< The last caculated cholesky decomposition of \c cov. */
  State<> sigmaPoints[5]; /**< The last calculated sigma points. */

  Vector3BH<> sigmaReadings[5]; /**< The expected sensor values at the sigma points. */
  Vector3BH<> readingMean; /**< The mean of the expected sensor values which was determined by using the sigma velocities. */
  Matrix3x3BH<> readingsCov;
  Matrix3x2BH<> readingsSigmaPointsCov;

  public:
  /**
  * Updates the OrientationData representation.
  * @param orientationData The orientation data representation which is updated by this module.
  */
  void update(OrientationData& orientationData,
          const InertiaSensorData& theInertiaSensorData,
          const SensorData& theSensorData,
          const RobotModel& theRobotModel,
          const FrameInfo& theFrameInfo,
          const MotionInfo& theMotionInfo,
          const WalkingEngineOutput& theWalkingEngineOutput);

  /**
  * Restores the initial state.
  */
  void reset();

  void predict(const RotationMatrixBH& rotationOffset);
  void readingUpdate(const Vector3BH<>& reading);

  void cholOfCov();
  void generateSigmaPoints();
  void meanOfSigmaPoints();
  void covOfSigmaPoints();

  void readingModel(const State<>& sigmaPoint, Vector3BH<>& reading);
  void meanOfSigmaReadings();
  void covOfSigmaReadingsAndSigmaPoints();
  void covOfSigmaReadings();

  inline Matrix2x2BH<> tensor(const Vector2BH<>& a, const Vector2BH<>& b) const
  {
    return Matrix2x2BH<>(a * b.x, a * b.y);
  }

  inline Matrix2x2BH<> tensor(const Vector2BH<>& a) const
  {
    return Matrix2x2BH<>(a * a.x, a * a.y);
  }

  inline Matrix3x2BH<> tensor(const Vector3BH<>& a, const Vector2BH<>& b)
  {
    return Matrix3x2BH<>(a * b.x, a * b.y);
  }

  inline Matrix3x3BH<> tensor(const Vector3BH<>& a)
  {
    return Matrix3x3BH<>(a * a.x, a * a.y, a * a.z);
  }
};
