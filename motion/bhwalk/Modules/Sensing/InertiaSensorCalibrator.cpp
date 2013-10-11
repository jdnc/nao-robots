/**
* @file InertiaSensorCalibrator.cpp
* Implementation of module InertiaSensorCalibrator.
* @author Colin Graf
*/

#include "InertiaSensorCalibrator.h"
#include "Tools/Math/Pose3D.h"

//MAKE_MODULE(InertiaSensorCalibrator, Sensing)

InertiaSensorCalibrator::InertiaSensorCalibrator()
{
  reset();

  p.timeFrame = 1500;
  p.gyroBiasProcessNoise = Vector2BH<>(0.05f, 0.05f);
  p.gyroBiasStandMeasurementNoise = Vector2BH<>(0.01f, 0.01f);
  p.gyroBiasWalkMeasurementNoise = Vector2BH<>(0.1f, 0.1f);
  p.accBiasProcessNoise = Vector3BH<>(0.01f, 0.01f, 0.01f);
  p.accBiasStandMeasurementNoise = Vector3BH<>(0.1f, 0.1f, 0.1f);
  p.accBiasWalkMeasurementNoise = Vector3BH<>(1.f, 1.f, 1.f);
}

void InertiaSensorCalibrator::reset()
{
  lastTime = 0;
  lastMotion = MotionRequest::specialAction;
  calibrated = false;
  collectionStartTime = 0;
  cleanCollectionStartTime = 0;
  safeGyro = Vector2BH<>();
  safeAcc = Vector3BH<>(0.f, 0.f, -9.80665f);
  inertiaSensorDrops = 1000;
}

void InertiaSensorCalibrator::update(InertiaSensorData& inertiaSensorData,
        const InspectedInertiaSensorData& theInspectedInertiaSensorData,
        const FrameInfo& theFrameInfo,
        const RobotModel& theRobotModel,
        const GroundContactState& theGroundContactState,
        const MotionSelection& theMotionSelection,
        const MotionInfo& theMotionInfo,
        const WalkingEngineOutput& theWalkingEngineOutput,
        const DamageConfiguration& theDamageConfiguration,
        bool penalized)
{
//  MODIFY("module:InertiaSensorCalibrator:parameters", p);

  // frame time check
  if(theFrameInfo.time <= lastTime)
  {
    if(theFrameInfo.time == lastTime)
      return; // done!
    reset();
  }

  // detect changes in joint calibration

#ifndef RELEASE
  bool jointCalibrationChanged = false;
  for(int i = JointData::LHipYawPitch; i <= JointData::LAnkleRoll; ++i)
    if(theJointCalibration.joints[i].offset != lastJointCalibration.joints[i].offset)
    {
      jointCalibrationChanged = true;
      lastJointCalibration.joints[i].offset = theJointCalibration.joints[i].offset;
    }
  for(int i = JointData::RHipYawPitch; i <= JointData::RAnkleRoll; ++i)
    if(theJointCalibration.joints[i].offset != lastJointCalibration.joints[i].offset)
    {
      jointCalibrationChanged = true;
      lastJointCalibration.joints[i].offset = theJointCalibration.joints[i].offset;
    }
  if(jointCalibrationChanged)
    reset();
#endif

  // check for dropped sensor readings
  const Vector2BH<>& newGyro = theInspectedInertiaSensorData.gyro;
  const Vector3BH<>& newAcc = theInspectedInertiaSensorData.acc;
  if(theInspectedInertiaSensorData.acc.x == InertiaSensorData::off)
    ++inertiaSensorDrops;
  else
  {
    inertiaSensorDrops = 0;
    safeGyro = newGyro;
    safeAcc = newAcc;
  }

  // it's prediction time!
  if(lastTime && calibrated)
  {
    const float timeDiff = float(theFrameInfo.time - lastTime) * 0.001f; // in seconds
    accXBias.predict(0.f, sqrBH(p.accBiasProcessNoise.x * timeDiff));
    accYBias.predict(0.f, sqrBH(p.accBiasProcessNoise.y * timeDiff));
    accZBias.predict(0.f, sqrBH(p.accBiasProcessNoise.z * timeDiff));
    gyroXBias.predict(0.f, sqrBH(p.gyroBiasProcessNoise.x * timeDiff));
    gyroYBias.predict(0.f, sqrBH(p.gyroBiasProcessNoise.y * timeDiff));
  }

  // detect unstable stuff...
  const MotionRequest::Motion& currentMotion(theMotionSelection.targetMotion);
  bool unstable = false;
  if(currentMotion != lastMotion || // motion change
     currentMotion != theMotionInfo.motion ||  // interpolating
     (!theGroundContactState.contact && theDamageConfiguration.useGroundContactDetectionForSensorCalibration))
    unstable = true;
  else if(currentMotion == MotionRequest::walk) // let's not recalibrate while walking
  //else if(currentMotion == MotionRequest::walk &&
          //(abs(theWalkingEngineOutput.speed.translation.y) > 10.f ||
           //abs(theWalkingEngineOutput.speed.translation.x) > 20.f ||
           //abs(theWalkingEngineOutput.speed.rotation) > 0.15f))
    unstable = true;
  else if(inertiaSensorDrops >= 2)
    unstable = true;
  else if(currentMotion != MotionRequest::walk && currentMotion != MotionRequest::stand &&
          !(currentMotion == MotionRequest::specialAction && theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::sitDownKeeper))
    unstable = true;
  else if(penalized)
    unstable = true;
  else if(accValues.getNumberOfEntries() >= accValues.getMaxEntries())
    unstable = true;

  // update cleanCollectionStartTime
  if(unstable)
    cleanCollectionStartTime = 0;
  else if(!cleanCollectionStartTime)
    cleanCollectionStartTime = theFrameInfo.time;

  // restart sensor value collecting?
  if(unstable || (currentMotion == MotionRequest::walk && theWalkingEngineOutput.positionInWalkCycle < lastPositionInWalkCycle) ||
     (currentMotion == MotionRequest::stand && theFrameInfo.time - collectionStartTime > 2000))
  {
    // add collection within the time frame to the collection buffer
    ASSERT(accValues.getNumberOfEntries() == gyroValues.getNumberOfEntries());
    if(cleanCollectionStartTime && theFrameInfo.time - cleanCollectionStartTime > p.timeFrame &&
       accValues.getNumberOfEntries())
    {
      ASSERT(collections.getNumberOfEntries() < collections.getMaxEntries());
      collections.add(Collection(accValues.getSum() / float(accValues.getNumberOfEntries()),
                                 gyroValues.getSum() / float(gyroValues.getNumberOfEntries()),
                                 collectionStartTime + (theFrameInfo.time - collectionStartTime) / 2, currentMotion));
    }

    // restart collecting
    accValues.init();
    gyroValues.init();
    collectionStartTime = 0;

    // look if there are any useful buffered collections
    for(int i = collections.getNumberOfEntries() - 1; i >= 0; --i)
    {
      const Collection& collection(collections.getEntry(i));
      if(theFrameInfo.time - collection.timeStamp < p.timeFrame)
        break;
      if(cleanCollectionStartTime && cleanCollectionStartTime < collection.timeStamp)
      {
        std::cout << "Updating calibration" << std::endl;
        // use this collection
        Vector3BH<>& accBiasMeasurementNoise = collection.motion == MotionRequest::stand ? p.accBiasStandMeasurementNoise : p.accBiasWalkMeasurementNoise;
        Vector2BH<>& gyroBiasMeasurementNoise = collection.motion == MotionRequest::stand ? p.gyroBiasStandMeasurementNoise : p.gyroBiasWalkMeasurementNoise;
        if(!calibrated)
        {
          calibrated = true;
          accXBias.init(collection.accAvg.x, sqrBH(accBiasMeasurementNoise.x));
          accYBias.init(collection.accAvg.y, sqrBH(accBiasMeasurementNoise.y));
          accZBias.init(collection.accAvg.z, sqrBH(accBiasMeasurementNoise.z));
          gyroXBias.init(collection.gyroAvg.x, sqrBH(gyroBiasMeasurementNoise.x));
          gyroYBias.init(collection.gyroAvg.y, sqrBH(gyroBiasMeasurementNoise.y));
        }
        else
        {
          accXBias.update(collection.accAvg.x, sqrBH(accBiasMeasurementNoise.x));
          accYBias.update(collection.accAvg.y, sqrBH(accBiasMeasurementNoise.y));
          accZBias.update(collection.accAvg.z, sqrBH(accBiasMeasurementNoise.z));
          gyroXBias.update(collection.gyroAvg.x, sqrBH(gyroBiasMeasurementNoise.x));
          gyroYBias.update(collection.gyroAvg.y, sqrBH(gyroBiasMeasurementNoise.y));
        }
        std::cout << "Collection values: "
          << collection.accAvg.x << "(" << sqrBH(accBiasMeasurementNoise.x) << ") "
          << collection.accAvg.y << "(" << sqrBH(accBiasMeasurementNoise.y) << ") "
          << collection.accAvg.z << "(" << sqrBH(accBiasMeasurementNoise.z) << ") "
          << collection.gyroAvg.x << "(" << sqrBH(gyroBiasMeasurementNoise.x) << ") "
          << collection.gyroAvg.y << "(" << sqrBH(gyroBiasMeasurementNoise.y) << ") "
          << std::endl;

        std::cout << "new BIASES: "
          << accXBias.value << "(" << accXBias.variance << ") "
          << accYBias.value << "(" << accYBias.variance << ") "
          << accZBias.value << "(" << accZBias.variance << ") "
          << gyroXBias.value << "(" << gyroXBias.variance << ") "
          << gyroYBias.value << "(" << gyroYBias.variance << ") "
          << std::endl;
      }
      collections.removeFirst();
    }
  }

  // collecting....
  if(!unstable)
  {
    // calculate rotation based on foot - torso transformation
    const Pose3DBH& footLeft(theRobotModel.limbs[MassCalibrationBH::footLeft]);
    const Pose3DBH& footRight(theRobotModel.limbs[MassCalibrationBH::footRight]);
    const Pose3DBH footLeftInvert(footLeft.invert());
    const Pose3DBH footRightInvert(footRight.invert());
    if(abs(footLeftInvert.translation.z - footRightInvert.translation.z) < 3.f/* magic number */)
    {
      // use average of the calculated rotation of each leg
      calculatedRotation = RotationMatrixBH(Vector3BH<>(
                                            (atan2(footLeftInvert.rotation.c1.z, footLeftInvert.rotation.c2.z) + atan2(footRightInvert.rotation.c1.z, footRightInvert.rotation.c2.z)) * 0.5f,
                                            (atan2(-footLeftInvert.rotation.c0.z, footLeftInvert.rotation.c2.z) + atan2(-footRightInvert.rotation.c0.z, footRightInvert.rotation.c2.z)) * 0.5f,
                                            0.f));
    }
    else if(footLeftInvert.translation.z > footRightInvert.translation.z)
    {
      // use left foot
      calculatedRotation = footLeftInvert.rotation;
    }
    else
    {
      // use right foot
      calculatedRotation = footRightInvert.rotation;
    }

    // calculate expected acceleration sensor reading
    Vector3BH<> accGravOnly(calculatedRotation.c0.z, calculatedRotation.c1.z, calculatedRotation.c2.z);
    accGravOnly *= -9.80665f;

    // add sensor reading to the collection
    ASSERT(accValues.getNumberOfEntries() < accValues.getMaxEntries());
    accValues.add(safeAcc - accGravOnly);
    gyroValues.add(safeGyro);
    if(!collectionStartTime)
      collectionStartTime = theFrameInfo.time;
  }

  // provide calibrated inertia readings
  inertiaSensorData.calibrated = calibrated;
  if(!calibrated || newGyro.x == SensorData::off)
  {
    inertiaSensorData.gyro.x = inertiaSensorData.gyro.y = InertiaSensorData::off;
    inertiaSensorData.acc.x = inertiaSensorData.acc.y = inertiaSensorData.acc.z = InertiaSensorData::off;
  }
  else
  {
    inertiaSensorData.gyro.x = newGyro.x - gyroXBias.value;
    inertiaSensorData.gyro.y = newGyro.y - gyroYBias.value;
    inertiaSensorData.acc.x = newAcc.x - accXBias.value;
    inertiaSensorData.acc.y = newAcc.y - accYBias.value;
    inertiaSensorData.acc.z = newAcc.z - accZBias.value;
  }

//  MODIFY("module:InertiaSensorCalibrator:calibrated", calibrated);
//  MODIFY("module:InertiaSensorCalibrator:gyroXBias", gyroXBias.value);
//  MODIFY("module:InertiaSensorCalibrator:gyroYBias", gyroYBias.value);
//  MODIFY("module:InertiaSensorCalibrator:accXBias", accXBias.value);
//  MODIFY("module:InertiaSensorCalibrator:accYBias", accYBias.value);
//  MODIFY("module:InertiaSensorCalibrator:accZBias", accZBias.value);

  // store some values for the next iteration
  lastTime = theFrameInfo.time;
  lastMotion = theMotionSelection.targetMotion;
  lastPositionInWalkCycle = theWalkingEngineOutput.positionInWalkCycle;
}
