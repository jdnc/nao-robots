/**
* @file TorsoMatrixProvider.cpp
* Implementation of module TorsoMatrixProvider.
* @author Colin Graf
*/

#include "TorsoMatrixProvider.h"
//#include "Tools/Debugging/DebugDrawings.h"

//MAKE_MODULE(TorsoMatrixProvider, Sensing)

void TorsoMatrixProvider::update(TorsoMatrixBH& torsoMatrix,
        const FilteredSensorData& theFilteredSensorData,
        const RobotDimensionsBH& theRobotDimensions,
        const RobotModel& theRobotModel,
        const GroundContactState& theGroundContactState,
        const DamageConfiguration& theDamageConfiguration)
{
  // generate rotation matrix from measured angleX and angleY
  const Vector3BH<> axis((float) theFilteredSensorData.data[SensorData::angleX], (float) theFilteredSensorData.data[SensorData::angleY], 0);
  RotationMatrixBH torsoRotation(axis);

  // calculate "center of hip" position from left foot
  Pose3DBH fromLeftFoot(torsoRotation);
  fromLeftFoot.conc(theRobotModel.limbs[MassCalibrationBH::footLeft]);
  fromLeftFoot.translate(0, 0, (float) -theRobotDimensions.heightLeg5Joint);
  fromLeftFoot.translation *= -1.;
  fromLeftFoot.rotation = torsoRotation;

  // calculate "center of hip" position from right foot
  Pose3DBH fromRightFoot(torsoRotation);
  fromRightFoot.conc(theRobotModel.limbs[MassCalibrationBH::footRight]);
  fromRightFoot.translate(0, 0, (float) -theRobotDimensions.heightLeg5Joint);
  fromRightFoot.translation *= -1.;
  fromRightFoot.rotation = torsoRotation;

  // get foot z-rotations
  const Pose3DBH& leftFootInverse(theRobotModel.limbs[MassCalibrationBH::footLeft].invert());
  const Pose3DBH& rightFootInverse(theRobotModel.limbs[MassCalibrationBH::footRight].invert());
  const float leftFootZRotation = leftFootInverse.rotation.getZAngle();
  const float rightFootZRotation = rightFootInverse.rotation.getZAngle();

  // determine used foot
  const bool useLeft = fromLeftFoot.translation.z > fromRightFoot.translation.z;

  // calculate foot span
  const Vector3BH<> newFootSpan(fromRightFoot.translation - fromLeftFoot.translation);

  // and construct the matrix
  Pose3DBH newTorsoMatrix;
  newTorsoMatrix.translate(newFootSpan.x / (useLeft ? 2.f : -2.f), newFootSpan.y / (useLeft ? 2.f : -2.f), 0);
  newTorsoMatrix.conc(useLeft ? fromLeftFoot : fromRightFoot);

  // calculate torso offset
  if(torsoMatrix.translation.z != 0) // the last torso matrix should be valid
  {
    Pose3DBH& torsoOffset(torsoMatrix.offset);
    torsoOffset = torsoMatrix.invert();
    torsoOffset.translate(lastFootSpan.x / (useLeft ? 2.f : -2.f), lastFootSpan.y / (useLeft ? 2.f : -2.f), 0);
    torsoOffset.rotateZ(useLeft ? float(leftFootZRotation - lastLeftFootZRotation) : float(rightFootZRotation - lastRightFootZRotation));
    torsoOffset.translate(newFootSpan.x / (useLeft ? -2.f : 2.f), newFootSpan.y / (useLeft ? -2.f : 2.f), 0);
    torsoOffset.conc(newTorsoMatrix);
  }

  // adopt new matrix and footSpan
  (Pose3DBH&)torsoMatrix = newTorsoMatrix;
  lastLeftFootZRotation = leftFootZRotation;
  lastRightFootZRotation = rightFootZRotation;
  lastFootSpan = newFootSpan;

  // valid?
  torsoMatrix.isValid = (theGroundContactState.contact || !theDamageConfiguration.useGroundContactDetection);

  //
//  PLOT("module:TorsoMatrixProvider:footSpanX", newFootSpan.x);
//  PLOT("module:TorsoMatrixProvider:footSpanY", newFootSpan.y);
//  PLOT("module:TorsoMatrixProvider:footSpanZ", newFootSpan.z);
//
//  PLOT("module:TorsoMatrixProvider:torsoMatrixX", torsoMatrix.translation.x);
//  PLOT("module:TorsoMatrixProvider:torsoMatrixY", torsoMatrix.translation.y);
//  PLOT("module:TorsoMatrixProvider:torsoMatrixZ", torsoMatrix.translation.z);
}

/*
void TorsoMatrixProvider::update(FilteredOdometryOffset& odometryOffset)
{
  Pose2DBH odometryOffset;

  if(lastTorsoMatrix.translation.z != 0.)
  {
    Pose3DBH odometryOffset3D(lastTorsoMatrix);
    odometryOffset3D.conc(theTorsoMatrix.offset);
    odometryOffset3D.conc(theTorsoMatrix.invert());

    odometryOffset.translation.x = odometryOffset3D.translation.x;
    odometryOffset.translation.y = odometryOffset3D.translation.y;
    odometryOffset.rotation = odometryOffset3D.rotation.getZAngle();
  }

  PLOT("module:TorsoMatrixProvider:odometryOffsetX", odometryOffset.translation.x);
  PLOT("module:TorsoMatrixProvider:odometryOffsetY", odometryOffset.translation.y);
  PLOT("module:TorsoMatrixProvider:odometryOffsetRotation", toDegrees(odometryOffset.rotation));

  (Pose3DBH&)lastTorsoMatrix = theTorsoMatrix;
}
*/
void TorsoMatrixProvider::update(OdometryData& odometryData,
        const TorsoMatrixBH& theTorsoMatrix)
{
  Pose2DBH odometryOffset;

  if(lastTorsoMatrix.translation.z != 0.)
  {
    Pose3DBH odometryOffset3D(lastTorsoMatrix);
    odometryOffset3D.conc(theTorsoMatrix.offset);
    odometryOffset3D.conc(theTorsoMatrix.invert());

    odometryOffset.translation.x = odometryOffset3D.translation.x;
    odometryOffset.translation.y = odometryOffset3D.translation.y;
    odometryOffset.rotation = odometryOffset3D.rotation.getZAngle();
  }

//  PLOT("module:TorsoMatrixProvider:odometryOffsetX", odometryOffset.translation.x);
//  PLOT("module:TorsoMatrixProvider:odometryOffsetY", odometryOffset.translation.y);
//  PLOT("module:TorsoMatrixProvider:odometryOffsetRotation", toDegrees(odometryOffset.rotation));

  odometryData += odometryOffset;

  (Pose3DBH&)lastTorsoMatrix = theTorsoMatrix;
}

