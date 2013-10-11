#!/bin/bash

types="Vector3 Matrix2x2 Matrix3x3 Matrix3x2 Matrix2x3 RotationMatrix Pose3D Vector2 Range Pose2D RingBufferWithSum RingBuffer TorsoMatrix MassCalibration RobotDimensions"

for type in $types; do
  echo $type
  cp BHWalkModule.cpp BHWalkModule.cpp.bak.tmp
  cp BHWalkModule.h BHWalkModule.h.bak.tmp
  perl -pi -w -e "s/\b${type}\b/${type}BH/g;" *.h *.cpp */*.h */*.cpp */*/*.h */*/*.cpp
  perl -pi -w -e "s/${type}BH\.h/${type}\.h/g;" *.h *.cpp */*.h */*.cpp */*/*.h */*/*.cpp
  cp BHWalkModule.cpp.bak.tmp BHWalkModule.cpp
  cp BHWalkModule.h.bak.tmp BHWalkModule.h
done
