/**
* @file RobotModelProvider.cpp
*
* This file implements a module that provides information about the current state of the robot's limbs.
*
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander H�rtl</a>
*/

#include "RobotModelProvider.h"
//#include "Tools/Debugging/DebugDrawings.h"
//#include "Tools/Debugging/DebugDrawings3D.h"


void RobotModelProvider::update(RobotModel& robotModel,
        const FilteredJointData& theFilteredJointData,
        const RobotDimensionsBH& theRobotDimensions,
        const MassCalibrationBH& theMassCalibration)
{
  robotModel.setJointData(theFilteredJointData, theRobotDimensions, theMassCalibration);

//  DECLARE_DEBUG_DRAWING3D("module:RobotModelProvider:massOffsets", "origin");
//  COMPLEX_DRAWING3D("module:RobotModelProvider:massOffsets",
//  {
//    for(int i = 0; i < MassCalibrationBH::numOfLimbs; ++i)
//    {
//      const Vector3BH<>& v(robotModel.limbs[i] * theMassCalibration.masses[i].offset);
//      SPHERE3D("module:RobotModelProvider:massOffsets", v.x, v.y, v.z, 5, ColorRGBA(200, 0, 0));
//    }
//  });
}


//MAKE_MODULE(RobotModelProvider, Sensing)
