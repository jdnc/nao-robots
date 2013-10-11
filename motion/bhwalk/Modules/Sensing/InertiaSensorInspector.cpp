/**
* @file InertiaSensorInspector.cpp
* Implementation of module InertiaSensorInspector.
* @author Colin Graf
*/

#include "InertiaSensorInspector.h"

//MAKE_MODULE(InertiaSensorInspector, Sensing)

InertiaSensorInspector::InertiaSensorInspector() : lastAcc(0.f, 0.f, -9.80665f), inertiaSensorDrops(1000)
{
  p.maxGyroOffset = Vector2BH<>(1.f, 1.f);
  p.maxAccOffset = Vector3BH<>(10.f, 10.f, 10.f);
}

void InertiaSensorInspector::update(InspectedInertiaSensorData& inertiaSensorData,
                                    const SensorData& theSensorData)
{
//  MODIFY("module:InertiaSensorInspector:parameters", p);

  // drop corrupted sensor readings
  Vector2BH<>& newGyro = inertiaSensorData.gyro;
  Vector3BH<>& newAcc = inertiaSensorData.acc;
  newGyro = Vector2BH<>(theSensorData.data[SensorData::gyroX], theSensorData.data[SensorData::gyroY]);
  newAcc = Vector3BH<>(theSensorData.data[SensorData::accX], theSensorData.data[SensorData::accY], theSensorData.data[SensorData::accZ]);
  newAcc *= 9.80665f; // strange unit => metric unit :)
  if(abs(newGyro.x - lastGyro.x) > p.maxGyroOffset.x ||
     abs(newGyro.y - lastGyro.y) > p.maxGyroOffset.y ||
     abs(newAcc.x - lastAcc.x) > p.maxAccOffset.x ||
     abs(newAcc.y - lastAcc.y) > p.maxAccOffset.y ||
     abs(newAcc.z - lastAcc.z) > p.maxAccOffset.z)
  {
    if(++inertiaSensorDrops > 3)
    {
      lastGyro = newGyro;
      lastAcc = newAcc;
    }
    newGyro.x = newGyro.y = newAcc.x = newAcc.y = newAcc.z = InertiaSensorData::off;
  }
  else
  {
    inertiaSensorDrops = 0;
    lastGyro = newGyro;
    lastAcc = newAcc;
  }
}
