/**
* @file WalkingEngine.cpp
* Implementation of a module that creates the walking motions
* @author Colin Graf
*/

#include <cstdio>

#include "WalkingEngine.h"
//#include "Tools/Debugging/DebugDrawings.h" // PLOT
//#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/InverseKinematic.h"
//#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Streams/InStreams.h"
//#include "Tools/Settings.h"
#include "Tools/Math/Matrix.h"
//#include "Platform/SoundPlayer.h"

#include <math/Geometry.h>
#include <math/Spline3D.h>

inline float saveAsinh(float xf)
{
  double x = xf; // yes, we need double here
#ifdef _MSC_VER
  return float(log(x + sqrt(x * x + 1.)));
#else
  return float(asinh(x));
#endif
}

inline float saveAcosh(float xf)
{
  //ASSERT(xf >= 1.f);
  double x = xf; // yes, we need double here
  if(x < 1.)
    return 0.000001f;
#ifdef WIN32
  x = log(x + sqrt(x * x - 1.));
#else
  x = acosh(x);
#endif
  if(x < 0.000001)
    return 0.000001f;
  return float(x);
}

#define DEBUG_BHWALK
#ifdef DEBUG_BHWALK
#define bhwalk_out std::cout
#else
#define bhwalk_out (*NullStream::NullInstance())
#endif


//MAKE_MODULE(WalkingEngine, Motion Control)

//PROCESS_WIDE_STORAGE(WalkingEngine) WalkingEngine::theInstance = 0;

WalkingEngine::WalkingEngine(const std::string &configDir) :
  configDir(configDir),
  emergencyShutOff(false), currentMotionType(stand),
  kickPlayer(configDir),
  testing(false), testingNextParameterSet(0),
//  optimizeStarted(false), optimizeStartTime(0),
  instable(true), beginOfStable(0), lastExecutedWalkingKick(WalkRequest::none),
  lastWalkingKickCompletion(0),
  walk_decides_finished_with_target_(false),
  time_walk_target_started_(0)
{
//  theInstance = this;
  observedPendulumPlayer.walkingEngine = this;

  // default parameters
  p.standBikeRefX = 20.f;
  p.standStandbyRefX = 3.f;
  p.standComPosition = Vector3BH<>(/*20.5f*/ 3.5f /*0.f*/, 50.f, /*259.f*/ /*261.5f*/ 258.0f);
  p.standBodyTilt = 0.f;
  p.standArmJointAngles = Vector2BH<>(0.2f, 0.f);

  p.standHardnessAnklePitch = 75;
  p.standHardnessAnkleRoll = 75;

  p.walkRefX = 15.f;
  p.walkRefXAtFullSpeedX = 7.f;
  p.walkRefY = 50.f;
  p.walkRefYAtFullSpeedX = 38.f;
  p.walkRefYAtFullSpeedY = 50.f;
  p.walkStepDuration = 500.f;
  p.walkStepDurationAtFullSpeedX = 480.f;
  p.walkStepDurationAtFullSpeedY = 440.f;
  p.walkHeight = Vector2BH<>(p.standComPosition.z, 300.f);
  p.walkArmRotation = 0.4f;
  p.walkRefXSoftLimit.min = -3.f;
  p.walkRefXSoftLimit.max = 5.f;
  p.walkRefXLimit.min = -15.f;
  p.walkRefXLimit.max = 13.f;
  p.walkRefYLimit.min = -3.f;
  p.walkRefYLimit.max = 3.f;
  p.walkRefYLimitAtFullSpeedX.min = -30.f;
  p.walkRefYLimitAtFullSpeedX.max = 30.f;
  p.walkLiftOffset = Vector3BH<>(0.f, -5.f, 17.f);
  p.walkLiftOffsetJerk = 0.f;
  p.walkLiftOffsetAtFullSpeedY = Vector3BH<>(0.f, -20.f, 25.f);
  p.walkLiftRotation = Vector3BH<>(-0.05f, -0.05f, 0.f);
  p.walkAntiLiftOffset = Vector3BH<>(0.f, 0.f, 2.3f);
  p.walkAntiLiftOffsetAtFullSpeedY = Vector3BH<>(0.f, 0.f, 2.3f);

  p.walkComBodyRotation = 0.05f;
  p.walkFadeInShape = Parameters::sine;

  p.kickComPosition = Vector3BH<>(20.f, 0.f, 245.f);
  p.kickX0Y = 1.f;
  p.kickHeight = Vector2BH<>(p.standComPosition.z, 300.f);

  p.speedMax = Pose2DBH(0.8f, 60.f * 2.f, 50.f);
  p.speedMaxMin = Pose2DBH(0.2f, 10.f, 0.f);
  p.speedMaxBackwards = 50 * 2.f;
  p.speedMaxChange = Pose2DBH(0.3f, 8.f, 20.f);

  p.observerMeasurementMode = Parameters::torsoMatrix;
  p.observerMeasurementDelay = 40.f;

  p.observerErrorMode = Parameters::mixed;
  p.observerProcessDeviation = Vector4f(0.1f, 0.1f, 3.f, 3.f);
  p.observerMeasurementDeviation = Vector2f(20.f, 20.f);
  p.observerMeasurementDeviationAtFullSpeedX = Vector2f(20.f, 20.f);
  p.observerMeasurementDeviationWhenInstable = Vector2f(20.f, 10.f);

  p.balance = true;
  p.balanceMinError = Vector3BH<>(0.f, 0.f, 0.f);
  p.balanceMaxError = Vector3BH<>(10.f, 10.f, 10.f);
  //p.balanceCom.x = PIDCorrector::Parameters(0.f, 0.01f, 0.f, 3.f);
  //p.balanceCom.y = PIDCorrector::Parameters(0.f, 0.01f, 0.f, 3.f);
  //p.balanceCom.z = PIDCorrector::Parameters(0.2f, 0.2f, 0.f, 3.f);
  p.balanceBodyRotation.x = PIDCorrector::Parameters(0.f, 0.0f, 0.f, 30.f);
  p.balanceBodyRotation.y = PIDCorrector::Parameters(0.f, 0.0f, 0.f, 30.f);
  p.balanceStepSize = Vector2BH<>(0.08f, -0.04f);
  p.balanceStepSizeWhenInstable = Vector2BH<>(0.16f, -0.06f);
  p.balanceStepSizeWhenPedantic = Vector2BH<>(0.04f, -0.02f);
  p.balanceStepSizeInterpolation = 0.1f;

  p.stabilizerOnThreshold = 200.f;
  p.stabilizerOffThreshold = 100.f;
  p.stabilizerDelay = 200;

  p.odometryUseTorsoMatrix = true;
  p.odometryScale = Pose2DBH(1.f, 1.2f, 1.f);
  p.odometryUpcomingScale = Pose2DBH(1.f, 1.8f, 1.2f);
  p.odometryUpcomingOffset = Pose2DBH(0.f, 0.f, 0.f);


  fallDownStateDetector = new FallDownStateDetector(configDir);
  this->init(BHWalkParameters());
}

void WalkingEngine::init(const BHWalkParameters &params)
{
  std::cout << "WalkingEngine::init()" << std::endl;
    // load parameters from config file
    InConfigMap massesStream(configDir + "/masses.cfg");
    if (massesStream.exists()) {
        massesStream >> theMassCalibration;
        //cout << theMassCalibration.masses[2].mass << endl;
    } else {
        cout << "Could not find masses.cfg!" << endl;
    }

    InConfigMap robotDimStream(configDir + "/robotDimensions.cfg");
    if (robotDimStream.exists()) {
        robotDimStream >> theRobotDimensions;
    } else {
        cout << "Could not find robotDims.cfg!" << endl;
    }

    InConfigMap jointCalibrateStream(configDir + "/jointCalibration.cfg");
    if (jointCalibrateStream.exists()) {
        jointCalibrateStream >> theJointCalibration;
    } else {
        cout << "Could not find jointCalibration.cfg!" << endl;
    }

    InConfigMap sensorCalibrateStream(configDir + "/sensorCalibration.cfg");
    if (sensorCalibrateStream.exists()) {
        sensorCalibrateStream >> theSensorCalibration;
    } else {
        cout << "Could not find sensorCalibration.cfg!" << endl;
    }

    InConfigMap hardnessStream(configDir + "/jointHardness.cfg");
    if (hardnessStream.exists()) {
        hardnessStream >> defaultHardnessData;
    } else {
        cout << "Could not find jointHardness.cfg!" << endl;
    }

  InConfigMap stream(configDir + "walking.cfg");
  if(stream.exists())
    stream >> p;
  else
  {
      cout << "Could not find walking.cfg!" << endl;
  }
  if (params.set) {
    std::cout << "Setting specified walk parameters" << std::endl;
    p.speedMax.translation.x = params.speedMax.translation.x;
    p.speedMax.translation.y = params.speedMax.translation.y;
    p.speedMax.rotation = params.speedMax.rotation;

    p.speedMaxBackwards = params.speedMaxBackwards;
    p.speedMaxChange.translation.x = params.speedMaxChange.translation.x;
    p.speedMaxChange.translation.y = params.speedMaxChange.translation.y;
    p.speedMaxChange.rotation = params.speedMaxChange.rotation;
  }
  p.computeContants();

  kickPlayer.load(configDir);

  //TODO: make the ground contact detector work
  //right now the ground contact detection is off - trick motion into thinking it's broken
  theDamageConfiguration.useGroundContactDetection = true;
  theDamageConfiguration.useGroundContactDetectionForLEDs = false;
  theDamageConfiguration.useGroundContactDetectionForSafeStates = false;
  theDamageConfiguration.useGroundContactDetectionForSensorCalibration = true;

#ifdef TARGET_SIM
  p.observerMeasurementDelay = 60.f;
#endif

  currentRefX = p.standStandbyRefX;
  balanceStepSize = p.balanceStepSize;
}

void WalkingEngine::update(bool penalized, bool abortWalk)
{

  //std::cout << p.speedMax.translation.y << std::endl;
    //get new joint, sensor and frame info data // now set in BHWalkModule to be whatever we have in the frame info
    //theFrameInfo.cycleTime = 0.01f;
    //theFrameInfo.time = theJointData.timeStamp = theSensorData.timeStamp = SystemCall::getCurrentSystemTime();

    //calibrate joints
    for(int i = 0; i < JointData::numOfJoints; ++i) {
        theJointData.angles[i] = theJointData.angles[i] * theJointCalibration.joints[i].sign - theJointCalibration.joints[i].offset;
    }
    theHeadJointRequest.tilt = theJointData.angles[JointData::HeadPitch];
    theHeadJointRequest.pan = theJointData.angles[JointData::HeadYaw];

    //calibrate sensors
    theSensorData.data[SensorData::gyroX] *= theSensorCalibration.gyroXGain / 1600;
    theSensorData.data[SensorData::gyroY] *= theSensorCalibration.gyroYGain / 1600;
    theSensorData.data[SensorData::accX] *= theSensorCalibration.accXGain;
    theSensorData.data[SensorData::accX] += theSensorCalibration.accXOffset;
    theSensorData.data[SensorData::accY] *= theSensorCalibration.accYGain;
    theSensorData.data[SensorData::accY] += theSensorCalibration.accYOffset;
    theSensorData.data[SensorData::accZ] *= theSensorCalibration.accZGain;
    theSensorData.data[SensorData::accZ] += theSensorCalibration.accZOffset;

    theSensorData.data[SensorData::fsrLFL] = ((theSensorData.data[SensorData::fsrLFL] + theSensorCalibration.fsrLFLOffset) * theSensorCalibration.fsrLFLGain);
    theSensorData.data[SensorData::fsrLFR] = ((theSensorData.data[SensorData::fsrLFR] + theSensorCalibration.fsrLFROffset) * theSensorCalibration.fsrLFRGain);
    theSensorData.data[SensorData::fsrLBL] = ((theSensorData.data[SensorData::fsrLBL] + theSensorCalibration.fsrLBLOffset) * theSensorCalibration.fsrLBLGain);
    theSensorData.data[SensorData::fsrLBR] = ((theSensorData.data[SensorData::fsrLBR] + theSensorCalibration.fsrLBROffset) * theSensorCalibration.fsrLBRGain);
    theSensorData.data[SensorData::fsrRFL] = ((theSensorData.data[SensorData::fsrRFL] + theSensorCalibration.fsrRFLOffset) * theSensorCalibration.fsrRFLGain);
    theSensorData.data[SensorData::fsrRFR] = ((theSensorData.data[SensorData::fsrRFR] + theSensorCalibration.fsrRFROffset) * theSensorCalibration.fsrRFRGain);
    theSensorData.data[SensorData::fsrRBL] = ((theSensorData.data[SensorData::fsrRBL] + theSensorCalibration.fsrRBLOffset) * theSensorCalibration.fsrRBLGain);
    theSensorData.data[SensorData::fsrRBR] = ((theSensorData.data[SensorData::fsrRBR] + theSensorCalibration.fsrRBROffset) * theSensorCalibration.fsrRBRGain);

    //filter joints
    jointFilter.update(theFilteredJointData, theJointData);
    //update the robot model - this computes the CoM
    robotModelProvider.update(theRobotModel, theFilteredJointData,
                              theRobotDimensions, theMassCalibration);
    inertiaSensorInspector.update(theInspectedInertiaSensorData, theSensorData);
    inertiaSensorCalibrator.update(theInertiaSensorData, theInspectedInertiaSensorData,
            theFrameInfo, theRobotModel, theGroundContactState, theMotionSelection, theMotionInfo,
            walkingEngineOutput, theDamageConfiguration,penalized);
    inertiaSensorFilter.update(theOrientationData, theInertiaSensorData, theSensorData, theRobotModel,
            theFrameInfo, theMotionInfo, walkingEngineOutput);
    sensorFilter.update(theFilteredSensorData, theInertiaSensorData, theSensorData, theOrientationData);

    fallDownStateDetector->update(theFallDownState, theFilteredSensorData, theFrameInfo, theInertiaSensorData);
    torsoMatrixProvider.update(theTorsoMatrix, theFilteredSensorData, theRobotDimensions, theRobotModel,
            theGroundContactState, theDamageConfiguration);


    static bool prevFinishedWithTarget = finishedWithTarget;
    finishedWithTarget = false;

    if (currentMotionType == stepping) {
      if (walk_decides_finished_with_target_ && (theMotionRequest.walkRequest.mode == WalkRequest::targetMode)) {
        Pose2DBH walkTarget = requestedWalkTarget;
        Pose2DBH upcomingOdometryOffset = observedPendulumPlayer.next.s - stepOffset * 0.5f; // == (observedPendulumPlayer.next.s - stepOffset) * 0.5f + observedPendulumPlayer.next.s * 0.5f
        //upcomingOdometryOffset -= observedPendulumPlayer.s * 0.5f;

        upcomingOdometryOffset.translation.x *= p.odometryUpcomingScale.translation.x;
        upcomingOdometryOffset.translation.y *= p.odometryUpcomingScale.translation.y;
        upcomingOdometryOffset.rotation *= p.odometryUpcomingScale.rotation;

        float sign = observedPendulumPlayer.supportLeg == left ? -1.f : 1.f;
        Pose2DBH up(p.odometryUpcomingOffset.rotation * sign, p.odometryUpcomingOffset.translation.x, p.odometryUpcomingOffset.translation.y * sign);
        upcomingOdometryOffset += up;

        walkTarget -= upcomingOdometryOffset;
        walkTarget -= up;

        if ((walkTarget.translation.x < finished_with_target_max_x_error_) &&
            (walkTarget.translation.y < finished_with_target_max_y_error_) &&
            (walkTarget.translation.y > finished_with_target_min_y_error_) &&
            (theFrameInfo.time > time_walk_target_started_ + 0.5 * 1000)) {
          ////finishedTarget = true;
          finishedWithTarget = true;
          theMotionRequest.motion = MotionRequest::stand;
        }
      }
    }
    if (finishedWithTarget)
      finished_with_target_count_++;
    else
      finished_with_target_count_ = 0;
    finishedWithTarget = (finished_with_target_count_ >= 5);
    if (prevFinishedWithTarget != finishedWithTarget) {
      std::cout << "walk: finishedWithTarget went from " << finishedWithTarget << " to " << prevFinishedWithTarget << " " << theFrameInfo.time << endl;
      prevFinishedWithTarget = finishedWithTarget;
    }

    motionSelector.update(theMotionSelection, theMotionRequest, walkingEngineOutput,
            theGroundContactState, theDamageConfiguration, theFrameInfo);
    if (abortWalk) {
      float *ratios = theMotionSelection.ratios;
      for (int i = 0; i < MotionRequest::numOfMotions; i++)
        ratios[i] = 0;
      ratios[MotionRequest::specialAction] = 1.0;
    }

    //motion selector surrogate
//    theMotionSelection.walkRequest = theMotionRequest.walkRequest;
//    theMotionSelection.targetMotion = theMotionRequest.motion;
//    for (int i = 0 ; i < MotionRequest::numOfMotions; i++) {
//        theMotionSelection.ratios[i] = 0;
//    }
//    theMotionSelection.ratios[theMotionSelection.targetMotion] = 1.0f;


    static bool calibrated = false;
    if (calibrated != theInertiaSensorData.calibrated) {
        bhwalk_out << "Calibration status changed to " << theInertiaSensorData.calibrated << endl;
    }
    calibrated = theInertiaSensorData.calibrated;

  if(theMotionSelection.ratios[MotionRequest::walk] > 0.f || theMotionSelection.ratios[MotionRequest::stand] > 0.f)
  {
    Vector2BH<> targetBalanceStepSize = theMotionRequest.walkRequest.pedantic ? p.balanceStepSizeWhenPedantic : p.balanceStepSize;
    Vector2BH<> step = targetBalanceStepSize - balanceStepSize;
    Vector2BH<> maxStep(std::abs(balanceStepSize.x - p.balanceStepSizeWhenPedantic.x) * p.balanceStepSizeInterpolation,
                      std::abs(balanceStepSize.y - p.balanceStepSizeWhenPedantic.y) * p.balanceStepSizeInterpolation);
    if(step.x > maxStep.x)
      step.x = maxStep.x;
    else if(step.x < -maxStep.x)
      step.x = -maxStep.x;
    if(step.y > maxStep.y)
      step.y = maxStep.y;
    else if(step.y < -maxStep.y)
      step.y = -maxStep.y;

    balanceStepSize += step;

    updateMotionRequest();
    updateObservedPendulumPlayer();
    computeMeasuredStance();
    computeExpectedStance();
    computeError();
    updatePendulumPlayer();
    updateKickPlayer();
    generateTargetStance();
    generateJointRequest();
    computeOdometryOffset();
    generateOutput(walkingEngineOutput);
  }
  else
  {
    currentMotionType = stand;
    if(theMotionSelection.ratios[MotionRequest::specialAction] >= 1.f)
      if(theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead || theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::sitDown)
        currentRefX = p.standStandbyRefX;
    generateDummyOutput(walkingEngineOutput);
  }

  //this is what motion combinator does more or less
  if (theMotionSelection.ratios[MotionRequest::walk] > .99f) {
    theOdometryData += walkingEngineOutput.odometryOffset;
  }

  theMotionInfo.motion = theMotionRequest.motion;
  theMotionInfo.isMotionStable = true;
  theMotionInfo.walkRequest = walkingEngineOutput.executedWalk;
  theMotionInfo.upcomingOdometryOffset = walkingEngineOutput.upcomingOdometryOffset;
  theMotionInfo.upcomingOdometryOffsetValid = walkingEngineOutput.upcomingOdometryOffsetValid;

  for(int i = 0; i < JointData::numOfJoints; ++i) {
      if (walkingEngineOutput.jointHardness.hardness[i] == HardnessData::useDefault) {
          walkingEngineOutput.jointHardness.hardness[i] = defaultHardnessData.hardness[i];
      }

      if(walkingEngineOutput.angles[i] == JointData::off) {
          joint_angles[i] = 0.0f;
          joint_hardnesses[i] = 0.0f; // hardness off
      } else {
          joint_angles[i] = (walkingEngineOutput.angles[i] + theJointCalibration.joints[i].offset) * float(theJointCalibration.joints[i].sign);
          joint_hardnesses[i] = float(walkingEngineOutput.jointHardness.hardness[i]) / 100.f;
      }
  }
}

void WalkingEngine::updateMotionRequest()
{
  if(theMotionRequest.motion == MotionRequest::walk)
  {
    if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
    {
      if(theMotionRequest.walkRequest.target != Pose2DBH()) {
        requestedWalkTarget = theMotionRequest.walkRequest.target;
        theMotionRequest.walkRequest.target = Pose2DBH();
      }
    }
    else
      requestedWalkTarget = theMotionRequest.walkRequest.speed; // just for sgnBH(requestedWalkTarget.translation.y)
  }

  // get requested motion state
  requestedMotionType = stand;
  if((theGroundContactState.contactSafe || !theDamageConfiguration.useGroundContactDetectionForSafeStates) && !walkingEngineOutput.enforceStand && theMotionSelection.ratios[MotionRequest::walk] > 0.999f && !instable) {
    if(theMotionRequest.motion == MotionRequest::walk)
    {
      if(theMotionRequest.walkRequest.kickType != WalkRequest::none && kickPlayer.isKickStandKick(theMotionRequest.walkRequest.kickType))
      {
        bool mirrored = kickPlayer.isKickMirrored(theMotionRequest.walkRequest.kickType);
        requestedMotionType = mirrored ? standLeft : standRight;
      }
      else
        requestedMotionType = stepping;
    }
  }

  // detect whether the walking engine changed modes
  static bool warned = false;
  msg = "";
  if (requestedMotionType != currentMotionType) {
      if (!warned) {
          bhwalk_out << "The walking engine is switching to "
                  << getName(requestedMotionType)
                  << "!" << " " << theFrameInfo.time << endl;
          msg = getName(requestedMotionType);
          if (instable) {
              std::cout << "Warning - the walk engine is set to stand because of stability issues" << " " << theFrameInfo.time << endl;
              msg = "unstable";
          }
      }
      warned = true;
  } else {
      warned = false;
  }
}

void WalkingEngine::reset() {
  currentMotionType = stand;
  SupportLeg supportLeg = SupportLeg(0);
  Vector2BH<> r;
  Vector2BH<> x0;
  Vector2BH<> k = p.walkK;
  StepType stepType = fromStand;
  supportLeg = left;
  r = Vector2BH<>(currentRefX, p.walkRefY);
  x0 = Vector2BH<>(0.f, -p.walkRefY);
  lastNextSupportLeg = supportLeg;
  lastSelectedSpeed = Pose2DBH();
  nextPendulumParameters.s = StepSize();
  observedPendulumPlayer.init(stepType, p.observerMeasurementDelay * -0.001f, supportLeg, r, x0, k, theFrameInfo.cycleTime);
  kickPlayer.stop();
}

void WalkingEngine::updateObservedPendulumPlayer()
{
  // motion update
  if(observedPendulumPlayer.isActive())
    observedPendulumPlayer.seek(theFrameInfo.cycleTime);

  // change motion type
  switch(currentMotionType)
  {
  case stand:
  case standLeft:
  case standRight:
    if(kickPlayer.isActive())
      break;

    if(requestedMotionType != currentMotionType)
    {
      SupportLeg supportLeg = SupportLeg(0);
      Vector2BH<> r;
      Vector2BH<> x0;
      Vector2BH<> k = p.walkK;
      StepType stepType = fromStand;
      switch(currentMotionType)
      {
      case standRight:
        ASSERT(false);
        supportLeg = right;
        r = Vector2BH<>(0.f, -(p.standComPosition.y - p.kickComPosition.y + p.kickX0Y));
        x0 = Vector2BH<>(currentRefX, p.kickX0Y);
        k = p.kickK;
        stepType = fromStandLeft;
        break;
      case standLeft:
        ASSERT(false);
        supportLeg = left;
        r = Vector2BH<>(0.f, p.standComPosition.y - p.kickComPosition.y + p.kickX0Y);
        x0 = Vector2BH<>(currentRefX, -p.kickX0Y);
        k = p.kickK;
        stepType = fromStandRight;
        break;
      case stand:
        if(requestedMotionType == standRight || (requestedMotionType == stepping && requestedWalkTarget.translation.y > 0.f))
        {
          supportLeg = left;
          r = Vector2BH<>(currentRefX, p.walkRefY);
          x0 = Vector2BH<>(0.f, -p.walkRefY);
        }
        else
        {
          supportLeg = right;
          r = Vector2BH<>(currentRefX, -p.walkRefY);
          x0 = Vector2BH<>(0.f, p.walkRefY);
        }
        break;
      default:
        ASSERT(false);
        break;
      }
      lastNextSupportLeg = supportLeg;
      lastSelectedSpeed = Pose2DBH();
      nextPendulumParameters.s = StepSize();
      observedPendulumPlayer.init(stepType, p.observerMeasurementDelay * -0.001f, supportLeg, r, x0, k, theFrameInfo.cycleTime);
      
      currentMotionType = stepping;
    }
    break;
  default:
    break;
  }
}

void WalkingEngine::computeMeasuredStance()
{
  switch(p.observerMeasurementMode)
  {
  case Parameters::robotModel:
  {
    Pose3DBH comToLeft = Pose3DBH(-theRobotModel.centerOfMass).conc(theRobotModel.limbs[MassCalibrationBH::footLeft]).translate(0.f, 0.f, -theRobotDimensions.heightLeg5Joint);
    Pose3DBH comToRight = Pose3DBH(-theRobotModel.centerOfMass).conc(theRobotModel.limbs[MassCalibrationBH::footRight]).translate(0.f, 0.f, -theRobotDimensions.heightLeg5Joint);
    Pose3DBH leftToCom = comToLeft.invert();
    Pose3DBH rightToCom = comToRight.invert();

    RotationMatrixBH rotationLeft(Vector3BH<>(
                                  atan2(leftToCom.rotation.c1.z, leftToCom.rotation.c2.z),
                                  atan2(-leftToCom.rotation.c0.z, leftToCom.rotation.c2.z),
                                  0.f));
    RotationMatrixBH rotationRight(Vector3BH<>(
                                   atan2(rightToCom.rotation.c1.z, rightToCom.rotation.c2.z),
                                   atan2(-rightToCom.rotation.c0.z, rightToCom.rotation.c2.z),
                                   0.f));

    RotationMatrixBH& bodyRotation = observedPendulumPlayer.supportLeg == left ? rotationLeft : rotationRight;
    // TODO: optimize

    measuredLeftToCom = -Pose3DBH(bodyRotation).conc(comToLeft).translation;
    measuredRightToCom = -Pose3DBH(bodyRotation).conc(comToRight).translation;
  }
  break;

  default:
    measuredLeftToCom = -Pose3DBH(theTorsoMatrix.rotation).translate(-theRobotModel.centerOfMass).conc(theRobotModel.limbs[MassCalibrationBH::footLeft]).translate(0.f, 0.f, -theRobotDimensions.heightLeg5Joint).translation;
    measuredRightToCom = -Pose3DBH(theTorsoMatrix.rotation).translate(-theRobotModel.centerOfMass).conc(theRobotModel.limbs[MassCalibrationBH::footRight]).translate(0.f, 0.f, -theRobotDimensions.heightLeg5Joint).translation;
    break;
  }

//  PLOT("module:WalkingEngine:measuredLeftToComX", measuredLeftToCom.x);
//  PLOT("module:WalkingEngine:measuredLeftToComY", measuredLeftToCom.y);
//  PLOT("module:WalkingEngine:measuredLeftToComZ", measuredLeftToCom.z);
//  PLOT("module:WalkingEngine:measuredRightToComX", measuredRightToCom.x);
//  PLOT("module:WalkingEngine:measuredRightToComY", measuredRightToCom.y);
//  PLOT("module:WalkingEngine:measuredRightToComZ", measuredRightToCom.z);
}

void WalkingEngine::computeExpectedStance()
{
  LegStance* expectedStance;
  if(legStances.getNumberOfEntries() == 0)
  {
    legStances.add();
    expectedStance = &legStances.getEntry(0);
    getStandStance(*expectedStance);
    stepOffset = StepSize();
  }
  else
  {
    int index = std::min(int(p.observerMeasurementDelay / 10.f - 0.5f), legStances.getNumberOfEntries() - 1);
    expectedStance = &legStances.getEntry(index);
    if(observedPendulumPlayer.isActive() && !observedPendulumPlayer.isLaunching())
      observedPendulumPlayer.getStance(*expectedStance, 0, 0, &stepOffset);
    else {
      stepOffset = StepSize();
    }
  }

  expectedLeftToCom = expectedStance->leftOriginToCom - expectedStance->leftOriginToFoot.translation;
  expectedRightToCom = expectedStance->rightOriginToCom - expectedStance->rightOriginToFoot.translation;

}

void WalkingEngine::computeError()
{

  static int startedWalkTime = 0;
  if((theGroundContactState.contactSafe || !theDamageConfiguration.useGroundContactDetectionForSafeStates) && theFallDownState.state != FallDownState::onGround && theFallDownState.state != FallDownState::falling)
  {
    leftError = measuredLeftToCom - expectedLeftToCom;
    rightError = measuredRightToCom - expectedRightToCom;
    //if (theMotionSelection.ratios[MotionRequest::specialAction] > 0.01) { // assume we're stable in our special actions
      //instability.add(0);
      //instable = false;
      ////std::cout << "stable from special action" << std::endl;
      //startedWalkTime = 0;
    //} else if(theMotionSelection.ratios[MotionRequest::walk] < 0.99f)
    if(theMotionSelection.ratios[MotionRequest::walk] < 0.99f)
    {
      instability.add(((leftError + rightError) * 0.5f).squareAbs());
      instable = true;
      //std::cout << "instable not walking" << std::endl;
      startedWalkTime = 0;
    }
    else if(kickPlayer.isActive())
    {
      lastWalkingKickCompletion = theFrameInfo.time;
      instability.add(0);
      //instability.add((observedPendulumPlayer.supportLeg == left ? leftError : rightError).squareAbs());
    }
    else
    {
      if ((startedWalkTime == 0) && !walkingEngineOutput.enforceStand)
        startedWalkTime = theFrameInfo.time;
      if (theFrameInfo.getTimeSince(startedWalkTime) < 500) { // assume stable at walk start
        instability.add(0);
        //std::cout << "assuming stable because of walk start" << std::endl;
      } else {
        instability.add(((leftError + rightError) * 0.5f).squareAbs());
        //std::cout << "adding error: " << ((leftError + rightError) * 0.5f).squareAbs() << std::endl;
      }
      float stabilizerOnThreshold = p.stabilizerOnThreshold;
      //float secondsSinceLastWalkKick = (theFrameInfo.time - lastWalkingKickCompletion) / 1000.0;
      //if (secondsSinceLastWalkKick < 2.0) // recent walk kick
        //stabilizerOnThreshold = 500;
      if(instability.getAverage() > stabilizerOnThreshold)
      {
        instable = true;
        beginOfStable = 0;
        //std::cout << "instable in walk: " << instability.getAverage() << " " << p.stabilizerOnThreshold << std::endl;
      }
      else if(instability.getAverage() < p.stabilizerOffThreshold)
      {
        if(!beginOfStable)
          beginOfStable = theFrameInfo.time;
        else if(theFrameInfo.getTimeSince(beginOfStable) > p.stabilizerDelay) {
          instable = false;
          //std::cout << "stable after wait" << std::endl;
        }
      }
    }
    for(int i = 0; i < 2; ++i)
    {
      Vector3BH<>& error = i == 0 ? leftError : rightError;
      for(int i = 0; i < 3; ++i)
      {
        if(error[i] > -p.balanceMinError[i] && error[i] < p.balanceMinError[i])
          error[i] = 0.f;
        else if(error[i] > p.balanceMinError[i])
          error[i] -= p.balanceMinError[i];
        else
          error[i] += p.balanceMinError[i];

        if(error[i] > p.balanceMaxError[i])
          error[i] = p.balanceMaxError[i];
        else if(error[i] < -p.balanceMaxError[i])
          error[i] = -p.balanceMaxError[i];
      }
    }
  }
  else
    leftError = rightError = Vector3BH<>();

  float errorX = 0.f;
  if(errorX != 0.f)
  {
    leftError.x = errorX;
    rightError.x = errorX;
  }
}

void WalkingEngine::updatePendulumPlayer()
{
  if(currentMotionType == stepping)
  {
    if(p.balance)
      observedPendulumPlayer.applyCorrection(leftError, rightError, theFrameInfo.cycleTime);

    computeExpectedStance(); // HACK

    pendulumPlayer = observedPendulumPlayer;
    pendulumPlayer.seek(p.observerMeasurementDelay * 0.001f);

    if(!pendulumPlayer.isActive())
    {
      currentRefX = pendulumPlayer.next.r.x;
      switch(pendulumPlayer.type)
      {
      case toStand:
        currentMotionType = stand;
        break;
      case toStandLeft:
        currentMotionType = standLeft;
        break;
      case toStandRight:
        currentMotionType = standRight;
        break;
      default:
        ASSERT(false);
        break;
      }
      //if(currentMotionType == requestedMotionType && (requestedMotionType == standLeft || requestedMotionType == standRight) && theMotionRequest.walkRequest.kickType != WalkRequest::none)
      //kickPlayer.init(theMotionRequest.walkRequest.kickType, theMotionRequest.walkRequest.kickBallPosition, theMotionRequest.walkRequest.kickTarget);
    }
  }
}

void WalkingEngine::updateKickPlayer()
{
  if(currentMotionType == stepping)
  {
    if(!kickPlayer.isActive() && pendulumPlayer.kickType != WalkRequest::none)
    {
      kickPlayer.init(pendulumPlayer.kickType, theMotionRequest.walkRequest.kickBallPosition, theMotionRequest.walkRequest.kickTarget);
    }
    if(kickPlayer.isActive())
    {
      if(kickPlayer.getType() != pendulumPlayer.kickType)
        kickPlayer.stop();
      else
      {
        float length = kickPlayer.getLength();
        ASSERT(length >= 0.f);
        float pos = length * (pendulumPlayer.t - pendulumPlayer.tb) / (pendulumPlayer.te - pendulumPlayer.tb);
        kickPlayer.seek(std::max(pos - kickPlayer.getCurrentPosition(), 0.f));
      }
    }
  }
  else
  {
    if(kickPlayer.isActive())
      kickPlayer.seek(theFrameInfo.cycleTime);
    else if(theMotionRequest.walkRequest.kickType != WalkRequest::none && currentMotionType == requestedMotionType && (requestedMotionType == standLeft || requestedMotionType == standRight)) {
      kickPlayer.init(theMotionRequest.walkRequest.kickType, theMotionRequest.walkRequest.kickBallPosition, theMotionRequest.walkRequest.kickTarget);
    }
  }
}

void WalkingEngine::generateTargetStance()
{
  targetStance.headJointAngles[0] = theHeadJointRequest.pan;
  targetStance.headJointAngles[1] = theHeadJointRequest.tilt;

  float leftArmAngle = 0.f, rightArmAngle = 0.f;
  if(currentMotionType == stepping)
    pendulumPlayer.getStance(targetStance, &leftArmAngle, &rightArmAngle, 0);
  else
    getStandStance(targetStance);

  float frac = 0;
  if (pendulumPlayer.type == fromStand) {
    float t = (pendulumPlayer.t - pendulumPlayer.tb) / (pendulumPlayer.te - pendulumPlayer.tb);
    float q = 0.25;
    if (t < q)
      frac = t / q;
    else
      frac = 1.0 - (t - q) / (1.0 - q);
    //std::cout << "from stand: " << frac << std::endl;
    if (frac < 0)
      frac = 0;
    if (frac > 1)
      frac = 1;
    float adj = 20 * frac;
    if (pendulumPlayer.supportLeg == left)
      adj *= -1;
    targetStance.leftOriginToFoot.translation.y += adj;
    targetStance.rightOriginToFoot.translation.y += adj;
    //std::cout << "adj: " << adj << std::endl;
  }

  // set arm angles
  targetStance.leftArmJointAngles[0] = theJointData.angles[JointData::LShoulderPitch];
  targetStance.leftArmJointAngles[1] = theJointData.angles[JointData::LShoulderRoll];
  targetStance.leftArmJointAngles[2] = theJointData.angles[JointData::LElbowYaw];
  targetStance.leftArmJointAngles[3] = theJointData.angles[JointData::LElbowRoll];
  targetStance.rightArmJointAngles[0] = theJointData.angles[JointData::RShoulderPitch];
  targetStance.rightArmJointAngles[1] = theJointData.angles[JointData::RShoulderRoll];
  targetStance.rightArmJointAngles[2] = theJointData.angles[JointData::RElbowYaw];
  targetStance.rightArmJointAngles[3] = theJointData.angles[JointData::RElbowRoll];
  //targetStance.leftArmJointAngles[0] = pi / 180.0 * -116;
  //targetStance.leftArmJointAngles[1] = pi / 180.0 * 8;
  //targetStance.leftArmJointAngles[2] = pi / 180.0 * 25;
  //targetStance.leftArmJointAngles[3] = pi / 180.0 * -53;
  //targetStance.rightArmJointAngles[0] = pi / 180.0 * -116;
  //targetStance.rightArmJointAngles[1] = pi / 180.0 * 8;
  //targetStance.rightArmJointAngles[2] = pi / 180.0 * 25;
  //targetStance.rightArmJointAngles[3] = pi / 180.0 * -53;

  // playing a kick motion!?
  if(kickPlayer.isActive())
  {
    kickPlayer.setParameters(theMotionRequest.walkRequest.kickBallPosition, theMotionRequest.walkRequest.kickTarget);
    kickPlayer.apply(targetStance, theMotionRequest.walkRequest.kickType);
  }

  legStances.add(targetStance);
}

void WalkingEngine::getStandStance(LegStance& stance) const
{
  ASSERT(currentMotionType == standLeft || currentMotionType == standRight || currentMotionType == stand);
  stance.leftOriginToFoot = Pose3DBH(Vector3BH<>(0.f, p.standComPosition.y, 0.f));
  stance.rightOriginToFoot = Pose3DBH(Vector3BH<>(0.f, -p.standComPosition.y, 0.f));
  if (theMotionSelection.ratios[MotionRequest::walk] > 0.0) {
    float frac = theMotionSelection.ratios[MotionRequest::walk];
    float amt = frac * 0;
    stance.leftOriginToFoot.translation.y += amt;
    stance.rightOriginToFoot.translation.y += amt;
  }
  if(currentMotionType == stand)
    stance.leftOriginToCom = stance.rightOriginToCom = Vector3BH<>(p.standComPosition.x + currentRefX, 0.f, p.standComPosition.z);
  else
  {
    const float sign = currentMotionType == standLeft ? 1.f : -1.f;
    stance.leftOriginToCom =  stance.rightOriginToCom = Vector3BH<>(p.kickComPosition.x + currentRefX, (p.standComPosition.y - p.kickComPosition.y) * sign, p.kickComPosition.z);
  }
}

void WalkingEngine::generateJointRequest()
{
  Vector3BH<> correctedLeftOriginToCom = targetStance.leftOriginToCom;
  Vector3BH<> correctedRightOriginToCom = targetStance.rightOriginToCom;

  if(p.balance)
  {
    correctedLeftOriginToCom.x += leftControllerX.getCorrection(leftError.x, theFrameInfo.cycleTime, p.observerMeasurementDelay, p.balanceCom.x);
    correctedLeftOriginToCom.y += leftControllerY.getCorrection(leftError.y, theFrameInfo.cycleTime, p.observerMeasurementDelay, p.balanceCom.y);
    correctedLeftOriginToCom.z += leftControllerZ.getCorrection(leftError.z, theFrameInfo.cycleTime, p.observerMeasurementDelay, p.balanceCom.z);
    correctedRightOriginToCom.x += rightControllerX.getCorrection(rightError.x, theFrameInfo.cycleTime, p.observerMeasurementDelay, p.balanceCom.x);
    correctedRightOriginToCom.y += rightControllerY.getCorrection(rightError.y, theFrameInfo.cycleTime, p.observerMeasurementDelay, p.balanceCom.y);
    correctedRightOriginToCom.z += rightControllerZ.getCorrection(rightError.z, theFrameInfo.cycleTime, p.observerMeasurementDelay, p.balanceCom.z);
  }

  if(currentMotionType == stepping)
  {
    if(pendulumPlayer.supportLeg == left)
    {
      if(pendulumPlayer.l.z != 0.f)
        correctedRightOriginToCom.z -= p.walkLiftOffsetJerk;
    }
    else
    {
      if(pendulumPlayer.l.z != 0.f)
        correctedLeftOriginToCom.z -= p.walkLiftOffsetJerk;
    }
  }

  jointRequest.angles[JointData::HeadYaw] = targetStance.headJointAngles[0];
  jointRequest.angles[JointData::HeadPitch] = targetStance.headJointAngles[1];
  jointRequest.angles[JointData::LShoulderPitch] = targetStance.leftArmJointAngles[0];
  jointRequest.angles[JointData::LShoulderRoll] = targetStance.leftArmJointAngles[1];
  jointRequest.angles[JointData::LElbowYaw] = targetStance.leftArmJointAngles[2];
  jointRequest.angles[JointData::LElbowRoll] = targetStance.leftArmJointAngles[3];
  jointRequest.angles[JointData::RShoulderPitch] = targetStance.rightArmJointAngles[0];
  jointRequest.angles[JointData::RShoulderRoll] = targetStance.rightArmJointAngles[1];
  jointRequest.angles[JointData::RElbowYaw] = targetStance.rightArmJointAngles[2];
  jointRequest.angles[JointData::RElbowRoll] = targetStance.rightArmJointAngles[3];


  float bodyRotationX = 0.f, bodyRotationY = 0.f;
  if(p.balance)
  {
    bodyRotationY = atan(bodyControllerX.getCorrection((leftError.x + rightError.x) * 0.5f, theFrameInfo.cycleTime, p.observerMeasurementDelay, p.balanceBodyRotation.x) / p.walkHeight.x);
    bodyRotationX = atan(bodyControllerY.getCorrection((leftError.y + rightError.y) * 0.5f, theFrameInfo.cycleTime, p.observerMeasurementDelay, p.balanceBodyRotation.y) / p.walkHeight.y);
  }

  float additionalBodyRotation = (((targetStance.rightOriginToCom.y - targetStance.rightOriginToFoot.translation.y) - p.standComPosition.y) + ((targetStance.leftOriginToCom.y - targetStance.leftOriginToFoot.translation.y) + p.standComPosition.y)) * 0.5f;
  additionalBodyRotation *= 1.f / (22.5f - 50.f);
  additionalBodyRotation *= p.walkComBodyRotation;
  RotationMatrixBH bodyRotation(Vector3BH<>(additionalBodyRotation + bodyRotationX, bodyRotationY, 0.f));
  bodyRotation *= p.standBodyRotation;

  const Pose3DBH comToLeftOrigin = Pose3DBH(bodyRotation, correctedLeftOriginToCom).invert();
  const Pose3DBH comToRightOrigin = Pose3DBH(bodyRotation, correctedRightOriginToCom).invert();
  // TODO: optimize this by calculating the inverted left/rightOriginToCom pose directly
  const Pose3DBH comToLeftAnkle = Pose3DBH(comToLeftOrigin).conc(targetStance.leftOriginToFoot).translate(0.f, 0.f, theRobotDimensions.heightLeg5Joint);
  const Pose3DBH comToRightAnkle = Pose3DBH(comToRightOrigin).conc(targetStance.rightOriginToFoot).translate(0.f, 0.f, theRobotDimensions.heightLeg5Joint);
  const Vector3BH<> averageComToAnkle = (comToLeftAnkle.translation + comToRightAnkle.translation) * 0.5f;

  Vector3BH<> bodyToCom = this->bodyToCom;
  Vector3BH<> bodyToComOffset = lastAverageComToAnkle != Vector3BH<>() ? (averageComToAnkle - lastAverageComToAnkle) * 0.4f : Vector3BH<>();
  lastAverageComToAnkle = averageComToAnkle;
  bodyToCom += bodyToComOffset;

  Pose3DBH bodyToLeftAnkle(comToLeftAnkle.rotation, bodyToCom + comToLeftAnkle.translation);
  Pose3DBH bodyToRightAnkle(comToRightAnkle.rotation, bodyToCom + comToRightAnkle.translation);
  bool reachable = InverseKinematic::calcLegJoints(bodyToLeftAnkle, bodyToRightAnkle, jointRequest, theRobotDimensions, 0.5f);

  for(int i = 0; i < 7; ++i)
  {
    if(reachable || this->bodyToCom == Vector3BH<>())
    {
      if(reachable)
        this->bodyToCom = bodyToCom; // store the working bodyToCom offset

      RobotModel robotModel(jointRequest, theRobotDimensions, theMassCalibration);

      // TODO: improve this by not calculating the whole limb/mass model in each iteration

      Pose3DBH tmpComToLeftAnkle = Pose3DBH(-robotModel.centerOfMass).conc(robotModel.limbs[MassCalibrationBH::footLeft]);
      Pose3DBH tmpComToRightAnkle = Pose3DBH(-robotModel.centerOfMass).conc(robotModel.limbs[MassCalibrationBH::footRight]);

      Vector3BH<> tmpAverageComToAnkle = (tmpComToLeftAnkle.translation + tmpComToRightAnkle.translation) * 0.5f;
      bodyToComOffset = (averageComToAnkle - tmpAverageComToAnkle) * 1.3f;
    }
    else
    {
      bodyToCom = this->bodyToCom; // recover last working bodyToCom offset
      bodyToComOffset *= 0.5f; // reduce last bodyToComOffset
    }

    bodyToCom += bodyToComOffset;

    bodyToLeftAnkle.translation = bodyToCom + comToLeftAnkle.translation;
    bodyToRightAnkle.translation = bodyToCom + comToRightAnkle.translation;

    reachable = InverseKinematic::calcLegJoints(bodyToLeftAnkle, bodyToRightAnkle, jointRequest, theRobotDimensions, 0.5f);

    if(abs(bodyToComOffset.x) < 0.05 && abs(bodyToComOffset.y) < 0.05 && abs(bodyToComOffset.z) < 0.05)
      break;
  }
  /*
  #ifdef TARGET_SIM
  RobotModel robotModel(jointRequest, theRobotDimensions, theMassCalibration);
  Pose3DBH tmpComToLeftAnkle = Pose3DBH(-robotModel.centerOfMass).conc(robotModel.limbs[MassCalibrationBH::footLeft]);
  Pose3DBH tmpComToRightAnkle = Pose3DBH(-robotModel.centerOfMass).conc(robotModel.limbs[MassCalibrationBH::footRight]);
  ASSERT(abs(tmpComToLeftAnkle.translation.x - comToLeftAnkle.translation.x) < 0.1f && abs(tmpComToLeftAnkle.translation.y - comToLeftAnkle.translation.y) < 0.1f && abs(tmpComToLeftAnkle.translation.z - comToLeftAnkle.translation.z) < 0.1f);
  ASSERT(abs(tmpComToRightAnkle.translation.x - comToRightAnkle.translation.x) < 0.1f && abs(tmpComToRightAnkle.translation.y - comToRightAnkle.translation.y) < 0.1f && abs(tmpComToRightAnkle.translation.z - comToRightAnkle.translation.z) < 0.1f);
  #endif
  */

  jointRequest.jointHardness.hardness[JointData::LAnklePitch] = p.standHardnessAnklePitch;
  jointRequest.jointHardness.hardness[JointData::LAnkleRoll] = p.standHardnessAnkleRoll;
  jointRequest.jointHardness.hardness[JointData::RAnklePitch] = p.standHardnessAnklePitch;
  jointRequest.jointHardness.hardness[JointData::RAnkleRoll] = p.standHardnessAnkleRoll;

//  PLOT("module:WalkingEngine:leftTargetX", bodyToLeftAnkle.translation.x);
//  PLOT("module:WalkingEngine:leftTargetY", bodyToLeftAnkle.translation.y);
//  PLOT("module:WalkingEngine:leftTargetZ", bodyToLeftAnkle.translation.z);
//  PLOT("module:WalkingEngine:rightTargetX", bodyToRightAnkle.translation.x);
//  PLOT("module:WalkingEngine:rightTargetY", bodyToRightAnkle.translation.y);
//  PLOT("module:WalkingEngine:rightTargetZ", bodyToRightAnkle.translation.z);
}

void WalkingEngine::generateOutput(WalkingEngineOutput& walkingEngineOutput)
{
  if(observedPendulumPlayer.isActive())
  {
    const float stepDuration = (observedPendulumPlayer.te - observedPendulumPlayer.next.tb) * 2.f;
    walkingEngineOutput.speed.translation = Vector2BH<>(observedPendulumPlayer.s.translation.x + observedPendulumPlayer.next.s.translation.x, observedPendulumPlayer.s.translation.y + observedPendulumPlayer.next.s.translation.y) / stepDuration;
    walkingEngineOutput.speed.rotation = (observedPendulumPlayer.s.rotation + observedPendulumPlayer.next.s.rotation) / stepDuration;
  }
  else
    walkingEngineOutput.speed = Pose2DBH();

  walkingEngineOutput.odometryOffset = odometryOffset;
  walkingEngineOutput.upcomingOdometryOffset = upcomingOdometryOffset;
  walkingEngineOutput.upcomingOdometryOffsetValid = upcomingOdometryOffsetValid;
  walkingEngineOutput.isLeavingPossible = currentMotionType == stand;
  if(currentMotionType == stepping)
    walkingEngineOutput.positionInWalkCycle = 0.5f * ((observedPendulumPlayer.t - observedPendulumPlayer.tb) / (observedPendulumPlayer.te - observedPendulumPlayer.tb)) + (observedPendulumPlayer.supportLeg == left ? 0.5f : 0.f);
  else
    walkingEngineOutput.positionInWalkCycle = 0.f;
  walkingEngineOutput.enforceStand = false;
  walkingEngineOutput.instability = 0.f;
  walkingEngineOutput.executedWalk = theMotionRequest.walkRequest;
  walkingEngineOutput.executedWalk.kickType = kickPlayer.isActive() ? kickPlayer.getType() : WalkRequest::none;
  (JointRequestBH&)walkingEngineOutput = jointRequest;
}

void WalkingEngine::generateDummyOutput(WalkingEngineOutput& walkingEngineOutput)
{
  walkingEngineOutput.speed = Pose2DBH();
  walkingEngineOutput.odometryOffset = Pose2DBH();
  walkingEngineOutput.upcomingOdometryOffset = Pose2DBH();
  walkingEngineOutput.upcomingOdometryOffsetValid = true;
  walkingEngineOutput.isLeavingPossible = true;
  walkingEngineOutput.positionInWalkCycle = 0.f;
  walkingEngineOutput.enforceStand = false;
  walkingEngineOutput.instability = 0.f;
  walkingEngineOutput.executedWalk = WalkRequest();
  // leaving joint data untouched
}

void WalkingEngine::limitSpeedForTarget(float &req, float speedMax, float speedMaxChange, int dim) {
  float maxSpeedForTarget;
  // old way
  if (walk_decides_finished_with_target_) {
    maxSpeedForTarget = sqrt(2.f * abs(req) * speedMaxChange);
  } else {
    // new way - be aggressive, i.e. linear
    float factor = 0.5;
    //if (dim == 0)
      //factor = 0.75;
    if (dim == 1)
      factor = 1.0;
    maxSpeedForTarget = factor * abs(req);
  }
  // don't go too low
  if (maxSpeedForTarget < speedMaxChange)
    maxSpeedForTarget = speedMaxChange;

  // crop
  if (abs(req) > maxSpeedForTarget)
    req = req >= 0.f ? maxSpeedForTarget : -maxSpeedForTarget;
}

void WalkingEngine::generateNextStepSize(SupportLeg nextSupportLeg, StepType lastStepType, WalkRequest::KickType lastKickType, PendulumParameters& next)
{
  //finishedWithTarget = false;
  //bool finishedTarget = false;
  if(nextSupportLeg == lastNextSupportLeg)
    next = nextPendulumParameters;
  else
  {
    lastNextSupportLeg = nextSupportLeg;

    StepSize lastStepSize = next.s;
    //std::cout << "last step: " << next.s.translation.x << std::endl;

    const float sign = nextSupportLeg == right ? 1.f : -1.f;
    next.type = unknown;
    next.s = StepSize();
    next.l = Vector3BH<>(p.walkLiftOffset.x, p.walkLiftOffset.y * sign, p.walkLiftOffset.z);
    next.al = Vector3BH<>(p.walkAntiLiftOffset.x, p.walkAntiLiftOffset.y * sign, p.walkAntiLiftOffset.z);
    next.lRotation = Vector3BH<>();
    next.r = Vector2BH<>(p.walkRefX, p.walkRefY * (-sign));
    next.c = Vector2BH<>();
    next.k = p.walkK;
    next.te = p.te;
    next.tb = -p.te;
    next.kickType = WalkRequest::none;
    next.sXLimit.max = p.speedMax.translation.x * (1.1f * 0.5f);
    next.sXLimit.min = p.speedMaxBackwards * (-1.1f * 0.5f);
    next.rXLimit.max = next.r.x + p.walkRefXSoftLimit.max;
    next.rXLimit.min = next.r.x + p.walkRefXSoftLimit.min;
    next.rYLimit.max = p.walkRefY + p.walkRefYLimit.max;
    next.rYLimit.min = p.walkRefY + p.walkRefYLimit.min;

    switch(lastStepType)
    {
    case toStand:
      next.te = next.tb = 0.f;
      if(theMotionRequest.motion == MotionRequest::bike)
        next.r.x  = p.standBikeRefX;
      else
        next.r.x = p.walkRefX;
      next.x0 = Vector2BH<>(0.f, -next.r.y);
      next.xv0 = Vector2BH<>();
      next.xtb = Vector2BH<>(next.r.x, 0.f);
      next.xvtb = next.xv0;
      break;
    case toStandLeft:
    case toStandRight:
      ASSERT(false); // TODO!
      break;

    default:

      switch(requestedMotionType)
      {
      case stand:
        if(theMotionRequest.motion == MotionRequest::bike && (nextSupportLeg == left) == theMotionRequest.bikeRequest.mirror)
          break;
        if(abs(lastStepSize.translation.x) > p.speedMax.translation.x * 0.5f)
          break;
        next.type = toStand;
        next.te = p.te;
        next.tb = -p.te;
        break;

      case standLeft:
      case standRight:
        if((nextSupportLeg == left && requestedMotionType == standLeft) || (nextSupportLeg == right && requestedMotionType == standRight))
        {
          ASSERT(false); // TODO!
          next.type = requestedMotionType == standLeft ? toStandLeft : toStandRight;
          //next.r = Vector2BH<>(0.f, (p.standComPosition.y - p.kickComPosition.y + p.kickX0Y) * (-sign));
          //next.x0 = Vector2BH<>(0.f, p.kickX0Y * sign);
          //next.k = p.kickK;
        }
        break;
      default:
        ASSERT(next.type == unknown);
        break;
      }
      if(next.type == unknown)
      {
        if(!instable && theMotionRequest.walkRequest.kickType != WalkRequest::none && !kickPlayer.isKickStandKick(theMotionRequest.walkRequest.kickType) &&
           kickPlayer.isKickMirrored(theMotionRequest.walkRequest.kickType) == (nextSupportLeg == left) &&
           theMotionRequest.walkRequest.kickType != lastExecutedWalkingKick)
        {
          lastExecutedWalkingKick = theMotionRequest.walkRequest.kickType;
          next.kickType = theMotionRequest.walkRequest.kickType;
          // set params before asking step size
          kickPlayer.getKickPreStepSize(next.kickType, next.s.rotation, next.s.translation,theMotionRequest);
          next.r.x = kickPlayer.getKickRefX(next.kickType, next.r.x);
          next.rXLimit.max = next.r.x + p.walkRefXSoftLimit.max;
          next.rXLimit.min = next.r.x + p.walkRefXSoftLimit.min;
          next.rYLimit.max = p.walkRefY + p.walkRefYLimitAtFullSpeedX.max;
          next.rYLimit.min = p.walkRefY + p.walkRefYLimitAtFullSpeedX.min;
          float duration = kickPlayer.getKickDuration(next.kickType);
          if(duration != 0.f)
          {
            next.te = duration * 0.25f;
            next.tb = -next.te;
          }
        }
        else if(lastKickType != WalkRequest::none)
        {
          kickPlayer.getKickStepSize(lastKickType, next.s.rotation, next.s.translation,theMotionRequest);
          next.r.x = kickPlayer.getKickRefX(lastKickType, next.r.x);
          next.rXLimit.max = next.r.x + p.walkRefXSoftLimit.max;
          next.rXLimit.min = next.r.x + p.walkRefXSoftLimit.min;
          next.rYLimit.max = p.walkRefY + p.walkRefYLimitAtFullSpeedX.max;
          next.rYLimit.min = p.walkRefY + p.walkRefYLimitAtFullSpeedX.min;
          float duration = kickPlayer.getKickDuration(lastKickType);
          if(duration != 0.f)
          {
            next.te = duration * 0.25f;
            next.tb = -next.te;
          }
        }
        else if(instable)
        {
          // nothing
        }
        else
        {
          if(theMotionRequest.walkRequest.kickType == WalkRequest::none)
            lastExecutedWalkingKick = WalkRequest::none;

          // get requested walk target and speed
          Pose2DBH walkTarget = requestedWalkTarget;
          Pose2DBH requestedSpeed = theMotionRequest.walkRequest.speed;
          if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode) // remove upcoming odometry offset
          {

            Pose2DBH upcomingOdometryOffset = observedPendulumPlayer.next.s - stepOffset * 0.5f; // == (observedPendulumPlayer.next.s - stepOffset) * 0.5f + observedPendulumPlayer.next.s * 0.5f
            //upcomingOdometryOffset -= observedPendulumPlayer.s * 0.5f;

            upcomingOdometryOffset.translation.x *= p.odometryUpcomingScale.translation.x;
            upcomingOdometryOffset.translation.y *= p.odometryUpcomingScale.translation.y;
            upcomingOdometryOffset.rotation *= p.odometryUpcomingScale.rotation;

            float sign = observedPendulumPlayer.supportLeg == left ? -1.f : 1.f;
            Pose2DBH up(p.odometryUpcomingOffset.rotation * sign, p.odometryUpcomingOffset.translation.x, p.odometryUpcomingOffset.translation.y * sign);
            upcomingOdometryOffset += up;

            walkTarget -= upcomingOdometryOffset;
            walkTarget -= up;

            //if ((fabs(walkTarget.translation.x) < 30) && (fabs(walkTarget.translation.y) < 30)) {
              ////finishedTarget = true;
              //finishedWithTarget = true;
            //}

            requestedSpeed = Pose2DBH(walkTarget.rotation * 2.f / p.odometryUpcomingScale.rotation, walkTarget.translation.x * 2.f / p.odometryUpcomingScale.translation.x, walkTarget.translation.y * 2.f / p.odometryUpcomingScale.translation.y);
            //std::cout << "requestedSpeed: " << requestedSpeed.translation.x << " " << requestedSpeed.translation.y << std::endl;

            // x-speed clipping to handle limited deceleration
            //float maxSpeedForTargetX = sqrt(2.f * abs(requestedSpeed.translation.x) * p.speedMaxChangeX);
            //if(abs(requestedSpeed.translation.x) > maxSpeedForTargetX)
            //requestedSpeed.translation.x = requestedSpeed.translation.x >= 0.f ? maxSpeedForTargetX : -maxSpeedForTargetX;
          }
          else if(theMotionRequest.walkRequest.mode == WalkRequest::percentageSpeedMode)
          {
            requestedSpeed.rotation *= p.speedMax.rotation;
            requestedSpeed.translation.x *= (theMotionRequest.walkRequest.speed.translation.x >= 0.f ? p.speedMax.translation.x : p.speedMaxBackwards);
            requestedSpeed.translation.y *= p.speedMax.translation.y;
          }

          //std::cout << "requested speed: " << requestedSpeed.translation.x << " " << requestedSpeed.translation.y << " " << _180_pi * requestedSpeed.rotation << std::endl;

          // compute max speeds for the requested walk direction
          Pose2DBH maxSpeed(p.speedMax.rotation, requestedSpeed.translation.x < 0.f ? p.speedMaxBackwards : p.speedMax.translation.x, p.speedMax.translation.y);
          Vector3BH<> tmpSpeed(
            requestedSpeed.translation.x / (p.speedMaxMin.translation.x + maxSpeed.translation.x),
            requestedSpeed.translation.y / (p.speedMaxMin.translation.y + maxSpeed.translation.y),
            requestedSpeed.rotation / (p.speedMaxMin.rotation + maxSpeed.rotation));
          const float tmpSpeedAbs = tmpSpeed.abs();
          if(tmpSpeedAbs > 1.f)
          {
            tmpSpeed /= tmpSpeedAbs;
            tmpSpeed.x *= (p.speedMaxMin.translation.x + maxSpeed.translation.x);
            tmpSpeed.y *= (p.speedMaxMin.translation.y + maxSpeed.translation.y);
            tmpSpeed.z *= (p.speedMaxMin.rotation + maxSpeed.rotation);
            maxSpeed.translation.x = min(abs(tmpSpeed.x), maxSpeed.translation.x);
            maxSpeed.translation.y = min(abs(tmpSpeed.y), maxSpeed.translation.y);
            maxSpeed.rotation = min(abs(tmpSpeed.z), maxSpeed.rotation);
          }

          // x-speed clipping to handle limited deceleration
          if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
          {
            limitSpeedForTarget(requestedSpeed.translation.x,p.speedMax.translation.x,p.speedMaxChange.translation.x,0);
            limitSpeedForTarget(requestedSpeed.translation.y,p.speedMax.translation.y,p.speedMaxChange.translation.y,1);
            limitSpeedForTarget(requestedSpeed.rotation,p.speedMax.rotation,p.speedMaxChange.rotation,2);

            //float maxSpeedForTargetXOld = sqrt(2.f * abs(requestedSpeed.translation.x) * p.speedMaxChange.translation.x);
            //// sbarrett, let's do it differently
            ////float maxSpeedForTargetX = p.speedMax.translation.x;
            ////if (fabs(requestedSpeed.translation.x) < maxSpeedForTargetX * 2.0)
              ////maxSpeedForTargetX = maxSpeedForTargetX * (1.0 - (maxSpeedForTargetX * 2.0 - fabs(requestedSpeed.translation.x)) / (maxSpeedForTargetX * 2.0));
            //float maxLastSpeed = 20.f; // just a guess of what we want
            //float maxSpeedForTargetX = abs(requestedSpeed.translation.x) * 0.66;
            //if (maxSpeedForTargetX < maxLastSpeed)
              //maxSpeedForTargetX = maxLastSpeed;
            //std::cout << "constraining: " << requestedSpeed.translation.x << " " << maxSpeedForTargetXOld << " " << maxSpeedForTargetX << std::endl;
            //// end sbarrett change
            //if(abs(requestedSpeed.translation.x) > maxSpeedForTargetX)
              //requestedSpeed.translation.x = requestedSpeed.translation.x >= 0.f ? maxSpeedForTargetX : -maxSpeedForTargetX;

            //float maxSpeedForTargetY = sqrt(2.f * abs(requestedSpeed.translation.y) * p.speedMaxChange.translation.y);
            //if(abs(requestedSpeed.translation.y) > maxSpeedForTargetY)
              //requestedSpeed.translation.y = requestedSpeed.translation.y >= 0.f ? maxSpeedForTargetY : -maxSpeedForTargetY;

            //float maxSpeedForTargetR = sqrt(2.f * abs(requestedSpeed.rotation) * p.speedMaxChange.rotation);
            //if(abs(requestedSpeed.rotation) > maxSpeedForTargetR)
              //requestedSpeed.rotation = requestedSpeed.rotation >= 0.f ? maxSpeedForTargetR : -maxSpeedForTargetR;
          }
          //std::cout << "requestedSpeedv3: " << requestedSpeed.translation.x << " " << requestedSpeed.translation.y << std::endl;

          // max speed change clipping (y-only)
          // just clip y and r since x will be clipped by min/maxRX in computeRefZMP
          requestedSpeed.translation.y = RangeBH<>(lastSelectedSpeed.translation.y - p.speedMaxChange.translation.y, lastSelectedSpeed.translation.y + p.speedMaxChange.translation.y).limit(requestedSpeed.translation.y);
          requestedSpeed.rotation = RangeBH<>(lastSelectedSpeed.rotation - p.speedMaxChange.rotation, lastSelectedSpeed.rotation + p.speedMaxChange.rotation).limit(requestedSpeed.rotation);

          // clip requested walk speed to the computed max speeds
          if(abs(requestedSpeed.rotation) > maxSpeed.rotation)
            requestedSpeed.rotation = requestedSpeed.rotation > 0.f ? maxSpeed.rotation : -maxSpeed.rotation;
          if(abs(requestedSpeed.translation.x) > maxSpeed.translation.x)
            requestedSpeed.translation.x = requestedSpeed.translation.x > 0.f ? maxSpeed.translation.x : -maxSpeed.translation.x;
          if(abs(requestedSpeed.translation.y) > maxSpeed.translation.y)
            requestedSpeed.translation.y = requestedSpeed.translation.y > 0.f ? maxSpeed.translation.y : -maxSpeed.translation.y;
          
          //std::cout << "speed: " << requestedSpeed.translation.x << " " << p.speedMax.translation.x * theMotionRequest.walkRequest.speed.translation.x << std::endl;

          // clip requested walk speed to a target walk speed limit
          if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
          {
            requestedSpeed.translation.x = RangeBH<>(-p.speedMax.translation.x * theMotionRequest.walkRequest.speed.translation.x, p.speedMax.translation.x * theMotionRequest.walkRequest.speed.translation.x).limit(requestedSpeed.translation.x);
            requestedSpeed.translation.y = RangeBH<>(-p.speedMax.translation.y * theMotionRequest.walkRequest.speed.translation.y, p.speedMax.translation.y * theMotionRequest.walkRequest.speed.translation.y).limit(requestedSpeed.translation.y);
            requestedSpeed.rotation = RangeBH<>(-p.speedMax.rotation * theMotionRequest.walkRequest.speed.rotation, p.speedMax.rotation * theMotionRequest.walkRequest.speed.rotation).limit(requestedSpeed.rotation);
          }
          

          // generate step size from requested walk speed
          next.s = StepSize(requestedSpeed.rotation, requestedSpeed.translation.x * 0.5f, requestedSpeed.translation.y);

          // adjust step duration according to the actual desired step size
          {
            // do this before the "just move the outer foot" clipping
            const float accClippedSpeedX = RangeBH<>(lastSelectedSpeed.translation.x - p.speedMaxChange.translation.x, lastSelectedSpeed.translation.x + p.speedMaxChange.translation.x).limit(requestedSpeed.translation.x);
            const float accClippedStepSizeX = accClippedSpeedX * 0.5f;

            {
              const float xxSpeedFactor = (p.teAtFullSpeedX - p.te) / (p.speedMax.translation.x * 0.5f);
              const float yySpeedFactor = (p.teAtFullSpeedY - p.te) / p.speedMax.translation.y;
              next.te += abs(next.s.translation.y) * yySpeedFactor;
              next.te += abs(accClippedStepSizeX) * xxSpeedFactor;
              next.tb = -next.te;
            }

            {
              float xSpeedFactor = (p.walkRefXAtFullSpeedX -  p.walkRefX) / (p.speedMax.translation.x * 0.5f);
              next.r.x += abs(accClippedStepSizeX) * xSpeedFactor;
              next.rXLimit.max = next.r.x + p.walkRefXSoftLimit.max;
              next.rXLimit.min = next.r.x + p.walkRefXSoftLimit.min;
            }

            {
              float walkRefYLimitMax = p.walkRefYLimit.max;
              float walkRefYLimitMin = p.walkRefYLimit.min;
              {
                float xSpeedFactor = (p.walkRefYLimitAtFullSpeedX.max -  p.walkRefYLimit.max) / (p.speedMax.translation.x * 0.5f);
                walkRefYLimitMax += abs(accClippedStepSizeX) * xSpeedFactor;
              }
              {
                float xSpeedFactor = (p.walkRefYLimitAtFullSpeedX.min -  p.walkRefYLimit.min) / (p.speedMax.translation.x * 0.5f);
                walkRefYLimitMin += abs(accClippedStepSizeX) * xSpeedFactor;
              }

              float ySpeedFactor = (p.walkRefYAtFullSpeedY -  p.walkRefY) / p.speedMax.translation.y;
              float xSpeedFactor = (p.walkRefYAtFullSpeedX -  p.walkRefY) / (p.speedMax.translation.x * 0.5f);
              next.r.y += (abs(requestedSpeed.translation.y) * ySpeedFactor + abs(accClippedStepSizeX) * xSpeedFactor) * (-sign);
              next.rYLimit.max = abs(next.r.y) + walkRefYLimitMax;
              next.rYLimit.min = abs(next.r.y) + walkRefYLimitMin;
            }
          }

          // just move the outer foot, when walking sidewards or when rotating
          if((next.s.translation.y < 0.f && nextSupportLeg == left) || (next.s.translation.y > 0.f && nextSupportLeg != left))
            next.s.translation.y = 0.f;
          if((next.s.rotation < 0.f && nextSupportLeg == left) || (next.s.rotation > 0.f && nextSupportLeg != left))
            next.s.rotation = 0.f;
          
          //std::cout << "step size: " << next.s.translation.x << " ";
          // clip to walk target
          if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
          {
            if((next.s.translation.x > 0.f && walkTarget.translation.x > 0.f && next.s.translation.x * p.odometryUpcomingScale.translation.x > walkTarget.translation.x) || (next.s.translation.x < 0.f && walkTarget.translation.x < 0.f && next.s.translation.x * p.odometryUpcomingScale.translation.x < walkTarget.translation.x))
              next.s.translation.x = walkTarget.translation.x / p.odometryUpcomingScale.translation.x;
            if((next.s.translation.y > 0.f && walkTarget.translation.y > 0.f && next.s.translation.y * p.odometryUpcomingScale.translation.y > walkTarget.translation.y) || (next.s.translation.y < 0.f && walkTarget.translation.y < 0.f && next.s.translation.y * p.odometryUpcomingScale.translation.y < walkTarget.translation.y))
              next.s.translation.y = walkTarget.translation.y / p.odometryUpcomingScale.translation.y;
            if((next.s.rotation > 0.f && walkTarget.rotation > 0.f && next.s.rotation * p.odometryUpcomingScale.rotation > walkTarget.rotation) || (next.s.rotation < 0.f && walkTarget.rotation < 0.f && next.s.rotation * p.odometryUpcomingScale.rotation < walkTarget.rotation))
              next.s.rotation = walkTarget.rotation / p.odometryUpcomingScale.rotation;
          }
          //std::cout << next.s.translation.x << " | " << walkTarget.translation.x << std::endl;
        }
          
        //std::cout << "step size: " << next.s.translation.x << " " << next.s.translation.y << " " << _180_pi * next.s.rotation << std::endl;
        //std::cout << "step size: " << next.s.translation.x << std::endl;

        next.lRotation = Vector3BH<>(
                           p.walkLiftRotation.x * sign * fabs(next.s.translation.y) / p.speedMax.translation.y,
                           next.s.translation.x > 0.f ? (p.walkLiftRotation.y * next.s.translation.x / (p.speedMax.translation.x * 0.5f)) : 0,
                           p.walkLiftRotation.z * sign);

        {
          float xSpeedFactor = (p.walkLiftOffsetAtFullSpeedY.x - p.walkLiftOffset.x) / p.speedMax.translation.y;
          float ySpeedFactor = (p.walkLiftOffsetAtFullSpeedY.y - p.walkLiftOffset.y) / p.speedMax.translation.y;
          float zSpeedFactor = (p.walkLiftOffsetAtFullSpeedY.z - p.walkLiftOffset.z) / p.speedMax.translation.y;
          next.l.x += abs(next.s.translation.y) * xSpeedFactor;
          next.l.y += abs(next.s.translation.y) * ySpeedFactor * sign;
          next.l.z += abs(next.s.translation.y) * zSpeedFactor;
        }

        {
          float xSpeedFactor = (p.walkAntiLiftOffsetAtFullSpeedY.x - p.walkAntiLiftOffset.x) / p.speedMax.translation.y;
          float ySpeedFactor = (p.walkAntiLiftOffsetAtFullSpeedY.y - p.walkAntiLiftOffset.y) / p.speedMax.translation.y;
          float zSpeedFactor = (p.walkAntiLiftOffsetAtFullSpeedY.z - p.walkAntiLiftOffset.z) / p.speedMax.translation.y;
          next.al.x += abs(next.s.translation.y) * xSpeedFactor;
          next.al.y += abs(next.s.translation.y) * ySpeedFactor * sign;
          next.al.z += abs(next.s.translation.y) * zSpeedFactor;
        }
      }
/*
      if (finishedTarget) {
        std::cout << "FINISHED TARGET" << std::endl;
        next.type = toStand;
        next.te = p.te;
        next.tb = -p.te;
        next.s = StepSize();
        next.l = Vector3BH<>(p.walkLiftOffset.x, p.walkLiftOffset.y * sign, p.walkLiftOffset.z);
        next.al = Vector3BH<>(p.walkAntiLiftOffset.x, p.walkAntiLiftOffset.y * sign, p.walkAntiLiftOffset.z);
        next.lRotation = Vector3BH<>();
        next.r = Vector2BH<>(p.walkRefX, p.walkRefY * (-sign));
        next.c = Vector2BH<>();
      }
*/
      lastSelectedSpeed = Pose2DBH(lastStepSize.rotation + next.s.rotation, lastStepSize.translation.x + next.s.translation.x, lastStepSize.translation.y + next.s.translation.y);

      ASSERT(next.tb == -next.te);

      // next.r.y + next.x0.y * cosh(next.k * next.tb) = 0.f
      // => next.x0.y = - next.r.y / cosh(next.k * next.tb)
      next.x0 = Vector2BH<>(0.f, -next.r.y / cosh(next.k.y * next.tb));

      // next.xv0.x * sinh(next.k * next.tb) / next.k = next.s.translation.x * -0.5f
      // => next.xv0.x = next.s.translation.x * -0.5f * next.k / sinh(next.k * next.tb)
      next.xv0 = Vector2BH<>(next.s.translation.x * -0.5f * next.k.x / sinh(next.k.x * next.tb), 0.f);

      // next.r.y + next.x0.y * cosh(next.k * next.tb) = next.s.translation.y * -0.5f
      // => next.tb = -acosh((next.s.translation.y * -0.5f - next.r.y) / next.x0.y) / next.k
      next.tb = -saveAcosh((next.s.translation.y * -0.5f - next.r.y) / next.x0.y) / next.k.y;

      // next.r.x + next.xv0.x * sinh(next.k * next.tb) / k  = next.xtb.x
      next.xtb = Vector2BH<>(next.r.x + next.xv0.x * sinh(next.k.x * next.tb) / next.k.x, next.s.translation.y * -0.5f);

      // next.xvtb.x = next.xv0.x * cosh(next.k * next.tb)
      // next.xvtb.y = next.x0.y * next.k * sinh(next.k * next.tb)
      next.xvtb = Vector2BH<>(next.xv0.x * cosh(next.k.x * next.tb), next.x0.y * next.k.y * sinh(next.k.y * next.tb));

      next.originalRX = next.r.x;
    }
    nextPendulumParameters = next;
  }
}

void WalkingEngine::computeOdometryOffset()
{
  if(p.odometryUseTorsoMatrix)
  {
    // "measured" odometry
    if(lastTorsoMatrix.translation.z != 0.)
    {
      Pose3DBH odometryOffset3D(lastTorsoMatrix);
      odometryOffset3D.conc(theTorsoMatrix.offset);
      odometryOffset3D.conc(theTorsoMatrix.invert());
      odometryOffset.translation.x = odometryOffset3D.translation.x;
      odometryOffset.translation.y = odometryOffset3D.translation.y;
      odometryOffset.rotation = odometryOffset3D.rotation.getZAngle();
    }
    lastTorsoMatrix = theTorsoMatrix;
  }
  else
  {
    // calculated odometry
    if(observedPendulumPlayer.supportLeg == lastSupportLeg)
      odometryOffset = (stepOffset - lastStepOffset) * 0.5f;
    else
      odometryOffset = (stepOffset + observedPendulumPlayer.s * 2.f - lastStepOffset) * 0.5f; // == ((observedPendulumPlayer.s - lastStepOffset) + (stepOffset - (observedPendulumPlayer.s * -1f))) * 0.5f;
  }
#ifdef TARGET_SIM
  {
    Pose2DBH odometryOffset;
    if(observedPendulumPlayer.supportLeg == lastSupportLeg)
      odometryOffset = (stepOffset - lastStepOffset) * 0.5f;
    else
      odometryOffset = (stepOffset + observedPendulumPlayer.s * 2.f - lastStepOffset) * 0.5f; // == ((observedPendulumPlayer.s - lastStepOffset) + (stepOffset - (observedPendulumPlayer.s * -1f))) * 0.5f;
    PLOT("module:WalkingEngine:calculatedOdometryOffsetX", odometryOffset.translation.x);
    PLOT("module:WalkingEngine:calculatedOdometryOffsetY", odometryOffset.translation.y);
    PLOT("module:WalkingEngine:calculatedOdometryOffsetRotation", toDegrees(odometryOffset.rotation));
  }
#endif

  upcomingOdometryOffset = observedPendulumPlayer.next.s - stepOffset * 0.5f; // == (observedPendulumPlayer.next.s - stepOffset) * 0.5f + observedPendulumPlayer.next.s * 0.5f

  // HACK: somehow this improves the accuracy of the upcoming odometry offset for target walks (but i have no idea why)
  upcomingOdometryOffset -= (observedPendulumPlayer.s + observedPendulumPlayer.next.s) * 0.5f;

  float sign = observedPendulumPlayer.supportLeg == left ? -1.f : 1.f;
  Pose2DBH up(p.odometryUpcomingOffset.rotation * sign, p.odometryUpcomingOffset.translation.x, p.odometryUpcomingOffset.translation.y * sign);
  upcomingOdometryOffset += up;
  upcomingOdometryOffsetValid = observedPendulumPlayer.supportLeg == pendulumPlayer.supportLeg;
  if(!upcomingOdometryOffsetValid)
    upcomingOdometryOffset += pendulumPlayer.next.s;
  else
    upcomingOdometryOffsetValid = (observedPendulumPlayer.te - observedPendulumPlayer.t) > 0.040f;

  lastSupportLeg = observedPendulumPlayer.supportLeg;
  lastStepOffset = stepOffset;

  odometryOffset.translation.x *= p.odometryScale.translation.x;
  odometryOffset.translation.y *= p.odometryScale.translation.y;
  odometryOffset.rotation *= p.odometryScale.rotation;

  upcomingOdometryOffset.translation.x *= p.odometryUpcomingScale.translation.x;
  upcomingOdometryOffset.translation.y *= p.odometryUpcomingScale.translation.y;
  upcomingOdometryOffset.rotation *= p.odometryUpcomingScale.rotation;

  if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
    requestedWalkTarget -= odometryOffset;
}

//bool WalkingEngine::handleMessage(InMessage& message)
//{
////  return theInstance && theInstance->kickPlayer.handleMessage(message);
//}

void WalkingEngine::ObservedPendulumPlayer::init(StepType stepType, float t, SupportLeg supportLeg, const Vector2BH<>& r, const Vector2BH<>& x0, const Vector2BH<>& k, float deltaTime)
{
  Parameters& p = walkingEngine->p;

  active = true;
  launching = true;

  this->supportLeg = supportLeg;

  this->l = this->al = Vector3BH<>();
  this->s = StepSize();
  this->type = stepType;

  this->k = k;
  this->te = p.te;
  this->tb = -p.te;
  this->t = this->tb + t;
  this->xv0 = Vector2BH<>();
  this->x0 = x0;
  this->r = r;
  this->originalRX  = r.x;
  this->c = Vector2BH<>();
  this->sXLimit.max = p.speedMax.translation.x * (1.1f * 0.5f);
  this->sXLimit.min = p.speedMaxBackwards * (-1.1f * 0.5f);
  this->rXLimit.max = this->r.x + p.walkRefXSoftLimit.max;
  this->rXLimit.min = this->r.x + p.walkRefXSoftLimit.min;
  this->rYLimit.max = p.walkRefY + p.walkRefYLimit.max;
  this->rYLimit.min = p.walkRefY + p.walkRefYLimit.min;

  cov = Matrix4x4f(
          Vector4f(sqrBH(p.observerProcessDeviation[0]), 0.f, p.observerProcessDeviation[0] * p.observerProcessDeviation[2], 0.f),
          Vector4f(0.f, sqrBH(p.observerProcessDeviation[1]), 0.f, p.observerProcessDeviation[1] * p.observerProcessDeviation[3]),
          Vector4f(p.observerProcessDeviation[0] * p.observerProcessDeviation[2], 0.f, sqrBH(p.observerProcessDeviation[2]), 0.f),
          Vector4f(0.f, p.observerProcessDeviation[1] * p.observerProcessDeviation[3], 0.f, sqrBH(p.observerProcessDeviation[3])));

  generateNextStepSize();

  ASSERT(r.y != 0.f);
  computeSwapTimes(this->tb, 0.f, 0.f, 0.f);

  computeRefZmp(this->tb, r.x, 0.f, 0.f);
}

void WalkingEngine::PendulumPlayer::seek(float deltaT)
{
  t += deltaT;
  if(t >= 0.f)
  {
    launching = false;
    if(t >= te)
    {
      if(type == toStand || type == toStandLeft || type == toStandRight)
      {
        active = false;
        return;
      }

      float const xTeY = r.y + c.y * te + x0.y * cosh(k.y * te) + xv0.y * sinh(k.y * te) / k.y;
      float const xvTeY = c.y + k.y * x0.y * sinh(k.y * te) + xv0.y * cosh(k.y * te);
      float const xTeX = r.x + c.x * te + x0.x * cosh(k.x * te) + xv0.x * sinh(k.x * te) / k.x;
      float const xvTeX = c.x + k.x * x0.x * sinh(k.x * te) + xv0.x * cosh(k.x * te);

      supportLeg = supportLeg == left ? right : left;
      t = next.tb + (t - te);
      (PendulumParameters&)*this = next;
      generateNextStepSize();

      computeSwapTimes(tb, xTeY - s.translation.y, xvTeY, 0.f);
      computeRefZmp(tb, xTeX - s.translation.x, xvTeX, 0.f);
    }
  }
}

void WalkingEngine::ObservedPendulumPlayer::applyCorrection(const Vector3BH<>& leftError, const Vector3BH<>& rightError, float deltaTime)
{
  Parameters& p = walkingEngine->p;
  Vector3BH<> error;
  switch(p.observerErrorMode)
  {
  case Parameters::indirect:
    error = supportLeg != left ? leftError : rightError;
    break;
  case Parameters::direct:
    error = supportLeg == left ? leftError : rightError;
    break;
  case Parameters::mixed:
  {
    float x = (t - tb) / (te - tb);
    if(x < 0.5f)
      x = 0.f;
    else
      x = (x - 0.5f) * 2.f;
    if(supportLeg != left)
      x = 1.f - x;
    error = leftError * (1.f - x) + rightError * x;
  }
  default:
    error = (leftError + rightError) * 0.5f;
    break;
  }

  static const Matrix2x4f c(Vector2f(1, 0), Vector2f(0, 1), Vector2f(), Vector2f());
  static const Matrix4x2f cTransposed = c.transpose();
  static const Matrix4x4f a(Vector4f(1, 0, 0, 0), Vector4f(0, 1, 0, 0),
                            Vector4f(deltaTime, 0, 1, 0), Vector4f(0, deltaTime, 0, 1));
  static const Matrix4x4f aTransponsed = a.transpose();

  cov = a * cov * aTransponsed;

  for(int i = 0; i < 4; ++i)
    cov[i][i] += sqrBH(p.observerProcessDeviation[i]);

  Matrix2x2f covPlusSensorCov = c * cov * cTransposed;
  Vector2f observerMeasurementDeviation = p.observerMeasurementDeviation;
  if(walkingEngine->instable)
    observerMeasurementDeviation = p.observerMeasurementDeviationWhenInstable;
  else if(next.s.translation.x > 0.f)
  {
    observerMeasurementDeviation.x += (p.observerMeasurementDeviationAtFullSpeedX.x - p.observerMeasurementDeviation.x) * abs(next.s.translation.x) / (p.speedMax.translation.x * 0.5f);
    observerMeasurementDeviation.y += (p.observerMeasurementDeviationAtFullSpeedX.y - p.observerMeasurementDeviation.y) * abs(next.s.translation.x) / (p.speedMax.translation.x * 0.5f);
  }
  covPlusSensorCov[0][0] += sqrBH(observerMeasurementDeviation[0]);
  covPlusSensorCov[1][1] += sqrBH(observerMeasurementDeviation[1]);
  Matrix4x2f kalmanGain = cov * cTransposed * covPlusSensorCov.invert();
  Vector2f innovation(error.x, error.y);
  Vector4f correction = kalmanGain * innovation;
  cov -= kalmanGain * c * cov;

  // compute updated xt and xvt
  Vector2BH<> xt(
    r.x + this->c.x * t + x0.x * cosh(k.x * t) + xv0.x * sinh(k.x * t) / k.x + correction[0],
    r.y + this->c.y * t + x0.y * cosh(k.y * t) + xv0.y * sinh(k.y * t) / k.y + correction[1]);
  Vector2BH<> xvt(
    this->c.x + k.x * x0.x * sinh(k.x * t) + xv0.x * cosh(k.x * t) + correction[2],
    this->c.y + k.y * x0.y * sinh(k.y * t) + xv0.y * cosh(k.y * t) + correction[3]);

  computeSwapTimes(t, xt.y, xvt.y, error.y);
  computeRefZmp(t, xt.x, xvt.x, error.x);
}

void WalkingEngine::PendulumPlayer::generateNextStepSize()
{
  walkingEngine->generateNextStepSize(supportLeg == right ? left : right, type, kickType, next);
}



void WalkingEngine::PendulumPlayer::computeSwapTimes(float t, float xt, float xvt, float errory)
{
  if(te - t < 0.005f)
    return;

  switch(type)
  {
  case toStand:
  case toStandLeft:
  case toStandRight:
  case fromStand:
  case fromStandLeft:
  case fromStandRight:
  {
    float const xte = next.s.translation.y + next.xtb.y;
    float const xvte = next.xvtb.y;

    // r * 1 + c * te + x0 * cosh(k * te)     + xv0 * sinh(k * te) / k  = xte
    //       + c * 1  + x0 * k * sinh(k * te) + xv0 * cosh(k * te)      = xvte
    // r * 1 + c * t  + x0 * cosh(k * t)      + xv0 * sinh(k * t) / k   = xt
    //       + c * 1  + x0 * k * sinh(k * t)  + xv0 * cosh(k * t)       = xvt

    Matrix<4, 4> a(
      Vector<4>(1.f, 0.f, 1.f, 0.f),
      Vector<4>(te, 1.f, t, 1.f),
      Vector<4>(cosh(k.y * te), k.y * sinh(k.y * te), cosh(k.y * t), k.y * sinh(k.y * t)),
      Vector<4>(sinh(k.y * te) / k.y, cosh(k.y * te), sinh(k.y * t) / k.y, cosh(k.y * t)));
    Vector<4> b(xte, xvte, xt, xvt);

    Vector<4> x;
    if(!a.solve(b, x))
    {
      ASSERT(false);
      return;
    }

    r.y = x[0];
    c.y = x[1];
    x0.y = x[2];
    xv0.y = x[3];
  }
  return;
  default:
    break;
  }

  Parameters& p = walkingEngine->p;

  if(errory != 0.f && walkingEngine->balanceStepSize.y != 0.f && kickType == WalkRequest::none /*&& !walkingEngine->theMotionRequest.walkRequest.pedantic*/)
  {
    ASSERT(next.xv0.y == 0.f);
    float sy = next.xtb.y * -2.f;
    sy += errory * (walkingEngine->instable ? p.balanceStepSizeWhenInstable.y : walkingEngine->balanceStepSize.y);
    next.tb = -saveAcosh((sy * -0.5f - next.r.y) / next.x0.y) / next.k.y;
    next.xtb.y =  sy * -0.5f;
    next.xvtb.y =  next.x0.y * next.k.y * sinh(next.k.y * next.tb);
  }

  ASSERT(next.xv0.y == 0.f);
  //float const xte = next.s.translation.y + next.xtb.y;
  float const xvte = next.xvtb.y;

  //           x0.y * k * sinh(k * te) + xv0.y * cosh(k * te)     = xvte
  // r.y * 1 + x0.y * cosh(k * t)      + xv0.y * sinh(k * t) / k  = xt
  //           x0.y * k * sinh(k * t)  + xv0.y * cosh(k * t)      = xvt

  Matrix<3, 3> a(
    Vector<3>(0.f, 1.f, 0.f),
    Vector<3>(k.y * sinh(k.y * te), cosh(k.y * t), k.y * sinh(k.y * t)),
    Vector<3>(cosh(k.y * te), sinh(k.y * t) / k.y, cosh(k.y * t)));
  Vector<3> b(xvte, xt, xvt);

  Vector<3> x;
  if(!a.solve(b, x))
  {
    ASSERT(false);
    return;
  }

  r.y = x[0];
  c.y = 0.f;
  x0.y = x[1];
  xv0.y = x[2];

  float newXte = r.y  + x0.y * cosh(k.y * te) + xv0.y * sinh(k.y * te) / k.y;
  next.s.translation.y = newXte - next.xtb.y;
}

void WalkingEngine::PendulumPlayer::computeRefZmp(float t, float xt, float xvt, float errorx)
{
  if(te - t < 0.005f)
    return;

  switch(type)
  {
  case toStand:
  case toStandLeft:
  case toStandRight:
  {
    float const xte = next.s.translation.x + next.xtb.x;
    float const xvte = next.xvtb.x;

    // r * 1 + c * te + x0 * cosh(k * te)     + xv0 * sinh(k * te) / k  = xte
    //       + c * 1  + x0 * k * sinh(k * te) + xv0 * cosh(k * te)      = xvte
    // r * 1 + c * t  + x0 * cosh(k * t)      + xv0 * sinh(k * t) / k   = xt
    //       + c * 1  + x0 * k * sinh(k * t)  + xv0 * cosh(k * t)       = xvt

    Matrix<4, 4> a(
      Vector<4>(1.f, 0.f, 1.f, 0.f),
      Vector<4>(te, 1.f, t, 1.f),
      Vector<4>(cosh(k.x * te), k.x * sinh(k.x * te), cosh(k.x * t), k.x * sinh(k.x * t)),
      Vector<4>(sinh(k.x * te) / k.x, cosh(k.x * te), sinh(k.x * t) / k.x, cosh(k.x * t)));
    Vector<4> b(xte, xvte, xt, xvt);

    Vector<4> x;
    if(!a.solve(b, x))
    {
      ASSERT(false);
      return;
    }

    r.x = x[0];
    c.x = x[1];
    x0.x = x[2];
    xv0.x = x[3];
  }
  return;
  default:
    break;
  }

  Parameters& p = walkingEngine->p;
  if(errorx != 0.f && walkingEngine->balanceStepSize.x != 0.f && kickType == WalkRequest::none  /*&& !walkingEngine->theMotionRequest.walkRequest.pedantic */)
  {
    ASSERT(next.x0.x == 0.f);
    float sx = next.xv0.x * sinh(next.k.x * next.tb) / (-0.5f * next.k.x);
    sx += errorx * (walkingEngine->instable ? p.balanceStepSizeWhenInstable.x : walkingEngine->balanceStepSize.x);
    next.xv0.x = sx * -0.5f * next.k.x / sinh(next.k.x * next.tb);
    next.xtb.x = next.r.x + next.xv0.x * sinh(next.k.x * next.tb) / next.k.x;
    next.xvtb.x = next.xv0.x * cosh(next.k.x * next.tb);
  }


  ASSERT(next.x0.x == 0.f);
  //float const xte = next.s.translation.x + next.xtb.x;
  float const xvte = next.xvtb.x;

  //           x0.x * k * sinh(k * te) + xv0.x * cosh(k * te)     = xvte
  // r.x * 1 + x0.x * cosh(k * t)      + xv0.x * sinh(k * t) / k  = xt
  //           x0.x * k * sinh(k * t)  + xv0.x * cosh(k * t)      = xvt

  Matrix<3, 3> a(
    Vector<3>(0.f, 1.f, 0.f),
    Vector<3>(k.x * sinh(k.x * te), cosh(k.x * t), k.x * sinh(k.x * t)),
    Vector<3>(cosh(k.x * te), sinh(k.x * t) / k.x, cosh(k.x * t)));
  Vector<3> b(xvte, xt, xvt);

  Vector<3> x;
  if(!a.solve(b, x))
  {
    ASSERT(false);
    return;
  }

  r.x = x[0];
  c.x = 0.f;
  x0.x = x[1];
  xv0.x = x[2];

  float newXte = r.x +  x0.x * cosh(k.x * te) + xv0.x * sinh(k.x * te) / k.x;
  float newXvte = x0.x * k.x * sinh(k.x * te) + xv0.x * cosh(k.x * te);
  float newNextXvtb = newXvte;
  float newNextXv0 = newNextXvtb / cosh(next.k.x * next.tb);
  float newNextXtb = next.r.x + newNextXv0 * sinh(next.k.x * next.tb) / next.k.x;
  next.s.translation.x = newXte - newNextXtb;

  if(!rXLimit.isInside(r.x))
  {
    r.x = rXLimit.limit(r.x);

    // x0.x * cosh(k * t)      + xv0.x * sinh(k * t) / k  = xt - r.x
    // x0.x * k * sinh(k * t)  + xv0.x * cosh(k * t)      = xvt

    Matrix<2, 2> a(
      Vector<2>(cosh(k.x * t), k.x * sinh(k.x * t)),
      Vector<2>(sinh(k.x * t) / k.x, cosh(k.x * t)));
    Vector<2> b(xt - r.x, xvt);

    Vector<2> x;
    if(!a.solve(b, x))
    {
      ASSERT(false);
      return;
    }

    x0.x = x[0];
    xv0.x = x[1];

    float newXte = r.x +  x0.x * cosh(k.x * te) + xv0.x * sinh(k.x * te) / k.x;
    float newXvte = x0.x * k.x * sinh(k.x * te) + xv0.x * cosh(k.x * te);
    float newNextXvtb = newXvte;
    float newNextXv0 = newNextXvtb / cosh(next.k.x * next.tb);
    float newNextXtb = next.r.x + newNextXv0 * sinh(next.k.x * next.tb) / next.k.x;
    //if(kickType == WalkRequest::none)
    next.s.translation.x = newXte - newNextXtb;
    if(type == unknown)
    {
      next.xv0.x = newNextXv0;
      next.xtb.x = newNextXtb;
      next.xvtb.x = newNextXvtb;
    }
  }

  if(!sXLimit.isInside(next.s.translation.x)/* && kickType == WalkRequest::none*/)
  {
    next.s.translation.x = sXLimit.limit(next.s.translation.x);

    // r + x0 * cosh(k * t)      + xv0 * sinh(k * t) / k                                                                     = xt
    //     x0 * k * sinh(k * t)  + xv0 * cosh(k * t)                                                                         = xvt
    //     x0 * k * sinh(k * te) + xv0 * cosh(k * te)          - nx0 * nk * sinh(nk * ntb) - nxv0 * cosh(nk * ntb)           = 0      // <=> xvte = nxvtb
    // r + x0 * cosh(k * te)     + xv0 * sinh(k * te) / k - nr - nx0 * cosh(nk * ntb)      - nxv0 * sinh(nk * ntb) / nk - ns = 0      // <=> xte - nxtb = ns

    // nx0 = 0

    // r + x0 * cosh(k * t)      + xv0 * sinh(k * t) / k                                = xt
    //     x0 * k * sinh(k * t)  + xv0 * cosh(k * t)                                    = xvt
    //     x0 * k * sinh(k * te) + xv0 * cosh(k * te)      - nxv0 * cosh(nk * ntb)      = 0
    // r + x0 * cosh(k * te)     + xv0 * sinh(k * te) / k  - nxv0 * sinh(nk * ntb) / nk = ns + nr

    Matrix<4, 4> a(
      Vector<4>(1.f, 0.f, 0.f, 1.f),
      Vector<4>(cosh(k.x * t), k.x * sinh(k.x * t), k.x * sinh(k.x * te), cosh(k.x * te)),
      Vector<4>(sinh(k.x * t) / k.x, cosh(k.x * t), cosh(k.x * te), sinh(k.x * te) / k.x),
      Vector<4>(0.f, 0.f, -cosh(next.k.x * next.tb), -sinh(next.k.x * next.tb) / next.k.x));
    Vector<4> b(xt, xvt, 0, next.s.translation.x + next.r.x);

    Vector<4> x;
    if(!a.solve(b, x))
    {
      ASSERT(false);
      return;
    }

    r.x = x[0];
    x0.x = x[1];
    xv0.x = x[2];

    if(type == unknown)
    {
      next.xvtb.x = x0.x * k.x * sinh(k.x * te) + xv0.x * cosh(k.x * te);
      next.xv0.x = next.xvtb.x / cosh(next.k.x * next.tb);
      next.xtb.x = next.r.x + next.xv0.x * sinh(next.k.x * next.tb) / next.k.x;
    }
  }

  if(type == unknown)
  {
    Parameters& p = walkingEngine->p;
    rXLimit.max = originalRX + p.walkRefXLimit.max;
    if(rXLimit.max > p.walkRefXAtFullSpeedX + p.walkRefXLimit.max)
      rXLimit.max = p.walkRefXAtFullSpeedX + p.walkRefXLimit.max;
    rXLimit.min = originalRX + p.walkRefXLimit.min;
    type = normal;
  }
}

void WalkingEngine::PendulumPlayer::getStance(LegStance& stance, float* leftArmAngle, float* rightArmAngle, StepSize* stepOffset) const
{
  const float phase = (t - tb) / (te - tb);

  Parameters& p = walkingEngine->p;
  const float swingMoveFadeIn = phase < p.walkMovePhase.start ? 0.f :
                                phase > p.walkMovePhase.start + p.walkMovePhase.duration ? 1.f :
                                smoothShape((phase - p.walkMovePhase.start) / p.walkMovePhase.duration);
  const float swingMoveFadeOut = 1.f - swingMoveFadeIn;
  const float sl = phase < p.walkLiftPhase.start || phase > p.walkLiftPhase.start + p.walkLiftPhase.duration ? 0.f :
                   smoothShape((phase - p.walkLiftPhase.start) / p.walkLiftPhase.duration * 2.f);

  Vector3BH<> r(this->r.x + this->c.x * t, this->r.y + this->c.y * t, 0.f);
  Vector3BH<> refToCom(
    p.standComPosition.x + x0.x * cosh(k.x * t) + xv0.x * sinh(k.x * t) / k.x,
    x0.y * cosh(k.y * t) + xv0.y * sinh(k.y * t) / k.y,
    p.standComPosition.z);
  switch(type)
  {
  case toStandLeft:
  case toStandRight:
  case fromStandLeft:
  case fromStandRight:
  {
    const float ratio = smoothShape(type == toStandLeft || type == toStandRight ? 1.f - t / tb : 1.f - t / te);
    refToCom.z += ratio * (p.kickComPosition.z - p.standComPosition.z);
    refToCom.x += ratio * (p.kickComPosition.x - p.standComPosition.x);
  }
  break;
  default:
    refToCom += next.al * sl;
    break;
  }

  if(supportLeg == left)
  {
    const Vector3BH<> rightStepOffsetTranslation = next.l * sl - (next.s.translation + s.translation) * swingMoveFadeOut;

    Vector3BH<> rightStepOffsetRotation = next.lRotation * sl;
    rightStepOffsetRotation.z += next.s.rotation * swingMoveFadeIn;
    const Vector3BH<> leftStepOffsetRotation(0.f, 0.f, s.rotation * swingMoveFadeOut);

    stance.leftOriginToCom = refToCom + r;
    stance.leftOriginToFoot = Pose3DBH(RotationMatrixBH(leftStepOffsetRotation), Vector3BH<>(0.f, p.standComPosition.y, 0.f));

    stance.rightOriginToCom = refToCom + r - next.s.translation;
    stance.rightOriginToFoot = Pose3DBH(RotationMatrixBH(rightStepOffsetRotation), Vector3BH<>(0.f, -p.standComPosition.y, 0.f) + rightStepOffsetTranslation);

    if(leftArmAngle)
      *leftArmAngle = (next.s.translation.x * swingMoveFadeIn - s.translation.x * swingMoveFadeOut) / p.speedMax.translation.x * p.walkArmRotation;
    if(rightArmAngle)
      *rightArmAngle = (s.translation.x * swingMoveFadeOut - next.s.translation.x * swingMoveFadeIn) / p.speedMax.translation.x * p.walkArmRotation;
  }
  else
  {
    const Vector3BH<> leftStepOffsetTranslation = next.l * sl - (next.s.translation + s.translation) * swingMoveFadeOut;

    Vector3BH<> leftStepOffsetRotation = next.lRotation * sl;
    leftStepOffsetRotation.z += next.s.rotation * swingMoveFadeIn;
    const Vector3BH<> rightStepOffsetRotation(0.f, 0.f, s.rotation * swingMoveFadeOut);

    stance.rightOriginToCom = refToCom + r;
    stance.rightOriginToFoot = Pose3DBH(RotationMatrixBH(rightStepOffsetRotation), Vector3BH<>(0.f, -p.standComPosition.y, 0.f));

    stance.leftOriginToCom = refToCom + r - next.s.translation;
    stance.leftOriginToFoot = Pose3DBH(RotationMatrixBH(leftStepOffsetRotation), Vector3BH<>(0.f, p.standComPosition.y, 0.f) + leftStepOffsetTranslation);

    if(rightArmAngle)
      *rightArmAngle = (next.s.translation.x * swingMoveFadeIn - s.translation.x * swingMoveFadeOut) / p.speedMax.translation.x * p.walkArmRotation;
    if(leftArmAngle)
      *leftArmAngle = (s.translation.x * swingMoveFadeOut - next.s.translation.x * swingMoveFadeIn) / p.speedMax.translation.x * p.walkArmRotation;
  }

  if(stepOffset)
  {
    stepOffset->translation.x = next.s.translation.x * swingMoveFadeIn - s.translation.x * swingMoveFadeOut;
    stepOffset->translation.y = next.s.translation.y * swingMoveFadeIn - s.translation.y * swingMoveFadeOut;
    stepOffset->translation.z = 0.f;
    stepOffset->rotation = next.s.rotation * swingMoveFadeIn - s.rotation * swingMoveFadeOut;
  }
}

float WalkingEngine::PendulumPlayer::smoothShape(float r) const
{
  switch(walkingEngine->p.walkFadeInShape)
  {
  case WalkingEngine::Parameters::sine:
    return 0.5f - cos(r * pi) * 0.5f;
  case WalkingEngine::Parameters::sqrBH:
    if(r > 1.f)
      r = 2.f - r;
    return r < 0.5f ? 2.f * r * r : (4.f - 2.f * r) * r - 1.f;
  default:
    ASSERT(false);
    return 0;
  }
}

WalkingEngine::KickPlayer::KickPlayer(const std::string &configDir) : kick(0) {
  startKick = false;
  load(configDir);
}

void WalkingEngine::KickPlayer::load(const std::string &configDir) {
  ASSERT((WalkRequest::numOfKickTypes - 1) % 2 == 0);
  for(int i = 0; i < (WalkRequest::numOfKickTypes - 1) / 2; ++i)
  {
    std::string path = configDir + "Kicks/" + WalkRequest::getName(WalkRequest::KickType(i * 2 + 1)) + ".cfg";
    kicks[i].load(path.c_str());
  }
}

bool WalkingEngine::KickPlayer::isKickStandKick(WalkRequest::KickType type) const
{
  bool mirrored = (type - 1) % 2 != 0;
  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
  return kick.isStandKick();
}

void WalkingEngine::KickPlayer::getKickStepSize(WalkRequest::KickType type, float& rotation, Vector3BH<>& translation, const MotionRequest &theMotionRequest) const
{
  bool mirrored = (type - 1) % 2 != 0;
  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
  kick.getStepSize(rotation, translation);
  if(mirrored)
  {
    translation.y = -translation.y;
    rotation = -rotation;
  }
}

void WalkingEngine::KickPlayer::getKickPreStepSize(WalkRequest::KickType type, float& rotation, Vector3BH<>& translation, const MotionRequest &theMotionRequest) const
{
  bool mirrored = (type - 1) % 2 != 0;
  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
  kick.getPreStepSize(rotation, translation);

  if(mirrored)
  {
    translation.y = -translation.y;
    rotation = -rotation;
  }
  std::cout << "WalkingEngineKick starting: " << WalkRequest::getName(type) << std::endl;
  
  //float dy = 1 * theMotionRequest.walkRequest.kickBallPosition.y;
  //translation.y += dy;
  //std::cout << "changing by " << dy << " to " << translation.y;
  //float maxVal = -20;
  //float minVal = -50;
  //if (mirrored) {
    //float temp = minVal;
    //minVal = -maxVal;
    //maxVal = -temp;
  //}
  //translation.y = crop(translation.y,minVal,maxVal);
  //std::cout << " finally: " << translation.y << std::endl;
  
  //float dx = theMotionRequest.walkRequest.kickBallPosition.x;
  //if (dx > 80)
    //dx = 80;
  //translation.x += dx;
  //std::cout << "fwd: " << dx << " to " << translation.x << std::endl;
}

float WalkingEngine::KickPlayer::getKickDuration(WalkRequest::KickType type) const
{
  bool mirrored = (type - 1) % 2 != 0;
  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
  return kick.getDuration();
}

float WalkingEngine::KickPlayer::getKickRefX(WalkRequest::KickType type, float defaultValue) const
{
  bool mirrored = (type - 1) % 2 != 0;
  const WalkingEngineKick& kick = kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
  return kick.getRefX(defaultValue);
}

void WalkingEngine::KickPlayer::init(WalkRequest::KickType type, const Vector2BH<>& ballPosition, const Vector2BH<>& target)
{
  ASSERT(!kick);
  mirrored = (type - 1) % 2 != 0;
  this->type = type;
  kick = &kicks[mirrored ? (type - 2) / 2 : (type - 1) / 2];
  setParameters(ballPosition, target);
  startKick = true;
  kick->init();
}

void WalkingEngine::KickPlayer::seek(float deltaT)
{
  if(kick)
    if(!kick->seek(deltaT))
      kick = 0;
}

float WalkingEngine::KickPlayer::getLength() const
{
  if(kick)
    return kick->getLength();
  ASSERT(false);
  return -1.f;
}

float WalkingEngine::KickPlayer::getCurrentPosition() const
{
  if(kick)
    return kick->getCurrentPosition();
  ASSERT(false);
  return -1.f;
}

void WalkingEngine::KickPlayer::apply(Stance& stance, WalkRequest::KickType type)
{
  if(!kick)
    return;
  Vector3BH<> additionalFootRotation;
  Vector3BH<> additionFootTranslation;
  float additionHeadAngles[2];
  float additionLeftArmAngles[4];
  float additionRightArmAngles[4];

  for(int i = 0; i < 2; ++i)
    additionHeadAngles[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::headYaw + i), 0.f);
  for(int i = 0; i < 4; ++i)
  {
    additionLeftArmAngles[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::lShoulderPitch + i), 0.f);
    additionRightArmAngles[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::rShoulderPitch + i), 0.f);
  }
  for(int i = 0; i < 3; ++i)
  {
    additionFootTranslation[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::footTranslationX + i), 0.f);
    additionalFootRotation[i] = kick->getValue(WalkingEngineKick::Track(WalkingEngineKick::footRotationX + i), 0.f);
  }
  // begin sbarrett changes to walk kick
  /*float frac = kick->currentPosition / (float)kick->length;
  float liftTime = 0.075;
  float dx = 50;
  float dy = 30;
  float negdy = 30;
  float dz = 10;
  float timeStayFwd = 0.0;
  float timeMoveFwd = (1.0 - timeStayFwd) / 2.0;
  if ((frac < liftTime) || (frac > (1.0 - liftTime))) {
    float f = frac / liftTime;
    if (f > (1.0 - liftTime))
      f = (1.0 - frac) / liftTime;
    f = 0.5 - 0.5 * cos(M_PI * 2 * f);
    additionFootTranslation[0] = 0;
    additionFootTranslation[1] = 0;
    additionFootTranslation[2] = f * dz;
  } else {
    float f = (frac - liftTime) / (1.0 - 2 * liftTime);
    // x
    if (f < timeMoveFwd)
      additionFootTranslation[0] = dx * (0.5 - 0.5 * cos(M_PI * 2 * f * 0.5 / timeMoveFwd));
    else if (f < timeMoveFwd + timeStayFwd)
      additionFootTranslation[0] = dx;
    else
      additionFootTranslation[0] = dx * (0.5 + 0.5 * cos(M_PI * 2 * (f  - timeMoveFwd - timeStayFwd) * 0.5 / timeMoveFwd));
    // y
    additionFootTranslation[1] = (-cos(M_PI * 2 * f + M_PI * 0.5));
    //additionFootTranslation[1] = dy * (-cos(M_PI * 2 * f + M_PI * 0.5));
    if (additionFootTranslation[1] > 0)
      additionFootTranslation[1] *= dy;
    else
      additionFootTranslation[1] *= negdy;
    // z
    additionFootTranslation[2] = dz;
  }*/
  // the following should be able to change the stance leg, but it's currently untested
  //Vector3BH<> stanceTranslation(0,0,0);
  //(mirrored ? stance.leftOriginToFoot : stance.rightOriginToFoot).conc(Pose3DBH(stanceTranslation));
  // end sbarrett changes to walk kick

  // begin katie changes to walk kick
  float frac = kick->currentPosition / (float)kick->length;
  if (startKick && ((type == WalkRequest::left) || (type == WalkRequest::right) || (type == WalkRequest::stepForLeftKick) || (type == WalkRequest::stepForRightKick))) { // straight kick
    float dx = 50;
    float dy = 0;
    float dz = 10;
    float stancedx = -20;
    float liftTime = 0.44;
    float timeStayFwd = 0.1;
    float timeMoveFwd = 0.01;
    startKick = false;
    int pts = 8;
    double time[] = {0,0.01,liftTime,liftTime+timeMoveFwd,liftTime+timeMoveFwd+timeStayFwd,1-liftTime,0.99,1};
    double xs[] = {0,0,0,dx,dx,0,0,0};
    double ys[] = {0,0,0,dy,dy,0,0,0};
    double zs[] = {0,0,dz,dz,dz,dz,0,0};
    swingSpline.set(pts,time,xs,ys,zs,true);
    double stancexs[] = {0,0,0,stancedx,stancedx,0,0,0};
    double stanceys[] = {0,0,0,0,0,0,0,0};
    double stancezs[] = {0,0,0,0,0,0,0,0};
    stanceSpline.set(pts,time,stancexs,stanceys,stancezs,true);
  } else if (startKick && (type == WalkRequest::sidewardsLeft || type == WalkRequest::sidewardsRight)) { // side kick
    float dx = 50;
    float dy = 45;
    float dz = 10;
    float stancedx = -30;
    if (mirrored) { // ie, kicking leftward with right leg
      dx = 55;
      dy = 40;
      stancedx = -25;
    }
    float negdy = -30;
    float liftTime = 0.075;
    float timeStayFwd = 0.05;
    float timeRemain = 1-liftTime-timeStayFwd-liftTime;
    float timeMoveFwd = timeRemain*0.5;
    startKick = false;
    int pts = 8;
    double time[] = {0,0.01,liftTime,liftTime+timeMoveFwd,liftTime+timeMoveFwd+timeStayFwd,1-liftTime,0.99,1};
    double xs[] = {0,0,0,dx,dx,0,0,0};
    double ys[] = {0,0,0,dy,negdy,0,0,0};
    double zs[] = {0,0,dz,dz,dz,dz,0,0};
    swingSpline.set(pts,time,xs,ys,zs,true);
    double stancexs[] = {0,0,0,stancedx,stancedx,0,0,0};
    double stanceys[] = {0,0,0,0,0,0,0,0};
    double stancezs[] = {0,0,0,0,0,0,0,0};
    stanceSpline.set(pts,time,stancexs,stanceys,stancezs,true);
  } else if (startKick && (type == WalkRequest::angleLeft || type == WalkRequest::angleRight)) { // 45 degree kick
    float dx = 45;
    float dy = 50;
    float dz = 10;
    float stancedx = -10;
    float stancedy = 0;
    if (!mirrored) { // ie, kicking rightward  with left leg
      dx = 40;
      dy = 45;
      stancedx = -20;
    }
    float negdy = 0;
    float liftTime = 0.1;
    float sideTime = 0.1;
    float timeMoveFwd = 0.3;
    float timeStayForward = 0.1;
    startKick = false;
    int pts = 9;
    double time[] = {0,0.01,liftTime,liftTime+sideTime,liftTime+sideTime+timeMoveFwd,liftTime+sideTime+timeMoveFwd+timeStayForward,1-liftTime,0.99,1};
    double xs[] = {0,0,0,0,dx,dx,0,0,0};
    double ys[] = {0,0,0,dy,negdy,negdy,0,0,0};
    double zs[] = {0,0,dz,dz,dz,dz,dz,0,0};
    swingSpline.set(pts,time,xs,ys,zs,true);
    double stancexs[] = {0,0,0,0,stancedx,stancedx,0,0,0};
    double stanceys[] = {0,0,0,stancedy,0,0,0,0,0};
    double stancezs[] = {0,0,0,0,0,0,0,0,0};
    stanceSpline.set(pts,time,stancexs,stanceys,stancezs,true);
  }

  Vector3<float> swing;
  Vector3<float> stanceFoot;
  frac = crop(frac,0.0001,0.99999);
  swingSpline.calc(frac,swing);
  stanceSpline.calc(frac,stanceFoot);
  additionFootTranslation[0] = swing.x;
  additionFootTranslation[1] = swing.y;
  additionFootTranslation[2] = swing.z;
  //std::cout << "swing - x: " << swing.x << ", y: " << swing.y << ", z: " << swing.z << std::endl;

  // change the stance leg
  if (mirrored) {
    stanceFoot.y = -stanceFoot.y;
  }
  Vector3BH<> stanceTranslation(stanceFoot.x,stanceFoot.y,stanceFoot.z);
  (mirrored ? stance.leftOriginToFoot : stance.rightOriginToFoot).conc(Pose3DBH(stanceTranslation));
  // end katie changes to walk kick

  if(mirrored)
  {
    additionalFootRotation.x = -additionalFootRotation.x;
    additionalFootRotation.z = -additionalFootRotation.z;
    additionFootTranslation.y = -additionFootTranslation.y;

    for(unsigned int i = 0; i < sizeof(stance.leftArmJointAngles) / sizeof(*stance.leftArmJointAngles); ++i)
    {
      float tmp = additionLeftArmAngles[i];
      additionLeftArmAngles[i] = additionRightArmAngles[i];
      additionRightArmAngles[i] = tmp;
    }
    additionHeadAngles[0] = -additionHeadAngles[0];
  }

  (mirrored ? stance.rightOriginToFoot : stance.leftOriginToFoot).conc(Pose3DBH(RotationMatrixBH(additionalFootRotation), additionFootTranslation));
  //std::cout << "kick: " << WalkRequest::getName(type) << " " << additionFootTranslation.x << " " << additionFootTranslation.y << " " << additionFootTranslation.z << std::endl;
  for(int i = 0; i < 2; ++i)
    if(stance.headJointAngles[i] != JointData::off)
      stance.headJointAngles[i] += additionHeadAngles[i];
  for(int i = 0; i < 4; ++i)
  {
    stance.leftArmJointAngles[i] += additionLeftArmAngles[i];
    stance.rightArmJointAngles[i] += additionRightArmAngles[i];
  }
}

void WalkingEngine::KickPlayer::setParameters(const Vector2BH<>& ballPosition, const Vector2BH<>& target)
{
  if(!kick)
    return;
  if(mirrored)
    kick->setParameters(Vector2BH<>(ballPosition.x, -ballPosition.y), Vector2BH<>(target.x, -target.y));
  else
    kick->setParameters(ballPosition, target);
}

//bool WalkingEngine::KickPlayer::handleMessage(InMessage& message)
//{
//  if(message.getMessageID() == idWalkingEngineKick)
//  {
//    unsigned int id, size;
//    message.bin >> id >> size;
//    ASSERT(id < WalkRequest::numOfKickTypes);
//    char* buffer = new char[size + 1];
//    message.bin.read(buffer, size);
//    buffer[size] = '\0';
//    char filePath[256];
//    sprintf(filePath, "Kicks/%s.cfg", WalkRequest::getName(WalkRequest::KickType(id)));
//    if(kicks[(id - 1) / 2].load(filePath, buffer))
//    {
////      OUTPUT(idText, text, filePath << ": ok");
//    }
//    delete[] buffer;
//    return true;
//  }
//  else
//    return false;
//}
