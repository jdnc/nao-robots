/**
 * @file InverseKinematic.h
 * @author Alexander H�rtl
 * @author jeff
 */

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Math/Pose3D.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Tools/Range.h"


class InverseKinematic
{
public:
  /**
  * The method calculates the joint angles for the legs of the robot from a Pose3DBH for each leg
  * @param positionLeft The desired position (translation + rotation) of the left foots ankle point
  * @param positionRight The desired position (translation + rotation) of the right foots ankle point
  * @param jointData The instance of JointData where the resulting joints are written into
  * @param robotDimensions The Robot Dimensions needed for calculation
  * @param ratio The ratio of
  * @return Whether the target position was reachable or not (if the given target position is not reachable the computation proceeds using the closest reachable position near the target)
  */
  static bool calcLegJoints(const Pose3DBH& positionLeft, const Pose3DBH& positionRight, JointData& jointData, const RobotDimensionsBH& robotDimensions, float ratio = 0.5f)
  {
    bool reachable = true;
    if(!calcLegJoints(positionLeft, jointData, true, robotDimensions))
      reachable = false;
    if(!calcLegJoints(positionRight, jointData, false, robotDimensions))
      reachable = false;
    RangeBH<> clipping(0.0f, 1.0f);
    ratio = clipping.limit(ratio);
    // the hip joints of both legs must be equal, so it is computed as weighted mean and the foot positions are
    // recomputed with fixed joint0 and left open foot rotation (as possible failure)
    float joint0 = jointData.angles[JointData::LHipYawPitch] * ratio + jointData.angles[JointData::RHipYawPitch] * (1 - ratio);
    if(!calcLegJoints(positionLeft, jointData, joint0, true, robotDimensions))
      reachable = false;
    if(!calcLegJoints(positionRight, jointData, joint0, false, robotDimensions))
      reachable = false;
    return reachable;
  }

private:
  /**
  * The method calculates the joint angles of one leg of the robot from a Pose3DBH
  * @param position The desired position (translation + rotation) of the foots ankle point
  * @param jointData The instance of JointData where the resulting joints are written into
  * @param left Determines if the left or right leg is calculated
  * @param robotDimensions The Robot Dimensions needed for calculation
  * @return Whether the target position was reachable or not (if the given target position is not reachable the computation proceeds using the closest reachable position near the target)
  */
  static bool calcLegJoints(const Pose3DBH& position, JointData& jointData, bool left, const RobotDimensionsBH& robotDimensions)
  {
    Pose3DBH target(position);
    JointData::Joint firstJoint(left ? JointData::LHipYawPitch : JointData::RHipYawPitch);
    int sign(left ? -1 : 1);
    target.translation.y += (float) robotDimensions.lengthBetweenLegs / 2.f * sign; // translate to origin of leg
    // rotate by 45� around origin for Nao
    // calculating sqrt(2) is faster than calculating the resp. rotation matrix with getRotationX()
    static const float sqrt2_2 = sqrt(2.0f) * 0.5f;
    RotationMatrixBH rotationX_pi_4 = RotationMatrixBH(Vector3BH<>(1, 0, 0), Vector3BH<>(0, sqrt2_2, sqrt2_2 * sign), Vector3BH<>(0, sqrt2_2 * -sign, sqrt2_2));
    target.translation = rotationX_pi_4 * target.translation;
    target.rotation = rotationX_pi_4 * target.rotation;

    target = target.invert(); // invert pose to get position of body relative to foot

    // use geometrical solution to compute last three joints
    float length = target.translation.abs();
    float sqrLength = length * length;
    float upperLeg = robotDimensions.upperLegLength;
    float sqrUpperLeg = upperLeg * upperLeg;
    float lowerLeg = robotDimensions.lowerLegLength;
    float sqrLowerLeg = lowerLeg * lowerLeg;
    float cosLowerLeg = (sqrLowerLeg + sqrLength - sqrUpperLeg) / (2 * lowerLeg * length);
    float cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * upperLeg * lowerLeg);

    // clip for the case of unreachable position
    const RangeBH<> clipping(-1.0f, 1.0f);
    bool reachable = true;
    if(!clipping.isInside(cosKnee) || clipping.isInside(cosLowerLeg))
    {
      cosKnee = clipping.limit(cosKnee);
      cosLowerLeg = clipping.limit(cosLowerLeg);
      reachable = false;
    }
    float joint3 = pi - acos(cosKnee); // implicitly solve discrete ambiguousness (knee always moves forward)
    float joint4 = -acos(cosLowerLeg);
    joint4 -= atan2(target.translation.x, Vector2BH<>(target.translation.y, target.translation.z).abs());
    float joint5 = atan2(target.translation.y, target.translation.z) * sign;

    // calulate rotation matrix before hip joints
    RotationMatrixBH hipFromFoot;
    hipFromFoot.rotateX(joint5 * -sign);
    hipFromFoot.rotateY(-joint4 - joint3);

    // compute rotation matrix for hip from rotation before hip and desired rotation
    RotationMatrixBH hip = hipFromFoot.invert() * target.rotation;

    // compute joints from rotation matrix using theorem of euler angles
    // see http://www.geometrictools.com/Documentation/EulerAngles.pdf
    // this is possible because of the known order of joints (z, x, y seen from body resp. y, x, z seen from foot)
    float joint1 = asin(-hip[2].y) * -sign;
    joint1 -= pi_4; // because of the 45�-rotational construction of the Nao legs
    float joint2 = -atan2(hip[2].x, hip[2].z);
    float joint0 = atan2(hip[0].y, hip[1].y) * -sign;

    // set computed joints in jointData
    jointData.angles[firstJoint + 0] = joint0;
    jointData.angles[firstJoint + 1] = joint1;
    jointData.angles[firstJoint + 2] = joint2;
    jointData.angles[firstJoint + 3] = joint3;
    jointData.angles[firstJoint + 4] = joint4;
    jointData.angles[firstJoint + 5] = joint5;

    return reachable;
  }

  /**
  * The method calculates the joint angles of one leg of the Nao from a Pose3DBH with a fixed first joint
  * This is necessary because the Nao has mechanically connected hip joints, hence not every
  * combination of foot positions can be reached and has to be recalculated with equal joint0 for both legs
  * the rotation of the foot around the z-axis through the ankle-point is left open as "failure"
  * @param position The desired position (translation + rotation) of the foots ankle point
  * @param jointData The instance of JointData where the resulting joints are written into
  * @param joint0 Fixed value for joint0 of the respective leg
  * @param left Determines if the left or right leg is calculated
  * @param robotDimensions The Robot Dimensions needed for calculation
  * @return Whether the target position was reachable or not (if the given target position is not reachable the computation proceeds using the closest reachable position near the target)
  */
  static bool calcLegJoints(const Pose3DBH& position, JointData& jointData, float joint0, bool left, const RobotDimensionsBH& robotDimensions)
  {
    Pose3DBH target(position);
    JointData::Joint firstJoint(left ? JointData::LHipYawPitch : JointData::RHipYawPitch);
    const int sign(left ? -1 : 1);
    target.translation.y += robotDimensions.lengthBetweenLegs / 2 * sign; // translate to origin of leg
    target = Pose3DBH().rotateZ(joint0 * -sign).rotateX(pi_4 * sign).conc(target); // compute residual transformation with fixed joint0

    // use cosine theorem and arctan to compute first three joints
    float length = target.translation.abs();
    float sqrLength = length * length;
    float upperLeg = robotDimensions.upperLegLength;
    float sqrUpperLeg = upperLeg * upperLeg;
    float lowerLeg = robotDimensions.lowerLegLength;
    float sqrLowerLeg = lowerLeg * lowerLeg;
    float cosUpperLeg = (sqrUpperLeg + sqrLength - sqrLowerLeg) / (2 * upperLeg * length);
    float cosKnee = (sqrUpperLeg + sqrLowerLeg - sqrLength) / (2 * upperLeg * lowerLeg);
    // clip for the case that target position is not reachable
    const RangeBH<> clipping(-1.0f, 1.0f);
    bool reachable = true;
    if(!clipping.isInside(cosKnee) || clipping.isInside(upperLeg))
    {
      cosKnee = clipping.limit(cosKnee);
      cosUpperLeg = clipping.limit(cosUpperLeg);
      reachable = false;
    }
    float joint1 = target.translation.z == 0.0f ? 0.0f : atan(target.translation.y / -target.translation.z) * sign;
    float joint2 = -acos(cosUpperLeg);
    joint2 -= atan2(target.translation.x, Vector2BH<>(target.translation.y, target.translation.z).abs() * -sgnBH(target.translation.z));
    float joint3 = pi - acos(cosKnee);
    RotationMatrixBH beforeFoot = RotationMatrixBH().rotateX(joint1 * sign).rotateY(joint2 + joint3);
    joint1 -= pi_4; // because of the strange hip of Nao

    // compute joints from rotation matrix using theorem of euler angles
    // see http://www.geometrictools.com/Documentation/EulerAngles.pdf
    // this is possible because of the known order of joints (y, x, z) where z is left open and is seen as failure
    RotationMatrixBH foot = beforeFoot.invert() * target.rotation;
    float joint5 = asin(-foot[2].y) * -sign * -1;
    float joint4 = -atan2(foot[2].x, foot[2].z) * -1;
    //float failure = atan2(foot[0].y, foot[1].y) * sign;

    // set computed joints in jointData
    jointData.angles[firstJoint + 0] = joint0;
    jointData.angles[firstJoint + 1] = joint1;
    jointData.angles[firstJoint + 2] = joint2;
    jointData.angles[firstJoint + 3] = joint3;
    jointData.angles[firstJoint + 4] = joint4;
    jointData.angles[firstJoint + 5] = joint5;

    return reachable;
  }

public:
  static bool calcArmJoints(const Pose3DBH& left, const Pose3DBH& right, JointData& targetJointData, const RobotDimensionsBH& theRobotDimensions, const JointCalibration& theJointCalibration)
  {
    const Vector3BH<> leftDir = left.rotation * Vector3BH<>(0, -1, 0),
                    rightDir = right.rotation * Vector3BH<>(0, 1, 0);

    //transform to "shoulder"-coordinate-system
    Vector3BH<> leftTarget = left.translation - Vector3BH<>(theRobotDimensions.armOffset.x,
                           theRobotDimensions.armOffset.y,
                           theRobotDimensions.armOffset.z),
                           rightTarget = right.translation - Vector3BH<>(theRobotDimensions.armOffset.x,
                                         -theRobotDimensions.armOffset.y,
                                         theRobotDimensions.armOffset.z);

    //avoid straigt arm
    static const float maxLength = (theRobotDimensions.upperArmLength + theRobotDimensions.lowerArmLength) * 0.9999f;
    if(leftTarget.squareAbs() >= sqrBH(maxLength))
      leftTarget.normalizeBH(maxLength);

    if(rightTarget.squareAbs() >= sqrBH(maxLength))
      rightTarget.normalizeBH(maxLength);

    bool res1, res2;
    res1 = calcArmJoints(leftTarget, leftDir, 1, targetJointData, theRobotDimensions, theJointCalibration);
    res2 = calcArmJoints(rightTarget, rightDir, -1, targetJointData, theRobotDimensions, theJointCalibration);

    return res1 && res2;
  }

  static bool calcArmJoints(Vector3BH<> target, Vector3BH<> targetDir, int side, JointData& targetJointData, const RobotDimensionsBH& theRobotDimensions, const JointCalibration& theJointCalibration)
  {
    //hacked mirror
    target.y *= (float)side;
    targetDir.y *= (float)side;

    const int offset = side == -1 ? JointData::RShoulderPitch : JointData::LShoulderPitch;

    Vector3BH<> elbow;
    if(!calcElbowPosition(target, targetDir, side, elbow, theRobotDimensions, theJointCalibration))
      return false;

    calcJointsForElbowPos(elbow, target, targetJointData, offset, theRobotDimensions);

    return true;
  }

  static bool calcElbowPosition(Vector3BH<> &target, const Vector3BH<> &targetDir, int side, Vector3BH<> &elbow, const RobotDimensionsBH& theRobotDimensions, const JointCalibration& theJointCalibration)
  {
    const Vector3BH<> M1(0, 0, 0); //shoulder
    const Vector3BH<> M2(target); //hand
    const float r1 = theRobotDimensions.upperArmLength;
    const float r2 = theRobotDimensions.lowerArmLength;
    const Vector3BH<> M12 = M2 - M1;

    Vector3BH<> n = target;
    n.normalizeBH();

    //center of intersection circle of spheres around shoulder and hand
    const Vector3BH<> M3 = M1 + M12 * ((sqrBH(r1) - sqrBH(r2)) / (2 * M12.squareAbs()) + 0.5f);

    //calculate radius of intersection circle
    const Vector3BH<> M23 = M3 - M2;
    float diff = sqrBH(r2) - M23.squareAbs();
    const float radius = sqrt(diff);

    //determine a point on the circle
    const bool specialCase = n.x == 1 && n.y == 0 && n.z == 0 ? true : false;
    const Vector3BH<> bla(specialCase ? 0.0f : 1.0f, specialCase ? 1.0f : 0.0f, 0.0f);
    const Vector3BH<> pointOnCircle = M3 + (n ^ bla).normalizeBH(radius);

    //find best point on circle
    float angleDiff = pi * 2.0f / 3.0f;
    Vector3BH<> bestMatch = pointOnCircle;
    float bestAngle = 0.0f;
    float newBestAngle = bestAngle;
    float bestQuality = -2.0f;

    Vector3BH<> tDir = targetDir;
    tDir.normalizeBH();

    const int offset = side == 1 ? 0 : 4;
    int iterationCounter = 0;
    const float maxAngleEpsilon = 1.0f * pi / 180.0f;
    while(2.0f * angleDiff > maxAngleEpsilon)
    {
      for(int i = -1; i <= 1; i++)
      {
        if(i == 0 && iterationCounter != 1)
          continue;

        iterationCounter++;

        const Pose3DBH elbowRotation(RotationMatrixBH(n, bestAngle + angleDiff * i));
        const Vector3BH<> possibleElbow = elbowRotation * pointOnCircle;
        const Vector3BH<> elbowDir = (M3 - possibleElbow).normalizeBH();
        float quality = elbowDir * tDir;
        if(quality > bestQuality)
        {
          bestQuality = quality;
          bestMatch = possibleElbow;
          newBestAngle = bestAngle + angleDiff * i;
        }
      }
      angleDiff /= 2.0f;
      bestAngle = newBestAngle;
    }
    //printf("iterations %d\n", iterationCounter);
    if(bestQuality == -2.0f)
      return false;

    //special case of target-out-of-joints-limit problem
    JointData tAJR;
    calcJointsForElbowPos(bestMatch, target, tAJR, offset, theRobotDimensions);

    const JointCalibration::JointInfo ji = theJointCalibration.joints[JointData::LShoulderPitch + offset + 1];
    if(tAJR.angles[offset + 1] < ji.minAngle)
    {
      tAJR.angles[offset + 1] = ji.minAngle;
      Pose3DBH shoulder2Elbow;
      shoulder2Elbow.translate(0, -theRobotDimensions.upperArmLength, 0);
      shoulder2Elbow.rotateX(-(tAJR.angles[offset + 1] - pi / 2.0f));
      shoulder2Elbow.rotateY(tAJR.angles[offset + 0] + pi / 2.0f);
      Vector3BH<> handInEllbow = shoulder2Elbow * target;

      handInEllbow.normalizeBH(theRobotDimensions.lowerArmLength);
      target = shoulder2Elbow.invert() * handInEllbow;
      bestMatch = shoulder2Elbow.invert() * Vector3BH<>(0, 0, 0);
    }

    elbow = bestMatch;
    return true;
  }

  static void calcJointsForElbowPos(const Vector3BH<> &elbow, const Vector3BH<> &target, JointData& targetJointData, int offset, const RobotDimensionsBH& theRobotDimensions)
  {
    //set elbow position with the pitch/yaw unit in the shoulder
    targetJointData.angles[offset + 0] = atan2(elbow.z, elbow.x);
    targetJointData.angles[offset + 1] = atan2(elbow.y, sqrt(sqrBH(elbow.x) + sqrBH(elbow.z)));

    //calculate desired elbow "klapp"-angle
    const float c = target.abs(),
                a = theRobotDimensions.upperArmLength,
                b = theRobotDimensions.lowerArmLength;

    //cosine theorem
    float cosAngle = (-sqrBH(c) + sqrBH(b) + sqrBH(a)) / (2.0f * a * b);
    if(cosAngle < -1.0f)
    {
//      ASSERT(cosAngle > -1.1);
      cosAngle = -1.0f;
    }
    else if(cosAngle > 1.0f)
    {
      ASSERT(cosAngle < 1.1);
      cosAngle = 1.0f;
    }
    targetJointData.angles[offset + 3] = acos(cosAngle) - pi;

    //calculate hand in elbow coordinate system and calculate last angle
    Pose3DBH shoulder2Elbow;
    shoulder2Elbow.translate(0, -theRobotDimensions.upperArmLength, 0);
    shoulder2Elbow.rotateX(-(targetJointData.angles[offset + 1] - pi / 2.0f));
    shoulder2Elbow.rotateY(targetJointData.angles[offset + 0] + pi / 2.0f);
    const Vector3BH<> handInEllbow = shoulder2Elbow * target;

    targetJointData.angles[offset + 2] = -(atan2(handInEllbow.z, handInEllbow.x) + pi / 2.f);
    while(targetJointData.angles[offset + 2] > pi)
      targetJointData.angles[offset + 2] -= 2 * pi;
    while(targetJointData.angles[offset + 2] < -pi)
      targetJointData.angles[offset + 2] += 2 * pi;
  }

  /**
  * Solves the inverse kinematics for the arms of the Nao with arbitrary elbow yaw.
  * @param position Position of the arm in cartesian space relative to the robot origin.
  * @param elbowYaw The fixed angle of the elbow yaw joint.
  * @param jointData The instance of JointData where the resulting joints are written into.
  * @param left Determines whether the left or right arm is computed.
  * @param robotDimensions The robot dimensions needed for the calculation.
  */
  static void calcArmJoints(const Vector3BH<>& position, const float elbowYaw, JointData& jointData, bool left, const RobotDimensionsBH& robotDimensions)
  {
    JointData::Joint firstJoint(left ? JointData::LShoulderPitch : JointData::RShoulderPitch);
    const int sign(left ? -1 : 1);
    const Vector3BH<> pos(position - Vector3BH<>(robotDimensions.armOffset.x, robotDimensions.armOffset.y * -sign, robotDimensions.armOffset.z));
    float& joint0 = jointData.angles[firstJoint + 0];
    float& joint1 = jointData.angles[firstJoint + 1];
    const float& joint2 = jointData.angles[firstJoint + 2] = elbowYaw;
    float& joint3 = jointData.angles[firstJoint + 3];

    // distance of the end effector position to the origin
    const float positionAbs = pos.abs();

    // the upper and lower arm form a triangle with the air line to the end effector position being the third edge. Elbow angle can be computed using cosine theorem
    const float actualUpperArmLength = Vector2BH<>(robotDimensions.upperArmLength, robotDimensions.yElbowShoulder).abs();
    float cosElbow = (sqrBH(actualUpperArmLength) + sqrBH(robotDimensions.lowerArmLength) - sqrBH(positionAbs)) / (2.0f * robotDimensions.upperArmLength * robotDimensions.lowerArmLength);
    // clip for the case of unreachable position
    cosElbow = RangeBH<>(-1.0f, 1.0f).limit(cosElbow);
    // elbow is streched in zero-position, hence pi - innerAngle
    joint3 = -(pi - acos(cosElbow));
    // compute temporary end effector position from known third and fourth joint angle
    const Pose3DBH tempPose = Pose3DBH(robotDimensions.upperArmLength, robotDimensions.yElbowShoulder * -sign, 0).rotateX(joint2 * -sign).rotateZ(joint3 * -sign).translate(robotDimensions.lowerArmLength, 0, 0);

    /* offset caused by third and fourth joint angle */                /* angle needed to realise y-component of target position */
    joint1 = atan2(tempPose.translation.y * sign, tempPose.translation.x) + asin(pos.y / Vector2BH<>(tempPose.translation.x, tempPose.translation.y).abs()) * -sign;
    // refresh temporary endeffector position with known joint1
    const Pose3DBH tempPose2 = Pose3DBH().rotateZ(joint1 * -sign).conc(tempPose);

    /* first compensate offset from temporary position */       /* angle from target x- and z-component of target position */
    joint0 = -atan2(tempPose2.translation.z, tempPose2.translation.x) + atan2(pos.z, pos.x);
  }
};
