#!/usr/bin/env python

import core
import time
import copy

deepcopy = copy.deepcopy

def epochTime():
  #TODO: this might not actually be epoch time, maybe check that or whatever
  return time.time()

def currentFrame():
  return core.vision_frame_info.frame_id

def getPoseJoint(joint, pose, reversed = False, reverseRolls = False):
  if joint not in pose: return None
  if not reversed: return pose[joint]

  if joint == core.LHipYawPitch or joint == core.RHipYawPitch:
    return pose[joint]

  val = 0
  # get the joint value for the joint on the opposite side
  if joint >= core.LShoulderPitch and joint <= core.LElbowRoll:
    #left arm
    val = pose[joint+(core.RShoulderPitch-core.LShoulderPitch)]
  elif joint >= core.LHipYawPitch and joint <= core.LAnkleRoll:
    #left leg
    val = pose[joint+(core.RHipYawPitch-core.LHipYawPitch)]
  elif joint >= core.RHipYawPitch and joint <= core.RAnkleRoll:
    #right leg
    val = pose[joint-(core.RHipYawPitch-core.LHipYawPitch)]
  elif joint >= core.RShoulderPitch and joint <= core.RElbowRoll:
    #right arm
    val = pose[joint-(core.RShoulderPitch-core.LShoulderPitch)]

  if reverseRolls:
    # reverse the roll directions
    directionReversedJoints = [core.LHipRoll,core.RHipRoll,core.LAnkleRoll,core.RAnkleRoll,core.LShoulderRoll,core.RShoulderRoll,core.LElbowRoll,core.RElbowRoll,core.LElbowYaw,core.RElbowYaw]
    for i in range(1, len(directionReversedJoints)):
      if joint == directionReversedJoints[i]:
        return -val
  return val


class Timer(object):
  def __init__(self):
    self._start = 0.0
    self.reset()

  def start(self):
    self._start = time.time()
  
  def stop(self):
    self._elapsed += time.time() - self._start

  def reset(self):
    self._elapsed = 0.0
    self._start = time.time()

  def elapsed(self):
    return time.time() - self._start + self._elapsed 
