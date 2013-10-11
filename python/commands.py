#!/usr/bin/env python

import core
import cfgstiff
import math

def stand():
  core.walk_request.stand()

def setWalkVelocity(velX, velY, velTheta):
  core.walk_request.setWalk(velX, velY, velTheta)

def setStiffness(cfg = cfgstiff.One, time = 0.3):
  if isAtStiffness(cfg): return

  for i in range(core.NUM_JOINTS):
    core.joint_commands.setJointStiffness(i, cfg[i])
  core.joint_commands.send_stiffness_ = True
  core.joint_commands.stiffness_time_ = time * 1000.0

def isAtStiffness(cfg):
  for i in range(core.NUM_JOINTS):
    stiff = core.pythonC.getFloat(core.joint_angles.stiffness_, i)
    error = abs(stiff - cfg[i])
    if error > 0.05: return False
  return True

def getTurnOffset(offsets,velX,velY,sprint):
  turn = offsets.turnInPlace
  if velX > 0:
    if sprint:
      turn = turn + math.abs(velX) * (offsets.turnSprint  - offsets.turnInPlace)
    else:
      turn = turn + math.abs(velX) * (offsets.turnFwd  - offsets.turnInPlace)
  else:
    turn = turn + math.abs(velX) * (offsets.turnBack  - offsets.turnInPlace)
  if velY > 0:
    turn = turn + math.abs(velY) * (offsets.turnLeft - offsets.turnInPlace)
  else:
    turn = turn + math.abs(velY) * (offsets.turnRight - offsets.turnInPlace)
  return turn

def setHeadPanTilt(pan = 0, tilt = -21, time = 2.0, isChange = False):
  setHeadTilt(tilt)
  setHeadPan(pan, time, isChange)

def setHeadTilt(tilt = -21):
  core.joint_commands.setHeadTilt(core.DEG_T_RAD * tilt, 200.0, False)

def setHeadPan(target_pos, target_time, isChange = None):
  if (isChange == None): isChange = False
  # make sure tilt is correct
  setHeadTilt()
  
  core.joint_commands.setHeadPan(target_pos, target_time*1000.0, isChange)

def setKickParameters(params, paramsSuper):
  core.kick_params.send_params_ = True
  core.kick_params.params_ = params
  core.kick_params.params_super_ = paramsSuper
