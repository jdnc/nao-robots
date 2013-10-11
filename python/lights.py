#!/usr/bin/env python
import core

ledsC = core.ledsC
world_objects = core.world_objects
behavior_mem = core.behavior_mem

def processFrame():
  doFootLights()
  doStateLights()
  doEyeLights()
  doEarLights()


def doEarLights():
  if (core.game_state.state == core.INITIAL or core.game_state.state == core.FINISHED):
    doInitialEarLights()
  else:
    doPlayingEarLights()

def doInitialEarLights():
  p1frames = (core.vision_frame_info.frame_id - core.team_packets.getFrameReceived(1))
  p2frames = (core.vision_frame_info.frame_id - core.team_packets.getFrameReceived(2))
  p3frames = (core.vision_frame_info.frame_id - core.team_packets.getFrameReceived(3))  
  p4frames = (core.vision_frame_info.frame_id - core.team_packets.getFrameReceived(4))
  p5frames = (core.vision_frame_info.frame_id - core.team_packets.getFrameReceived(5))

  if (p1frames < 30):
    ledsC.partLeftEar(1, 0, 4)
  else:
    ledsC.partLeftEar(0, 0, 4)

  if (p2frames < 30):
    ledsC.partLeftEar(1, 5, 4)
  else:
    ledsC.partLeftEar(0, 5, 4)
 
  # front top
  if (p3frames < 30):
    ledsC.partRightEar(1, 0, 3)
  else:
    ledsC.partRightEar(0, 0, 3)

  # bottom
  if (p4frames < 30):
    ledsC.partRightEar(1, 4, 2)
  else:
    ledsC.partRightEar(0, 4, 2)
 
  # back top
  if (p5frames < 30):
    ledsC.partRightEar(1, 7, 2)
  else:
    ledsC.partRightEar(0, 7, 2)


def doPlayingEarLights():
  # chaser
  if (core.robot_state.role_ == core.CHASER):
    ledsC.frontRightEar(1)
  else:
    ledsC.frontRightEar(0)

  # I'd like not to send this every frame, rather
  # only send when seen changes
  if (world_objects.getObjPtr(core.WO_BALL).seen):
    ledsC.frontLeftEar(1)
  else:
    ledsC.frontLeftEar(0)

  # default to off
  ledsC.backLeftEar(0)
  ledsC.backRightEar(0)

  # left arm bumper
  if (core.processed_sonar.bump_left_):
    ledsC.backLeftEar(1)
  if (core.processed_sonar.bump_right_):
    ledsC.backRightEar(1)


def doStateLights():
  # do chest led based on state
  state = core.game_state.state
  if (state == core.INITIAL): 
    ledsC.chest(0,0,0)
  elif (state == core.READY):
    ledsC.chest(0,0,1)
  elif (state == core.SET):
    ledsC.chest(1,1,0)
  elif (state == core.PLAYING):
    ledsC.chest(0,1,0)
  elif (state == core.PENALISED):
    ledsC.chest(1,0,0)
  elif (state == core.TESTING):
    ledsC.chest(1,0,1)
  elif (state == core.FINISHED):
    ledsC.chest(1,1,1)

def doFootLights():
  # do feet leds
  if (core.robot_state.team_== 0): #blue
    ledsC.rightFoot(0,0,1)
  elif (core.robot_state.team_==1): #red
    ledsC.rightFoot(1,0,0)
  
  if (core.game_state.ourKickOff):
    ledsC.leftFoot(1,1,1)
  else:
    ledsC.leftFoot(0,0,0)

def doEyeLights():
  if (core.game_state.state == core.INITIAL):
    doInitialEyeLights()
  elif (core.game_state.state == core.FINISHED):
    doFinishedEyeLights()
  else:
    doPlayingEyeLights()
  if (core.robot_state.ignore_comms_):
    ledsC.allLeftEye(1,0,0)

def doInitialEyeLights():
  doEyeBalls()
  doEyePower()
  doEyeHeat()

def doFinishedEyeLights():
  doEyeBalls()
  doEyePower()
  doEyeHeat()

def doPlayingEyeLights():
  #doLeftEyeObjects()
  #doRightEyeSonar()
  doLeftEyeKicks()
  doRightEyeKicks()

def doLeftEyeObjects():
  goal = world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
  circle = world_objects.getObjPtr(core.WO_CENTER_CIRCLE)
  if goal.seen:
    ledsC.allLeftEye(0,0,1)
  elif circle.seen:
    ledsC.allLeftEye(1,0,0)
  else:
    ledsC.allLeftEye(0,0,0)

def doRightEyeSonar():
  if not(core.processed_sonar.sonar_module_enabled_):
    ledsC.allRightEye(0,0,0)
  else:
    if core.processed_sonar.on_left_:
      ledsC.allRightEye(1,0,0)
    elif core.processed_sonar.on_center_:
      ledsC.allRightEye(0,1,0)
    elif core.processed_sonar.on_right_:
      ledsC.allRightEye(0,0,1)
    else:
      ledsC.allRightEye(1,1,1)

def doEyePower():
  battery = core.sensors.getValue(core.battery)
  if (battery > 0.95):
    ledsC.allBottomLeftEye(1,1,1)
    return False
  elif (battery > 0.8):
    ledsC.allBottomLeftEye(1,1,0)
    return False
  elif (battery > 0.65):
    ledsC.allBottomLeftEye(1,.5,1)
    return False
  else:
    ledsC.allBottomLeftEye(1,0,0)
    return True

def doEyeHeat():
  max_temp = getMaxTemp()
  if (max_temp > 74):
    ledsC.allTopLeftEye(1,0,0)
    return True
  elif (max_temp > 64):
    ledsC.allTopLeftEye(1,.5,0)
    return True
  elif (max_temp > 54):
    ledsC.allTopLeftEye(1,1,0)
    return True
  else:
    ledsC.allTopLeftEye(1,1,1)
    return False

def doEyeBalls():
  ball = world_objects.getObjPtr(core.WO_BALL)
  if (ball.seen):
    if ball.fromTopCamera:
      ledsC.allTopRightEye(0,0,1)
      ledsC.allBottomRightEye(0,1,0)
    else:
      ledsC.allTopRightEye(0,1,0)
      ledsC.allBottomRightEye(0,0,1)
  else:
    ledsC.allTopRightEye(0,1,0)
    ledsC.allBottomRightEye(0,1,0)


def doLeftEyeKicks():
  ledsC.allLeftEye(0,0,0)
  if (behavior_mem.chooseKick):
    if (behavior_mem.kickChoice == core.FwdLongStraightKick or
        behavior_mem.kickChoice == core.FwdLongLeftwardOutKick or
        behavior_mem.kickChoice == core.FwdLongLeftwardinKick):
      ledsC.allLeftEye(1,0,0)
    elif (behavior_mem.kickChoice == core.FwdMediumStraightKick or
        behavior_mem.kickChoice == core.FwdMediumLeftwardOutKick or
        behavior_mem.kickChoice == core.FwdMediumLeftwardinKick):
      ledsC.allLeftEye(0,1,0)
    elif (behavior_mem.kickChoice == core.WalkKickFront or
            behavior_mem.kickChoice == core.WalkKickLeftward):
      ledsC.allLeftEye(0,0,1)
    elif (behavior_mem.kickChoice == core.WalkKickLeftwardSide):
      ledsC.allLeftEye(1,1,1)

def doRightEyeKicks():
  ledsC.allRightEye(0,0,0)
  if (behavior_mem.chooseKick):
    if (behavior_mem.kickChoice == core.FwdLongStraightKick or
        behavior_mem.kickChoice == core.FwdLongRightwardOutKick or
        behavior_mem.kickChoice == core.FwdLongRightwardinKick):
      ledsC.allRightEye(1,0,0)
    elif (behavior_mem.kickChoice == core.FwdMediumStraightKick or
        behavior_mem.kickChoice == core.FwdMediumRightwardOutKick or
        behavior_mem.kickChoice == core.FwdMediumRightwardinKick):
      ledsC.allRightEye(0,1,0)
    elif (behavior_mem.kickChoice == core.WalkKickFront or
            behavior_mem.kickChoice == core.WalkKickRightward):
      ledsC.allRightEye(0,0,1)
    elif (behavior_mem.kickChoice == core.WalkKickRightwardSide):
      ledsC.allRightEye(1,1,1)



def getMaxTemp():
  max_temp = 0
  for i in range(core.NUM_JOINTS):
    temp = core.sensors.getJointTemperature(i)
    if temp > max_temp:
      max_temp = temp
  return max_temp
