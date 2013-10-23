#!/usr/bin/env python
import core
core.init() # this has to be run before anything else can be imported

import sys, os, traceback
import UTdebug
import pose
import percepts
import lights
import classBvr
import util
import cfgkick
import commands
behavior = classBvr

def init():
  global firstFrame
  firstFrame = True
  initMemory()
  initNonMemory()
  print "Python initialized"

def initMemory():
  core.initMemory()

def initNonMemory():
  pass

def processFrame():
  try:
    global firstFrame
    core.instance.preVision()
    if firstFrame:
      commands.setKickParameters(cfgkick.StraightKick, cfgkick.StraightSuperKick)
      core.world_objects.init(core.robot_state.team_)
      core.visionC.initSpecificModule()
      core.localizationC.initSpecificModule()
      core.speech.say("Vision")
      firstFrame = False

    #TODO: test for getting up
    core.visionC.processFrame()
    core.instance.postVision()
    percepts.update()
    core.localizationC.processFrame()
    core.opponentsC.processFrame()
    behavior.processFrame()
    lights.processFrame()
    core.instance.publishData()
  except:
    handle()

def handle():
  lines = traceback.format_exception(*(sys.exc_info()))
  message = ''.join('!! ' + line for line in lines)
  UTdebug.log(0,'PYTHON ERROR:')
  UTdebug.log(0, message)
  core.speech.say("python")
  core.pythonC.python_ok_ = False
