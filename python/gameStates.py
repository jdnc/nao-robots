#!/usr/bin/env python

import core
from task import Task, HeadBodyTask, MachineTask
import pose, head, kicks
import commands, cfgstiff
import testFSM
import math
import head

def areDistinct(state1, state2):
  if state1 == core.INITIAL and state2 == core.FINISHED: return False
  if state1 == core.FINISHED and state2 == core.INITIAL: return False
  if state1 == state2: return False
  return True

def createStateTask(state):
  if state == core.INITIAL: return Initial()
  if state == core.READY: return Ready()
  if state == core.PLAYING: return Playing()
  if state == core.FINISHED: return Finished()
  if state == core.TESTING: return Testing()
  if state == core.PENALISED: return Penalised()
  return None

Penalised = Initial = Finished = pose.Sit

class Ready(HeadBodyTask):
  def __init__(self):
    HeadBodyTask.__init__(self, 
      head.Scan(period = 3.0, maxPan = 105.0 * core.DEG_T_RAD, numSweeps = 4),
      pose.Stand()
    )

  def run(self):
    commands.setStiffness()
    HeadBodyTask.run(self)

class Playing(Task):
  def __init__(self):
    """
    HeadBodyTask.__init__(self, 
      head.Scan(period = 6.0, maxPan = 90.0 * core.DEG_T_RAD, numSweeps = 4),
      Walk()
    )
    """
    Task.__init__(self)
    

  def run(self):  
    commands.setStiffness()
    commands.setWalkVelocity(0.5, 0.0, 0.0)    
    #HeadBodyTask.run(self)
    #Walk.run()
   

class Testing(Task):
  def run(self):
    commands.setStiffness()
    commands.setWalkVelocity(.5, .2, 0.0)
    if self.getTime() > 5.0:
      self.subtask = pose.Sit()
      self.finish


class Walk(Task):
  def __init__(self):
    Task.__init__(self)
  def run(self):
    #pose.Stand()
    commands.setWalkVelocity(0.3, 0, 0.0)
