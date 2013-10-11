#!/usr/bin/env python

import core, util
from task import Task
import commands, state, percepts

class MoveHead(Task):
  def __init__(self, tilt = 0.0, pan = 0.0, time = 2.0):
    Task.__init__(self)
    self.tilt = tilt
    self.pan = pan
    self.time = time

  def run(self):
    commands.setHeadPan(self.pan, self.time)
    commands.setHeadTilt(self.tilt)

    if self.getTime() > self.time: self.finish()

class Scan(Task):
  def __init__(self, maxPan = 2.0, period = 3.0, numSweeps = 1, direction = 1):
    Task.__init__(self)
    self.maxPan = maxPan
    self.period = period
    self.numSweeps = numSweeps
    self.direction = direction
    self.intDirection = direction
    self.sweepCounter = 0
    self.state = state.SimpleStateMachine(['firstScan', 'nextScans'])

  def run(self):
    numSteps = self.period / DiscreteScan.stepTime + 1
    stepSize = (2 * self.maxPan / numSteps) * 1.05

    st = self.state

    if st.inState(st.firstScan):
      self.intDirection = self.direction
      self.subtask = DiscreteScan(dest = self.direction * self.maxPan, stepSize = stepSize)
      st.transition(st.nextScans)
      return

    if st.inState(st.nextScans) and self.subtask.finished():
      self.sweepCounter += 1
      self.intDirection *= -1
      core.behavior_mem.completeBallSearchTime = core.vision_frame_info.seconds_since_start
      self.subtask = DiscreteScan(dest = self.intDirection * self.maxPan, stepSize = stepSize, skipFirstPause = True)
      return

    if self.sweepCounter > self.numSweeps: self.finish()

class DiscreteScan(Task):
  stepTime = 0.4
  def __init__(self, dest = 115 * core.DEG_T_RAD, stepSize = 26 * core.DEG_T_RAD, skipFirstPause = False):
    Task.__init__(self)
    self.dest = dest
    self.stepSize = stepSize
    self.pauseTime = 0.2083
    self.skipFirstPause = skipFirstPause
    self.isPaused = True
    self.timer = util.Timer()

  def run(self):
    moveTime = self.stepTime - self.pauseTime
    if self.skipFirstPause or (self.isPaused and (self.timer.elapsed() > self.pauseTime)):
      self.isPaused = False
      self.skipFirstPause = False
      currPan = percepts.joint_angles[core.HeadPan]
      diff = self.dest - currPan
      dir = (diff / abs(diff))
      target = currPan + self.stepSize * dir
      if dir > 0 and target > self.dest:
        target = self.dest
      elif dir < 0 and target < self.dest:
        target = self.dest
      commands.setHeadTilt()
      commands.setHeadPan(target,moveTime)
      if abs(diff) < 5 * core.DEG_T_RAD:
        self.finish()
      self.timer.reset()

    if not self.isPaused and self.timer.elapsed() > moveTime:
      self.timer.reset()
      self.isPaused = True
