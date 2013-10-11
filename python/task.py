#!/usr/bin/env python

import util

class BaseTask(object):
  def __init__(self):
    self._started = False
    self._finished = False
    self._aborted = False
    self._timer = util.Timer()

  def _startrun(self):
    if not self._started:
      self.start()
    elif not self._finished:
      self.run()

  def start(self):
    if self._started: return
    self._timer.start()
    self._started = True

  def started(self):
    return self._started

  def finish(self):
    self._finished = True

  def finished(self):
    return self._finished

  def abort(self):
    self._aborted = True
    self.finish()

  def aborted(self):
    return self._aborted

  def getTime(self):
    return self._timer.elapsed()

class Task(BaseTask):
  def __init__(self):
    BaseTask.__init__(self)
    self.subtask = None
    self.chain = []
    self.chainIndex = None

  def run(self):
    if not self.chain: return
    for s in self.chain:
      if not s.finished():
        s._startrun()
        self.subtask = s
        return
    self.finish()

  def processFrame(self):
    if self.aborted(): return
    if self.subtask:
      if self.subtask.finished():
        self._startrun()
      else:
        self.subtask.processFrame()
    else:
      self._startrun()

  def abort(self):
    BaseTask.abort(self)
    if self.subtask: self.subtask.abort()

class MultiTask(BaseTask):
  def __init__(self, independentChains = True):
    BaseTask.__init__(self)
    self.subtasks = []
    self.chains = []
    self.chainIndexes = []
    self.idpChains = independentChains

  def run(self):
    if not self.chains: return
    if not self.subtasks:
      self.subtasks = [None] * len(self.chains)
    if not self.chainIndexes:
      self.chainIndexes = [0] * len(self.chains)
    ci = 0
    finished = True
    for c in self.chains:
      si = 0
      for s in c:
        self.chainIndexes[ci] = si
        if not s.finished():
          s._startrun()
          self.subtasks[ci] = s
          finished = False
          break
        si += 1
      ci += 1
    if finished: self.finish()

  def processFrame(self):
    if self.aborted(): return
    someFinished = False
    allFinished = True
    for s in self.subtasks:
      if not s:
        someFinished = True
      elif s.finished():
        someFinished = True
      else:
        allFinished = False
        s.processFrame()
    if someFinished and self.idpChains: self._startrun()
    elif allFinished: self._startrun()

  def abort(self):
    BaseTask.abort(self)
    for s in self.subtasks:
      if s: s.abort()

class MachineTask(BaseTask):
  def __init__(self, machine):
    BaseTask.__init__(self)
    self.machine = machine

  def processFrame(self):
    if self.aborted(): return
    if self.machine:
      self.machine.processFrame()

class HeadBodyTask(MultiTask):
  def __init__(self, htask, btask):
    MultiTask.__init__(self)
    self.chains = [[htask], [btask]]

