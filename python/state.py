#!/usr/bin/env python

import util

class SimpleStateMachine(object):
  def __init__(self, states):
    if not states:
      raise Exception("State machine can't be empty")
    self.states = list(states)
    self.istates = range(len(states))
    self.initial = states[0]
    self._timer = util.Timer()
    self.transition(self.initial)
    for i in range(len(states)):
      setattr(self, states[i], i)

  def timeSinceTransition(self):
    return self._timer.elapsed()

  def framesSinceTransition(self):
    return util.currentFrame() - self.startFrame

  def transition(self, state):
    if isinstance(state, int):
      self.istate = state
      if self.istate < 0 or self.istate >= len(self.states):
        raise Exception("Invalid state index: %i" % state)
      self.state = self.states[self.istate]
    elif isinstance(state, basestring):
      self.state = state
      if not state in self.states:
        raise Exception("State does not exist in the state machine: %s" % state)
      self.istate = self.states.index(state)
    else:
      raise Exception("Invalid state passed: %s" % str(state))
    self._timer.start()
    self.startFrame = util.currentFrame()

  def inState(self, state):
    return self.istate == state or self.state == state

  def isFirstFrameInState(self):
    return self.startFrame == util.currentFrame()

  def numStates(self):
    return len(self.states)

class StateMachine(object):
  def __init__(self):
    self._node = None
    self.setup()

  def _addTransition(self, *args):
    si, ei, ti = 0, 1, 2
    if not self._node: self._node = args[si]
    while ti < len(args):    
      source = args[si]
      event = args[ei]
      target = args[ti]
      if hasattr(event, '__call__'): event = event()
      source.events += [event]
      event.source = source
      event.target = target
      si += 2
      ei += 2
      ti += 2

  _adt = _addTransition

  def processFrame(self):
    for e in self._node.events:
      if not e.started(): e.start()
      if not e.fired() and e.ready():
        e.fire()
        e.reset()
        self._node.reset()
        self._node = e.target

    self._node.startrun()

class Event(object):
  def __init__(self):
    self.source = None
    self.target = None
    self._started = False
    self._fired = False

  def started(self):
    return self._started

  def start(self):
    self._started = True

  def fired(self):
    return self._fired

  def fire(self):
    self._fired = True

  def reset(self):
    self._started = False
    self._fired = False

class SignalEvent(Event):
  def __init__(self, signal):
    super(SignalEvent, self).__init__()
    self.signal = signal

  def ready(self):
    return self.source.getSignal() == self.signal

class CompletionEvent(Event):
  def ready(self):
    return self.source.complete()

class SuccessEvent(Event):
  def ready(self):
    return self.source.success()

class FailureEvent(Event):
  def ready(self):
    return self.source.failure()

class NullEvent(Event):
  def ready(self):
    return True

class TimeEvent(Event):
  def __init__(self, time):
    super(TimeEvent, self).__init__()
    self.time = time

  def ready(self):
    return self.source.getTime() > self.time

class IterationEvent(Event):
  def __init__(self, iterations):
    super(IterationEvent, self).__init__()
    self.iterations = iterations

  def ready(self):
    return self.source.iterations() > self.iterations

def S(signal = None):
  if signal == None: return SuccessEvent()
  return SignalEvent(signal)

I = IterationEvent
F = FailureEvent
N = NullEvent
T = TimeEvent
C = CompletionEvent

class Node(object):
  def __init__(self):
    self._iterations = 0
    self.events = list()
    self.reset()

  def startrun(self):
    if not self._started: self.start()
    self.run()

  def run(self):
    pass

  def start(self):
    self._started = True
    self._timer.start()
    self._startFrame = util.currentFrame()
    self._iterations += 1

  def started(self):
    return self._started

  def reset(self):
    self._success = False
    self._failure = False
    self._complete = False
    self._signal = None
    self._started = False
    self._timer = util.Timer()
    self._startFrame = None

  def postSuccess(self):
    self._success = True

  def success(self):
    return self._success

  def postFailure(self):
    self._failure = True

  def failure(self):
    return self._failure

  def postCompleted(self):
    self._complete = True

  def complete(self):
    return self._complete

  def iterations(self):
    return self._iterations

  def getSignal(self):
    return self._signal

  def postSignal(self, signal):
    self._signal = signal

  def getTime(self):
    return self._timer.elapsed()

  def getFrames(self):
    return util.currentFrame() - self._startFrame
