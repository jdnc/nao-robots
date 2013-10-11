#/usr/bin/env python

import core
import gameStates
import testFSM


currentState = None
currentTask = None

def processFrame():
  global currentState, currentTask
  lastState = currentState
  currentState = core.game_state.state

  if gameStates.areDistinct(currentState, lastState):
    if currentTask: currentTask.finish()
    currentTask = gameStates.createStateTask(currentState)
    currentTask.start()

  currentTask.processFrame()
