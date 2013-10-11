#!/usr/bin/env python

import core
from task import Task
from state import SimpleStateMachine
import commands

class Kick(Task):
  def __init__(self, foot = core.Kick.RIGHT, desiredDistance = 2000.0):
    Task.__init__(self)
    self.kickRunning = False
    self.postKick = False
    self.foot = foot
    self.desiredDistance = desiredDistance
    self.sm = SimpleStateMachine(['startup', 'kicking', 'finish'])


  def run(self):
    sm = self.sm
    ball = core.world_objects.getObjPtr(core.WO_BALL)

    kreq = core.kick_request
    kreq.ball_seen_ = ball.seen
    kreq.ball_image_center_x_ = ball.imageCenterX
    kreq.ball_image_center_y_ = ball.imageCenterY
    kreq.ball_rel_x_ = ball.relPos.x
    kreq.ball_rel_y_ = ball.relPos.y

    if sm.inState(sm.startup):
      commands.stand()
      if sm.timeSinceTransition() > 3.0:
        sm.transition(sm.kicking)
        kreq.kick_running_ = True
      else: return

    if sm.inState(sm.kicking):
      core.walk_request.noWalk()
      if sm.timeSinceTransition() > 1.0:
        sm.transition(sm.finish)
      kreq.set(core.Kick.STRAIGHT, self.foot, 0, self.desiredDistance)
    
    if sm.inState(sm.finish):
      self.finish()

