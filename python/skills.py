#!/usr/bin/env python

from task import Task, HeadBodyTask
import body, head, cfgpose, commands

class ToPoseMoveHead(HeadBodyTask):
  def __init__(self, pose, tilt = 0.0, time = 2.0):
    self.bpose = body.ToPose(pose = pose, time = time)
    self.mhead = head.MoveHead(tilt = tilt, time = time)
    HeadBodyTask.__init__(self, self.bpose, self.mhead)

class PoseSequence(Task):
  def __init__(self, *args):
    super(PoseSequence, self).__init__()
    if len(args) % 2 != 0:
      raise Exception("Pose sequence arguments must be (pose, time) pairs.")
    pi, ti = 0, 1
    while ti < len(args):
      pose = args[pi]
      time = args[ti]
      self.chain += [body.ToPose(pose = pose, time = time)]
      pi += 2
      ti += 2

  def start(self):
    commands.setStiffness()
    super(PoseSequence, self).start()
