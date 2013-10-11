#!/usr/bin/env python

import core, task, util, state, skills
import commands
from task import Task
import util
import cfgpose, cfgstiff
import percepts

class Sit(Task):
  def __init__(self):
    Task.__init__(self)
    core.kick_request.setNoKick()
    core.walk_request.noWalk()
    core.kick_request.kick_running_ = False
    core.behavior_mem.keeperDiving = core.Dive.NONE
    self.state = state.SimpleStateMachine(['stop', 'checkarms', 'movearms', 'sit', 'relaxknee', 'relaxbody', 'finish'])
    self.skippedState = False
    self.lower_time = 0

  def run(self):
        
    if self.getTime() < 2.0:
      core.walk_request.noWalk()
      core.kick_request.setNoKick()
      commands.setStiffness(cfgstiff.One, 0.3)
      return

    st = self.state

    if st.inState(st.stop):
      st.transition(st.checkarms)

    if st.inState(st.checkarms):
      shoulderCutoff = core.DEG_T_RAD * -90
      if percepts.joint_angles[core.LShoulderPitch] > shoulderCutoff and percepts.joint_angles[core.RShoulderPitch] > shoulderCutoff:
        st.transition(st.sit)
      else:
        st.transition(st.movearms)
    elif st.inState(st.movearms):
      pose = util.deepcopy(cfgpose.sittingPoseV3)
      for joint, val in cfgpose.armSidePose.items():
        pose[joint] = val
      self.subtask = skills.ToPoseMoveHead(tilt = 0.0, pose = pose)
      self.subtask.start()
      st.transition(st.sit)
    elif st.inState(st.sit) and (not self.subtask or self.subtask.finished()):
      self.skippedState = False
      self.subtask = skills.ToPoseMoveHead(pose = cfgpose.sittingPoseV3, time = 1.0)
      self.subtask.start()
      st.transition(st.relaxknee)
    elif st.inState(st.relaxknee) and self.subtask.finished():
      self.lower_time = self.getTime()
      commands.setStiffness(cfgstiff.ZeroKneeAnklePitch, 0.3)
      st.transition(st.relaxbody)
    elif st.inState(st.relaxbody) and st.timeSinceTransition() > 0.7:
      commands.setStiffness(cfgstiff.Zero, 0.3)
      st.transition(st.finish)
    elif st.inState(st.finish):
      self.finish()

class Stand(Task):
  def __init__(self):
    Task.__init__(self)

  def run(self):
    core.kick_request.setNoKick()
    core.walk_request.stand()
    if self.getTime() > 2.0:
      self.finish()

class Squat(Task):
  def __init__(self, time = 3.0):
    Task.__init__(self)
    self.chain = [ 
      skills.PoseSequence(
        cfgpose.goalieSquatPart1, 0.4,
        cfgpose.goalieSquatPart2, 0.2,
        cfgpose.goalieSquatPart2, time,
        cfgpose.goalieSquat5, 0.2,
        cfgpose.goalieSquat5, 0.3,
        cfgpose.goalieSquatPart2, 0.3,
        cfgpose.goalieSquatGetup15, 0.4,
        cfgpose.goalieSquatGetup2, 0.6,
        cfgpose.goalieSquatGetup7, 0.3
      ),
      Stand()
    ]

class BlockRight(Task):
  def __init__(self, time = 3.0):
    Task.__init__(self)
    self.subtask = skills.PoseSequence(
      cfgpose.blockright, 1.0,
      cfgpose.blockright, time, 
      cfgpose.sittingPoseNoArms, 2.0,
      cfgpose.standingPose, 2.0
    )

class BlockLeft(Task):
  def __init__(self, time = 3.0):
    Task.__init__(self)
    self.subtask = skills.PoseSequence(
      cfgpose.blockleft, 1.0,
      cfgpose.blockleft, time, 
      cfgpose.sittingPoseNoArms, 2.0,
      cfgpose.standingPose, 2.0
    )
