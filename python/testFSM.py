from state import * 
import commands, core, util, pose
import time

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    sit = SitNode()
    stand = StandNode()
    choose = ChooseNode()
    self._adt(start, N, stand)
    self._adt(stand, C, choose)
    self._adt(choose, S(Choices.Forward), WalkNode(), S, choose)
    self._adt(choose, S(Choices.Left), TurnLeftNode(), S, choose)
    self._adt(choose, S(Choices.Right), TurnRightNode(), S, choose)
    self._adt(choose, I(4), SpeakNode('completed'), S, sit)
    self._adt(sit, C, finish)


def rand():
  t = int(time.time() * 1000) % 1000
  return t

class Choices:
  Left = 0
  Right = 1
  Forward = 2
  NumChoices = 3

class ChooseNode(Node):
  _choices = 0
  def run(self):
    if ChooseNode._choices > 10:
      self.poseCompletion()
      return
    if self.getTime() > 4.0:
      choice = (rand() % Choices.NumChoices)
      self.postSignal(choice)
      ChooseNode._choices += 1

class TurnLeftNode(Node):
  def run(self):
    commands.setWalkVelocity(0, 0, .25)
    if self.getTime() > 4.0:
      commands.stand()
      self.postSuccess()

class TurnRightNode(Node):
  def run(self):
    commands.setWalkVelocity(0, 0, -.25)
    if self.getTime() > 4.0:
      commands.stand()
      self.postSuccess()

class SpeakNode(Node):
  def __init__(self, phrase):
    super(SpeakNode, self).__init__()
    self.phrase = phrase

  def run(self):
    if self.getFrames() == 0:
      core.speech.say(self.phrase)
    if self.getTime() > 4.0:
      self.postSuccess()

class WalkNode(Node):
  def run(self):
    commands.setWalkVelocity(.5, 0, 0)
    if self.getTime() > 2.0:
      commands.stand()
      self.postSuccess()

class SitNode(Node):
  def __init__(self):
    super(SitNode, self).__init__()
    self.task = pose.Sit()

  def run(self):
    self.task.processFrame() 
    if self.task.finished():
      self.postCompleted()

class StandNode(Node):
  def __init__(self):
    super(StandNode, self).__init__()
    self.task = pose.Stand()

  def run(self):
    self.task.processFrame()
    if self.task.finished():
      self.postCompleted()
