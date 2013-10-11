import core, UTdebug
StraightKick = core.KickParameters()

# times in milliseconds
liftAmount = 40 #25
backAmount = -80
throughAmount = 80
liftAlignAmount = 60
liftKickAmount = 25
comHeight = 175 # this should be the height of the walk
comOffset = 10
comOffsetX = 15
kick_time = 0

info = StraightKick.getStateInfoPtr(core.KickState.STAND)
info.state_time = 200 # decrease this if possible
info.joint_time = 200
info.com = core.vector3_float(comOffsetX,50,comHeight)
info.swing = core.vector3_float(0,100,0)
kick_time = kick_time + info.state_time

info = StraightKick.getStateInfoPtr(core.KickState.SHIFT)
info.state_time = 150
info.joint_time = 150
info.com = core.vector3_float(comOffsetX,comOffset + 30,comHeight)
info.swing = core.vector3_float(0,100,0)
kick_time = kick_time + info.state_time

info = StraightKick.getStateInfoPtr(core.KickState.LIFT)
info.state_time = 50
info.joint_time = 50
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,10)
kick_time = kick_time + info.state_time

info = StraightKick.getStateInfoPtr(core.KickState.ALIGN)
info.state_time = 150
info.joint_time = 150
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(backAmount,100,liftAlignAmount)
kick_time = kick_time + info.state_time

info = StraightKick.getStateInfoPtr(core.KickState.KICK1)
info.state_time = 0
info.joint_time = 50
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,(liftKickAmount+liftAlignAmount)/2)
kick_time = kick_time + info.state_time

info = StraightKick.getStateInfoPtr(core.KickState.KICK2)
info.state_time = 0
info.joint_time = 50
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(throughAmount,100,liftKickAmount)
kick_time = kick_time + info.state_time

info = StraightKick.getStateInfoPtr(core.KickState.RESETFOOT)
info.state_time = 150
info.joint_time = 150
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,liftAmount)

info = StraightKick.getStateInfoPtr(core.KickState.FOOTDOWN)
info.state_time = 50
info.joint_time = 50
info.com = core.vector3_float(comOffsetX,comOffset + 20,comHeight)
info.swing = core.vector3_float(0,100,liftAmount/2)

info = StraightKick.getStateInfoPtr(core.KickState.SHIFTBACK)
info.state_time = 500 #75
info.joint_time = 75
info.com = core.vector3_float(comOffsetX,50,comHeight)
info.swing = core.vector3_float(0,100,0)

info = StraightKick.getStateInfoPtr(core.KickState.FINISHSTAND)
info.state_time = 500
info.joint_time = 500
info.com = core.vector3_float(comOffsetX/2,50,comHeight)
info.swing = core.vector3_float(0,100,0)   

# SPLINE STATE
info = StraightKick.getStateInfoPtr(core.KickState.SPLINE)
info.state_time = 100
info.joint_time = info.state_time
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,0)

# these aren't used for this kick, but are needed to make lua happy
StraightKick.l_hip_roll_before_ = -0.4;
StraightKick.l_hip_roll_after_ = -0.4;
StraightKick.l_hip_pitch_before_ = 0.2;
StraightKick.l_hip_pitch_after_ = -1.12;
StraightKick.l_knee_pitch_before_ = 1.4;
StraightKick.l_knee_pitch_after_ = 1.4;
StraightKick.l_ankle_roll_before_ = 0.8;
StraightKick.l_ankle_roll_after_ = 0.8;
StraightKick.l_ankle_pitch_before_ = -0.95;
StraightKick.l_ankle_pitch_after_ = -0.4;

StraightKick.step_into_kick_ = False
StraightKick.use_stance_spline = False
StraightKick.kick_time_ = (450 + kick_time) / 1000.0
StraightKick.offset_ = 10

StraightKick.swing_length_ = 120
StraightKick.swing_time_ = 200

StraightKick.align_height_ = 32.5
StraightKick.kick_height_ = 32.5

# ideal ball placements - i.e. what the kick was developed for
StraightKick.ideal_ball_side_left_swing_ = 15
StraightKick.ideal_ball_side_right_swing_ = -30

##############################
# will eventually be the params for a Super Long Kick
StraightSuperKick = core.KickParameters()
# times in milliseconds
liftAmount = 40 #25
backAmount = -40
throughAmount = 100
liftAlignAmount = 40
liftKickAmount = 50
comHeight = 175 # this should be the height of the walk
comOffset = -5 # can balance on one foot with this offset
comOffsetX = 15
info
kick_time = 0

info = StraightSuperKick.getStateInfoPtr(core.KickState.STAND)
info.state_time = 300 # decrease this if possible
info.joint_time = 300
info.com = core.vector3_float(comOffsetX,50,comHeight)
info.swing = core.vector3_float(0,100,0)
kick_time = kick_time + info.state_time

info = StraightSuperKick.getStateInfoPtr(core.KickState.SHIFT)
info.state_time = 250
info.joint_time = 250
info.com = core.vector3_float(comOffsetX,comOffset + 30,comHeight)
info.swing = core.vector3_float(0,100,0)
kick_time = kick_time + info.state_time

info = StraightSuperKick.getStateInfoPtr(core.KickState.LIFT)
info.state_time = 350
info.joint_time = 350
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,10)
kick_time = kick_time + info.state_time

info = StraightSuperKick.getStateInfoPtr(core.KickState.ALIGN)
info.state_time = 2500
info.joint_time = 1500
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(backAmount,100,liftAlignAmount) # -80,100,60
kick_time = kick_time + info.state_time

info = StraightSuperKick.getStateInfoPtr(core.KickState.KICK1)
info.state_time = 0
info.joint_time = 50
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,(liftKickAmount+liftAlignAmount)/2)
kick_time = kick_time + info.state_time

info = StraightSuperKick.getStateInfoPtr(core.KickState.KICK2)
info.state_time = 0
info.joint_time = 50
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(throughAmount,100,liftKickAmount) # 80,100,35
kick_time = kick_time + info.state_time

info = StraightSuperKick.getStateInfoPtr(core.KickState.RESETFOOT)
info.state_time = 3000
info.joint_time = 3000
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,liftAmount)

info = StraightSuperKick.getStateInfoPtr(core.KickState.FOOTDOWN)
info.state_time = 150
info.joint_time = 150
info.com = core.vector3_float(comOffsetX,comOffset + 20,comHeight)
info.swing = core.vector3_float(0,100,liftAmount/2)

info = StraightSuperKick.getStateInfoPtr(core.KickState.SHIFTBACK)
info.state_time = 500 #75
info.joint_time = 250
info.com = core.vector3_float(comOffsetX,50,comHeight)
info.swing = core.vector3_float(0,100,0)

info = StraightSuperKick.getStateInfoPtr(core.KickState.FINISHSTAND)
info.state_time = 500
info.joint_time = 500
info.com = core.vector3_float(comOffsetX,50,comHeight)
info.swing = core.vector3_float(0,100,0)   

# SPLINE STATE
info = StraightSuperKick.getStateInfoPtr(core.KickState.SPLINE)
info.state_time = 5000
info.joint_time = info.state_time
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,0)

StraightSuperKick.l_hip_roll_before_ = -0.4;
StraightSuperKick.l_hip_roll_after_ = -0.4;
StraightSuperKick.l_hip_pitch_before_ = -0.2;
StraightSuperKick.l_hip_pitch_after_ = -1.12;
StraightSuperKick.l_knee_pitch_before_ = 1.4;
StraightSuperKick.l_knee_pitch_after_ = 1.4;
StraightSuperKick.l_ankle_roll_before_ = 0.8;
StraightSuperKick.l_ankle_roll_after_ = 0.8;
StraightSuperKick.l_ankle_pitch_before_ = -0.95;
StraightSuperKick.l_ankle_pitch_after_ = -0.4;

StraightSuperKick.step_into_kick_ = True
StraightSuperKick.use_stance_spline = False
StraightSuperKick.kick_time_ = (450 + kick_time) / 1000.0
StraightSuperKick.offset_ = 10

StraightSuperKick.swing_length_ = 120
StraightSuperKick.swing_time_ = 200

StraightSuperKick.align_height_ = 32.5
StraightSuperKick.kick_height_ = 32.5

# ideal ball placements - i.e. what the kick was developed for
StraightSuperKick.ideal_ball_side_left_swing_ = 15
StraightSuperKick.ideal_ball_side_right_swing_ = -30

##############################


def setRobotSpecificKickParams():
  id = robot_state.robot_id_
  if id == 92: # Gouda
    UTdebug.log(0,'Gouda Parameters')
    #StraightKick.ideal_ball_side_right_swing_ = 20
  else:
    UTdebug.log(0,'*** No Robot specific kick params')
  UTdebug.log(0,'Done setting robot specific kick params')


kick_speech_text = [
  "Large Gap",
  "Small Gap",
  "Long Straight",
  "Super Straight",
  "Mid Straight",
  "Pass 4",
  "Pass 3",
  "Pass 2",
  "Short Straight",
  "Dribble",
  "Walk Front",
  "Walk Left",
  "Walk Right",
  "Walk Left Side",
  "Walk Right Side"
]
