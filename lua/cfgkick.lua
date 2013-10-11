cfgStraightKick = core.KickParameters()

-- times in milliseconds
local liftAmount = 40 --25
local backAmount = -80
local throughAmount = 80
local liftAlignAmount = 60
local liftKickAmount = 25
local comHeight = 175 -- this should be the height of the walk
local comOffset = 10
local comOffsetX = 15
local info
local kick_time = 0

info = cfgStraightKick:getStateInfoPtr(core.KickState_STAND)
info.state_time = 200 -- decrease this if possible
info.joint_time = 200
info.com = core.vector3_float(comOffsetX,50,comHeight)
info.swing = core.vector3_float(0,100,0)
kick_time = kick_time + info.state_time

info = cfgStraightKick:getStateInfoPtr(core.KickState_SHIFT)
info.state_time = 150
info.joint_time = 150
info.com = core.vector3_float(comOffsetX,comOffset + 30,comHeight)
info.swing = core.vector3_float(0,100,0)
kick_time = kick_time + info.state_time

info = cfgStraightKick:getStateInfoPtr(core.KickState_LIFT)
info.state_time = 50
info.joint_time = 50
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,10)
kick_time = kick_time + info.state_time

info = cfgStraightKick:getStateInfoPtr(core.KickState_ALIGN)
info.state_time = 150
info.joint_time = 150
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(backAmount,100,liftAlignAmount)
kick_time = kick_time + info.state_time

info = cfgStraightKick:getStateInfoPtr(core.KickState_KICK1)
info.state_time = 0
info.joint_time = 50
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,(liftKickAmount+liftAlignAmount)/2)
kick_time = kick_time + info.state_time

info = cfgStraightKick:getStateInfoPtr(core.KickState_KICK2)
info.state_time = 0
info.joint_time = 50
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(throughAmount,100,liftKickAmount)
kick_time = kick_time + info.state_time

info = cfgStraightKick:getStateInfoPtr(core.KickState_RESETFOOT)
info.state_time = 150
info.joint_time = 150
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,liftAmount)

info = cfgStraightKick:getStateInfoPtr(core.KickState_FOOTDOWN)
info.state_time = 50
info.joint_time = 50
info.com = core.vector3_float(comOffsetX,comOffset + 20,comHeight)
info.swing = core.vector3_float(0,100,liftAmount/2)

info = cfgStraightKick:getStateInfoPtr(core.KickState_SHIFTBACK)
info.state_time = 500 --75
info.joint_time = 75
info.com = core.vector3_float(comOffsetX,50,comHeight)
info.swing = core.vector3_float(0,100,0)

info = cfgStraightKick:getStateInfoPtr(core.KickState_FINISHSTAND)
info.state_time = 500
info.joint_time = 500
info.com = core.vector3_float(comOffsetX/2,50,comHeight)
info.swing = core.vector3_float(0,100,0)   

-- SPLINE STATE
info = cfgStraightKick:getStateInfoPtr(core.KickState_SPLINE)
info.state_time = 100
info.joint_time = info.state_time
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,0)

-- these aren't used for this kick, but are needed to make lua happy
cfgStraightKick.l_hip_roll_before_ = -0.4;
cfgStraightKick.l_hip_roll_after_ = -0.4;
cfgStraightKick.l_hip_pitch_before_ = 0.2;
cfgStraightKick.l_hip_pitch_after_ = -1.12;
cfgStraightKick.l_knee_pitch_before_ = 1.4;
cfgStraightKick.l_knee_pitch_after_ = 1.4;
cfgStraightKick.l_ankle_roll_before_ = 0.8;
cfgStraightKick.l_ankle_roll_after_ = 0.8;
cfgStraightKick.l_ankle_pitch_before_ = -0.95;
cfgStraightKick.l_ankle_pitch_after_ = -0.4;

cfgStraightKick.step_into_kick_ = false
cfgStraightKick.use_stance_spline = false
cfgStraightKick.kick_time_ = (450 + kick_time) / 1000.0
cfgStraightKick.offset_ = 10

cfgStraightKick.swing_length_ = 120
cfgStraightKick.swing_time_ = 200

cfgStraightKick.align_height_ = 32.5
cfgStraightKick.kick_height_ = 32.5

-- ideal ball placements - i.e. what the kick was developed for
cfgStraightKick.ideal_ball_side_left_swing_ = 15
cfgStraightKick.ideal_ball_side_right_swing_ = -30

------------------------------------------------------------
-- will eventually be the params for a Super Long Kick
cfgStraightSuperKick = core.KickParameters()
-- times in milliseconds
local liftAmount = 40 --25
local backAmount = -40
local throughAmount = 100
local liftAlignAmount = 40
local liftKickAmount = 50
local comHeight = 175 -- this should be the height of the walk
local comOffset = -5 -- can balance on one foot with this offset
local comOffsetX = 15
local info
local kick_time = 0

info = cfgStraightSuperKick:getStateInfoPtr(core.KickState_STAND)
info.state_time = 300 -- decrease this if possible
info.joint_time = 300
info.com = core.vector3_float(comOffsetX,50,comHeight)
info.swing = core.vector3_float(0,100,0)
kick_time = kick_time + info.state_time

info = cfgStraightSuperKick:getStateInfoPtr(core.KickState_SHIFT)
info.state_time = 250
info.joint_time = 250
info.com = core.vector3_float(comOffsetX,comOffset + 30,comHeight)
info.swing = core.vector3_float(0,100,0)
kick_time = kick_time + info.state_time

info = cfgStraightSuperKick:getStateInfoPtr(core.KickState_LIFT)
info.state_time = 350
info.joint_time = 350
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,10)
kick_time = kick_time + info.state_time

info = cfgStraightSuperKick:getStateInfoPtr(core.KickState_ALIGN)
info.state_time = 2500
info.joint_time = 1500
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(backAmount,100,liftAlignAmount) -- -80,100,60
kick_time = kick_time + info.state_time

info = cfgStraightSuperKick:getStateInfoPtr(core.KickState_KICK1)
info.state_time = 0
info.joint_time = 50
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,(liftKickAmount+liftAlignAmount)/2)
kick_time = kick_time + info.state_time

info = cfgStraightSuperKick:getStateInfoPtr(core.KickState_KICK2)
info.state_time = 0
info.joint_time = 50
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(throughAmount,100,liftKickAmount) -- 80,100,35
kick_time = kick_time + info.state_time

info = cfgStraightSuperKick:getStateInfoPtr(core.KickState_RESETFOOT)
info.state_time = 3000
info.joint_time = 3000
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,liftAmount)

info = cfgStraightSuperKick:getStateInfoPtr(core.KickState_FOOTDOWN)
info.state_time = 150
info.joint_time = 150
info.com = core.vector3_float(comOffsetX,comOffset + 20,comHeight)
info.swing = core.vector3_float(0,100,liftAmount/2)

info = cfgStraightSuperKick:getStateInfoPtr(core.KickState_SHIFTBACK)
info.state_time = 500 --75
info.joint_time = 250
info.com = core.vector3_float(comOffsetX,50,comHeight)
info.swing = core.vector3_float(0,100,0)

info = cfgStraightSuperKick:getStateInfoPtr(core.KickState_FINISHSTAND)
info.state_time = 500
info.joint_time = 500
info.com = core.vector3_float(comOffsetX,50,comHeight)
info.swing = core.vector3_float(0,100,0)   

-- SPLINE STATE
info = cfgStraightSuperKick:getStateInfoPtr(core.KickState_SPLINE)
info.state_time = 5000
info.joint_time = info.state_time
info.com = core.vector3_float(comOffsetX,comOffset,comHeight)
info.swing = core.vector3_float(0,100,0)

cfgStraightSuperKick.l_hip_roll_before_ = -0.4;
cfgStraightSuperKick.l_hip_roll_after_ = -0.4;
cfgStraightSuperKick.l_hip_pitch_before_ = -0.2;
cfgStraightSuperKick.l_hip_pitch_after_ = -1.12;
cfgStraightSuperKick.l_knee_pitch_before_ = 1.4;
cfgStraightSuperKick.l_knee_pitch_after_ = 1.4;
cfgStraightSuperKick.l_ankle_roll_before_ = 0.8;
cfgStraightSuperKick.l_ankle_roll_after_ = 0.8;
cfgStraightSuperKick.l_ankle_pitch_before_ = -0.95;
cfgStraightSuperKick.l_ankle_pitch_after_ = -0.4;

cfgStraightSuperKick.step_into_kick_ = true
cfgStraightSuperKick.use_stance_spline = false
cfgStraightSuperKick.kick_time_ = (450 + kick_time) / 1000.0
cfgStraightSuperKick.offset_ = 10

cfgStraightSuperKick.swing_length_ = 120
cfgStraightSuperKick.swing_time_ = 200

cfgStraightSuperKick.align_height_ = 32.5
cfgStraightSuperKick.kick_height_ = 32.5

-- ideal ball placements - i.e. what the kick was developed for
cfgStraightSuperKick.ideal_ball_side_left_swing_ = 15
cfgStraightSuperKick.ideal_ball_side_right_swing_ = -30

------------------------------------------------------------


function setRobotSpecificKickParams()
  local id = robot_state.robot_id_
  if id == 92 then -- Gouda
    UTdebug.log(0,'Gouda Parameters')
    --cfgStraightKick.ideal_ball_side_right_swing_ = 20
  else
    UTdebug.log(0,'*** No Robot specific kick params')
  end
  UTdebug.log(0,'Done setting robot specific kick params')
end


kick_speech_text = { 
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
}
