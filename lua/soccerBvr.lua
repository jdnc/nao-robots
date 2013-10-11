require 'body'
require 'commands'
require 'head'
require 'roleSwitch'
require 'skills'
require 'stateMachine'
require 'task'
require 'testBvr'
module(..., package.seeall);

-- This is the behavior which is actived when the Nao is set
-- to the Playing state. Currently it approaches and kicks
-- the ball.
CompTask('soccerBvr_PlayingState',{
           lastState = core.INITIAL,
           init = true
         })
function soccerBvr_PlayingState:choose(as)

  -- end of half, losing, do desperation kick
  --if (game_state.secsRemaining < 30 and (game_state.ourScore - game_state.opponentScore) < 0) then
  if (game_state.secsRemaining < 30 and (game_state.ourScore - game_state.opponentScore) < 0) then
    UTdebug.log(0, "time nearly out", game_state.secsRemaining, "DESPERATION strategy")
    behavior_params.mainStrategy = cfgstrategy.cfgDesperationStrategy
    behavior_params.clusterStrategy = cfgstrategy.cfgNoClusterStrategy
  end

  if (game_state.isPenaltyKick and game_state.state == core.PLAYING) then
    UTdebug.log(10, "set pen kick strategy", game_state.secsRemaining, (game_state.secsRemaining > 47))
    -- start with early strategy - superKick
    if (game_state.secsRemaining > 47) then
      behavior_params.mainStrategy = cfgstrategy.cfgPenaltyKickEarlyStrategy
    else
      -- then switch - permissive superKick
      behavior_params.mainStrategy = cfgstrategy.cfgPenaltyKickStrategy
    end
  end

  if (self.init) then
    self.init = false
    commands.setStiffnessCommands(cfgStiffOne,0.1)
    commands.setHeadTilt()
  end
  
  if (game_state.lastStateChangeFromButton and (vision_frame_info.seconds_since_start - game_state.lastTimeLeftPenalized < 7.0)) then
    commands.stand()
    return head_SweepScan:set{period = 3.0, maxPan = 105.0*DEG_T_RAD, numSweeps = 4} -- stay in ready for longer by doing more sweeps
  end
  
  -- handle the new rules about entering the center circle on their kickoff
  local timeInPlaying = vision_frame_info.seconds_since_start - behavior_mem.timePlayingStarted
  local ball = world_objects:getObjPtr(core.WO_BALL)
  if not(game_state.ourKickOff) and (timeInPlaying < 10) and (ball.loc:getDistanceTo(core.Point2D(0,0)) < 250) then
    return skills_MoveHomeBAL:set{period = 3.0, maxPan = 105.0*DEG_T_RAD}
  end

  -- should we switch roles? only if not keeper and not penKick
  -- no role switch for first second in playing

  if (not game_state.isPenaltyKick and (vision_frame_info.frame_id % 5) == 0) then
    roleSwitch.roleSwitch()
  end
  
  if (robot_state.WO_SELF == core.KEEPER) then
    roleSwitch.checkKeeper()
  end

  -- varies based on what player we are
  if (robot_state.role_ == core.CHASER) then
    return skills_PlayAttacker:set()
  elseif (robot_state.role_ == core.KEEPER) then
    if (game_state.isPenaltyKick) then
      return skills_PlayPenaltyKickKeeper:set()
    else
      return skills_PlaySweepingKeeper:set()
    end
    -- dont switch off chasing while kicking!
  elseif (kick_request.kick_running_) then
    return skills_PlayAttacker:set()
  elseif (game_state.isPenaltyKick) then
    return skills_PlayAttacker:set()
  elseif (robot_state.role_ == core.SET_PLAY_RECEIVER) then
    return skills_PlaySetPlayReceiver:set()
  else
    local role = behavior_params.roleStrategy:getRolePositionConfigPtr(robot_state.role_)
    return skills_PlayPosition:set{role=role}
  --elseif (robot_state.role_ == core.DEFENDER) then
    --return skills_PlayDefender:set()
  --elseif (robot_state.role_ == core.SUPPORTER) then
    --return skills_PlaySupporter:set()
  --elseif (robot_state.role_ == core.FORWARD) then
    --return skills_PlayForward:set()
  --elseif (robot_state.role_ == core.MIDFIELD) then
    --return skills_PlayMidfield:set()
  --else
    --print('UNKNOWN ROLE:',robot_state.role_)
    --return task_NullTask:set()
  end
end

-- stuff for initial state
CompTask('soccerBvr_InitialState',{
           init = true,
           lower_stiff = false
         })
function soccerBvr_InitialState:choose(as)

  if (self.init) then
    self.init = false
    commands.setStiffnessCommands(cfgStiffOne,0.25)
    kick_request:setNoKick()
    walk_request:noWalk()
    kick_request.kick_running_ = false
    behavior_mem.keeperDiving = core.Dive_NONE
    commands.setHeadTilt()
    return task_NullTask:set()
  end

  -- after we've stood up, set standing stiffness
  if (self:getTime() > 3.3 and not self.lower_stiff) then
    self.lower_stiff = true
    if (robot_state.WO_SELF == core.KEEPER) then
      commands.setStiffnessCommands(cfgStiffKeeperStand,0.25)
    else
      commands.setStiffnessCommands(cfgStiffStand,0.25)
    end
  end

  -- wait for stiffness to go to 1
  if (self:getTime() > 0.3) then
    commands.stand()
    return head_MoveHead:set{}
  end

end

-- stuff for ready state
CompTask('soccerBvr_ReadyState',{
           init = true,
           lastState = core.INITIAL,
           state = stateMachine.StateMachine.create('Ready State',{'scan','position'})
         })
function soccerBvr_ReadyState:choose(as)

  -- should we switch roles? only if not penKick
  -- no role switch for first second of ready
  if (not game_state.isPenaltyKick and (vision_frame_info.frame_id % 5) == 0 and self:getTime() > 1.0) then
    roleSwitch.roleSwitch()
  end

  -- init
  if (self.init) then
    strategy.setStrategy()
    walk_request.keep_arms_out_ = (robot_state.WO_SELF == core.KEEPER)
    self.init = false
    -- set role back to player id
    robot_state.role_ = robot_state.WO_SELF
    -- except actually on kick, player 2 should be a forward
    --if (robot_state.WO_SELF == 2) then
    --  robot_state.role_ = core.FORWARD
    --end
    commands.setHeadTilt()

    behavior_mem.keeperClearing = false
    behavior_mem.keeperDiving = core.Dive_NONE
    kick_request.kick_running_ = false
    kick_request:setNoKick()
    commands.stand()
    self.state:transition('scan')
    commands.setStiffnessCommands(cfgStiffOne,0.1)
    -- straight to position if coming from anything but finished
    if (self.lastState ~= core.FINISHED and self.lastState ~= core.INITIAL) then
      self.state:transition('position')
    end
  end

  if (as ~= nil and as:finished()) then
    if (self.state:inState('scan')) then
      self.state:transition('position')
    end
  end
  if (robot_state.WO_SELF == core.KEEPER) then
    walk_request.keep_arms_out_ = true
  end

  -- dont stand there scanning for more than 10 seconds
  if (self.state:inState('scan') and self:getTime() > 10) then
    self.state:transition('position')
  end

  if (self.state:inState('scan')) then
    commands.stand()
    return head_SweepScan:set{period = 3.0, maxPan = 105.0*DEG_T_RAD, numSweeps = 4} -- stay in ready for longer by doing more sweeps
  elseif (self.state:inState('position')) then
    -- auto-position during ready
    -- not for keeper if far away
    if (robot_state.WO_SELF == core.KEEPER and world_objects:getObjPtr(robot_state.WO_SELF).loc:getDistanceTo(core.Point2D(-2800,0)) > 1400) then
      commands.stand()
      return head_LookForBall:set{}
    else
      return skills_MoveHomeSweepScan:set{period = 3.0, maxPan = 105.0*DEG_T_RAD}
    end
  end
end



-- stuff for set state
CompTask('soccerBvr_SetState',{
           init = true,
           lower_stiff = false
         })
function soccerBvr_SetState:choose(as)
  if (self.init) then
    commands.setHeadTilt()

    kick_request:setNoKick()
    commands.stand()
    commands.setStiffnessCommands(cfgStiffOne,0.1)
    self.init = false
    behavior_mem.keeperDiving = core.Dive_NONE
  end

  -- should we switch roles? only if not keeper and not penKick
  if (not game_state.isPenaltyKick and (vision_frame_info.frame_id % 5) == 0) then
    roleSwitch.roleSwitch()
  end

  -- after its gone from sit->stand, lower stiffness
  if (self:getTime() > 3.0 and not self.lower_stiff) then
    self.lower_stiff = true
    if (robot_state.WO_SELF == core.KEEPER) then
      commands.setStiffnessCommands(cfgStiffKeeperStand,0.1)
    else
      commands.setStiffnessCommands(cfgStiffStand,0.1)
    end
  end

  -- normal
  --return skills_ToPoseLookForBall:set{pose = standingPose}
  commands.stand()
  if (robot_state.WO_SELF == core.KEEPER) then
    walk_request.keep_arms_out_ = true
  else
    walk_request.keep_arms_out_ = false
  end
  return head_LookForBall:set{}
end

-- stuff for set state
CompTask('soccerBvr_PKSetState',{
           init = true,
           lower_stiff = false
         })
function soccerBvr_PKSetState:choose(as)
  if (self.init) then
    commands.setHeadTilt()

    behavior_mem.keeperDiving = core.Dive_NONE
    kick_request.kick_running_ = false
    kick_request:setNoKick()
    walk_request:noWalk()
    commands.setStiffnessCommands(cfgStiffOne,0.1)
    self.init = false
  end

  -- after its gone from sit->stand, lower stiffness
  if (self:getTime() > 3.0 and not self.lower_stiff) then
    self.lower_stiff = true
    if (robot_state.WO_SELF == core.KEEPER) then
      commands.setStiffnessCommands(cfgStiffKeeperStand,0.1)
    else
      commands.setStiffnessCommands(cfgStiffStand,0.1)
    end
  end

  -- for pk, we want to localize while we stand here
  -- cause we know where the ball is
  commands.stand()
  return head_SweepScan:set{period = 3.0, maxPan = 105.0*DEG_T_RAD}

end

-- stuff for set state
CompTask('soccerBvr_PenalisedState',{
           init = true
         })
function soccerBvr_PenalisedState:choose(as)

  if (self.init) then
    if (robot_state.WO_SELF == core.KEEPER) then
      commands.setStiffnessCommands(cfgStiffKeeperStand,0.1)
    else
      commands.setStiffnessCommands(cfgStiffStand,0.1)
    end
    kick_request:setNoKick()
    commands.stand()
    behavior_mem.keeperDiving = core.Dive_NONE
    self.init = false
    commands.setHeadTilt()

  end

  -- look for circle
  commands.stand()
  -- look straight ahead, hopefully see circle to one side (since its unique)
  return head_MoveHead:set{time = 0.5, pan = DEG_T_RAD*0.0} 

end

-- stuff for finished state
CompTask('soccerBvr_FinishedState',{
           init = true,
           state = 0,
           lower_time = 0,
           skippedState = false
         })
function soccerBvr_FinishedState:choose(as)
  if (self.init) then
    self:resetTime()
    self.init = false
    self.state = 0
    self.skippedState = false
    kick_request:setNoKick()
    walk_request:noWalk()
    kick_request.kick_running_ = false
    behavior_mem.keeperDiving = core.Dive_NONE
    --commands.setArmPose(armSidePose,2000)
    --joint_commands.body_angle_time_ = 2000
    commands.setHeadTilt()

    local shoulderCutoff = DEG_T_RAD * -90
    if (percepts.joint_angles[core.LShoulderPitch] > shoulderCutoff) and (percepts.joint_angles[core.RShoulderPitch] > shoulderCutoff) then
      self.state = 1
      self.skippedState = true
    end
  end

  -- first 2 seconds
  -- tell walk to stand
  if (self:getTime() < 2.0) then
    walk_request:noWalk()
    kick_request:setNoKick()
    commands.setStiffnessCommands(cfgStiffOne, 0.3)
    return task_NullTask:set()
  end

  if self.state == 0 then
    self.state = 1
    local pose = deepcopy(sittingPoseV3)
    for joint,val in pairs(armSidePose) do
      pose[joint] = val
    end
    return skills_ToPoseMoveHead:set{tilt = 0.0*DEG_T_RAD, pose=pose, init = first_in_state,init=true}
  elseif self.state == 1 then
    if as:finished() or self.skippedState then
      self.state = 2
      self.skippedState = false
      return skills_ToPoseMoveHead:set{tilt = 0.0*DEG_T_RAD,pose=sittingPoseV3,time=1.0,init=true}
    end
  elseif self.state == 2 then
    if as:finished() then
      self.state = 3
      self.lower_time = self:getTime()
      commands.setStiffnessCommands(cfgStiffZeroKneeAnklePitch, 0.3)
    end
  elseif self.state == 3 then
    if self:getTime() - self.lower_time > 0.7 then
      self.state = 4
      commands.setStiffnessCommands(cfgStiffZero, 0.3)
    end
  end

  return as
end

-- the main state selection
CompTask('SoccerBehavior',{
           lastState = core.INITIAL,
           currState = core.INITIAL
         })
function SoccerBehavior:choose(as)
  -- set penalty kick(1) or no kick (0) in init.lua!!

  self.lastState = self.currState
  self.currState = game_state.state

  if (self.currState == core.PLAYING) and (self.lastState ~= core.PLAYING) and (self.lastState ~= core.PENALISED) then
    -- switched into playing, from something other than penalized
    behavior_mem.timePlayingStarted = vision_frame_info.seconds_since_start
  end

  -- check for high temps
  -- stiffness drops with temp > 75
  if ((vision_frame_info.frame_id % 30) == 0) then
    checkTemperatures()
  end

  if ((vision_frame_info.frame_id % 30) == 0) then
    checkCommunication()
  end

  -- select a set play
  strategy.selectSetPlay()

  -- dont do stuff while getting up
  if (odometry.getting_up_side_ ~= core.Getup_NONE) then
    setFallCounter()
    UTdebug.log(20, "odom getting up, set no walk or kick")
    walk_request:noWalk()
    kick_request:abortKick()
    self.currState = core.FALLING
    return task_NullTask:set()
  end
  
  if (self.lastState == core.FALLING) and (self.currState ~= core.FALLING) then
    -- we're not doing a getup from the keeper dive for sure now
    walk_request.getup_from_keeper_dive_ = false
  end

  if (checkFallen()) then
    UTdebug.log(0, "is falling, call getup")
    -- might as well turn stiffness off if we havent quite fallen yet
    commands.setStiffnessCommands(cfgStiffZero, 0.01)
    kick_request:abortKick()
    self.currState = core.FALLING

    -- dont get up in center of field
    local me = world_objects:getObjPtr(robot_state.WO_SELF)
    if (false and core.Point2D(0,0):getDistanceTo(me.loc) < 800 and state == core.PLAYING) then
      UTdebug.log(10, "in center of field, dont do getup, take penalty")
      commands.stand()
      return task_NullTask:set()
    end
    walk_request:setFalling()
    return task_NullTask:set()
  end

  if (self.currState == core.PLAYING) then
    return soccerBvr_PlayingState:set{ lastState = self.lastState }
    -- PK, non playing mode (whether ready, set or pen)
  elseif (self.currState == core.FINISHED) then
    return soccerBvr_FinishedState:set()
  elseif (self.currState == core.INITIAL) then
    --return soccerBvr_InitialState:set()
    return soccerBvr_FinishedState:set()
  elseif (game_state.isPenaltyKick) then
    return soccerBvr_PKSetState:set()
  elseif (self.currState == core.READY) then
    --return testBvr_TestWalk:set{fwd = 1, side = 0, turn = 0.1}
    --return testBvr_TestingState:set()
    return soccerBvr_ReadyState:set{ lastState = self.lastState }
  elseif (self.currState == core.SET) then
    return soccerBvr_SetState:set()
  elseif (self.currState == core.TESTING) then
    return testBvr_TestingState:set()
  elseif (self.currState == core.PENALISED) then
    return soccerBvr_PenalisedState:set()
  elseif (self.currState == core.TEST_ODOMETRY) then
    local init = behavior_mem.test_odom_new
    behavior_mem.test_odom_new = false
    return testBvr_TestWalk:set{fwd = behavior_mem.test_odom_fwd, side = behavior_mem.test_odom_side, turn = behavior_mem.test_odom_turn, walkTime=behavior_mem.test_odom_walk_time,init=init}
  else
    return task_NullTask:set()
  end
end

function setFallCounter()
  local tilt = sensors:getValue(core.angleY)
  local roll = sensors:getValue(core.angleX)
  local tiltFalling = DEG_T_RAD * 45
  local rollFalling = DEG_T_RAD * 45
  if (math.abs(tilt) > tiltFalling) then
    if tilt > 0 then
      -- on stomach
      if walk_request.tilt_fallen_counter_ < 0 then
        walk_request.tilt_fallen_counter_ = 1
      else
        walk_request.tilt_fallen_counter_ = walk_request.tilt_fallen_counter_ + 1
      end
    else
      -- on back
      if walk_request.tilt_fallen_counter_ > 0 then
        walk_request.tilt_fallen_counter_ = -1
      else
        walk_request.tilt_fallen_counter_ = walk_request.tilt_fallen_counter_ - 1
      end
    end
  else
    walk_request.tilt_fallen_counter_ = 0
  end  
  if (math.abs(roll) > rollFalling) then
    if roll > 0 then
      -- on stomach
      if walk_request.roll_fallen_counter_ < 0 then
        walk_request.roll_fallen_counter_ = 1
      else
        walk_request.roll_fallen_counter_ = walk_request.roll_fallen_counter_ + 1
      end
    else
      -- on back
      if walk_request.roll_fallen_counter_ > 0 then
        walk_request.roll_fallen_counter_ = -1
      else
        walk_request.roll_fallen_counter_ = walk_request.roll_fallen_counter_ - 1
      end
    end
  else
    walk_request.roll_fallen_counter_ = 0
  end

  UTdebug.log(10, "fall counter: ", walk_request.tilt_fallen_counter_, walk_request.roll_fallen_counter_)
  return tilt,roll,tiltFalling,rollFalling
end

function checkFallen()

  -- leave this off for now
  --if (true) then
   --return false
  --end

  -- Check if sensors think we have fallen
  --accx = sensors:getValue(core.accelX)
  --accy = sensors:getValue(core.accelY)
  --accz = sensors:getValue(core.accelZ)

  ---- check for bad accel
  --if ((vision_frame_info.frame_id % 30) == 0) then
    --if (math.abs(accx) < 0.01 and math.abs(accy) < 0.01 and math.abs(accz) < 0.01) then
      --UTdebug.log(10, "ERROR, all accelerometers read 0s:", accx, accy, accz)
    --end
  --end

  -- upright should be ~9.8 on z and 0 on x,y
  --if ((math.abs(accz) < 9) and (math.abs(accy) > 4.0 or math.abs(accx) > 4.0)) then
    --if (accx > 0) then
      --if (walk_request.fallen_counter_ < 0) then
        --walk_request.fallen_counter_ = walk_request.fallen_counter_ -1 --on back.
      --else
        --walk_request.fallen_counter_= -1
      --end
    --else
      --if (walk_request.fallen_counter_ > 0) then
        --walk_request.fallen_counter_= walk_request.fallen_counter_ +1 --// on stomach
      --else
        --walk_request.fallen_counter_ = 1;
      --end
    --end
  --else
    --walk_request.fallen_counter_ = 0;
  --end
  
  tilt,roll,tiltFalling,rollFalling = setFallCounter()

  --UTdebug.log(80,"acc:", accx, accy, accz, "fallen counter: ", walk_request.fallen_counter_)
  UTdebug.log(80,"tilt,roll:", tilt, roll, "fallen counter: ", walk_request.tilt_fallen_counter_, walk_request.roll_fallen_counter_)

  -- Check if sensors think we have fallen
  if (math.abs(walk_request.tilt_fallen_counter_) > 2 or math.abs(walk_request.roll_fallen_counter_) > 2) then
    state = game_state.state

    -- set fall direction
    if (roll > rollFalling) then
      odometry.fall_direction_ = core.Fall_RIGHT
      --speech:say("right");
    elseif (roll < -rollFalling) then
      odometry.fall_direction_ = core.Fall_LEFT
      --speech:say("left");
    elseif (tilt > tiltFalling) then
      odometry.fall_direction_ = core.Fall_FORWARD
      --speech:say("forward");
    elseif (tilt < -tiltFalling) then
      odometry.fall_direction_ = core.Fall_BACKWARD
      --speech:say("backward");
    else
      -- default option??
      UTdebug.log(0, "default fall is forward")
      odometry.fall_direction_ = core.Fall_FORWARD
      --speech:say("default");
    end
      
    UTdebug.log(30, "fall detected in direction", odometry.fall_direction_, tilt, roll)
    

    -- Get up only in playing or ready
    if (state == core.PLAYING or state == core.READY) then
      -- and not keeper (keeper will decide on its own when to do get up)
      if (robot_state.WO_SELF ~= core.KEEPER) then
        return true
      else
        -- if keeper is not diving, we can still call get up
        if (behavior_mem.keeperDiving == core.Dive_NONE) then
          return true
        else
          -- wait for dive to tell us to get up
          return false
        end
      end
    end
  end
  odometry.fall_direction_ = core.Fall_NONE
  return false
end

function checkTemperatures()
  for i = 0, core.NUM_JOINTS-1 do
    local temp = sensors:getJointTemperature(i)
    if (temp > 65) then
      -- figure out stiffness
      local stiff = 1.0
      if (temp > 75) then
        local pct = (temp-75.0) / 10.0
        stiff = 1-pct
        if (stiff < 0) then stiff = 0 end
      end
      UTdebug.log(10, "Joint ", i, core.getJointName(i), " has temperature ", temp, " stiffness: ", stiff)
    end
  end
end




function checkCommunication()

  -- see if we've heard from people lately
  for i = core.WO_TEAM_FIRST,core.WO_TEAM_LAST do
    local frameRecv = team_packets:getFrameReceived(i)
    local frames_missed = vision_frame_info.frame_id - frameRecv
    if (frames_missed > 30) then
      -- this one might be ok (no other robots on)
      UTdebug.log(20, "have not heard from ",i,"in ",frames_missed, "frames")
    else
      -- see if they've heard from us
      local missedFromUs = team_packets:getMissedFromUs(i,robot_state.WO_SELF)
      if (missedFromUs > 30) then
        -- this one definitely bad.. they exist, we can hear them, they cant hear us
        UTdebug.log(10, "mate",i,"has not heard from us in", missedFromUs,"frames")      end
    end
  end

end

