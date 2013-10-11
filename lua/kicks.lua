require 'task'

module(...,package.seeall)

-- list of possible kicks
-- Kick Info class decleration
KickInfo = {}
KickInfo.__index = KickInfo
function KickInfo.create(switch,heading,distance,alignment,time,foot,isSlow,isSet)
  local tab = {} 
  setmetatable(tab,KickInfo)
  tab.switchable = switch
  tab.heading = heading
  tab.distance = distance
  tab.alignment = alignment
  tab.foot = foot
  tab.time = time
  tab.isSlow = isSlow -- kicks where we have to stop walking
  tab.isSet = isSet -- for variable kicks (gap, passes), if its set and valid
  return tab
end

KickAlignment = {}
KickAlignment.__index = KickAlignment
function KickAlignment.create(des_x, max_stop_x, des_y, min_stop_y, max_stop_y)
  local tab = {}
  setmetatable(tab,KickAlignment)
  tab.des_x = des_x
  tab.max_stop_x_err = max_stop_x
  tab.des_y = des_y
  tab.min_stop_y_err = min_stop_y
  tab.max_stop_y_err = max_stop_y
  return tab
end

-- des_x, max_stop_x, des_y, min_stop_y, max_stop_y
local fwdKickAlignment       = KickAlignment.create(  15, 5, 30, -10, 15)
local fwdKickSuperAlignment   = KickAlignment.create( 45, 10, 40, -10, 10)
local walkSideKickAlignment  = KickAlignment.create( 10, 10, -15, -15, 10) --(2.5,  5, -5, -10,  5)
local walkFrontKickAlignment = KickAlignment.create( 10, 10, 20, -20, 20)
local walkAngleKickAlignment = KickAlignment.create( 10,  20, 0, -10,  10)
local dribbleAlignment       = KickAlignment.create(45, -999, 0, -50, 50) -- desx not really used for now

-- setup indices to the passes
Passes = {}
Passes[2] = core.FwdPass2Kick
Passes[3] = core.FwdPass3Kick
Passes[4] = core.FwdPass4Kick
Passes[5] = core.FwdPass5Kick

-- Each kick needs set some kick parameters
-- 0) Switchable, i.e. can we choose left or right leg at the last moment
-- 1) Heading of the kick (degrees)
-- 2) Distance, approximate length (mm)
-- 3) Desired Y, location of the ball (to the side) (mm)
-- 4) Execution Time, time to kick (seconds)

local fwdShort = 1500
local fwdMedium = 2400
local fwdLong = 3950
local fwdSuper = 8000
local fwdYOffset = 40.0
local angYOffset = 20.0

kickData = {}
-- short forward kicks
kickData[core.FwdShortStraightKick] = KickInfo.create(true, 0.0, fwdShort, fwdKickAlignment, 2.0, core.Kick_RIGHT, true, false)

-- medium forward kicks
kickData[core.FwdMediumStraightKick] = KickInfo.create(true, 0.0, fwdMedium, fwdKickAlignment, 2.0, core.Kick_RIGHT, true, true)

-- long forward kicks
kickData[core.FwdLongStraightKick] = KickInfo.create(true, 0.0, fwdLong, fwdKickAlignment, 2.0, core.Kick_RIGHT, true, true)

-- super forward kicks
kickData[core.FwdSuperStraightKick] = KickInfo.create(true, 0.0, fwdSuper, fwdKickSuperAlignment, 5.6, core.Kick_RIGHT, true, true)

-- aiming for goal gaps
kickData[core.FwdLongLargeGapKick] = KickInfo.create(true, 0.0, fwdLong, fwdKickAlignment, 2.0, core.Kick_RIGHT, true, false)
kickData[core.FwdLongSmallGapKick] = KickInfo.create(true, 0.0, fwdLong, fwdKickAlignment, 2.0, core.Kick_RIGHT, true, false)

-- passing to teammates
kickData[core.FwdPass5Kick] = KickInfo.create(true, 0.0, fwdLong, fwdKickAlignment, 2.0, core.Kick_RIGHT, true, false)
kickData[core.FwdPass4Kick] = KickInfo.create(true, 0.0, fwdLong, fwdKickAlignment, 2.0, core.Kick_RIGHT, true, false)
kickData[core.FwdPass3Kick] = KickInfo.create(true, 0.0, fwdLong, fwdKickAlignment, 2.0, core.Kick_RIGHT, true, false)
kickData[core.FwdPass2Kick] = KickInfo.create(true, 0.0, fwdLong, fwdKickAlignment, 2.0, core.Kick_RIGHT, true, false)

-- really fast kick
kickData[core.Dribble] = KickInfo.create(true, 0.0, 250.0, dribbleAlignment, 0.5, core.Kick_RIGHT, false, true)

--function KickInfo.create(switch,heading,distance,y,time,foot,isSet)
-- kicking from the walk engine
kickData[core.WalkKickFront] = KickInfo.create(true,0.0,1500.0,walkFrontKickAlignment,0.25,core.Kick_RIGHT,false, true)
kickData[core.WalkKickLeftwardSide] = KickInfo.create(false,DEG_T_RAD * 90,500.0,walkSideKickAlignment,0.25,core.Kick_RIGHT,false, true)
kickData[core.WalkKickRightwardSide] = KickInfo.create(false,DEG_T_RAD * -90,500.0,walkSideKickAlignment,0.25,core.Kick_LEFT,false, true)
kickData[core.WalkKickLeftward] = KickInfo.create(false,DEG_T_RAD * 45,1000.0,walkAngleKickAlignment,0.25,core.Kick_RIGHT,false, true)
kickData[core.WalkKickRightward] = KickInfo.create(false,DEG_T_RAD * -45,1000.0,walkAngleKickAlignment,0.25,core.Kick_LEFT,false, true)

function checkKicks()
  -- check for kicks we haven't set up corrects
  for i = 0, core.NUM_KICKS-1 do
    if kickData[i] == nil then
      UTdebug.log(0,'*** KICK DATA MISSING FOR:',luaC:getString(core.kickNames,i))
      kickData[i] = KickInfo.create(true,0,0,fwdKickAlignment,10.0,core.Kick_LEFT,true,false)
    end
  end
end

CompTask('kicks_ExecuteKick',{
           init = true,
           kickRunning = false,
           desiredDistance = 3000,
           desiredHeading = 0.0,
           postKick = false,
           kickChoice = core.FwdLongStraightKick,
           foot = core.Kick_SWITCHABLE,
           params = NIL_INIT,
           setKickRequest = false,
           prevFinishedWithStep = false
         })
function kicks_ExecuteKick:choose(as)
  -- set some ball stuff for all kicks
  local ball = world_objects:getObjPtr(core.WO_BALL)
  kick_request.ball_seen_ = ball.seen
  kick_request.ball_image_center_x_ = ball.imageCenterX
  kick_request.ball_image_center_y_ = ball.imageCenterY
  kick_request.ball_rel_x_ = localizationC.filtered_close_ball_.x
  kick_request.ball_rel_y_ = localizationC.filtered_close_ball_.y

  commands.last_arm_command = vision_frame_info.seconds_since_start
  -- handle walk kicks
  if (self.kickChoice == core.WalkKickFront) or (self.kickChoice == core.WalkKickLeftwardSide) or (self.kickChoice == core.WalkKickRightwardSide) or (self.kickChoice == core.WalkKickLeftward) or (self.kickChoice == core.WalkKickRightward)  then
    if as == nil then
      self.kickRunning = true
    else
      self.kickRunning = as.kickRunning
    end
    return kicks_WalkKick:set{kickType=self.kickChoice,desiredHeading=self.desiredHeading,desiredDistance=self.desiredDistance}
  end
  
  if (self.init) then 
    self.init = false
    self:resetTime()
    self.setKickRequest = false
    kick_request.kick_running_ = true
    kick_request.finished_with_step_ = false
    self.prevFinishedWithStep = false
    -- get params
    self.params = kick_params.params_
    if (self.kickChoice == core.FwdSuperStraightKick) then
      self.params = kick_params.params_super_
    end
    -- set foot
    if kickData[self.kickChoice].switchable then
      if self.params.step_into_kick_ then
        -- step into kick needs a specific foot
        if kick_request.ball_rel_y_ > 0 then
          self.foot = core.Kick_LEFT
        else
          self.foot = core.Kick_RIGHT
        end
      else
        self.foot = core.Kick_SWITCHABLE
      end
    else
      self.foot = kickData[self.kickChoice].foot
    end
  end
  if not(self.prevFinishedWithStep) and kick_request.finished_with_step_ then
    self:resetTime()
  end
  self.prevFinishedWithStep = kick_request.finished_with_step_

  -- handle some initial funniness by making sure we know kick is running
  if self:getTime() < 0.15 then
    kick_request.kick_running_ = true
  end
  -- handle super kick
  if (self.kickChoice == core.FwdSuperStraightKick) then
    if (as == nil) or (as == task_NullTask) then
      kick_request.kick_running_ = true
    else
      kick_request.kick_running_ = not as.done
    end
  end
  -- handle step into kick
  if self.params.step_into_kick_ and not(kick_request.finished_with_step_) then
    kick_request.kick_running_ = true
  end
    
  self.kickRunning = kick_request.kick_running_

  -- handle step into kick
  if self.params.step_into_kick_ and not(kick_request.finished_with_step_) then
    UTdebug.log(10,'Setting walk request to step into kick')
    walk_request:setKick(self.desiredDistance,self.desiredHeading,self.foot == core.Kick_LEFT,true)
    kick_request:setNoKick()
    --if (self.kickChoice == core.FwdLongStraightKick or self.kickChoice == core.FwdMediumStraightKick or self.kickChoice == core.FwdShortStraightKick) then
    --  walk_request:setKickStepParams(7,core.Pose2D(0,0,0),core.Pose2D(0,0,0),0)
    --end
    return task_NullTask:set()
  end

  -- super kick
  if (self.kickChoice == core.FwdSuperStraightKick) then
    walk_request:noWalk()
    return body_LongKick:set{stepInto=self.params.step_into_kick_,rightLeg=self.foot==core.Kick_RIGHT}
  end
  
  UTdebug.log(10,'kick running:',self.kickRunning)
  -- if not doing a kick, get out of here
  if not(self.kickRunning) then
    return task_NullTask:set()
  end


  -- katie's kick
  if not(self.setKickRequest) then
    self.setKickRequest = true
    -- set initial kick request
    kick_request:set(self.kickChoice, self.foot, self.desiredHeading, self.desiredDistance)
  end

  walk_request:noWalk()

  UTdebug.log(10,"kick running", kick_request.kick_running_, self.kickRunning)

  kick_request.allow_correction_walk_ = false --true
  --UTdebug.log(0,"vision ball loc:",ball.imageCenterX,ball.imageCenterY)

  kick_request.camera_tilt_offset_ = 0 

  if (odometry.didKick) then
    self.postKick = true
  end

  -- always set kick request in case its changing
  -- up until actual kick execution
  if (not self.postKick) then
    if (kickData[self.kickChoice].switchable) and not(self.params.step_into_kick_) then
      kick_request.kick_leg_ = core.Kick_SWITCHABLE
    else
      kick_request.kick_leg_ = kickData[self.kickChoice].foot
    end
    kick_request.desired_angle_ = self.desiredHeading
    kick_request.desired_distance_ = self.desiredDistance
  end

  if (self.postKick) then
    -- set head to look for ball
    -- look at point for 0.75 seconds, then call look for ball
    if (as == head_LookForBall or (as == head_LookAtPoint and as:getTime() > 1.0)) then
      return head_LookForBall:set()
    end
    
    return head_LookAtPoint:set{dist = self.desiredDistance, bear = self.desiredHeading}
  end

  return task_NullTask:set()
end

function kicks_ExecuteKick:finished()
  return not self.kickRunning
end

CompTask('kicks_WalkKick',{
    init = true,
    kickRunning = false,
    desiredDistance = 1000,
    desiredHeading = 0,
    kickType = core.WalkKickFront,
    leftFoot = true
  })
function kicks_WalkKick:choose(as)
  if self.init then
    local ball = world_objects:getObjPtr(core.WO_BALL)
    self.init = false
    self.kickRunning = true
    if self.kickType == core.WalkKickFront then
      self.leftFoot = localizationC.filtered_close_ball_.y > 0
    elseif (self.kickType == core.WalkKickLeftwardSide) or (self.kickType == core.WalkKickLeftward) then
      self.leftFoot = false
    elseif (self.kickType == core.WalkKickRightwardSide) or (self.kickType == core.WalkKickRightward) then
      self.leftFoot = true
    else
      UTdebug.log(0,'Invalid kick type sent to kicks_WalkKick:',self.kickType,luaC:getString(core.kickNames,self.kickType))
      self.leftFoot = true
    end
  end
  if self.kickRunning then
    commands.setWalkVelocity(0,0,0,false)
    walk_request:setKick(self.desiredDistance,self.desiredHeading,self.leftFoot,false)
    walk_request.new_command_ = true
  end
  if (odometry.didKick) or self:getTime() > 1.0 then
    self.kickRunning = false
  end
  if true then
    return head_LookForBall:set()
  end
end

function kicks_WalkKick:finished()
  return not self.kickRunning
end



CompTask('kicks_QuickStepKick',{
           state = 0,
           isLeft = true,
           done = false
  })
function kicks_QuickStepKick:choose(as)

  -- reset walk request
  walk_request:wait()

  if (self.state == 0) then
    commands.stand()
    if (odometry.standing) then
      self.state = 1
      self:resetTime()
    end
  end

  if self.state == 1 then
    if (self.isLeft) then
      --speech:say('Quick Left')
    else
      --speech:say('Quick Right')
    end
    self.state = 2
    self:resetTime()
    commands.setWalkMode(core.WalkMode_KICK)
    walk_request:setStep(self.isLeft, 0.1, 0.0, 0)
  end

  if (self.state == 2 and self:getTime() > 1.5) then
    commands.setALWalkParameters(cfgWalkKickBack)
    self.state = 3
    self:resetTime()
    walk_request:setStep(self.isLeft, 0.00, 0.0, 0)

    -- odometry update
    odometry.didKick = true
    odometry.kickHeading = 0
    odometry.kickVelocity = 500.0;

  end

  if (self.state == 3 and self:getTime() > 1.5) then
    self.done = true
  end


  return head_LookForBall:set()
end

function kicks_QuickStepKick:finished()
  return self.done
end

-- Todd: fake kick for behavior debug
CompTask('kicks_FakeKick',{
           init = true,
           desiredDistance = 2000,
           desiredHeading = 0
         })
function kicks_FakeKick:choose(as)
  if (self.init) then
    -- Todd: TODO: how do we access kick types and foot from lua???
    kick_request:set(1,2, self.desiredHeading, self.desiredDistance)
    self.init = false
  end
  return task_NullTask:set()
end

function kicks_FakeKick:finished()
  return self:getTime() > 2.0
end
