require 'task'
module(..., package.seeall);


PrimTask('head_MoveHead',{
           head_init = true,
           time = 2.0,
           pan = 0.0
         })
function head_MoveHead:run()

  if (self.head_init) then
    self.head_init = false
    commands.setHeadPan(self.pan,self.time)
  end
end
function head_MoveHead:finished()
  return (self:getTime() > self.time)
end

--------------------------------
-- now that we never change tilt
-- only one scan!

---------------------------------------------
-- Basic Sweep scani

CompTask('head_SweepScan',{
           init = true,
           maxPan = 2.0,
           period = 3.0,
           numSweeps = 1,
           direction = 1,
           -- internal params
           intDirection = 1,
           sweepCounter = 0
         })
function head_SweepScan:choose(as)
  local scan = head_DiscreteScan
  local numSteps = self.period / scan.stepTime + 1
  local stepSize = (2 * self.maxPan / numSteps) * 1.05 -- 1.05 fudge factor to make it end where we want

  if self.init then
    self.init = false
    self.sweepCounter = 0
    self.intDirection = self.direction
    return scan:set{dest = self.direction * self.maxPan,stepSize = stepSize, init=true}
  end

  if as:finished() then
    self.sweepCounter = self.sweepCounter + 1
    self.intDirection = -self.intDirection
    behavior_mem.completeBallSearchTime = vision_frame_info.seconds_since_start
    return scan:set{dest = self.intDirection * self.maxPan,stepSize = stepSize, init=true,skipFirstPause=true}
  end

  return as
end

function head_SweepScan:finished()
  return self.sweepCounter >= self.numSweeps
end
--[[
PrimTask('head_SweepScan', {
           -- External Params
           maxPan = 1.3, -- How far left/right to pan the head
           period = 3.0, -- Time for one full sweep
           numSweeps = 1,

           -- Internal Params
           init = true,
           direction = 1,
           panTimer = NIL_INIT,
           pauseTimer = NIL_INIT,
           pauseRatio = 1.25,
           numPans = 0,
           numSteps = 4, -- The number of discrete steps on each side of the sweep
           stepsTaken = 0,
           nextStep = 0,
           nextPan = 0,
           isPaused = false,
           skipCurrent = false
         })

function head_SweepScan:run()
  local stepSize = self.maxPan / self.numSteps
  if self.init then
    -- self.period = 2
    -- camera ok, now we can start scan
    self.init = false
    self:resetTime()

    -- get the closest current pan
    local currPan = joint_angles[core.HeadPan]
    self.nextStep = math.max(math.min(math.floor(currPan / self.maxPan * self.numSteps + 0.5), self.numSteps),-1 * self.numSteps)
    if self.skipCurrent then
      self.nextStep = self.nextStep + self.direction
    end
    self.skipCurrent = false
    self.nextPan = self.nextStep * stepSize

    UTdebug.log(0,'  ',self.nextStep,RAD_T_DEG * self.nextPan)

    -- calculate times to stay still and move
    local timePerStep = self.period / (self.numSteps * 2)
    self.panTimer = timePerStep / (1 + self.pauseRatio)
    self.pauseTimer = timePerStep - self.panTimer

    commands.setHeadTilt()
    commands.setHeadPan(self.nextPan, self.panTimer)
    self.isPaused = true
    self.numPans = 0
  end
  -- we've finished panning to one side: pan to other side
  if (self.isPaused and (self:getTime() > self.pauseTimer)) then
    self.isPaused = false
    self:resetTime()
  elseif ((not self.isPaused) and (self:getTime() > self.panTimer)) then
    self.isPaused = true
    self:resetTime()
    if (self.nextStep >= self.numSteps) or (self.nextStep <= -1 * self.numSteps) then
      UTdebug.log(0,'  finished sweep')
      if self.nextStep < 0 then
        self.direction = 1
      else
        self.direction = -1
      end
      self.numPans = self.numPans + 1
    end
    self.nextStep = self.nextStep + self.direction
    self.nextPan = self.nextStep * stepSize
    UTdebug.log(0,'    moving ',self.nextStep,RAD_T_DEG * self.nextPan)
    --print("moving, step: " .. self.nextStep .. ", pan: " .. self.nextPan)
    commands.setHeadPan(self.nextPan, self.panTimer)
    -- possibly set that we just completed a ball scan
    if (robot_vision.doHighResBallScan) then
      behavior_mem.completeBallSearchTime = vision_frame_info.seconds_since_start
    end
  end
end

function head_SweepScan:finished()
  return self.numPans >= self.numSweeps
end
--]]


------------------------------------------------------------

CompTask('head_LookForBall')
function head_LookForBall:choose(as)
  robot_vision.doHighResBallScan = true
  return head_LookForObject:set{ object = core.WO_BALL }
end

function head_LookForBall:finished()
  return true
end

--------

-- try to look where object should be from localization
CompTask('head_LookAtObject',{
           object = 0
         })
function head_LookAtObject:choose(as)
  --UTdebug.log(20, "lookatObj time", self:getTime())
  local wo = world_objects:getObjPtr(self.object)
  -- If we've seen the object recently then these are more accurate
  if(vision_frame_info.frame_id - wo.frameLastSeen < 15) then
    return head_LookAtPoint:set{ 
        dist = wo.visionDistance, 
        bear = wo.visionBearing, 
        elev = wo.visionElevation 
      }
  else
    return head_LookAtPoint:set{ 
        dist = wo.distance, 
        bear = wo.bearing, 
        elev = wo.elevation 
      }
  end
end

------------

CompTask('head_LookForObject',{
           object = 9,
           lfoState = 0,
         })

function head_LookForObject:choose(as)

  local wo = world_objects:getObjPtr(self.object)

  local object = self.object
  local seen = wo.seen

  -- if its the goal, we may want to track the post or something instead
  if (wo:isGoal()) then
    object = head_goalSeenType(object)
    wo = world_objects:getObjPtr(object)
    seen = wo.seen
    --UTdebug.log(0, "Looking for Goal", self.object, " Going to track ", object, seen)
  end

  --UTdebug.log(80, "LFO", seen, self.lfoState, self.object)

  -- if we see it, track using vision bearing and distance
  -- if not, try looking where it should be
  -- if that fails after some time, try scan

  if as == nil then
    as = task_NullTask:set()
  end

  -- seen, track object
  if (seen) then
    UTdebug.log(30, "Tracking Ball")
    self.lfoState = 1
    return head_TrackObject:set{ object = object }
  else
    if (self.lfoState == 0 or self.lfoState == 1) then
      -- first time without seeing ball
      UTdebug.log(30, "First time not seeing ball")
      behavior_mem.startBallSearchTime = vision_frame_info.seconds_since_start
    end

    --UTdebug.log(20, "SATO, ", seen, self.lfoState)
    -- not seen
    -- do scan if we've looked and its not where we thought

    -- if its the ball, and we're not sure where it is... go straight to scan
    -- rather than try to look
    -- it's pretty cheap though, so err on the side of looking for the ball
    -- check ball sd relative to robot sd
    local me = world_objects:getObjPtr(robot_state.WO_SELF)
    local relSD = wo.sd - me.sd

    UTdebug.log(30, "ball abs sd", wo.sd, " rel sd", relSD)
    local sdThresh = 3000
    if (wo:isBall() and (relSD.x > sdThresh or relSD.y > sdThresh)) then
      UTdebug.log(30, "look for ball, high sd, skip look at point and go straight to scan")
      self.lfoState = 3
    end

    local lookAtPointTime = 2.5
    --local lookAtObjectTask = head_LookAtObjectWithMove
    local lookAtObjectTask = head_LookAtObject

    if (self.lfoState == 3 or
        (as == lookAtObjectTask and as:getTime() > lookAtPointTime)) then
      UTdebug.log(30, "going into sweep")
      self.lfoState = 3
      if (as == head_SweepScan) then
        return as
      end
      -- start scanning in the direction the wo was last seen
      local direction
      if (wo.bearing > 0) then
        direction = 1
      else
        direction = -1
      end
      return head_SweepScan:set{maxPan = 2.09,
                                period = 3.0, --2.6,
                                numSweeps = 1,
                                direction = direction}
    else
      UTdebug.log(30, "looking for object based on expected position")
      self.lfoState = 2
      return lookAtObjectTask:set{ object = self.object }
    end
  end
end

--------------------------------------------------------------------
-- try to look where object should be from localization
CompTask('head_LookAtObjectWithMove',{
           object = 0,
           init = false
         })
function head_LookAtObjectWithMove:choose(as)
  if self.init then
    self.init = false
    self:resetTime()
  end

  --UTdebug.log(20, "lookatObj time", self:getTime())
  local wo = world_objects:getObjPtr(self.object)
  local timeSpent = self:getTime()
  if (timeSpent < 0.2) then
    return head_LookAtPoint:set{ dist = wo.distance, bear = wo.bearing,
                                 elev = wo.elevation }
  elseif (timeSpent < 0.5) then
    return head_LookAtPoint:set{ dist = wo.distance, bear = wo.bearing+(DEG_T_RAD*15.0),
                                 elev = wo.elevation }
  elseif (timeSpent < 0.8) then
    return head_LookAtPoint:set{ dist = wo.distance, bear = wo.bearing+(DEG_T_RAD*-15.0),
                                 elev = wo.elevation }
  else
    return head_LookAtPoint:set{ dist = wo.distance, bear = wo.bearing,
                                 elev = wo.elevation }
  end
end

function head_LookAtObjectWithMove:finished()
  return (self:getTime() > 1.1)
end


-- look at the given point
PrimTask('head_LookAtPoint',{
           dist = 10,
           bear = 0,
           elev = 0,
           head_init = true
         })
function head_LookAtPoint:run()

  self.head_init = false

  local pan = self.bear;

  -- time should get smaller the longer we've run this method
  local targetTime = 0.35 - self:getTime()
  targetTime = math.max(0.05, targetTime)

  UTdebug.log(90, "LookAtPoint", self.dist, RAD_T_DEG*self.bear, self.elev, RAD_T_DEG*pan, targetTime)

  commands.setHeadPan(pan,targetTime)

end


--------
-- try to center object in image
PrimTask('head_TrackObject',{
           object = 0
         })

function head_TrackObject:run()

  local wo = world_objects:getObjPtr(self.object)

  if (wo.seen) then
    --print (vision_frame_info.frame_id, wo.seen)

    local panTrackSpeed = 0.5

    local relPan = wo.visionBearing - percepts.joint_angles[core.HeadPan]
  
    if(robot_state.WO_SELF == core.KEEPER and self.object == core.WO_BALL) then
      local woCross = world_objects:getObjPtr(core.WO_OWN_PENALTY_CROSS)
      if(woCross.seen) then
        if(math.abs(woCross.visionBearing - wo.visionBearing) < core.FOVx * .8) then
          local mid = (woCross.visionBearing + wo.visionBearing) / 2
          relPan = mid - percepts.joint_angles[core.HeadPan]
        end
      end
    end

    local panChange = panTrackSpeed * relPan
    UTdebug.log(25, "tracking, object center, track speed, change", wo.imageCenterX, panTrackSpeed, panChange)

    commands.setHeadPan(panChange,0.1,true)
  else
    return head_LookAtObject:set{object = self.object}
  end
end

----------


------------
--------------------------------------------------------
-- look for ball, but scan/look for goal every so often
-- if keeper, look for ball but scan/look for goals behind us (ideally penalty box corners, but not yet)
--------------------------------------------------------
CompTask('head_BallActiveLocalize',{
           init = true,
           orientationAtLastScan = 0.0,
           minTimeSinceScan = 1.5, --2.0,
           maxTimeSinceGoalSeen = 25.0,
           ballSeenCount = 0,
           numTimesGoalExpectedSeenSinceLastSeen = 0,
           useNegativeGoalInfo = true
         })

function head_BallActiveLocalize:choose(as)

  local ball = world_objects:getObjPtr(core.WO_BALL)
  local me = world_objects:getObjPtr(robot_state.WO_SELF)

  if (ball.seen) then
    self.ballSeenCount = self.ballSeenCount + 1
  else
    self.ballSeenCount = 0
  end


  if (self.init) then
    self.init = false
    self.orientationAtLastScan = me.orientation
    self.numTimesGoalExpectedSeenSinceLastSeen = 0

    -- if we're extremely lost.... just scan
    -- note: not opposite models because there looking at ball is actually more useful
    if (me.sdOrientation > DEG_T_RAD * 100.0 or localization_mem.fallenModels or localization_mem.bestAlpha < 0.4) then
      return head_GoalScan:set{numScans = 2}
    end
    
    return head_LookForBall:set()
  end
  
  if self.useNegativeGoalInfo and (shouldSeeObject(core.WO_OWN_GOAL,5000) or shouldSeeObject(core.WO_OPP_GOAL,5000)) then
    self.numTimesGoalExpectedSeenSinceLastSeen = self.numTimesGoalExpectedSeenSinceLastSeen + 1
  end

  -- time since goal
  local frameLastGoalSeen, objectLastSeen
  frameLastGoalSeen, objectLastSeen = head_BallActiveLocalize_timeSinceSeenGoal()
  if frameLastGoalSeen > behavior_mem.balSeenGoalFrame then
    -- never go back
    behavior_mem.balSeenGoalFrame = frameLastGoalSeen
    self.numTimesGoalExpectedSeenSinceLastSeen = 0
  end
  local timeSinceGoal = (vision_frame_info.frame_id-behavior_mem.balSeenGoalFrame) / 30.0 -- assuming 30 fps for now

  UTdebug.log(40,'BAL', (self:getTime()-behavior_mem.balScanTime), ball.distance, RAD_T_DEG * me.sdOrientation, RAD_T_DEG*ball.bearing, self.ballSeenCount, timeSinceGoal)

  -- continue current scan, if we haven't finished
  if ((as == head_GoalScan and not as:finished())) then
    behavior_mem.balScanTime = self:getTime()
    --UTdebug.log(40,'BAL continuing scan')
    return as
  end
  

  -- check if enough time has passed
  if ((self:getTime() - behavior_mem.balScanTime) < self.minTimeSinceScan) then
    --UTdebug.log(40,'BAL not enough time passod')
    return head_LookForBall:set()
  end

  -- for keeper, dont scan on ball moving toward us
  if (robot_state.WO_SELF == core.KEEPER and ball.relVel.x < -50) then
    return head_LookForBall:set()
  end

  -- if we're extremely lost.... just scan
  -- note: not opposite models because there looking at ball is actually more useful
  if (me.sdOrientation > DEG_T_RAD * 100.0 or localization_mem.fallenModels or localization_mem.bestAlpha < 0.4) then
    --UTdebug.log(40, "BAL goal scan due to uncertainty", me.sdOrientation*RAD_T_DEG, localization_mem.fallenModels, localization_mem.bestAlpha)
    return head_GoalScan:set()
  end

  -- should we be seeing a goal, if so, be more willing to goal scan because something might be going wrong
  local maxTimeSinceGoalSeen = self.maxTimeSinceGoalSeen - (self.numTimesGoalExpectedSeenSinceLastSeen / 30.0) -- take off a second for every second we think we should have seen the goal
  if maxTimeSinceGoalSeen < self.maxTimeSinceGoalSeen * 0.5 then
    maxTimeSinceGoalSeen = self.maxTimeSinceGoalSeen * 0.5
  end

  if (timeSinceGoal > maxTimeSinceGoalSeen) then
    --UTdebug.log(40,'BAL start goal scan on goal seen time frameLastGoalSeen:',elf.frameLastGoalSeen,'timeSinceGoal:',timeSinceGoal)
    --UTdebug.log(0,'BAL way too long since I saw a goal, scanning')
    return head_GoalScan:set{}
  end


  -- scan based on our orientation uncertainty and our distance from the ball
  local orientationThreshold = 35 - (ball.distance / 1000.0) * 7.5
  local maxThresh = 45
  local minThresh = 10

  -- lower if we're in a corner
  if (behavior_mem.robotIsInCorner) then
    orientationThreshold = orientationThreshold - 10
  end

  -- higher for keeper if ball is close
  if (robot_state.WO_SELF == core.KEEPER and ball.seen and ball.distance < 3500) then
    orientationThreshold = orientationThreshold + 5
  end

  -- be more sure for penalty kick
  if (game_state.isPenaltyKick) then
    maxThresh = 20
    minThresh = 10
  end
  orientationThreshold = core.crop(orientationThreshold,minThresh,maxThresh)



  UTdebug.log(40,'BAL', (self:getTime()-behavior_mem.balScanTime), ball.distance, orientationThreshold, RAD_T_DEG * me.sdOrientation, RAD_T_DEG*ball.bearing, self.ballSeenCount, timeSinceGoal)
  if (me.sdOrientation > DEG_T_RAD * orientationThreshold and self.ballSeenCount > 4 and math.abs(ball.bearing) < DEG_T_RAD*30.0) then
    behavior_mem.balScanTime = self:getTime()
    --UTdebug.log(40,'BAL starting goal scan on orient thresh',orientationThreshold)
    return head_GoalScan:set{}
  end

  -- otherwise, we do look for ball
  return head_LookForBall:set()
end

function head_BallActiveLocalize:finished()
  return true
end

function head_BallActiveLocalize_timeSinceSeenGoal()
  -- find out how recently we've seen any type of goal
  local frameLastGoalSeen = 0
  local objectLastSeen = -1

  if (robot_state.role_ == core.KEEPER) then
    -- use corners of the penalty box
    --[[
    local g1 = robot:getWorldObjPtr(core.WO_UNKNOWN_L_1)
    if (g1.frameLastSeen > frameLastGoalSeen) then
      frameLastGoalSeen = g1.frameLastSeen
      objectLastSeen = core.WO_UNKNOWN_L_1
    end
    --]]

    --if we are the goalie, only consider our goal (based on seen distance)
    for i = core.WO_GOAL_FIRST, core.WO_GOAL_LAST do
      local g1 = world_objects:getObjPtr(i)
      if ((g1.frameLastSeen > frameLastGoalSeen) and (g1.visionDistance < 2000)) then
        frameLastGoalSeen = g1.frameLastSeen
        objectLastSeen = i
      end
    end

    -- count the penalty cross too
    local g1 = world_objects:getObjPtr(core.WO_OWN_PENALTY_CROSS)
    if (g1.frameLastSeen > frameLastGoalSeen) then
      frameLastGoalSeen = g1.frameLastSeen
      objectLastSeen = core.WO_OWN_PENALTY_CROSS
    end

  else -- if not goalie
    for i = core.WO_GOAL_FIRST, core.WO_GOAL_LAST do
      local g1 = world_objects:getObjPtr(i)
      if (g1.frameLastSeen > frameLastGoalSeen) then
        frameLastGoalSeen = g1.frameLastSeen
        objectLastSeen = i
      end
    end
  end
  return frameLastGoalSeen, objectLastSeen
end

CompTask('head_GoalScan',{
           head_init = true,
           scanState = 0,
           seenCount = 0,
           scanDir = 0,
           closestBearing = 105.0,
           extraBearing = 15.0,
           initMoveTime = 0.4,
           scanTime = 2.6,
           maxPan = 105.0,
           corner = false,
           whichObj = 0,
           numScans = 1,
           partScanTime = 2.0,
           panAtGoalSighting = 0.0,
           panPastGoal = 240.0,
           seenAnyGoal = false,
           lookTime = 0.5
         })
function head_GoalScan:choose(as)

  -- check if we've seen any goals
  -- and get closest bearing
  local seenGoal = false
  local seenWholeGoal = false

  if (robot_state.WO_SELF == core.KEEPER) then
    self.extraBearing = 5.0
  end

  -- check corner
  if (self.corner) then

    if (self.head_init) then
      self.whichObj = math.random(0,1)
    end
    -- if self.corner is on, we're looking for corners rather than goals
    local g1 = world_objects:getObjPtr(core.WO_UNKNOWN_L_1)
    if (g1.seen) then
      seenGoal = true
      seenWholeGoal = true
    end
    local startIndex = core.WO_OWN_PEN_RIGHT_L
    if (robot_state.team_ == core.TEAM_RED) then
      startIndex = core.WO_OPP_PEN_RIGHT_L
    end
    local i = startIndex + self.whichObj
    local g2 = world_objects:getObjPtr(i)
    self.closestBearing = RAD_T_DEG*g2.bearing

    -- check our own goal for keeper, randomly picking a post
  elseif (robot_state.role_ == core.KEEPER) then
    local indexUnknown = core.WO_UNKNOWN_GOALPOST
    local indexLeft = core.WO_OWN_LEFT_GOALPOST
    local g1 = world_objects:getObjPtr(indexUnknown)
    seenGoal = g1.seen
    seenWholeGoal = false --g1.seen
    if (self.head_init) then
      self.whichObj = math.random(0,1)
    end
    local i = indexLeft + self.whichObj*2
    local g2 = world_objects:getObjPtr(i)
    self.closestBearing = RAD_T_DEG*g2.bearing

    -- normal
  else
    for i = core.WO_GOAL_FIRST, core.WO_GOAL_LAST do
      local g1 = world_objects:getObjPtr(i)
      if (i == core.WO_OWN_GOAL or i == core.WO_OPP_GOAL or i == core.WO_UNKNOWN_GOAL) then
        if (math.abs(RAD_T_DEG*g1.bearing) < math.abs(self.closestBearing)) then
          self.closestBearing = RAD_T_DEG*g1.bearing
        end
        if (g1.seen) then
          seenWholeGoal = true
        end
      end
      if (g1.seen) then
        seenGoal = true
        -- figure out angle we saw this at
        if (not self.seenAnyGoal and self.scanState == 1 and robot_state.WO_SELF ~= core.KEEPER) then
          self.seenAnyGoal = true
          self.panAtGoalSighting = percepts.joint_angles[core.HeadPan]
          -- how far from this should we pan (based on distance)
          self.panPastGoal = DEG_T_RAD * 46.4 + 2.0 * math.atan((core.GOAL_Y/2.0)/g1.distance)
        end
      end
    end
  end

  -- on init, lets go a few degrees outside of expected goal bearing
  if (self.head_init) then
    self.head_init = false
    self.scanState = 0
    self.seenCount = 0

    --UTdebug.log(10, "GoalScan, expectedBearing", self.closestBearing)

    if (self.closestBearing > 0.0) then
      --self.closestBearing = self.closestBearing + self.extraBearing
      self.scanDir = 1

    else
      --self.closestBearing = self.closestBearing - self.extraBearing
      self.scanDir = -1
    end

    -- if in corner, do full pan
    --[[
    if (behavior_mem.robotIsInCorner or
        robot_state.WO_SELF == core.KEEPER) then
      local currPan = percepts.joint_angles[core.HeadPan]
      if (currPan < 0) then
        self.closestBearing = -self.maxPan
      else
        self.closestBearing = self.maxPan
      end
    end
    --]]

    -- start pan, not past legal angles
    self.closestBearing = core.crop(self.closestBearing, -self.maxPan, self.maxPan)

    --return head_DiscreteScan:set{dest=DEG_T_RAD * self.closestBearing,init=true}
    local currPan = joint_angles[core.HeadPan]
    local stayTime = 0.25
    self.lookTime = math.abs(DEG_T_RAD * self.closestBearing - currPan) / (180 * DEG_T_RAD) + stayTime
    commands.setHeadPan(DEG_T_RAD * self.closestBearing,self.lookTime - stayTime)
    --return head_LookAtPoint:set{bear = DEG_T_RAD * self.closestBearing,head_init=true}
    return task_NullTask:set()
  end

  if (seenWholeGoal and robot_state.WO_SELF ~= core.KEEPER) then
    --if (seenGoal and robot_state.WO_SELF ~= core.KEEPER) then
    self.seenCount = self.seenCount + 1
  end


  local asFinished = (as == nil) or (as:finished())
  if (as == task_NullTask) and (self:getTime() > self.lookTime) then
    asFinished = true
  end

  -- otherwise, wait for first move to finish
  --if (self:getTime() > (self.initMoveTime+0.05) and self.scanState == 0 and not self.corner and not seenWholeGoal) then
  if (asFinished and self.scanState == 0 and not self.corner and not seenWholeGoal) then
    self.scanState = 1
    self:resetTime()
    -- scan the other way
    self.scanDir = -self.scanDir
    return head_DiscreteScan:set{dest = DEG_T_RAD * self.scanDir * self.maxPan,skipFirstPause=true,init=true}
  end

  --  UTdebug.log(0, "seen goal, angle at goal, pan past amount, current pan past",
  --            self.seenAnyGoal, self.panAtGoalSighting, self.panPastGoal,
  --          math.abs(percepts.sensorAngles[core.HeadPan] - self.panAtGoalSighting))

  -- finish on time, or panned far enough
  --if (self.scanState > 0 and
  --((self:getTime() > (self.partScanTime+0.05)) or
  --(math.abs(percepts.joint_angles[core.HeadPan] - self.panAtGoalSighting) > self.panPastGoal))) then
  if (self.scanState > 0 and asFinished) then
    self.scanState = self.scanState + 1
    -- only do extra scan if we're not done
    if (not self:finished()) then
      self:resetTime()
      -- scan the other way
      self.scanDir = -self.scanDir
      return head_DiscreteScan:set{dest = DEG_T_RAD * self.scanDir * self.maxPan,skipFirstPause=true,init=true}
    end
  end

  return as
end

function head_GoalScan:finished()
  -- either we've seen a goal for 4 frames, or we completed our scan
  return ((self.seenCount > 3) or (self.scanState > self.numScans))
end

PrimTask('head_DiscreteScan',{
           dest = 115 * DEG_T_RAD, -- how far left/right to pan
           stepSize = 26 * DEG_T_RAD,
           stepTime = 0.4,
           pauseTime = 0.2083,
           skipFirstPause = false,
           -- internal
           init = true,
           isFinished = false,
           isPaused = true,
         })
function head_DiscreteScan:run()
  if self.init then
    self.init = false
    self.isFinished = false
    self.isPaused = true
    self:resetTime()
    --UTdebug.log(0,'  DiscreteScanStart',self.dest)
  end
  --UTdebug.log(0,self.isPaused,self:getTime(),self:getTime() > self.pauseTime)

  local moveTime = self.stepTime - self.pauseTime

  if self.skipFirstPause or (self.isPaused and (self:getTime() > self.pauseTime)) then
    self.isPaused = false
    self.skipFirstPause = false
    -- set the new target
    local currPan = joint_angles[core.HeadPan]
    local diff = self.dest - currPan
    local dir = (diff / math.abs(diff))
    local target = currPan + self.stepSize * dir
    if (dir > 0) and (target > self.dest) then
      target = self.dest
    elseif (dir < 0) and (target < self.dest) then
      target = self.dest
    end
    commands.setHeadTilt()
    commands.setHeadPan(target,moveTime)
    --UTdebug.log(0,'TARGET:',target,'curr:',currPan)
    if math.abs(diff) < 5 * DEG_T_RAD then
      self.isFinished = true
    end
    self:resetTime()
  end

  if not self.isPaused and (self:getTime() > moveTime) then
    --local currPan = joint_angles[core.HeadPan]
    --commands.setHeadTilt()
    --commands.setHeadPan(currPan,0.01)
    self:resetTime()
    self.isPaused = true
  end
end

function head_DiscreteScan:finished()
  return self.isFinished
end

-- want to know what type of goal was seen
-- check if the goal or its posts were seen
function head_goalSeenType(goalType)

  local goal = world_objects:getObjPtr(core.WO_UNKNOWN_GOAL)

  local goalL = world_objects:getObjPtr(core.WO_UNKNOWN_LEFT_GOALPOST)
  local goalR = world_objects:getObjPtr(core.WO_UNKNOWN_RIGHT_GOALPOST)

  local goalU = world_objects:getObjPtr(core.WO_UNKNOWN_GOALPOST)

  if (goal.seen) then
    return goal.type
  elseif (goalL.seen) then
    return goalL.type
  elseif (goalR.seen) then
    return goalR.type
  elseif (goalU.seen) then
    return goalU.type
  else
    return goalType
  end

end

