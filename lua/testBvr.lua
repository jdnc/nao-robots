require 'commands'
require 'head'
require 'skills'
require 'cfgalwalk'
require 'body'
require 'cfgcam'

module(..., package.seeall);
   
-- what to run in testing state
CompTask('testBvr_TestWalk',{
           init = true,
           fwd = 1,
           side = 0,
           turn = 0,
           random = false,
           sprint = false,
           sentKick = false,
           standTime = 3.0,
           walkTime = 5.0
   }) 
function testBvr_TestWalk:choose(as)

  --commands.setWalkMode(core.WalkMode_SPRINT)
  if self.init then
    self:resetTime()
    self.init = false
    self.sentKick = false
    commands.setHeadPan(0,1.0)
  end
  --return skills_PlayAttacker:set()
  if (self:getTime() < self.standTime or self:getTime() > self.walkTime + self.standTime) then
    commands.stand()
  else
    if self.random then
      if (self:getTime() - math.floor(self:getTime()) < 0.1) then
        local fwd = math.random() * 2 - 1.0
        local side = math.random() * 2 - 1.0
        local turn = math.random() * 2 - 1.0
        commands.setWalkVelocity(fwd,side,turn,self.sprint)
      end
    else
      commands.setWalkVelocity(self.fwd,self.side,self.turn,self.sprint)
    end
  end
  --if (self:getTime() > 4.0) and not self.sentKick then
    --self.sentKick = true
    --walk_request:setKick(0,0,false)
  --end
  return task_NullTask:set()
end

CompTask('testBvr_TestWalkParamChange',{
    init = true,
    changeFreq = 2.0,
    lastChangeTime = 0,
    mode = core.WalkMode_SLOW,
    ind = 1
  })
function testBvr_TestWalkParamChange:choose(as)
  if (self:getTime() > 8.0) then
    commands.stand()
  else
    if (vision_frame_info.seconds_since_start - self.lastChangeTime > self.changeFreq) then
      print 'CHANGE'
      self.lastChangeTime = vision_frame_info.seconds_since_start
      --commands.setWalkMode(self.mode)
      if self.mode == core.WalkMode_SLOW then
        self.mode = core.WalkMode_SPRINT
      else
        self.mode = core.WalkMode_SLOW
      end
      commands.setWalkVelocity(1.0,0,-0.09)
    end
  end
  return task_NullTask:set()
end

CompTask('testBvr_TestApproach',{
    init = true
  })
function testBvr_TestApproach:choose(as)
  if self.init then
    self.init = false
    commands.setWalkVelocity(1,0,0,true)
    return head_LookForBall:set()
  end

  if (as:getTime() > 1.0 and as == head_LookForBall) then
    as = body_AlignToBallWithTargetWalk:set{yOffset=50.0,desiredFoot=core.Kick_LEFT,switchable=true,heading=0.0}
  elseif (as == body_AlignToBallWithTargetWalk and as:finished()) then
    as = kicks_ExecuteKick:set{kickChoice = core.FwdLongStraightKick, desiredHeading = 0.0, desiredDistance = 3000}
  end
  return as
end

CompTask('testBvr_TestWalkTo',{
    init = true
  })
function testBvr_TestWalkTo:choose(as)
  if self.init then
    self.init = false
    commands.setWalkMode(core.WalkMode_MID)
    commands.setWalkTarget(400,0)
  end
  return task_NullTask:set()
end

CompTask('testBvr_TestStep',{
           state = 0,
           isLeft = true
  })
function testBvr_TestStep:choose(as)

  -- reset walk request
  walk_request:wait()

  if self.state == 0 then
    self.state = 1
    commands.setWalkMode(core.WalkMode_KICK)
    walk_request:setStep(self.isLeft, 0.1, 0.0, 0)
  end

  if (self.state == 1 and self:getTime() > 1.25) then
    commands.setALWalkParameters(cfgWalkKickBack)
    self.state = 2
    self:resetTime()
    walk_request:setStep(self.isLeft, 0.00, 0.0, 0)
  end

  return task_NullTask:set()
end



CompTask('testBvr_TestWalkSideChange',{
    init = true,
    changeFreq = 0.2,
    lastChangeTime = 0,
    sideVel = 1.0
  })
function testBvr_TestWalkSideChange:choose(as)
  --commands.setWalkMode(core.WalkMode_SPRINT)
  --commands.setALWalkParameters(cfgWalkSlow)
  if (vision_frame_info.seconds_since_start - self.lastChangeTime > self.changeFreq) then
    self.lastChangeTime = vision_frame_info.seconds_since_start
    self.sideVel = -1 * self.sideVel

    local fwd_amount = 0---0.1
    local turn_amount = self.sideVel---0.09
    commands.setWalkVelocity(fwd_amount,self.sideVel,turn_amount,false)
  end
  return task_NullTask:set()
end


CompTask('testBvr_TestKick',{
    init = true
  })
function testBvr_TestKick:choose(as)
  if self.init then
    self.init = false
  end
  if (self:getTime() < 3.0) then
    commands.setWalkVelocity(0,0,0)
    return head_LookForBall:set()
  else
    return kicks_ExecuteKick:set{desiredHeading = DEG_T_RAD * 0, desiredDistance = 4000.0, kickChoice = core.WalkKickFront}
  end
end

CompTask('testBvr_TestNewApproach',{
           init = true,
           }) 
function testBvr_TestNewApproach:choose(as)

  if self.init then
    self:resetTime()
    self.init = false
  end
  
  if (self:getTime() < 3.0) then
    commands.stand()
    return head_LookForBall:set()
  else
    return skills_ApproachBall:set()
  end
  return task_NullTask:set()
end

-- what to run in testing state
CompTask('testBvr_TestingState',{
      init = true,
   }) 
function testBvr_TestingState:choose(as)

   --(body_AlignToBallWithTargetWalk.done)
  --if 
  --if self.init then
    --self.init = false
    --print('INIT')
  --if self:getTime() < 2.0 then
    --commands.stand()
    --return head_LookForBall:set()
  --else
    ----return body_AlignToBallWithTargetWalk:set{switchable = true}
    --return skills_ApproachBall:set()
  --end
  --else
    --return as
  --end
  --return testBvr_freeHead:set()
  --return skills_AlignToBallLookBall:set{yOffset = 50, desiredFoot=left, switchable = false, heading = 0, desiredX = 135}
  --return skills_ApproachBall:set()
  --return kicks_ExecuteKick:set{desiredDistance=1000,desiredHeading=0,kickChoice=core.WalkKickFront}
  --return testBvr_TestWalk:set{fwd = 1.0, side = 0.0, turn = DEG_T_RAD * 3, walkTime = 10}
  --if self.init then
    --self.init = false
    --walk_request:setWalkTarget(175,0,0)
  --end
  --return task_NullTask:set()
--  return head_TriangleScan:set{period = 5.0, minTilt = -30.0, maxTilt = -15.0, maxPan = 120.0}
  --current.motionMem.runInverseCOM = true
  --return testBvr_freeHead:set()
  --return testBvr_TestKick:set()
  --return testBvr_TestNewApproach:set()
  return testBvr_cameraOffsetTest:set()
  --return kicks_ExecuteKick:set{desiredHeading = DEG_T_RAD * 0, desiredDistance = 4000.0, kickChoice = core.FwdLongStraightKick}

  --return skills_KeeperSafeDiveLookForBall:set{reverseSides = true}
  --return task_NullTask:set()
  --return testBvr_cameraOffsetTest:set()
  
  --return testBvr_CheckFalling:set()
  -- return testBvr_Walk:set()
  --  return skills_CheerLead:set()
  --return soccerBvr_SetState:set()
  --return kicks_QuickSideKick:set{swingLeg=core.RIGHTLEG,kickType=kickChoice}
  --return com_FlattenFeet:set() 
  --return testBvr_WalkTimer:set()
  --return head_ImageScan:set()
  --return head_GoalScan:set{numScans = 15, scanTime = 2.0}
  --return testBvr_Camera:set()
  --return skills_PlaySupporter:set()
  --return kicks_GenericKick:set{kickType = core.FwdKick}
  --return skills_MoveToPointBoxScan:set{period = 4.0, maxTilt = -35.0, minTilt = -35.0, maxVel = 0.8,des = core.Point2D(-600,1500)}
  --return head_TriangleScan:set{period = 4.0, maxTilt = -15.0,                             minTilt = -35.0, maxPan = 95.0}
  --return skills_ToPoseTriangleScan:set{period = 4.0, maxTilt = -10, minTilt = -35, maxPan = 120}
  --return skills_RotateAroundBallBAL:set{useGoalBearing=true, minScanDist=0}
  --return testBvr_com:set()
  --return skills_PlayAttacker:set()
  --return body_GetUp:set()
  --return skills_PlayAttacker:set()
  --return head_BallActiveLocalize:set{}
  --return skills_Search:set()
  --return skills_DribbleBallBAL:set{xSpeedFactor = 1.0, xSpeedFactorFar = 1.0}
  --return skills_ToPoseTriangleScan:set{period = 5.0, maxTilt = -10.0, minTilt = -30.0, maxPan = 120.0}
  --return kicks_GenericCOMKick:set{kickType=core.COMFwdKick}
--[[
   if self.init then
     self.init = false
     self:resetTime()
     return body_ToPose:set{pose = GoalieLeftSplayFinalV4, time = 2.0, init = true}
  end
  if self:getTime() > 3.5 then
    self:resetTime()
    commands.interpAllJointStiffness(0,0.3)
  end
  return body_PrintPose:set()
--]]
--  return skills_BlockMoveHead:set{pan = 0.0, tilt = DEG_T_RAD *0.0, time = 0.5, head_init = true, yIntercept = -10}
 --[[
 if self.init then
   self.init = false
   commands.interpAllJointStiffness(0,0.3)
 end
 return body_PrintPose:set()
--]] 
end

CompTask('testBvr_cameraOffsetTest',{
    init = true
  })
function testBvr_cameraOffsetTest:choose(as)
  local ball = world_objects:getObjPtr(core.WO_BALL)
  --local x = ball.relPos.x
  --local y = ball.relPos.y
  local camOffsets = getCamOffsets()
  local x = behavior_mem.keeperRelBallPos.x
  local y = behavior_mem.keeperRelBallPos.y
  --UTdebug.log(0,ball.seen,ball.visionDistance,ball.visionBearing)
  --local x = ball.visionDistance * math.cos(ball.visionBearing)
  --local y = ball.visionDistance * math.sin(ball.visionBearing)
  UTdebug.log(0,'Ball, seen, x, y =',ball.seen,x,y)
  local desiredRelSide = 0
  local desiredRelFwd = 0
  UTdebug.log(0,'If ball centered between feet, rel_ball_side:', desiredRelSide - y)
  UTdebug.log(0,'If ball barely touching front of foot, rel_ball_fwd is:',desiredRelFwd - x)
  if self.init then
    self.init = false
    commands.setHeadPan(DEG_T_RAD * kickAimPose[core.HeadYaw],1.0,false)
    return body_ToPose:set{pose = kickAimPose, time = 1.0}
  else
    return as
  end
end

PrimTask('testBvr_testSonar',{
    init = true
  })
function testBvr_testSonar:run()
  -- process sonar information
  if (processed_sonar ~= nil) then
    -- avoid range on left
    if (processed_sonar.on_left_ ~= false) then
      UTdebug.log(0,"Sonar object detected on left: ", processed_sonar.left_distance_)
    end

    -- avoid range on center
    if (processed_sonar.on_center_ ~= false) then
      UTdebug.log(0,"Sonar object detected on center: ", processed_sonar.center_distance_)
    end

    -- avoid range on right
    if (processed_sonar.on_right_ ~= false) then
      UTdebug.log(0,"Sonar object detected on right: ", processed_sonar.right_distance_)
    end
  end
end

PrimTask('testBvr_freeHead',{
    init = true
  })
function testBvr_freeHead:run()
  if self.init then
    self.init = false
    commands.setStiffnessCommands(cfgStiffStandHeadFree, 1)
  end
end

PrimTask('testBvr_com',{
    init = true,
    state = 0,
    y = 0,
    lastVal = 0,
    errorSum = 0,
    Kp = -1,
    Ki = -0,
    Kd = 2,
  })
function testBvr_com:run()
  if self.init then
    self.init = false
--    inverseCoMC:setSupportLeg(core.LEFTLEG)
--    inverseCoMC:setLegEnable(core.RIGHTLEG,false)
--    inverseCoMC:setLegEnable(core.LEFTLEG,true)

--    inverseCoMC:setSupportLeg(core.RIGHTLEG)
--    inverseCoMC:setLegEnable(core.RIGHTLEG,true)
--    inverseCoMC:setLegEnable(core.LEFTLEG,false)
    local leg = math.random(0,1)
    inverseCoMC:setSupportLeg(leg)
    inverseCoMC:setLegEnable(leg,true)
    inverseCoMC:setLegEnable(1-leg,false)
  end

  local desiredPosition
  local actionTime = 0.25
  if self.state == 0 then
    -- shift left
    desiredPosition = core.VecPosition(0,0,245)
    if math.abs(inverseCoMC.com.m_y) < 2 then
      self.state = self.state + 1
    end
  end
  if self.state == 1 then
    -- stand up
    desiredPosition = core.VecPosition(0,0,280)
    if inverseCoMC.com.m_z > 278 then
      self.state = self.state + 1
    end
  end
  if self.state == 2 then
    -- dynamically balance
--[[
    local val = (percepts.fsrLeftSide + 1.54)
--    self.val = .75 * self.val + .25 * val
--    self.y = self.y - 3 * val
    self.errorSum = self.errorSum + val
    self.y = self.Kp * val + self.Ki * self.errorSum + self.Kd * (val - self.lastVal)
    self.lastVal = val
    print (val,self.y)
    actionTime = 0.1
--]]
    desiredPosition = core.VecPosition(0,self.y,280)
--    commands.setJointInterpRad(core.RKneePitch,2,actionTime)
--    commands.setJointInterpRad(core.RHipPitch,0,actionTime)
  end


  print (inverseCoMC.com.m_x,inverseCoMC.com.m_y,inverseCoMC.com.m_z)
  local diff = desiredPosition - inverseCoMC.com
  inverseCoMC:calculateInverseCoM(kinematicsC.current, 0, diff);
  local val
  local origVal
  local temp
  for i = 0,core.NUM_JOINTS do
    if inverseCoMC:isJointEnabled(i) then
      val = robot:getDouble(current.percepts.sensorAngles,i)
      diff = robot:getDouble(current.kinematicsMem.deltaTheta,i)
      val = getNewJointAngle(i,val,diff)
    
      commands.setJointInterpRad(i,val,actionTime)
    end
  end

--[[
  for i = core.RShoulderPitch,core.RElbowRoll do
    
--    val = robot:getDouble(current.percepts.sensorAngles,i)
--    temp = val
--    val = val + robot:getDouble(current.kinematicsMem.deltaTheta,i)
--    origVal = val
--    val = projectJointAngle(val,i,temp)
    val = robot:getDouble(current.percepts.sensorAngles,i)
    diff = robot:getDouble(current.kinematicsMem.deltaTheta,i)
    val = getNewJointAngle(i,val,diff)
    
    commands.setJointInterpRad(i,val,0.25)
--    local maxVal = robot:getDouble(core.maxJointLimits,i)
--    local minVal = robot:getDouble(core.minJointLimits,i)
--    print (origVal,'|',minVal,'..',maxVal,'|',val)
  end
  for i = core.LShoulderPitch,core.LElbowRoll do
    
--    val = robot:getDouble(current.percepts.sensorAngles,i)
--    temp = val
--    val = val + robot:getDouble(current.kinematicsMem.deltaTheta,i)
--    origVal = val
--    val = projectJointAngle(val,i,temp)
    val = robot:getDouble(current.percepts.sensorAngles,i)
    diff = robot:getDouble(current.kinematicsMem.deltaTheta,i)
    val = getNewJointAngle(i,val,diff)
    
    commands.setJointInterpRad(i,val,0.25)
--    local maxVal = robot:getDouble(core.maxJointLimits,i)
--    local minVal = robot:getDouble(core.minJointLimits,i)
--    print (origVal,'|',minVal,'..',maxVal,'|',val)
  end
--]]
end

function testBvr_com:finished()
  return true
end

function getNewJointAngle(joint,val,diff)
  local maxVal = robot:getDouble(core.maxJointLimits,joint)
  local minVal = robot:getDouble(core.minJointLimits,joint)
  
  local temp
  local diff1
  local diff2

  temp = val + diff
  if temp < maxVal then
    return temp
  end
  diff1 = temp - maxVal
  temp = val + diff - 2 * math.pi
  if temp > minVal then
    return temp
  end
  diff2 = minVal - temp
  return val
--  if diff1 < diff2 then
--    return maxVal
--  else
--    return minVal
--  end
end



PrimTask('testBvr_Stiff',{
      init = true
   })
function testBvr_Stiff:run()
   if (current.frameNum > 100 and self.init) then
      UTdebug.log(0, "setstiff")
      commands.setJointStiffness(0, 0.0)
      commands.setJointStiffness(1, 0.0)
      commands.setJointStiffness(2, 1.0)
      commands.setJointStiffness(3, 1.0)
      commands.setJointStiffness(4, 1.0)
      commands.setJointStiffness(5, 1.0)
      commands.setJointStiffness(6, 1.0)
      self.init = false
   end
end

PrimTask('testBvr_Walk',{
           init = true,
           speed = -100,
           dir = 1
         })
function testBvr_Walk:run()
  
  --commands.setRealWalkVelocity(self.speed, 0, 0)
  --commands.setRealWalkVelocity(0, self.speed, 0)
  --commands.setRealWalkVelocity(self.speed, self.speed, DEG_T_RAD*self.speed)
  commands.setWalkVelocity(1.0,0.00,0.00)
  commands.setJointStiffness(0, 0);
  commands.setJointStiffness(1, 0);

  self.speed = self.speed + self.dir

  if (math.abs(self.speed) > 100) then
    self.dir = -self.dir
  end
  
end

CompTask('testBvr_WalkTimer',{
           init = true,
           speed = -100,
           dir = 1
         })
function testBvr_WalkTimer:choose(as)
  if (self.init) then
    self.init=false
    --commands.setJointInterpRad(1,DEG_T_RAD*-25,0.01)
    --commands.setJointInterpRad(0,0,0.01)
    --commands.setJointStiffness(1, 0)
    commands.setWalkVelocity(0.0, 0, 0.1) 
    return task_NullTask:set()
  end

  if(self:getTime()  > 15.0) then
    commands.setWalkVelocity(0,0,0)
  end

  
  --commands.setJointStiffness(1, 0)
  return task_NullTask:set()
 
end


PrimTask('testBvr_Cart',{
      init = true
   })
function testBvr_Cart:run()

   if (current.frameNum == 50) then
      for i = 0, core.NUM_JOINTS-1 do
   commands.setJointStiffness(i, 1.0)
      end
   end

   if (current.frameNum > 100 and self.init) then
      for i = 0, core.NUM_JOINTS-1 do
   commands.setJointStiffness(i, 1.0)
      end
      commands.setChainInterpCartRel(1, 0.02, 0.05, 0.08, 1.0)
      self.init = false
   end


end


PrimTask('testBvr_Joints',{
      init = true
   })
function testBvr_Joints:run()
  if (current.frameNum > 100 and self.init) then
    for i = 0, core.NUM_JOINTS-1 do
      commands.setJointStiffness(i, 1.0)
    end

    -- try this command
    local commandType = core.interp
    local time = 5.0

    UTdebug.log(0, "setjoints")
    commands.setJointDeg(0, -10.0, commandType, time)
    commands.setJointDeg(1, 10.0, commandType, time)
    commands.setJointDeg(2, -10.0, commandType, time)
    commands.setJointDeg(3, -10.0, commandType, time)

    self.init = false
  end
end

function isFalling_SVM()
   accX=percepts.inertials[core.accX]
   accY=percepts.inertials[core.accY]
   accZ=percepts.inertials[core.accZ]
   spdX=percepts.inertials[core.spdX]
   spdY=percepts.inertials[core.spdY]
 
   k =sensorsC:isUnstable(accX,accY,accZ,spdX,spdY)
 
   return (k==1)
end


function isFalling_J48()
   accX=percepts.inertials[core.accX]
   accY=percepts.inertials[core.accY]
   accZ=percepts.inertials[core.accZ]
   spdX=percepts.inertials[core.spdX]
   spdY=percepts.inertials[core.spdY]


if(accZ <= -54)then
   if(accX <= -1.751786)then
      if(accX <= -2.802857)then
	 k  = 1 
      else
	 if(accY<= -0.700714)then
	    k = 1
	 else
	    if(spdX <= -1943)then
	       k = 0
            else
	       if(spdY <= -1764)then
		  k = 0
	       else
		  k = 1
	       end
            end
	 end
      end 
   else
      if(accY <= -2.277321)then
	 if(accY<= -2.978036)then
	    k = 1
	 else
	    if(accX <= -0.175179)then
	       k = 1
	    else
	       k = 0
	    end
	 end
      else
          if(accX <= 0.350357)then
	    if(accY <= 1.751786)then
	       k = 0
            else
	      if(accY <= 2.802857)then
		  if(accX <= -0.175179)then
		     if(spdY <= -1684)then
			k = 0
		     else
			if(spdX <= -1972)then
			   k = 0
			else
			   k = 1
		        end
		     end
		  else
		     k = 1
		  end
	      else
		  k = 1
	      end
	    end
         else
	       if(accZ <= -57)then
		  if(spdY <= -1795)then
		     k = 1
		  else
		     k = 0
		  end
	       else
		  k = 1
	       end
         end
      end 
   end	  
else
   if(accZ <= -52)then
      if(spdY <= -1679)then
	 if(spdX <= -2014)then
	    if(accX <= -1.401429)then
	       k = 0
	    else
	       k = 1
	    end
	 else
	    k = 1
	 end
      else
         if(accX <= 0.875893)then
	     if(accX <= -1.051071)then
		if(accX <= -2.978036)then
		   k = 1
		else
		   if(spdY <= -1589)then
		      k = 0
		   else
		      k = 1
		   end
		end
	     else
		k = 0
	     end
	 else
	     k = 1
	 end
      end
   else
      k = 1
   end
end
   
  return (k==1)
end
	
	
	
function arrayUpdate()	
     if(current.frameNum % 21 == 0)then
        index = 1 
	if(unstable)then
           recentStability[1] = 1	
         else	
           recentStability[1] = 0	
         end
     else
	 if(unstable)then	
           recentStability[index] = 1	
         else	
           recentStability[index] = 0	
         end 
     end
     index = index + 1  
end	
	
function isMajorityUnstable()	
   arrayUpdate()	
   zeroes = 0	
   ones = 0	
    for m=1,21 do	
       if( recentStability[m] == 1)then	
	  ones = ones + 1
       else	
	  zeroes = zeroes + 1
       end	
    end	
    print ("Unstable count = ", ones)
    return (ones/21 > 0.5)	
end     	

 

CompTask('testBvr_CheckFalling',{
           init = true,
         })
function testBvr_CheckFalling:choose(as)
   unstable = isFalling_SVM()
   -- unstable = isFalling_J48()     
   
   --Making the robot walk to notice if falling   
   commands.setWalkVelocity(1.0,1,0)
   
   if(unstable)then
      current.commands.fallingLights = true
      if(isMajorityUnstable())then
	 if(os.clock() - t1 > 1)then
	    commands.sayText("Falling! Save me")
	    t1 = os.clock()
	 end
      end
   else
      current.commands.fallingLights = false
   end

  return task_NullTask:set()
end



-- vim: ts=2:sw=2:expandtab:softtabstop=2
