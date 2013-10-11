require 'commands'
require 'stateMachine'
--require 'body'
require 'head'
require 'util'
module(..., package.seeall);

-----------------------------------------
-----------------------------------------
-- FRONT KICK
-----------------------------------------
-----------------------------------------

function setTime(time)
  joint_commands.send_body_angles_ = true
  joint_commands.body_angle_time_ = time * 1000.0
end

function setLeg(leg,hipRoll,hipPitch,kneePitch,anklePitch,ankleRoll)
  if (leg == core.RIGHTLEG) then
    joint_commands:setJointCommandDeg(core.RHipRoll,-hipRoll)
    joint_commands:setJointCommandDeg(core.RHipPitch,hipPitch)
    joint_commands:setJointCommandDeg(core.RKneePitch,kneePitch)
    joint_commands:setJointCommandDeg(core.RAnklePitch,anklePitch)
    joint_commands:setJointCommandDeg(core.RAnkleRoll,ankleRoll)
  else
    joint_commands:setJointCommandDeg(core.LHipRoll,-hipRoll)
    joint_commands:setJointCommandDeg(core.LHipPitch,hipPitch)
    joint_commands:setJointCommandDeg(core.LKneePitch,kneePitch)
    joint_commands:setJointCommandDeg(core.LAnklePitch,anklePitch)
    joint_commands:setJointCommandDeg(core.LAnkleRoll,-ankleRoll)
  end
end

function setShoulderBasedOnLeg(leg,shoulderPitch,shoulderRoll)
  if (leg == core.RIGHTLEG) then
    joint_commands:setJointCommandDeg(core.RShoulderPitch,-shoulderPitch)
    joint_commands:setJointCommandDeg(core.RShoulderRoll,-shoulderRoll)
  else
    joint_commands:setJointCommandDeg(core.LShoulderPitch,-shoulderPitch)
    joint_commands:setJointCommandDeg(core.LShoulderRoll,shoulderRoll)
  end
end

function setGroin(groin)
  joint_commands:setJointCommand(core.LHipYawPitch,groin)
  joint_commands:setJointCommand(core.RHipYawPitch,groin)
end

function ikSwingLeg(kickLeg,swingFwd,swingSide,swingUp,torsoAngle,groin,time,anklePitchOffset)
  if anklePitchOffset == nil then
    anklePitchOffset = 0
  end
  local commandType = core.interp
  UTdebug.log(80, "ikSwingLeg", kickLeg, swingFwd, swingSide, swingUp, torsoAngle,groin,time,anklePitchOffset)
  local hipPitch, kneePitch, hipRoll, ankleRoll, anklePitch
  if kickLeg == core.RIGHTLEG then
    swingSide = -swingSide

    hipPitch = core.RHipPitch
    kneePitch = core.RKneePitch
    hipRoll = core.RHipRoll
    ankleRoll = core.RAnkleRoll
    anklePitch = core.RAnklePitch
  else
    hipPitch = core.LHipPitch
    kneePitch = core.LKneePitch
    hipRoll = core.LHipRoll
    ankleRoll = core.LAnkleRoll
    anklePitch = core.LAnklePitch
  end
  
  ankleToAnkleInverseIgnoreBodyAngles(kickLeg,swingFwd,swingSide,swingUp,torsoAngle,groin)
  
  --joint_commands:setJointCommand(hipPitch,kinematicsC.hipPitch)
  --joint_commands:setJointCommand(kneePitch,kinematicsC.kneePitch)
  --joint_commands:setJointCommand(hipRoll,kinematicsC.hipRoll)
  --joint_commands:setJointCommand(ankleRoll, -kinematicsC.hipRoll - groin* 0.707 - current.percepts.bodyRoll)
  --joint_commands:setJointCommand(anklePitch,-(kinematicsC.hipPitch + kinematicsC.kneePitch + torsoAngle + groin * 0.707) + anklePitchOffset)
end

function ankleToAnkleInverseIgnoreBodyAngles(swingLeg,swingFwd,swingSide,swingUp,torsoAngle,groin)
  local stanceAnkle
  if swingLeg == core.RIGHTLEG then
    stanceAnkle = luaC:getPose3DPtr(body_model.rel_parts_,core.BodyPart_left_ankle).translation
  else
    stanceAnkle = luaC:getPose3DPtr(body_model.rel_parts_,core.BodyPart_right_ankle).translation
  end
--  stanceAnkle.m_z = stanceAnkle.m_z + lowestZ

  swingFwd = swingFwd + stanceAnkle.x
  swingSide = swingSide + stanceAnkle.y
  swingUp = swingUp + stanceAnkle.z
  originToInverseRaw(swingLeg,swingFwd,swingSide,swingUp,true,torsoAngle,groin)
end

-- Raw version that takes all the different variables
function originToInverseRaw(leg , x, y, z, toAnkle, torsoAngle, groinAngle) --f,s,h
--  print ('Request=',leg, x, y, z, torsoAngle)
--[[
  if (torsoAngle~=USECURRENTANGLE) then   
   theta4= -torsoAngle
   tempF = x
   x=z*math.sin(theta4)+tempF*math.cos(theta4)
   z=z*math.cos(theta4)-tempF*math.sin(theta4)
  end
  
  if (current.percepts.bodyRoll ~= 0) then
   theta4= current.percepts.bodyRoll
   tempF = y
   y= -1 * z*math.sin(theta4)+tempF*math.cos(theta4)
   z= z*math.cos(theta4)+tempF*math.sin(theta4)
  end
--]]
  if (toAnkle) then
    local left = leg == core.LEFTLEG
    core.InverseKinematics_calcLegJoints(core.Pose3D(x,y,z),joint_commands.angles_,left,robot_info.dimensions_)
    --kinematicsC:inverseKinematicsToAnkle(leg, x, y, z, groinAngle)  
  else
    kinematicsC:inverseKinematicsToFoot(leg, x, y, z, groinAngle)  
  end
  --return kinematicsC.hipPitch, kinematicsC.hipRoll, kinematicsC.kneePitch
end

CompTask('kicks_QuickKick',{
    init = true,
    kickType = core.FwdShortStraightKick,
    desiredKickHeading = 0,
    desiredKickDistance = 1000,
    -- probably don't overwrite these parameters when calling the kick
    state = stateMachine.StateMachine.create('Kick State',{'checkBall','shift','stand','kick','footBack','footDown','reset','finished'}),
    footSeparation = 100,
    stanceLegAngles = {-14.23,-44.48,77.17,-32.43,-22.23},
    stanceLegFootDownKnee = 72,
    stanceArmAngles = {128.20,-29.27},
    -- parameters below here are overwritten in chooseLegAndAim
    swingLeg = core.LEFTLEG,
    stanceLeg = core.RIGHTLEG,
    dir = 1,
    kickTime = 0.05,
    ballDistFromFootSide = 0,
    ballDistFromFootFwd = 0,
    kickHeading = 0
  })

function kicks_QuickKick:choose(as)
  if (self.init) then
    kicks_QuickKick_init(self)
  end
  --print ('kicks_QuickKick',self.state:currentName(),vision_frame_info.frame_id)
  --io.flush()
  --don't use a var for the current state, so if it changes, we move on immediately
  if (self.state:currentName() == 'checkBall') then
    as = kicks_QuickKick_checkBall(self)
  end
  if (self.state:currentName() == 'shift') then
    as = kicks_QuickKick_shift(self)
  end
  if (self.state:currentName() == 'stand') then
    as = kicks_QuickKick_stand(self)
  end
  if (self.state:currentName() == 'kick') then
    as = kicks_QuickKick_kick(self)
  end
  if (self.state:currentName() == 'footBack') then
    as = kicks_QuickKick_footBack(self)
  end
  if (self.state:currentName() == 'footDown') then
    as = kicks_QuickKick_footDown(self)
  end
  if (self.state:currentName() == 'reset') then
    as = kicks_QuickKick_reset(self)
  end

  if (self.state:currentName() == 'finished') then
    as = skills_ToPoseLookForBall:set{pose=standingPose,time = 2.0}
  end

  if as == nil then
    as = task_NullTask:set()
  end
  return as
end

function kicks_QuickKick_checkBall(self)
  local time = 0.5
  if (self.state:checkFirstTimeInState()) then
    commands.setStiffnessCommands(cfgStiffOldKick,0.15)
    return body_ToPose:set{pose=standingPose,time = time}
--    return com_FlattenFeet:set()
  end
 
  if (self.state:timeSinceTransition() > time) then
    self.state:transitionName('shift')
    kicks_QuickKick_chooseLegAndAim(self)

    --if (vision_frame_info.source == core.MEMORY_SIM) then
      --UTdebug.log(0, "sim kick", self.desiredKickHeading, self.desiredKickDistance)
      --local ball = world_objects:getObjPtr(core.WO_BALL)
      --robot:doSimKick(self.desiredKickDistance, self.desiredKickHeading)
      ---- update ball velocity
      --if (localizationC.pfActive) then
        --pfLocalizationC.ball:kickUpdate(0, self.desiredKickDistance/3.0, self.desiredKickHeading)
      --end
      --current.behaviorMem.attMode = 1
      --self.state:transitionName('finished')
    --end
  end
end  

function kicks_QuickKick_shift(self)
  local fastTime=0.60
  local slowTime=0.65

  if (vision_frame_info.source == core.MEMORY_SIM) then
    slowTime = 2.0
  end
 
  if self.state:checkFirstTimeInState() then
    --commands.setLeg(self.swingLeg,19.25,-43.77,80.33,-34.62,-18.28*self.dir,slowTime)
    --commands.setShoulderBasedOnLeg(self.swingLeg,105.38,1.58*self.dir,slowTime)

    --commands.setLeg(self.stanceLeg,self.stanceLegAngles[1],self.stanceLegAngles[2],self.stanceLegAngles[3],self.stanceLegAngles[4],self.stanceLegAngles[5]*self.dir,fastTime)
    --commands.setShoulderBasedOnLeg(self.stanceLeg,self.stanceArmAngles[1],self.stanceArmAngles[2]*self.dir,fastTime)
    setGroin(0)
    setLeg(self.swingLeg,19.25,-43.77,80.33,-34.62,-18.28*self.dir)
    setShoulderBasedOnLeg(self.swingLeg,105.38,1.58*self.dir)

    setLeg(self.stanceLeg,self.stanceLegAngles[1],self.stanceLegAngles[2],self.stanceLegAngles[3],self.stanceLegAngles[4],self.stanceLegAngles[5]*self.dir)
    setShoulderBasedOnLeg(self.stanceLeg,self.stanceArmAngles[1],self.stanceArmAngles[2]*self.dir)
    setTime(slowTime)
  end

  if (self.state:timeSinceTransition() > slowTime) then
    self.state:transitionName('stand')
  end
  
  return task_NullTask:set()
end

function kicks_QuickKick_stand(self)
  local fastTime=0.30
  local slowTime=0.35
  if self.state:checkFirstTimeInState() then
    local side,fwd,groin
    side,fwd,groin = kicks_QuickKick_getFootPosition(self,80)
    --commands.setJointInterpRad(core.LHipYawPitch,groin,fastTime)
    --kinematics.ikSwingLeg(self.swingLeg,fwd,side,30,current.percepts.bodyTilt,0,fastTime,0)
    --commands.setShoulderBasedOnLeg(self.swingLeg,105.38,1.58*self.dir,fastTime)

    --commands.setLeg(self.stanceLeg,self.stanceLegAngles[1],self.stanceLegAngles[2],self.stanceLegAngles[3],self.stanceLegAngles[4],self.stanceLegAngles[5]*self.dir,slowTime)
    --commands.setShoulderBasedOnLeg(self.stanceLeg,self.stanceArmAngles[1],self.stanceArmAngles[2]*self.dir,slowTime)
    
    setGroin(groin)
    ikSwingLeg(self.swingLeg,fwd,side,30,body_model.sensors_tilt_roll_.tilt_,0,fastTime,0)
    setShoulderBasedOnLeg(self.swingLeg,105.38,1.58*self.dir)

    setLeg(self.stanceLeg,self.stanceLegAngles[1],self.stanceLegAngles[2],self.stanceLegAngles[3],self.stanceLegAngles[4],self.stanceLegAngles[5]*self.dir)
    setShoulderBasedOnLeg(self.stanceLeg,self.stanceArmAngles[1],self.stanceArmAngles[2]*self.dir)
    
    setTime(slowTime)
  end
  if (self.state:timeSinceTransition() > slowTime) then
    self.state:transitionName('kick')
  end

  return task_NullTask:set()
end

function kicks_QuickKick_kick(self)
  local fastTime=self.kickTime
  if (self.state:checkFirstTimeInState() or (self.state:timeSinceTransition() > self.kickTime)) then
    local side,fwd,groin
    side,fwd,groin = kicks_QuickKick_getFootPosition(self,-40)
    --commands.setJointInterpRad(core.LHipYawPitch,groin,fastTime)
    --kinematics.ikSwingLeg(self.swingLeg,fwd,side,30,current.percepts.bodyTilt,groin,self.kickTime,DEG_T_RAD*-5)
    --commands.setShoulderBasedOnLeg(self.swingLeg,105.38,1.58*self.dir,fastTime)

    --commands.setLeg(self.stanceLeg,self.stanceLegAngles[1],self.stanceLegAngles[2],self.stanceLegAngles[3],self.stanceLegAngles[4],self.stanceLegAngles[5]*self.dir,fastTime)
    --commands.setShoulderBasedOnLeg(self.stanceLeg,self.stanceArmAngles[1],self.stanceArmAngles[2]*self.dir,fastTime)
    
    setGroin(groin)
    ikSwingLeg(self.swingLeg,fwd,side,30,body_model.sensors_tilt_roll_.tilt_,groin,self.kickTime,DEG_T_RAD*-5)
    setShoulderBasedOnLeg(self.swingLeg,105.38,1.58*self.dir)

    setLeg(self.stanceLeg,self.stanceLegAngles[1],self.stanceLegAngles[2],self.stanceLegAngles[3],self.stanceLegAngles[4],self.stanceLegAngles[5]*self.dir)
    setShoulderBasedOnLeg(self.stanceLeg,self.stanceArmAngles[1],self.stanceArmAngles[2]*self.dir)

    setTime(fastTime)
    -- update ball velocity
    if (localizationC.pfActive) then
      pfLocalizationC.ball:kickUpdate(0, self.desiredKickDistance/3.0, self.desiredKickHeading)
    end
    --current.behaviorMem.attMode = 1 -- TODO XXX
  end
  if (self.state:timeSinceTransition() > self.kickTime + 0.2) then
    self.state:transitionName('footBack')
  end
  return task_NullTask:set()
end

function kicks_QuickKick_footBack(self)
  local fastTime=0.30
  local slowTime=0.30
  if (self.state:checkFirstTimeInState()) then
    --kinematics.ikSwingLeg(self.swingLeg,10,self.footSeparation,30,current.percepts.bodyTilt,0,fastTime,0)   
    --commands.setShoulderBasedOnLeg(self.swingLeg,105.38,1.58*self.dir,fastTime)

    --commands.setLeg(self.stanceLeg,self.stanceLegAngles[1],self.stanceLegAngles[2],68,self.stanceLegAngles[4],self.stanceLegAngles[5]*self.dir,slowTime)
    --commands.setShoulderBasedOnLeg(self.stanceLeg,self.stanceArmAngles[1],self.stanceArmAngles[2]*self.dir,slowTime)
    setGroin(0)
    ikSwingLeg(self.swingLeg,10,self.footSeparation,30,body_model.sensors_tilt_roll_.tilt_,0,fastTime,0)
    setShoulderBasedOnLeg(self.swingLeg,105.38,1.58*self.dir)

    setLeg(self.stanceLeg,self.stanceLegAngles[1],self.stanceLegAngles[2],68,self.stanceLegAngles[4],self.stanceLegAngles[5]*self.dir)
    setShoulderBasedOnLeg(self.stanceLeg,self.stanceArmAngles[1],self.stanceArmAngles[2]*self.dir)
    setTime(slowTime)

    return head_MoveHead:set{ pan = self.kickHeading, tilt = DEG_T_RAD *-30.0, time = 0.5, head_init = true }
  end
  if (self.state:timeSinceTransition() > slowTime) then
    self.state:transitionName('footDown')
  end
  return task_NullTask:set()
end

function kicks_QuickKick_footDown(self)
  local fastTime=0.40
  local slowTime=0.40
  if (self.state:checkFirstTimeInState()) then
    commands.setStiffnessCommands(cfgStiffOldKickLegOne,0.01)
    --kinematics.ikSwingLeg(self.swingLeg,10,self.footSeparation,-5,current.percepts.bodyTilt,0,slowTime,0)   
    --commands.setShoulderBasedOnLeg(self.swingLeg,105.38,1.58*self.dir,slowTime)

    ----{-14.23,-44.48,77.17,-32.43,-22.23}
    ----{0,-25,55,-30,0}
    --commands.setLeg(self.stanceLeg,-10.23,-45,63,-30,-15.23*self.dir,fastTime)
----    commands.setLeg(self.stanceLeg,self.stanceLegAngles[1],self.stanceLegAngles[2],self.stanceLegFootDownKnee,self.stanceLegAngles[4],self.stanceLegAngles[5]*self.dir,slowTime) -- NOTE: changed stance knee
    --commands.setShoulderBasedOnLeg(self.stanceLeg,self.stanceArmAngles[1],self.stanceArmAngles[2]*self.dir,fastTime)
    setGroin(0)
    ikSwingLeg(self.swingLeg,10,self.footSeparation,-5,body_model.sensors_tilt_roll_.tilt_,0,slowTime,0)
    setShoulderBasedOnLeg(self.swingLeg,105.38,1.58*self.dir)
    setLeg(self.stanceLeg,-10.23,-45,63,-30,-15.23*self.dir)
    setShoulderBasedOnLeg(self.stanceLeg,self.stanceArmAngles[1],self.stanceArmAngles[2]*self.dir)
    setTime(slowTime)

    return head_MoveHead:set{ pan = self.kickHeading, tilt = DEG_T_RAD *-30.0, time = 0.5, head_init = true }
  end
  if (self.state:timeSinceTransition() > slowTime) then
    self.state:transitionName('reset')
  end
  return task_NullTask:set()
end

function kicks_QuickKick_reset(self)
  local time = 0.5
  if self.state:checkFirstTimeInState() then
    commands.setStiffnessCommands(cfgStiffALWalk,0.01)
  end
  if (self.state:timeSinceTransition() > time) then
    self.state:transitionName('finished')
  end
  return skills_ToPoseLookForBall:set{pose=standingPose,time = time}
end

function kicks_QuickKick_init(self) 
  if self.init then
    self.init = false
    
    self.state:transitionName('checkBall')
    commands.setHeadPanTilt(0, DEG_T_RAD * -40.0, 0.05, false)
  end
  
  -- don't do anything until walk is done
  if (walk_info.walk_is_active_) then
    if (vision_frame_info.source == core.MEMORY_SIM) then
      -- TODO - currently, no walk in sim
    else
      self.init = true
      commands.stand()
    end
  end 
end

function kicks_QuickKick_chooseLegAndAim(self)
  -- get the correct kick type
  local kick = self.kickType
  local ball = world_objects:getObjPtr(core.WO_BALL)

  -- XXX
  if (vision_frame_info.source == core.MEMORY_SIM) then
    ball.imageCenterY = 360
    ball.imageCenterX = 160
    ball.seen = true
  end

  -- check if ball is reasonable
  if not(ball.seen) or (ball.imageCenterY < 210) then
    UTdebug.log(0,'Aborting kick, ball not seen or too far. seen, imageCenterY:',ball.seen,ball.imageCenterY)
    self.state:transitionName('finished')
    return
  end

  -- set the kick speed (i.e. interp time) based on the desired distance
  -- linear wrt to log
  --self.kickTime = -0.21507680 * math.log(self.desiredKickDistance) + 1.8236703
 self.kickTime = -0.13383284 * math.log(self.desiredKickDistance) + 1.1441225
  self.kickTime = core.crop(self.kickTime,0.05,0.5) -- keep the interp time in bounds

  local offset = cfgCamOffsets[robot_state.robot_id_]
  if (offset == nil) then
    offset = cfgCamOffsets[0]
  end

  ball.imageCenterX = ball.imageCenterX + offset[1]
  --UTdebug.log(0,'Ball position in image:',ball.imageCenterX,ball.imageCenterY)  
  
-- Removed references to IMAGE_X because image resolution is no longer hard coded - JM 05/22/13
--   if (ball.imageCenterX < (core.IMAGE_X/2.0) ) then
--       self.swingLeg = core.LEFTLEG 
--   else
--       self.swingLeg = core.RIGHTLEG
--   end

  local maxOffset = 60
  if not(kicks.kickData[kick].switchable) then
--     if (kicks.kickData[kick].heading < 0) then
--       if (ball.imageCenterX < ((core.IMAGE_X / 2.0) + maxOffset)) then
--         self.swingLeg = core.LEFTLEG
--       else
--         commands.sayText('fudge')
--       end
--     else
--       if (ball.imageCenterX > ((core.IMAGE_X / 2.0) - maxOffset)) then
--         self.swingLeg = core.RIGHTLEG
--       else
--         commands.sayText('fudge')
--       end
--     end
  end
  
  if self.swingLeg == core.LEFTLEG then
    self.stanceLeg = core.RIGHTLEG
    self.dir = 1
    self.kickHeading = core.crop(self.desiredKickHeading,DEG_T_RAD * -30,DEG_T_RAD * 10)
  else
    self.stanceLeg = core.LEFTLEG
    self.dir = -1
    self.kickHeading = core.crop(self.desiredKickHeading,DEG_T_RAD * -10,DEG_T_RAD * 30)
  end
  UTdebug.log(0,'imageCenterX, leg=',ball.imageCenterX,self.swingLeg)

  if (self.swingLeg == core.LEFTLEG) then
    commands.setStiffnessCommands(cfgStiffOldKickLeftLegOne,0.01)
  else
    commands.setStiffnessCommands(cfgStiffOldKickRightLegOne,0.01)
  end
--  commands.interpLegJointStiffness(self.stanceLeg,1.0,0.01)
  --UTdebug.log(20,'Choosing kick leg, heading ',self.swingLeg, self.kickHeading)

  -- handle the aim stuff
  local xOffset = math.abs(320 - ball.imageCenterX)
  self.ballDistFromFootSide = 0.2976 * (xOffset - 102) -- 102 because pixel 218 is centered and 320 - 218 = 102
  self.ballDistFromFootFwd = 10 * (-0.04807 * ball.imageCenterY + 16.25 + 3.3) -- the first part is fit to the front of the ball, the 3.3 is the radius of the ball,  *10 for cm to mm

  --UTdebug.log(20,'Aim: ',self.ballDistFromFootSide, self.ballDistFromFootFwd)
  --UTdebug.log(0,'Performing kick with type, distance, heading:',robot:getString(core.kickNames,self.kickType), self.desiredKickDistance,RAD_T_DEG * self.desiredKickHeading)
  --UTdebug.log(20,'Actual kick parameters, time, heading:',self.kickTime,RAD_T_DEG * self.kickHeading)
  -- say the kick for now -- TODO disable this
  --commands.sayText(robot:getString(core.kickNames,self.kickType))
end

function kicks_QuickKick_getFootPosition(self,swingBackFromBall) 
  local theta = -1 * self.dir * self.kickHeading --self.desiredKickHeading
  local side = self.footSeparation + self.ballDistFromFootSide
  local fwd = swingBackFromBall

  local c = math.cos(theta)
  local s = math.sin(theta)
  side = side + fwd  * s
  fwd  = fwd  * c
  side = core.crop(side,self.footSeparation-25,self.footSeparation+50)
  fwd = -fwd + self.ballDistFromFootFwd --change to relative to origin
  fwd = core.crop(fwd,-40,150)
  
  local groin = 0 -- theta -- TODO
  return side,fwd,groin
end

function kicks_QuickKick:finished()
  return (self.state:currentName() == 'finished')
end


-- vim: ts=2:sw=2:expandtab:softtabstop=2
