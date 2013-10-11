require 'commands'
require 'task'
require 'poses'
require 'cfgstiff'
require 'strategy'
module(..., package.seeall)

function avoidSonarObstaclesByPos(desiredPt, isRelative)

  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  local desiredPtChange = false
  local savedDesiredPt = desiredPt
  if isRelative then
    desiredPt = desiredPt:relativeToGlobal(me.loc, me.orientation)
  end
  local myDirAbs = me.loc:getAngleTo(desiredPt)
  local myDir = core.normalizeAngle(myDirAbs - me.orientation)

  UTdebug.log(10, "  I am at ", me.loc, " with orientation ", me.orientation)
  UTdebug.log(10, "  I want to go to the location at ", desiredPt)
  UTdebug.log(10, "  For this the absolute orientation is ", myDirAbs)
  UTdebug.log(10, "  which relative to my orientation is ", myDir)

  local range_min = {}
  local range_max = {}

  local sonarMinDist = 0.3
  local sonarMaxDist = 0.8
  local sonarCurvePow = 1

  -- process sonar information
  if (processed_sonar ~= nil) then
    -- avoid range on left
    if (processed_sonar.on_left_ ~= false) then
      UTdebug.log(10,"Sonar object detected on left: ", processed_sonar.left_distance_)
      distance_diff = processed_sonar.left_distance_ - sonarMinDist
      if (distance_diff < 0) then
        distance_diff = 0
      end
      if (distance_diff < sonarMaxDist - sonarMinDist) then
        range = math.pi / (1 + 5 * math.pow(distance_diff, sonarCurvePow) / math.pow(sonarMaxDist - sonarMinDist, sonarCurvePow))
        table.insert(range_min, math.pi / 4 - range / 2)
        table.insert(range_max, math.pi / 4 + range / 2)
      end
    end

    -- avoid range on center
    if (processed_sonar.on_center_ ~= false) then
      UTdebug.log(10,"Sonar object detected on center: ", processed_sonar.center_distance_)
      distance_diff = processed_sonar.center_distance_ - sonarMinDist
      if (distance_diff < 0) then
        distance_diff = 0
      end
      if (distance_diff < sonarMaxDist - sonarMinDist) then
        range = math.pi / (1 + 5 * math.pow(distance_diff, sonarCurvePow) / math.pow(sonarMaxDist - sonarMinDist, sonarCurvePow))
        table.insert(range_min, -range / 2)
        table.insert(range_max, range / 2)
      end
    end

    -- avoid range on right
    if (processed_sonar.on_right_ ~= false) then
      UTdebug.log(10,"Sonar object detected on right: ", processed_sonar.right_distance_)
      distance_diff = processed_sonar.right_distance_ - sonarMinDist
      if (distance_diff < 0) then
        distance_diff = 0
      end
      if (distance_diff < sonarMaxDist - sonarMinDist) then
        --UTdebug.log(20," The difference in distance is: ", distance_diff)
        range = math.pi / (1 + 5 * math.pow(distance_diff, sonarCurvePow) / math.pow(sonarMaxDist - sonarMinDist, sonarCurvePow))
        table.insert(range_min, -math.pi / 4 - range / 2)
        table.insert(range_max, -math.pi / 4 + range / 2)
      end
    end

  end

  -- get rid of any overlapping ranges
  local size = table.getn(range_min)
  local i = 1
  while size > 1 do
    if range_max[i] >= range_min[i + 1] then
      range_max[i] = range_max[i + 1]
      table.remove(range_min, i + 1)
      table.remove(range_max, i + 1)
    else
      i = i + 1;
    end
    size = size - 1;
  end

  behavior_mem.sonarAvoidRangeCount = 0
  for i,v in ipairs(range_min) do
    luaC:setFloat(behavior_mem.sonarAvoidRangeMax, behavior_mem.sonarAvoidRangeCount, range_max[i])
    luaC:setFloat(behavior_mem.sonarAvoidRangeMin, behavior_mem.sonarAvoidRangeCount, range_min[i])
    behavior_mem.sonarAvoidRangeCount = behavior_mem.sonarAvoidRangeCount + 1

    UTdebug.log(20,"Sonar avoid range is (",v,",", range_max[i],")")
    UTdebug.log(20,"  and my Direction is ", myDir)
    if myDir > range_min[i] and myDir < range_max[i] then
      turnAngle = 0
      if math.abs(myDir - range_min[i]) < math.abs(myDir - range_max[i]) then
        turnAngle = range_min[i] - myDir
        myDir = range_min[i]
      else
        turnAngle = range_max[i] - myDir
        myDir = range_max[i]
      end
      turnAngle = turnAngle * 180 / math.pi
      UTdebug.log(10,"SONAR: Turning by ",turnAngle, "degrees")
      myDirAbs = core.normalizeAngle(myDir + me.orientation)
      newDesiredPt = core.Point2D_getPointFromPolar(750, myDirAbs)
      desiredPt = me.loc + newDesiredPt
      desiredPtChanged = true
      break
    end
  end

  if isRelative then
    if desiredPtChanged then
      desiredPt = desiredPt:globalToRelative(me.loc, me.orientation)
    else
      desiredPt = savedDesiredPt
    end
  end
  return desiredPt 
end

function avoidTeammates(desiredPt)
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  -- go through each teammate
  for i=core.WO_TEAM_FIRST,core.WO_TEAM_LAST do
    if (i ~= robot_state.WO_SELF and (vision_frame_info.frame_id - team_packets:getFrameReceived(i)) < 30) then
      -- get distance
      local tp = team_packets:getPktPtr(i)
      local mateLoc = core.Point2D(tp.locData.robotX, tp.locData.robotY)
      local mateDist = mateLoc:getDistanceTo(me.loc)
      local ourDir = me.loc:getAngleTo(desiredPt)
      local ourAngleToMate = me.loc:getAngleTo(mateLoc)
      local mateTargetAngleDiff = core.normalizeAngle(ourDir - ourAngleToMate)
      -- close and we're moving towards them
      if (mateDist < 500 and math.abs(mateTargetAngleDiff) < 1.57) then
        UTdebug.log(10,"At",me.loc,"within",mateDist,"of mate",i,"at",mateLoc)
        local rotateAngle = 60.0 * DEG_T_RAD
        -- pick a side to go around them
        -- if they're moving, go behind them
        local ourDir = me.loc:getAngleTo(desiredPt)
        local mateDest = core.Point2D(tp.bvrData.targetX,tp.bvrData.targetY)
        local theirDir = mateLoc:getAngleTo(mateDest)
        local mateTravelDist = mateLoc:getDistanceTo(mateDest)
        local mateAngleToUs = mateLoc:getAngleTo(me.loc)
        if (mateTravelDist > 250) then
          -- pick a side based on their direction
          local angleDiff = core.normalizeAngle(mateAngleToUs - theirDir)
          if (angleDiff > 0) then
            --rotate pos
            rotateAngle = 60.0 * DEG_T_RAD
          else
            rotateAngle = -60.0 * DEG_T_RAD
          end
        else
          -- pick solely based on our direction
          if (mateTargetAngleDiff < 0) then
            --rotate pos
            rotateAngle = 60.0 * DEG_T_RAD
          else
            rotateAngle = -60.0 * DEG_T_RAD
          end
        end
        -- put the destination point on a circle around mate
        local newDest = mateLoc+core.Point2D(500, mateAngleToUs+rotateAngle,core.POLAR)
        desiredPt = newDest
      end -- they're close
    end -- not us
  end -- mate loop
  return desiredPt
end

CompTask('body_MoveHome',{
           init = true,
           done = false
         })
function body_MoveHome:choose(as)
--[[
  keeper = core.Pose2D(0.0, -core.HALF_FIELD_X+200.0 ,0.0)  -- Orientation, X,Y

  -- Rules: keeper must be inside penalty area, no other players can be
  -- On our kick off, only one player inside circle, others anywhere in half
  -- On opp kick off, players behind cross (-1200)

  -- for our kickoff
  kickoff_attacker         = core.Pose2D(45, -300,-300)
  kickoff_attacker_no_wing = core.Pose2D(20, -300,-220)
  kickoff_left_wing        = core.Pose2D( 0, -300, 1400)
  kickoff_right_wing       = core.Pose2D( 0, -300,-1400)
  kickoff_left_defender    = core.Pose2D( 0,-1600, 400)
  kickoff_right_defender   = core.Pose2D( 0,-1600,-400)

  receive_middle = core.Pose2D(0,-1100,0)
  receive_left = core.Pose2D(0,-900,800)
  receive_right = core.Pose2D(0,-900,-800)
--]]

  if (as == body_MoveToPoint) then
    if (as:finished()) then
      self.done = true
    end
  end

  local desired_pos = core.Pose2D()
  if (game_state.ourKickOff) then
    desired_pos = luaC:getPose2D(core.ourKickoffPosesDesired,robot_state.role_)
  else
    desired_pos = luaC:getPose2D(core.theirKickoffPosesDesired,robot_state.role_)
  end
--[[
  if (robot_state.role_ == core.KEEPER) then
    desired_pos = keeper
  elseif (robot_state.role_ == core.CHASER) then
    if (game_state.ourKickOff) then
      desired_pos = kickoff_attacker
  
      local maxWingErrorAllowed = 600
      local maxWingOientErrorAllowed = 65 * DEG_T_RAD
      local wingError = 99999
      local orientError = 9999
      for i = core.WO_TEAM_FIRST,core.WO_TEAM_LAST do
        if (i ~= robot_state.WO_SELF) and (i ~= core.KEEPER) then 
          local mate = world_objects:getObjPtr(i)
          local err = (mate.loc - core.Point2D(kickoff_left_wing.translation.x,kickoff_left_wing.translation.y)):getMagnitude()
          --print('  mate:',i,mate.loc.x,mate.loc.y)
          --print('  kickoff_left_wing:',kickoff_left_wing.translation.x,kickoff_left_wing.translation.y)
          --print('  err:',err)
          if err < wingError then
            wingError = err
            orientError = math.abs(mate.orientation - kickoff_left_wing.rotation)
          end
        end
      end
      --print('minError:',minWingError)
      if (wingError > maxWingErrorAllowed) or (orientError > maxWingOientErrorAllowed) then
        --speech:say('No wing')
        desired_pos = kickoff_attacker_no_wing
      end

    else
      desired_pos = receive_middle
    end
  elseif (robot_state.role_ == core.SUPPORTER) then
    if (game_state.ourKickOff) then
      desired_pos = kickoff_left_wing
    else
      desired_pos = receive_left
    end
  else -- role 3
    if (game_state.ourKickOff) then
      desired_pos = kickoff_right_defender
    else
      desired_pos = kickoff_right_defender--receive_right
    end
  end
--]]
  return body_MoveToPoint:set{maxVel = 1.0,des = core.Point2D(desired_pos.translation.x,desired_pos.translation.y), desTheta = desired_pos.rotation}

end

function body_MoveHome:finished()
  return self.done
end


PrimTask('body_MoveToPoint', {
           des = core.Point2D(0,0),
           desTheta = 0.0,
           lookAtPoint = false,
           look = core.Point2D(0,0),
           close = false,
           walkStraight = true, -- if true, turns towards point and goes, otherwise tries to maintain target orientation
           maxVel = 1,
           maxTurn = 0.5,
           maxBackward = 0.25,
           orientThresh = 600.0,
           lastTurn = 0,
           doneCount = 0,
           doneDist = 50,
           doneAngle = 5.0,
           doneThresh = 20,
           --how close do we have to get to start slowing from max vel
           distThresh = 600,
           turnThresh = 1.8, -- 103 degrees
           turnToFace = false,
           lastSprint = false,
           maxSprintTurn = 0.2
         })
function body_MoveToPoint:run()

  -- get our location
  local selfRobot = world_objects:getObjPtr(robot_state.WO_SELF)
  local loc = selfRobot.loc
  local desTheta = self.desTheta
  self.turnToFace = false

  -- sprint in past orient thresh, so that we dont trash between modes
  local orientThresh = self.orientThresh
  if (self.lastSprint) then
    orientThresh = orientThresh - 200
  end

  -- two versions, one that looks at a point and one that does not
  if (self.lookAtPoint) then
    -- calculate bearing to point
    desTheta = loc:getBearingTo(self.look, 0)

  end

  -- even look at point may walk straight if far enough away
  if (self.walkStraight and loc:getDistanceTo(self.des) > orientThresh) then
    -- get the bearing to walk straight to the point
    desTheta = loc:getBearingTo(self.des, 0)
    self.turnToFace = true

    UTdebug.log(20, "Turn straight toward target point", desTheta*RAD_T_DEG)

  end

  self.close = (loc:getDistanceTo(self.des) < self.doneDist) and (math.abs(selfRobot.orientation - desTheta) < self.doneAngle*DEG_T_RAD)

  if (self.close) then
    self.doneCount = self.doneCount + 1
  elseif (self.doneCount > 0) then
    self.doneCount = self.doneCount - 1
  end

  -- get error in these
  local diff = self.des - loc

  local diffTheta = core.normalizeAngle(desTheta - selfRobot.orientation)

  -- bias towards last turn direction if its all the way around
  if (math.abs(diffTheta) > DEG_T_RAD * 160.0) then
    if (self.lastTurn > 0 and diffTheta < 0) then
      --UTdebug.log(20, "Over 160 deg, stay with last turn", diffTheta, self.lastTurn)
      diffTheta = core.normalizeAngle(diffTheta + 30.0*DEG_T_RAD)
    elseif (self.lastTurn < 0 and diffTheta > 0) then
      --UTdebug.log(20, "Over 160 deg, stay with last turn", diffTheta, self.lastTurn)
      diffTheta = core.normalizeAngle(diffTheta - 30.0*DEG_T_RAD)
    end
  end

  -- rotate to put into robot's coordinate frame
  diff:rotate(-selfRobot.orientation)

  behavior_mem.targetPt = diff
  behavior_mem.useTarget = true
  behavior_mem.absTargetPt = self.des
  behavior_mem.useAbsTarget = true
  behavior_mem.useTargetBearing = false

  UTdebug.log(20, "self, des, diff, diffTheta", loc, RAD_T_DEG*selfRobot.orientation,self.des, diff, RAD_T_DEG*diffTheta, self.close)

  -- set velocity
  local velX, velY, velTheta
  if (diff.x > self.distThresh) then
    velX = self.maxVel
  elseif (diff.x < -self.distThresh) then
    velX = -self.maxBackward
  else
    velX = self.maxVel * diff.x / self.distThresh
  end

  if (diff.y > self.distThresh) then
    velY = self.maxVel
  elseif (diff.y < -self.distThresh) then
    velY = -self.maxVel
  else
    velY = self.maxVel * diff.y / self.distThresh
  end

  if (diffTheta > self.turnThresh) then
    velTheta = self.maxTurn
  elseif (diffTheta < -self.turnThresh) then
    velTheta = -self.maxTurn
  else
    velTheta = self.maxTurn * diffTheta / self.turnThresh
  end

  self.lastTurn = 1
  if (velTheta < 0) then
    self.lastTurn = -1
  end

  local localMaxSprintTurn = self.maxSprintTurn + 0.1
  -- go into sprint more if we're really far
  if (loc:getDistanceTo(self.des) > 2500) then
    localMaxSprintTurn = localMaxSprintTurn + 0.1
  end

  -- if we're roughly facing point, lets sprint
  if (self.walkStraight and self.turnToFace) then
    UTdebug.log (10, "in walk straight, turn error is ", diffTheta *RAD_T_DEG, self.lastSprint, self.lastTurn, velTheta)
    -- if we were turning, continue turn vel under 0.14
    -- if we were sprinting, start turning if turn vel over 0.2
    if ((not self.lastSprint and math.abs(velTheta) > (0.7*localMaxSprintTurn)) or (math.abs(velTheta) > localMaxSprintTurn)) then
      velX = 0
      velY = 0
      UTdebug.log(10, "turn to face point first", diffTheta * RAD_T_DEG, "new vel", velX, velY, velTheta)
      self.lastSprint = false
      -- otherwise, sprint
    else
      velY = 0
      -- still crop to actual limit
      velTheta = core.crop(velTheta, -self.maxSprintTurn, self.maxSprintTurn)
      UTdebug.log(10, "sprint to point", diffTheta * RAD_T_DEG, "new vel", velX, velY, velTheta)
      self.lastSprint = true
    end
  else
    self.lastSprint = false
  end

  UTdebug.log(10, "vel before sonar", velX, velY, velTheta, self.lastSprint)
  local sonarChange = false
  local bumpChange = false
  velX, velY, velTheta, self.lastSprint, sonarChange = commands.avoidSonarObstacles(velX, velY, velTheta, self.lastSprint, diff)
  if (not sonarChange) then
    velX, velY, velTheta, self.lastSprint, bumpChange = commands.avoidBumpObstacles(velX, velY, velTheta, self.lastSprint, diff)
  end

  if (sonarChange or bumpChange) then
    UTdebug.log(10, "avoid", sonarChange, bumpChange, velX, velY, velTheta, self.lastSprint)
  end

  commands.setWalkVelocity(velX, velY, velTheta, self.lastSprint)
end

function body_MoveToPoint:finished()
  return (self.doneCount > self.doneThresh)
end

PrimTask('body_ArcToBallFacingGoal',{
           init = true,
           xSpeedFactor = 1.50,--0.75, --2.0,
           ySpeedFactor = 4.5, --3.0,
           turnFactor = 0.5,
           done = false,
           backDist = 600,
           heading = 0.0,
           tolerance = 0.174,
           lastSprint = false,
           maxSprintTurn = 0.2
         })
function body_ArcToBallFacingGoal:run()
  local ball = world_objects:getObjPtr(core.WO_BALL)
  local dx = ball.distance * math.cos(ball.bearing)
  local dy = ball.distance * math.sin(ball.bearing)

  local ballRelLoc = core.Point2D(dx,dy)

  -- get a point behind the ball, at the desired heading
  local backDist = math.min(ball.distance/2.0, self.backDist)
  backDist = math.max(backDist, 220.0)

  local maxArcingAngle = behavior_params.mainStrategy.maxArcAngle
  -- crop angle to be close to robot's angle to ball
  -- can't do this with crop since its angles and they wrap around
  -- if we need to crop
  local approachAngle = self.heading
  if (ball.loc.x < 0) then
    local ownGoal = world_objects:getObjPtr(core.WO_OWN_GOAL)
    approachAngle = core.normalizeAngle(ownGoal.bearing + 180.0*DEG_T_RAD)
  end
    
  local headingDiff = math.abs(approachAngle-ball.bearing)
  if (headingDiff > 180.0*DEG_T_RAD) then
    headingDiff = 360.0*DEG_T_RAD - headingDiff
  end
  if (headingDiff > maxArcingAngle) then
    -- find closer one
    local dist1 = math.abs(approachAngle-(ball.bearing-maxArcingAngle))
    if (dist1 > 180.0*DEG_T_RAD) then
      dist1 = 360.0*DEG_T_RAD - dist1
    end
    local dist2 = math.abs(approachAngle-(ball.bearing+maxArcingAngle))
    if (dist2 > 180.0*DEG_T_RAD) then
      dist2 = 360.0*DEG_T_RAD - dist2
    end
    if (dist1 < dist2) then
      approachAngle = core.normalizeAngle(ball.bearing-maxArcingAngle)
    else
      approachAngle = core.normalizeAngle(ball.bearing+maxArcingAngle)
    end
  end

  --current.behaviorMem.approachAngle = approachAngle -- TODO
  local ballOffset = core.Point2D(-backDist, approachAngle,core.POLAR)
  local targetPt = ballRelLoc + ballOffset

  -- always look point next to ball that we want to align to
  -- so not straight at ball since it should be on left/right foot
  local targetBearing = ballRelLoc:getDirection()

  -- Check if we need to change target pt to avoid illegal defender
  if (robot_state.WO_SELF ~= core.KEEPER) then
    -- only modify target point
    -- sprint mode will change bearing to face point first, once we get close
    -- we do want to turn to face ball
    targetPt = avoidPenaltyBox(targetPt,targetBearing)
  end

  local me = world_objects:getObjPtr(robot_state.WO_SELF)

  --UTdebug.log(20, "MTBOFAG", self.heading*RAD_T_DEG,ballRelLoc,ballOffset,targetPt)

  --UTdebug.log(20, "MTBOFAG dir", RAD_T_DEG*targetPt:getDirection(), RAD_T_DEG*self.heading, RAD_T_DEG*targetBearing)
  -- Todd: lets keep going forward quickly
  -- rather than slow down as we approach arc point 0.5m behind ball
  --local xSpeed = (targetPt.x) / (self.xSpeedFactor) -- * current.commands.walkParams.maxFwdVel)

  local xSpeed = 1.0
  if (targetPt.x < 0) then
    xSpeed = -0.1
  end
  local ySpeed = (targetPt.y) / (self.ySpeedFactor * commands.walk_max_vel_y)
  local turnSpeed = self.turnFactor*targetBearing

  -- min tolerance of 10 deg
  self.tolerance = math.max(self.tolerance, 10.0*DEG_T_RAD)

  self.done = math.abs(ySpeed) < 0.1 and math.abs(turnSpeed) < 0.1 and math.abs(self.heading) < self.tolerance

  behavior_mem.targetPt = targetPt
  behavior_mem.useTarget = true
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  behavior_mem.absTargetPt = targetPt:relativeToGlobal(me.loc, me.orientation)
  behavior_mem.useAbsTarget = true
  behavior_mem.targetBearing = targetBearing
  behavior_mem.useTargetBearing = true

  --UTdebug.log(20, "MTBOFAG speed", xSpeed, ySpeed, turnSpeed)

  -- after this threshold, we start walking omnidirectional to get to final pose
  local orientThresh = 600
  -- sprint past orient thresh, to prevent thrashing
  if (self.lastSprint) then
    orientThresh = orientThresh - 200
  end

  local localMaxSprintTurn = self.maxSprintTurn + 0.1
  -- go into sprint more if we're really far
  if (targetPt:getMagnitude() > 2500) then
    localMaxSprintTurn = localMaxSprintTurn + 0.1
  end


  -- turn to face target, then sprint
  if (targetPt:getMagnitude() > orientThresh) then
    local diffTheta = targetPt:getDirection()
    local turnThresh = 1.8 -- 103 degrees
    if (diffTheta > turnThresh) then
      turnSpeed = 0.5
    elseif (diffTheta < -turnThresh) then
      turnSpeed = -0.5
    else
      turnSpeed = 0.5 * diffTheta / turnThresh
    end
    -- if we were turning, continue turn vel under 0.14
    -- if we were sprinting, start turning if turn vel over 0.2
    if ((not self.lastSprint and math.abs(turnSpeed) > (0.7*localMaxSprintTurn)) or (math.abs(turnSpeed) > localMaxSprintTurn)) then
      xSpeed = 0
      ySpeed = 0
      UTdebug.log(10, "turn to face point first", diffTheta * RAD_T_DEG, "new vel", xSpeed, ySpeed, turnSpeed)
      self.lastSprint = false
      -- otherwise, sprint
    else
      ySpeed = 0
      -- still crop to actual turn limit
      turnSpeed = core.crop(turnSpeed, -self.maxSprintTurn, self.maxSprintTurn)
      UTdebug.log(10, "sprint to point", diffTheta * RAD_T_DEG, "new vel", xSpeed, ySpeed, turnSpeed)
      self.lastSprint = true
    end
  else
    -- then when close, turn back the way we want again
    self.lastSprint = false
  end

  local sonarChange = false
  local bumpChange = false
  xSpeed, ySpeed, turnSpeed, self.lastSprint, sonarChange = commands.avoidSonarObstacles(xSpeed, ySpeed, turnSpeed, self.lastSprint, targetPt)
  if (not sonarChange) then
    xSpeed, ySpeed, turnSpeed, self.lastSprint, bumpChange = commands.avoidBumpObstacles(xSpeed, ySpeed, turnSpeed, self.lastSprint, targetPt)
  end

  if (sonarChange or bumpChange) then
    UTdebug.log(10, "avoid", sonarChange, bumpChange, xSpeed, ySpeed, turnSpeed, self.lastSprint)
  end

  commands.setWalkVelocity(xSpeed,ySpeed,turnSpeed,self.lastSprint)
end

function body_ArcToBallFacingGoal:finished()
  return false --self.done
end

-- desired bearing
PrimTask('body_RotateAroundBall',{
           bearing = 0.0,
           targetDistance = 40, --200,
           speed = 2.0, --0.75, --1.0,
           useGoalBearing = false,
           done = false,
           doneCount = 0,
           tolerance = 0.08, -- 0.14
           init = true,
           desiredFoot = core.Kick_RIGHT
         })
function body_RotateAroundBall:run()
  local ball = world_objects:getObjPtr(core.WO_BALL)
  --local distance = ball.relPos:getMagnitude()
  local relPosBall = core.Point2D(localizationC.filtered_close_ball_.x,localizationC.filtered_close_ball_.y)
  --local relPosBall = core.Point2D(self.targetDistance,0)
  local distance = relPosBall:getMagnitude()
  local ballRelOrientation = relPosBall:getDirection()

  if (self.useGoalBearing) then
    local oppGoal = world_objects:getObjPtr(core.WO_OPP_GOAL)
    self.bearing = oppGoal.bearing
  end

  -- no tolerance for penalty kick, make kick selection say there is a valid kick
  if (game_state.isPenaltyKick) then
    self.tolerance = 2.0*DEG_T_RAD
  end

  -- for now
  if (math.abs(self.bearing) < self.tolerance) then
    self.doneCount = self.doneCount + 1
  elseif (self.doneCount > 0) then
    self.doneCount = self.doneCount - 1
  end

  self.done = (self.doneCount > 5)

  -- desired orient of ball to us
  -- depending on which way we're going, we want ball on left or right foot
  local desiredRelBallOrient = 0
  if desiredFoot == core.Kick_RIGHT then
    desiredRelBallOrient = -18.0 * DEG_T_RAD
  elseif desiredFoot == core.Kick_LEFT then
    desiredRelBallOrient = 18.0 * DEG_T_RAD
  end
  --if (self.bearing < 0) then
    ---- moving left, want left foot, positive bearing
    --desiredRelBallOrient = 18.0 * DEG_T_RAD
  --else
    --desiredRelBallOrient = -18.0 * DEG_T_RAD
  --end
  -- Todd: just do 0, these seem to be messing us up
  --desiredRelBallOrient = 0.0 * DEG_T_RAD

  local relBallOriError = ballRelOrientation - desiredRelBallOrient
  UTdebug.log(10, "Rotating", self.bearing*RAD_T_DEG, "want ball orient",desiredRelBallOrient*RAD_T_DEG,"actual",ballRelOrientation*RAD_T_DEG,"error",relBallOriError*RAD_T_DEG)
  
  local id = robot_state.robot_id_

  local maxStrafe = 1.0
  local strafeSpeed = -self.bearing * 2 * self.speed
  strafeSpeed = core.crop(strafeSpeed,-maxStrafe,maxStrafe)
  local turnSpeed = relBallOriError / 2.0 - strafeSpeed / 2.0 --8.0 --10.0
  local xSpeed = (distance - self.targetDistance) / (2.0 * commands.walk_max_vel_x)

  xSpeed = math.max(math.min(xSpeed,1.0),-1.0)
  turnSpeed = math.max(math.min(turnSpeed,0.8),-0.8)

  commands.setWalkVelocity(xSpeed,strafeSpeed,turnSpeed,false)

  UTdebug.log(20,"Rotate", self.bearing*RAD_T_DEG,ballRelOrientation*RAD_T_DEG,relBallOriError*RAD_T_DEG,distance,xSpeed,strafeSpeed,turnSpeed)
end

function body_RotateAroundBall:finished()
  return self.done
end


-- desired bearing
PrimTask('body_RotateAroundBallWithRotateWalk',{
           bearing = 0.0,
           targetDistance = 250, --200,
           useGoalBearing = false,
           done = false,
           doneCount = 0,
           tolerance = 0.174
         })
function body_RotateAroundBallWithRotateWalk:run()
  local ball = world_objects:getObjPtr(core.WO_BALL)
  local dx = ball.distance * math.cos(ball.bearing)
  local dy = ball.distance * math.sin(ball.bearing)

  if (self.useGoalBearing) then
    local oppGoal = world_objects:getObjPtr(core.WO_OPP_GOAL)
    self.bearing = oppGoal.bearing
  end

  self.tolerance = math.max(self.tolerance, 10.0*DEG_T_RAD)

  -- no tolerance for penalty kick, make kick selection say there is a valid kick
  if (game_state.isPenaltyKick) then
    self.tolerance = 0.0
  end

  -- for now
  if (math.abs(self.bearing) < self.tolerance) then
    self.doneCount = self.doneCount + 1
  elseif (self.doneCount > 0) then
    self.doneCount = self.doneCount - 1
  end

  self.done = (self.doneCount > 5)

  commands.setWalkRotate(dx, dy, self.targetDistance, self.bearing)

  UTdebug.log(12,"Rotate", self.bearing*RAD_T_DEG,ball.bearing*RAD_T_DEG,ball.distance)
end

function body_RotateAroundBallWithRotateWalk:finished()
  return self.done
end


CompTask('body_DribbleBall',{
           heading =  0,
           allowPositionCancel = true,
           init = true
         })

function body_DribbleBall:choose(as)
  local givenHeading = self.heading
  if (self.heading < DEG_T_RAD * 15.0) then
    givenHeading = 0
  end

  if (self.init) then
    self.init = false
    --speech:say("Dribble")
  end

  --return body_AlignToBallWithTargetWalk:set{desiredX=45, desiredY = 50, dribble = true, switchable = true, heading = givenHeading}
  return body_AlignToBallWithTargetWalk:set{dribble = true, switchable = true, heading = givenHeading, kickChoice = core.Dribble, alignment = kicks.kickData[core.Dribble].alignment}
  --return body_MoveAndAlignToBall:set{desiredX=45, desiredY = 50, dribble = true, switchable = true, heading = givenHeading}

end

function body_DribbleBall:finished()
  if (true) then
    return false
  end
  local ball = world_objects:getObjPtr(core.WO_BALL)
  if (ball.distance > 220) then
    return true
  end

  local onGoalLine = strategy.checkGoalLineDribble()
  local notInCorner = (not behavior_mem.ballIsInCorner)
  if (self.allowPositionCancel and ( notInCorner and onGoalLine==-1) ) then
    return true
  end

  return false
end

CompTask('body_MoveAndAlignToBall',{
           init = true,
           desiredX = 130, --170, --65, --60,--165 --155, --155,
           yOffset = 55, --
           desiredY = 55,
           xSpeedFactor = 1.0,--1.75,
           xSpeedFactorFar = 1.0,
           xSpeedFactorFarDist = 280,
           ySpeedFactor = 1.0, --3.0,
           turnFactor = 0.25,
           done = false,
           doneCount = 0,
           legChosen = false,
           dribble = false,
           useHeading = false,
           heading = 0.0,
           switchable = false,
           footSwitchCount = 0,
           currentFoot = core.Kick_RIGHT,
           desiredFoot = core.Kick_RIGHT,
           xError = 500
         })
function body_MoveAndAlignToBall:set()

  -- set current foot to match the one passed in from the kick
  if (self.init) then
    self.currentFoot = self.desiredFoot
    -- first time in, we can pick an initial foot based on bearing
    if (self.switchable) then
      if (self.heading > 0) then
        self.currentFoot = core.Kick_RIGHT
      else
        self.currentFoot = core.Kick_LEFT
      end
    end
    if (self.currentFoot == core.Kick_RIGHT) then
      self.desiredY = -self.yOffset
    else
      self.desiredY = self.yOffset
    end
    self.init = false
    self.doneCount = 0
  end

  local ball = world_objects:getObjPtr(core.WO_BALL)
  local dx = ball.distance * math.cos(ball.bearing)
  local dy = ball.distance * math.sin(ball.bearing)

  local realDesiredX = self.desiredX

  if (vision_frame_info.source == core.MEMORY_SIM) then
    realDesiredX = self.desiredX - 30
  end

  -- back up a bit if we have to strafe way sideways for it
  if (self.dribble and math.abs(dy)>100) then
    realDesiredX = self.desiredX + 190
  end
  -- kicking, ball is sideways
  if (not self.dribble and math.abs(dy) > 120) then
    realDesiredX = self.desiredX + 50
  end
  -- kicking, ball is on side of foot, actually closer than front of foot
  if (not self.dribble and math.abs(dy) > 90 and dx < 110) then
    realDesiredX = self.desiredX + 100
  end

  -- possibly switch feet
  if (self.switchable) then
    -- see if we really have a prefered foot (prefer outside foot as has more range)
    local prefFoot=self.currentFoot

    local oppGoal = world_objects:getObjPtr(core.WO_OPP_GOAL)
    -- large bearing, we want that foot
    if (math.abs(self.heading) > DEG_T_RAD*10) then
      if (oppGoal.bearing > 0) then
        prefFoot = core.Kick_RIGHT
      else
        prefFoot = core.Kick_LEFT
      end
    end

    UTdebug.log(10,'FEET:',self.currentFoot,prefFoot,RAD_T_DEG * self.heading,dy)

    -- based on y offset of ball, see if we can switch feet
    local switchThreshold = 35
    -- ball in middle, still pick based on heading
    -- otherwise, pick the one near our foot
    if (prefFoot~=self.currentFoot and (math.abs(dy) < switchThreshold)) then
      self.currentFoot = prefFoot
    elseif (self.currentFoot == core.Kick_LEFT and dy < -switchThreshold) then
      self.currentFoot = core.Kick_RIGHT
    elseif (self.currentFoot == core.Kick_RIGHT and dy > switchThreshold) then
      self.currentFoot = core.Kick_LEFT
    end
  else
    -- not switchable, current foot has to be desired foot
    self.currentFoot = self.desiredFoot
  end


  if (self.currentFoot == core.Kick_RIGHT) then
    self.desiredY = -self.yOffset
  else
    self.desiredY = self.yOffset
  end

-- stop farther for left foot, and have ball farther inward (hack for dribble from walk)
  if (self.currentFoot == core.Kick_LEFT) then
    realDesiredX = realDesiredX + 15
    self.desiredY = self.yOffset - 30
  end

  local origXError = (dx - self.desiredX)
  local xError = (dx - realDesiredX)
  self.xError = xError
  local yError = (dy - self.desiredY)

  local xSpeed
  if (xError < self.xSpeedFactorFarDist) then
    xSpeed = xError / (self.xSpeedFactor * commands.walk_max_vel_x)
  else
    xSpeed = xError / (self.xSpeedFactorFar * commands.walk_max_vel_x)
  end

  local ySpeed = yError / (self.ySpeedFactor * commands.walk_max_vel_y)

  if (self.dribble and ball.seen and (math.abs(yError) < 20)) then
    xSpeed = 1.0
  end
  xSpeed = core.crop(xSpeed,-1.0,1.0)
  local maxY = 0.5
  if xSpeed > 0.8 then
    maxY = 0.8
  end
  ySpeed = core.crop(ySpeed,-maxY,maxY)

  behavior_mem.targetPt = core.Point2D(xError,yError)
  behavior_mem.useTarget = true
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  behavior_mem.absTargetPt = behavior_mem.targetPt:relativeToGlobal(me.loc, me.orientation)
  behavior_mem.useAbsTarget = true
  behavior_mem.useTargetBearing = false

  -- tighter check for penalty kick
  local extra = 0
  if (game_state.isPenaltyKick and game_state.secsRemaining > 10) then
    extra = 15
  end

  local doneThreshold = 3
  -- check if we're close to target for some # of frames
  if (ball.seen and (not self.dribble) and (origXError < 30) and ( (math.abs(yError) < (30-extra)) or (self.switchable and (math.abs(dy) < (75-extra) and math.abs(dy)>(10+extra)) ) ) ) then
    UTdebug.log(10,'MTBOF',origXError)
    if (self.doneCount < doneThreshold + 2) then
      self.doneCount = self.doneCount + 1
    end
  elseif (self.doneCount > 0) then
    self.doneCount = self.doneCount - 1
  end

  self.done = (self.doneCount >= doneThreshold)
  --UTdebug.log(0,'done:',self.doneCount,doneThreshold,self.done)

   --UTdebug.log(0,"MTBOF", xError, yError, '|', dx, dy, '|', origXError, realDesiredX, '|', ball.distance, '|', xSpeed, ySpeed, '|', self.currentFoot)

  -- we've stopped walking?
  if self.done then
    commands.setWalkVelocity(0,0,0)
    return task_NullTask:set()
  end

  -- ball.bearing is wrong should be the bearing to desiredX,desiredY
  local origin = core.Point2D(0, self.desiredY)
  local dest = core.Point2D(dx, dy)
  local desiredBallBearing = origin:getAngleTo(dest)
  local desiredBearing = desiredBallBearing

  if (self.useHeading) then
    desiredBearing = self.heading
  end

  -- very little turn, since we started approach because we liked our heading
  local turnSpeed = (desiredBearing) * 0.03
  turnSpeed = core.crop(turnSpeed,-0.1,0.1)

  -- slow down when we're close to ball and not looking at it
  if (not ball.seen and (ball.distance < 1000)) then
    xSpeed = xSpeed * (ball.distance / 1000.0)
    ySpeed = ySpeed * (ball.distance / 1000.0)
    turnSpeed = turnSpeed * (ball.distance / 1000.0)
  end

  commands.setWalkVelocity(xSpeed,ySpeed,turnSpeed,false)
  return head_LookForBall:set()
end

function body_MoveAndAlignToBall:finished()
  return self.done
end

function adjustKickAlignment(inAlignment,id,kickChoice)
  local alignment = deepcopy(inAlignment)
  -- nothing robot specific yet
  return alignment
  --local walkKickTypes = {core.WalkKickFront,core.WalkKickLeftwardSide,core.WalkKickRightwardSide}
end

CompTask('body_AlignToBallWithTargetWalk',{
           init = true,
           desiredX = 155,--165, -- overwritten
           alignment = kicks.kickData[core.FwdShortStraightKick].alignment,
           --yOffset = 50,
           desiredY = 50, -- overwritten
           done = false,
           doneCount = 0,
           legChosen = false,
           dribble = false,
           switchable = false,
           footSwitchCount = 0,
           heading = 0,
           currentFoot = core.Kick_RIGHT,
           desiredFoot = core.Kick_RIGHT,
           needToSend = true,
           xError = 500,
           lastSendBallLoc = core.Point2D(0,0),
           kickChoice = core.FwdShortStraightKick,
           walkDecidesTargetFinished = false
         })
function body_AlignToBallWithTargetWalk:choose(as)
  self.alignment = adjustKickAlignment(self.alignment,robot_state.robot_id_,self.kickChoice)
  self.desiredX = self.alignment.des_x

  if (self.kickChoice == core.FwdLongStraightKick) or (self.kickChoice == core.FwdMediumStraightKick) or (self.kickChoice == core.FwdShortStraightKick) then
    self.walkDecidesTargetFinished = true
  else
    self.walkDecidesTargetFinished = false
  end

  -- set current foot to match the one passed in from the kick
  if (self.init) then
    UTdebug.log(10, "target approach init")
    self.currentFoot = self.desiredFoot
    -- first time in, we can pick an initial foot based on bearing
    if (self.switchable) then
      if (self.heading > 0) then
        self.currentFoot = core.Kick_RIGHT
      elseif (self.heading < 0) then
        self.currentFoot = core.Kick_LEFT
      else
        local oppGoal = world_objects:getObjPtr(core.WO_OPP_GOAL)
        -- large bearing, we want that foot
        if (oppGoal.bearing > 0) then
          self.currentFoot = core.Kick_RIGHT
        else
          self.currentFoot = core.Kick_LEFT
        end
      end
    end
    if (self.currentFoot == core.Kick_RIGHT) then
      self.desiredY = -self.alignment.des_y
    else
      self.desiredY = self.alignment.des_y
    end
    self.init = false
    self.needToSend = true
    self.doneCount = 0
  end

  local ball = world_objects:getObjPtr(core.WO_BALL)
  --local dx = ball.relPos.x
  --local dy = ball.relPos.y
  local dx = localizationC.filtered_close_ball_.x
  local dy = localizationC.filtered_close_ball_.y
  --local dx = ball.distance * math.cos(ball.bearing)
  --local dy = ball.distance * math.sin(ball.bearing)
  --local dx = ball.visionDistance * math.cos(ball.visionBearing)
  --local dy = ball.visionDistance * math.sin(ball.visionBearing)
  --UTdebug.log(10, "Ball debug","dist",ball.distance, "bearing",ball.bearing,"dx,dy",dx,dy)

  local realDesiredX = self.desiredX

  if (vision_frame_info.source == core.MEMORY_SIM) then
    realDesiredX = self.desiredX - 30
  end


  -- back up a bit if we have to strafe way sideways for it
  if (self.dribble and math.abs(dy)>130) then
    realDesiredX = self.desiredX + 100
  end
  -- kicking, ball is sideways
  if (not self.dribble and math.abs(dy) > 120) then
    realDesiredX = self.desiredX + 25
  end
  --[[
  -- kicking, ball is on side of foot, actually closer than front of foot
  if (not self.dribble and math.abs(dy) > 90 and dx < 110) then
    realDesiredX = self.desiredX + 100
  end
  --]]

  local origXError = (dx - self.desiredX)
  local xError = (dx - realDesiredX)

  -- from farther way... make sure to give us more room
  --if (xError > 200 and not self.dribble) then
    --realDesiredX = realDesiredX + 25
    --xError = (dx - realDesiredX)
  --end

  -- possibly switch feet
  -- no switch after we would stop sending walk to target commands
  if (self.switchable and (xError > 100 or self.needToSend)) then
    -- see if we really have a prefered foot (prefer outside foot as has more range)
    local prefFoot=self.currentFoot

    local oppGoal = world_objects:getObjPtr(core.WO_OPP_GOAL)
    -- large bearing, we want that foot
    if (math.abs(oppGoal.bearing) > DEG_T_RAD*10) then
      if (oppGoal.bearing > 0) then
        prefFoot = core.Kick_RIGHT
      else
        prefFoot = core.Kick_LEFT
      end
    end

    UTdebug.log(10,'FEET:',self.currentFoot,prefFoot,RAD_T_DEG * self.heading,RAD_T_DEG*oppGoal.bearing,dy)

    -- based on y offset of ball, see if we can switch feet
    local switchThreshold = 30
    -- ball in middle, still pick based on heading
    -- otherwise, pick the one near our foot
    if (prefFoot~=self.currentFoot and (math.abs(dy) < 15)) then
      self.currentFoot = prefFoot
      self.doneCount = 0
      self.needToSend = true
    elseif (self.currentFoot == core.Kick_LEFT and dy < -switchThreshold) then
      self.currentFoot = core.Kick_RIGHT
      self.doneCount = 0
      self.needToSend = true
    elseif (self.currentFoot == core.Kick_RIGHT and dy > switchThreshold) then
      self.currentFoot = core.Kick_LEFT
      self.doneCount = 0
      self.needToSend = true
    end
  elseif (not self.switchable) then
    -- not switchable, current foot has to be desired foot
    -- if we're switching... reset count
    if (self.currentFoot ~= self.desiredFoot) then
      self.doneCount = 0
      self.needToSend = true
    end
    self.currentFoot = self.desiredFoot
  end

  if (self.currentFoot == core.Kick_RIGHT) then
    self.desiredY = -self.alignment.des_y
  else
    self.desiredY = self.alignment.des_y
  end

  local yError = (dy - self.desiredY)
  
  local yAlignError = yError
  if (self.currentFoot == core.Kick_RIGHT) then
    yAlignError = -yError
  end

  if self.dribble and (yAlignError > self.alignment.min_stop_y_err) and (yAlignError < self.alignment.max_stop_y_err) then
    -- if we're dribbling and the ball is in a good spot, let's walk into the ball quickly
    xError = xError + 200
  end

  -- calculate body target from ball
  --UTdebug.log(0, "Ball",dx,dy, " desired",realDesiredX, self.desiredY, "error",xError,yError,yAlignError)

  behavior_mem.targetPt = core.Point2D(xError,yError)
  behavior_mem.useTarget = true
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  behavior_mem.absTargetPt = behavior_mem.targetPt:relativeToGlobal(me.loc, me.orientation)
  behavior_mem.useAbsTarget = true
  behavior_mem.useTargetBearing = false

  local doneThreshold = 3

  local extra = 0
  if (game_state.isPenaltyKick and game_state.secsRemaining > 10) then
    extra = 10
  end


  -- check if we're close to target for some # of frames
  --if (ball.seen and (not self.dribble) and (origXError < 20) and ( (math.abs(yError) < (20-extra)) or (self.switchable and (math.abs(dy) < (75-extra) and math.abs(dy)>(15+extra)) ) ) ) then
  --if (ball.seen and (not self.dribble) and origXError < 28 and math.abs(yError) < (28-extra)) then
  -- normal
  local sideBuffer = 0 --12
  local fwdBuffer = 0 --7
  local incrementDoneCounter = ball.seen and (not self.dribble) and origXError < (self.alignment.max_stop_x_err + fwdBuffer) and yAlignError > (self.alignment.min_stop_y_err - sideBuffer) and yAlignError < (self.alignment.max_stop_y_err + sideBuffer)
  -- hack for walk kick front, less slow down near ball
  -- do done count normally, but then change the xError on it
  if (self.kickChoice == core.WalkKickFront) and (yAlignError > self.alignment.min_stop_y_err) and (yAlignError < self.alignment.max_stop_y_err) then
  --if (yAlignError > self.alignment.min_stop_y_err) and (yAlignError < self.alignment.max_stop_y_err) then
    -- walk kick front, and ball reasonable side to side, let's not slow down as much
    local oldXError = xError -- just for debug statement below
    local maxErr = self.alignment.max_stop_x_err
    
    local stepTime = 0.25 -- in seconds, on average, this is about correct
    -- we want the time until starting the foot opposite of the kick foot (since there's the pre step)
    local timeUntilStart = walk_info.time_remaining_in_step_
    local wantLeftStanceForKick = (self.currentFoot ~= core.Kick_RIGHT)
    if walk_info.is_stance_left ~= wantLeftStanceForKick then
      -- we'll have to wait for a step with the swing leg
      timeUntilStart = timeUntilStart + stepTime
    end
    timeUntilStart = timeUntilStart + 0.05 -- give us some room for error

    local futureXError = origXError - timeUntilStart * walk_info.robot_velocity_.translation.x
    --if (origXError < maxErr) then
    if (futureXError < maxErr) then
      UTdebug.log(0,'Walk kick front, incrementing done',dx,origXError,futureXError)
      incrementDoneCounter = true
    else
      xError = xError + 50
      incrementDoneCounter = false
    end

    --UTdebug.log(0,'  Walk kick front, moving xError from',oldXError,'to',xError)
    --UTdebug.log(0,'    futureXError,vel,timeUntilStart,timeInStep:',futureXError,walk_info.robot_velocity_.translation.x,timeUntilStart,walk_info.time_remaining_in_step_)
  end


  if incrementDoneCounter then
    if (self.doneCount < doneThreshold + 2) then
      self.doneCount = self.doneCount + 1
    end
  elseif (self.doneCount > 0) then
    
    self.doneCount = self.doneCount - 1
  --elseif (not walk_info.walk_is_active_) then
    ---- if this happens when we're not walking... send new commands
    --self.needToSend = true
    --UTdebug.log(10, "restart approach, stopped in bad spot")
  end

  self.done = (self.doneCount >= doneThreshold)

  if self.walkDecidesTargetFinished then
    self.done = walk_info.finished_with_target_
  end
  --UTdebug.log(0, "done check", ball.seen, self.dribble, origXError, yAlignError, self.alignment.max_stop_x_err, self.alignment.min_stop_y_err, self.alignment.max_stop_y_err, self.doneCount, doneThreshold)

  -- also done if we're standing now
  UTdebug.log(30, "standing: ", odometry.standing, self.doneCount, xError, yError)
  --[[
  if (odometry.standing and self.doneCount > 0) then
    self.doneCount = doneThreshold + 1
    self.done = true
  end
  --]]

   UTdebug.log(10,"MTBOF", xError, yError, origXError, realDesiredX, ball.distance, self.currentFoot, self.needToSend, self.doneCount, self.done)
  
  -- so approach can know about our xError
  self.xError = xError

  -- we've stopped walking?
  --if self.done then
    ----commands.stand()
    --commands.setWalkVelocity(0,0,0)
    --return head_LookForBall:set()
  --end

  -- dont change target when not seeing ball
  if (not ball.seen) then
    --walk_request:wait()
    self.done = false
    return head_LookForBall:set()
  end
  
  --local rel_target_x = xError - walk_info.robot_relative_next_position_.translation.x * al_walk_param.fwd_odometry_factor_
  --local rel_target_y = yError - walk_info.robot_relative_next_position_.translation.y * al_walk_param.side_odometry_factor_
  --UTdebug.log(10,'rel_target',rel_target_x,rel_target_y)
  --if (math.abs(rel_target_x) < 2.0 * 20) and (math.abs(rel_target_y) < 2.0 * (20 -extra)) then
    ---- don't send a command
    --UTdebug.log(10,'NOT SENDING A COMMAND')
  --else
    --commands.setWalkTarget(xError,yError)
  --end

  -- check error 
  --UTdebug.log(10, "ball error", xError, yError, ball.distance, origXError, self.doneCount, self.done)

--  UTdebug.log(10, "last ball loc", self.lastSendBallLoc, ball.loc, ball.loc:getDistanceTo(self.lastSendBallLoc))
  local ballMovement = core.Point2D(dx,dy):getDistanceTo(self.lastSendBallLoc)

  -- Todd: I think 200 means there's about 4 steps left
  -- we could go to 170.. which most of the time is just before 2 steps left
  -- but seems safer to stay at 200
  -- if ball has moved more than 6 cm... send command
  if (self.needToSend or xError > 85 or ballMovement > 60) then
    --commands.setWalkTarget(xError,yError)
    UTdebug.log(10, "need to send, xError, ballMove", self.needToSend, xError, ballMovement)
    self.doneCount = 0
    self.done = false
  else
    --self.done = not(walk_request.target_walk_is_active_)
    --walk_request:wait()
  end
  
  --UTdebug.log(0,luaC:getString(core.kickNames,self.kickChoice),dx,self.desiredX,realDesiredX,xError,yError,self.dribble)

  local pedantic = false
  if (math.abs(xError) < 40) and (math.abs(yError) < 20) then
    -- turn on pedantic at the end
    pedantic = true
  end

  if not(self.done) then
    walk_request:setWalkTarget(xError,yError,0,pedantic)

    if self.walkDecidesTargetFinished then
      walk_request.walk_decides_finished_with_target_ = true
      walk_request.finished_with_target_max_x_error_ = self.alignment.max_stop_x_err
      if (self.currentFoot == core.Kick_LEFT) then
        -- normal
        walk_request.finished_with_target_max_y_error_ = self.alignment.max_stop_y_err
        walk_request.finished_with_target_min_y_error_ = self.alignment.min_stop_y_err
      else
        -- switched
        walk_request.finished_with_target_max_y_error_ = -self.alignment.min_stop_y_err
        walk_request.finished_with_target_min_y_error_ = -self.alignment.max_stop_y_err
      end
    end
  end

  --UTdebug.log(0,xError,yError)
  self.needToSend = false
  self.lastSendBallLoc = core.Point2D(dx,dy)

  return head_LookForBall:set()
  -- return head_BallActiveLocalize:set()

end

function body_AlignToBallWithTargetWalk:finished()
  return self.done
end



function avoidPenaltyBox(targetPt, targetBearing)
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  local ball = world_objects:getObjPtr(core.WO_BALL)

  -- Check if we anywhere near the box
  if (me.loc.x > -core.PENALTY_CROSS_X) then
    --UTdebug.log(10,"avoidPenaltyPoint - i'm too far up field")
    return targetPt, targetBearing
  end


  -- Make a copy of the original points so we can change them
  local origTarget = core.Point2D(targetPt.x,targetPt.y)
  local origBearing = targetBearing
  --UTdebug.log(10,"avoidPenaltyPoint - original target:",targetPt.x,targetPt.y, targetBearing)

  -- convert target to an absoulte position
  local absTarget = origTarget:rotate(me.orientation) + me.loc

  --UTdebug.log(10,"me loc:",me.loc.x,me.loc.y)
  --UTdebug.log(10,"abs target:",absTarget.x,absTarget.y)
  --UTdebug.log(10,"ball loc:",ball.loc.x,ball.loc.y)

  local finalTarget = core.Point2D(absTarget.x,absTarget.y)
  local targetChanged = false
  -- Doing this as different scenarios rather then a generic solution that has many corner cases

  local penaltyBoxLineX =  -core.FIELD_X/2.0 + core.PENALTY_X
  local halfPenaltyY = core.PENALTY_Y / 2.0
  local bufferPenaltyY = halfPenaltyY - 400
  -- Scenario 1, target in back corners.
  if ( (ball.loc.x < penaltyBoxLineX or absTarget.x < penaltyBoxLineX) and (math.abs(absTarget.y) > bufferPenaltyY or math.abs(ball.loc.y)  > bufferPenaltyY) ) then
    -- only do something if i'm in the region spanning the width of the box (i.e. not the edge of the field)
    if (math.abs(me.loc.y) < halfPenaltyY+200) then
      UTdebug.log(10,"avoidPenaltyPoint - now in scenario 1 :",ball.loc.y)
      -- we neet to walk to one of the corners of the box instead of to the point
      targetChanged = true
      if (ball.loc.y > 0) then
        finalTarget.y = halfPenaltyY + 300.0
      else
        finalTarget.y = -halfPenaltyY - 300.0
      end

      -- Todd: we can't solely use our location, or we keep going back to same corner pt
      -- we've cleared corner
      if ((me.loc.y > halfPenaltyY+100 and ball.loc.y > halfPenaltyY+100) or
        (me.loc.y < -halfPenaltyY-100 and ball.loc.y < -halfPenaltyY-100)) then
        finalTarget.x = absTarget.x
      else
        finalTarget.x = penaltyBoxLineX + 400.0
      end

    end
  end

  -- Scenario 2, we are in back corners.
  if (not targetChanged and me.loc.x < penaltyBoxLineX+200 and math.abs(me.loc.y) > bufferPenaltyY) then
    -- only do something if the ball is in the region spanning the width of the box (i.e. not the edge of the field)
    if ((me.loc.y > halfPenaltyY and absTarget.y < halfPenaltyY) or
      (me.loc.y < -halfPenaltyY and absTarget.y > -halfPenaltyY)) then
      --if (math.abs(absTarget.y) < halfPenaltyY and absTarget.x < (penaltyBoxLineX + 500)) then
      UTdebug.log(10,"avoidPenaltyPoint - now in scenario 2")
      -- we need to walk to one of the corners of the box instead of to the point
      targetChanged = true
      finalTarget.x = penaltyBoxLineX + 300.0
      -- Todd: we can't solely use our location, or we keep going back to same corner pt
      -- we've cleared corner
      if (me.loc.x > penaltyBoxLineX+100) then
        --UTdebug.log(10,"scenario 2a")
        finalTarget.y = absTarget.y
      else
        --UTdebug.log(10,"scenario 2b")
        if (me.loc.y > 0) then
          finalTarget.y = halfPenaltyY + 300.0
        else
          finalTarget.y = -halfPenaltyY - 300.0
        end
      end
    end
  end

  -- Scenario 3, ball in box
  if (not targetChanged and ball.loc.x < penaltyBoxLineX and math.abs(ball.loc.y) < (core.PENALTY_Y/2.0)) then
    UTdebug.log(10,"avoidPenaltyBox, scenario 3")
    targetChanged = true
    finalTarget.x = penaltyBoxLineX + 300.0
    if (ball.loc.y > 0) then
      finalTarget.y = halfPenaltyY + 300.0
    else
      finalTarget.y = -halfPenaltyY - 300.0
    end
  end

  -- convert final position to a relative position and return new target and heading
  if (targetChanged) then
    local cropAmt = 200
    -- no cropping, so that we'll go into sprint across front of box
    targetPt = (finalTarget - me.loc):rotate(-me.orientation)
    targetBearing = ball.bearing --core.normalizeAngle(targetPt:getDirection())
    --UTdebug.log(10,"new target pt", targetPt)
    --targetPt.x = core.crop(targetPt.x, -180, 180)
  end
  --UTdebug.log(10,"new target:",targetPt.x,targetPt.y,targetBearing)
  return targetPt,targetBearing
end


-------------------------------
-- keeper blocks and dives

CompTask('body_Block2',{
           init = true,
           state = 0,
           dir = "Center",
           yIntercept = 0,
           moveTime = 0.4,
           returnLength = 0.8,
           returnTime = 4.0,
           totalTime = 5.0,
           poseChangeFactor = 0.6,
           stateTimes = {0.4,0.4,0.6, 0.4,0.4,0.7,0.6,0.8},
           yIntercept = 100,
           reverseSides = false -- set in init, false for left, true for right
         })
function body_Block2:choose(as)

  if (self.reverseSides) then
    behavior_mem.keeperDiving = core.Dive_RIGHT
  else
    behavior_mem.keeperDiving = core.Dive_LEFT
  end

  -- move to pose very quickly
  if (self.init) then
    self.init = false
    self.state = 1
    self.reverseSides = (self.yIntercept < 0) --false for left, true for right
  end

  UTdebug.log(10,self.yIntercept,self.reverseSides)

  if (self.state == 1) then
    self.state = 2
    commands.setStiffnessCommands(cfgStiffLowArms, self.stateTimes[self.state-1])

    self:resetTime()
    return body_ToPose:set{pose = GoalieLeftSplayInter1, time = self.stateTimes[self.state-1], init = true, reverseSides=self.reverseSides}
  end

  if (self.state == 2) and (self:getTime() > self.stateTimes[self.state-1] * self.poseChangeFactor) then
    -- go back to standing
    self.state = 3
    self:resetTime()
    return body_ToPose:set{pose = GoalieLeftSplayInter2, time = self.stateTimes[self.state-1], init = true, reverseSides=self.reverseSides}

  end

  if (self.state == 3 and self:getTime() > self.stateTimes[self.state-1] * self.poseChangeFactor) then--self.returnTime + self.returnLength) then
    self.state = 4
    self:resetTime()
    return body_ToPose:set{pose = GoalieLeftSplayFinalV5, time = self.stateTimes[self.state-1], init = true, reverseSides=self.reverseSides}
    --    commands.interpAllJointStiffness(0.0, self.moveTime)
    --    return body_PrintPose:set()
  end

  if (self.state == 4) and (self:getTime() > self.returnTime - .8) then
    -- commands.interpAllJointStiffness(0.0, self.moveTime)
    --  return body_PrintPose:set()
    self.state = 5
    self:resetTime()
    return body_ToPose:set{init = true, pose = GoalieLeftSplayGetup1, time=self.stateTimes[self.state-1], reverseSides=self.reverseSides}
    --   return body_ToPose:set{pose = GoalieLeftSplayInter2, time = self.moveTime, init = true}
  end

  if (self.state == 5) and (self:getTime() > self.stateTimes[self.state-1]) then
    self.state = 6
    self:resetTime()
    return body_ToPose:set{init = true, pose = GoalieLeftSplayGetup2, time=self.stateTimes[self.state-1], reverseSides=self.reverseSides}
  end

  if (self.state == 6) and (self:getTime() > self.stateTimes[self.state-1]) then
    self.state = 7
    self:resetTime()
    return body_ToPose:set{init = true, pose = GoalieLeftSplayGetup3V6, time=self.stateTimes[self.state-1], reverseSides=self.reverseSides}
  end

  if (self.state == 7) and (self:getTime() > self.stateTimes[self.state-1]) then
    self.state = 8
    self:resetTime()
    return body_ToPose:set{init = true, pose = GoalieLeftSplayGetup4V3, time=self.stateTimes[self.state-1], reverseSides=self.reverseSides}
  end
  if (self.state == 8) and (self:getTime() > self.stateTimes[self.state-1]) then
    self.state = 9
    self:resetTime()
    commands.setStiffnessCommands(cfgStiffLowArms, self.stateTimes[self.state-1])

    return body_ToPose:set{init = true, pose = GoalieLeftSplayGetup5, time=self.stateTimes[self.state-1], reverseSides=self.reverseSides}
  end

end

function body_Block2:finished()
  --return (self:getTime() > self.totalTime)
  return (self.state == 9) and (self:getTime() > self.stateTimes[self.state-1])
end




CompTask('body_KeeperDive', {
           time = 0.3,
           dir = "RIGHT",
           init = true,
           fakeDive = false,
           done = false,
           state = 0
         })
function body_KeeperDive:choose(as)

  local poseIdOne = diveLeftTwo
  local poseIdTwo = diveLeftTwoB
  if (self.dir == "RIGHT") then
    poseIdOne = diveRightTwo
    poseIdTwo = diveRightTwoB
  elseif (self.dir == "CENTER") then
    poseIdOne = crabPose
    poseIdTwo = crabPose
  end

  if (self.dir == "LEFT") then
    behavior_mem.keeperDiving = core.Dive_LEFT
  elseif (self.dir == "RIGHT") then
    behavior_mem.keeperDiving = core.Dive_RIGHT
  else
    behavior_mem.keeperDiving = core.Dive_CENTER
  end

  -- done after being down for more than 3 sec
  if (self:getTime() > self.time +3.0 and not self.done) then
    self.done = true
    -- trigger get up routine (if not penalty kick)
    if (odometry.getting_up_side_ == core.Getup_NONE and
        not game_state.isPenaltyKick) then
      walk_request:setFalling()
      return task_NullTask:set()
    end
  end

  -- go to pose while lowering stiffness
  if (self.init) then
    self.init = false
    self.state = 1
    return body_ToPose:set{pose = poseIdOne, time = self.time}
  end

  -- after falling, extend lower leg again
  if (self.dir ~= "CENTER" and self.state == 1) then

    -- then rotate body over hip to fall over (and lower stiffness)
    if (self:getTime() > self.time) then
      self.state = 2
      commands.setStiffnessCommands(cfgStiffZeroPtOne, self.time)
      return body_ToPose:set{pose = poseIdTwo, time = self.time,
                             init = true}
    end

  end

end

function body_KeeperDive:finished()
  return self.done
end



CompTask('body_FakeDive',{
           init = true,
           reverseSides = false,
         })
function body_FakeDive:choose(as)

  if (self.init) then
    return body_ToPose:set{pose=standingLeftArmPose, init=true, time=0.3, reverseSides = self.reverseSides}
  end
  return task_NullTask:set()

end


-- 'Safe/soft' dive modeled after B-Human
CompTask('body_SafeDive',{
           init = true,
           state = 1,
           stateTimes = {0.1,0.05, 0.05, 2.2, 0.5, 0.3, 0.3},
           reverseSides = false, -- false for left, true for right (from goalie's perspective)
           done = false,
           performClear = false
         })
function body_SafeDive:choose(as)

  if self.state < 7 then
    walk_request:noWalk()
  end
  if (self.reverseSides) then
    behavior_mem.keeperDiving = core.Dive_RIGHT
  else
    behavior_mem.keeperDiving = core.Dive_LEFT
  end

  -- move to pose very quickly
  if (self.init) then
    self.init = false
    self.state = 2
    self:resetTime()
    local time = self.stateTimes[self.state-1]
    -- turn on the stiffness, or the arm won't make it where we need to
    commands.setStiffnessCommands(cfgStiffOne, 0.01)
    return body_ToPose:set{pose = KeeperDiveGoingDown1, time = time, init = true, reverseSides=self.reverseSides}
  end

  if (self.state == 2) and (self:getTime() > self.stateTimes[self.state-1]) then
    self.state = 3
    self:resetTime()
    commands.setStiffnessCommands(cfgStiffZeroPtThree, self.stateTimes[self.state-1])
    --[[ Todd: dont break arms
    --if (self.reverseSides) then
      joint_commands:setJointStiffness(core.RShoulderPitch,1.0)
      joint_commands:setJointStiffness(core.RShoulderRoll,1.0)
    --else
      joint_commands:setJointStiffness(core.LShoulderPitch,1.0)
      joint_commands:setJointStiffness(core.LShoulderRoll,1.0)
    --end
      --]]

    return body_ToPose:set{pose = KeeperDiveGoingDown2, time = self.stateTimes[self.state-1], init = true, reverseSides=self.reverseSides}
  end

  if (self.state == 3) and (self:getTime() > self.stateTimes[self.state-1]+0.3) then
    self.state = 4
    self:resetTime()
    commands.setStiffnessCommands(cfgStiffZero, self.stateTimes[self.state-1])

    return body_ToPose:set{pose = KeeperDiveGoingDown3, time = self.stateTimes[self.state-1], init = true, reverseSides=self.reverseSides}
  end

  -- state 4 is a long one... wait for fall, then we'll turn stiffness back on
  -- and prepare for side clearing motion
  if (self.state == 4) and (self:getTime() > self.stateTimes[self.state-1]+0.3) then
    -- in penalty kick, we dont do these clearing steps... skip to state 6
    if (game_state.isPenaltyKick) then
      self.state = 6
    else
      if self.performClear then
        self.state = 5
      else
        self.state = 6
      end
      self:resetTime()
      commands.setStiffnessCommands(cfgStiffOne, self.stateTimes[self.state-1])
      return body_ToPose:set{pose = KeeperDiveGoingDown3, time = self.stateTimes[self.state-1], init = true, reverseSides=self.reverseSides}
    end
  end

  -- after state 5, we're ready to try clearing
  if (self.state == 5) and (self:getTime() > self.stateTimes[self.state-1]+0.3) then
    self.state = 6
    self:resetTime()
    return body_ToPose:set{pose = KeeperDiveSideClear, time = self.stateTimes[self.state-1], init = true, reverseSides=self.reverseSides}
  end

  --now we can get up
  if (self.state == 6 and (self:getTime() > self.stateTimes[self.state-1]+0.3) and not self.done) then
    self.state = 7
    self:resetTime()
    self.done = true
    -- trigger get up routine (if not penalty kick)
    if (odometry.getting_up_side_ == core.Getup_NONE and not game_state.isPenaltyKick) then
      walk_request:setFalling()
      walk_request.getup_from_keeper_dive_ = true
      return task_NullTask:set()
    end
  end
  return task_NullTask:set()
end

function body_SafeDive:finished()
  return self.done
end




-------------------------------
-- set body pose and stiffness


PrimTask('body_ToPose',{
           pose = sittingPoseV3,
           init = true,
           time = 3.0,
           reverseSides = false
         })
function body_ToPose:run()
  -- only have to send command first time
  if (self.init) then
    self:resetTime()
    self.init = false

    local val

    -- for each body joint (not head)
    for i=2, core.NUM_JOINTS-1 do
      -- do interp to given angle
      val = DEG_T_RAD*getPoseJoint(i,self.pose,self.reverseSides,true)
      joint_commands:setJointCommand(i, val)
    end

    -- set time and send_angles_
    joint_commands.send_body_angles_ = true
    joint_commands.body_angle_time_ = self.time * 1000.0

    -- turn off walk request and kick request
    walk_request:noWalk()
    kick_request:setNoKick()

  end
end

function body_ToPose:finished()
  return (self:getTime() > self.time+0.1) --0.05)
end

-- NOTE: if a pose has 'leftLeg' in it, we'll treat it as a pose with IK legs
PrimTask('body_ToPoseSequence',{
    init = true,
    poses = {sittingPoseV3},
    times = {1.0},
    stiffness = cfgStiffOne,
    state = 0,
    reverseSides = false
  })
function body_ToPoseSequence:run()
  if self.init then
    self.init = false
    self.state = 0
    commands.setStiffnessCommands(self.stiffness,0.1)
  end
  walk_request:noWalk()
  kick_request:setNoKick()

  if self:finished() then
    return
  end

  if (self.state == 0) or self:getTime() > self.times[self.state] then
    self:resetTime()
    self.state = self.state + 1
    if self:finished() then
      return 
    end
  end

  local poseStart
  if self.poses[self.state].leftLeg == nil then
    poseStart = core.BODY_JOINT_OFFSET
  else
    local left  = self.poses[self.state].leftLeg
    local right = self.poses[self.state].rightLeg
    if self.reverseSides then
      local temp = left
      left = right
      right = temp
      right.translation.y = -right.translation.y
      left.translation.y = -left.translation.y
      right.rotation:rotateX(-2 * right.rotation:getXAngle())
      left.rotation:rotateX(-2 * left.rotation:getXAngle())
    end
    core.InverseKinematics_calcLegJoints(left,right,joint_commands.angles_,robot_info.dimensions_)
    poseStart = core.LShoulderPitch
  end
  local val
  -- for each body joint (not head)
  for i=poseStart, core.NUM_JOINTS-1 do
    -- do interp to given angle
    val = DEG_T_RAD*getPoseJoint(i,self.poses[self.state],self.reverseSides,false)
    joint_commands:setJointCommand(i, val)
  end

  local time = (self.times[self.state] - self:getTime()) * 1000
  if self.poses[self.state] == self.poses[self.state-1] then
    time = 0
  end
  joint_commands.send_body_angles_ = true
  joint_commands.body_angle_time_ = time

  return as
end

function body_ToPoseSequence:finished()
  return self.state > table.getn(self.poses)
end

CompTask('body_GoalieSquat',{
    init = true,
    stayTime = 2.75,
    done = false
  })
function body_GoalieSquat:choose(as)
  if self.init then
    self.init = false
    self.done = false
  elseif not self.done then
    self.done = as:finished()
  end
  behavior_mem.keeperDiving = core.Dive_CENTER

  local poses = {goalieSquatPart1,goalieSquatPart2,goalieSquatPart2,goalieSquat5,goalieSquat5,goalieSquatPart2,goalieSquatGetup15,goalieSquatGetup2,goalieSquatGetup7}--,goalieSquatGetup5}
  local times = {0.4,0.2,self.stayTime,0.2,0.3,0.3,0.4,0.6,0.1}--,0.3}
  --local poses = {goalieSquatPart1,goalieSquat5,goalieSquat5,goalieSquatGetup1,goalieSquatGetup2,goalieSquatGetup7}--,goalieSquatGetup5}
  --local times = {2.4,2.2,self.stayTime,0.4,0.6,0.1}--,0.3}
  commands.setHeadTilt()
  commands.setHeadPan(0,0.1,false)
  --if as ~= nil and as:finished() then
  if self.done then
    commands.stand()
    return task_NullTask:set()
  else
    return body_ToPoseSequence:set{poses=poses,times=times}
  end
end

function body_GoalieSquat:finished()
  return self.done
end

CompTask('body_LongKick',{
    init = true,
    rightLeg = false,
    done = false,
    setOdometry = false,
    stepInto = false
  })
function body_LongKick:choose(as)
  if self.init then
    self.init = false
    self.done = false
    self.setOdometry = true
    kick_request:initAccelX()
    kick_request:initAccelY()
  elseif not self.done then
    self.done = as:finished()
  end
  kick_request:addAccelX(sensors:getValue(core.accelX))
  kick_request:addAccelY(sensors:getValue(core.accelY))

  -- if we tap the ball and it becomes too far away early in the kick, abort
  if self.stepInto==false and self:getTime() < 1.0 and ((kick_request.ball_rel_x_ > 35.0) or (math.abs(kick_request.ball_rel_y_) > 90)) then
    UTdebug.log(0,"ABORTING SUPER KICK - time: ",self:getTime(),", x: ",kick_request.ball_rel_x_,", y: ",kick_request.ball_rel_y_)
    self.done = true
  end

  local kickState = 6 -- index of longKickKick, remember that lua starts indexing at 1
  local poses = {standingPose,longKickShift,longKickShift2,longKickLift,longKickCock,longKickKick,longKickShiftback,standingPose}
  local times = {1.0,0.6,0.8,0.8,0.8,0.1,0.7,0.5,2.0}
  if self.stepInto then
    kickState = 7
    poses = {longKickStepShort,longKickStepShort,longKickStep2,longKickShift2Step,longKickLiftStep,longKickCock,longKickKick,longKickShiftback,standingPose,standingPose,standingPose}
    times = {0.5,1.5,0.6,1.5,0.6,0.8,0.1,0.7,0.5,2.0,0.1}
    if (self:getTime()>0.1 and as.state==2 and math.abs(kick_request:getAvgAccelX()) < 0.3 and math.abs(kick_request:getAvgAccelY()) < 0.3) then
      as.state=3
      as:resetTime()
      --UTdebug.log(0,'pre-kick time: ',self:getTime(),", avg x: ",kick_request:getAvgAccelX(),", avg y: ",kick_request:getAvgAccelY())
    end
    if (self:getTime()>0.1 and as.state==10 and math.abs(kick_request:getAvgAccelX()) < 0.4 and math.abs(kick_request:getAvgAccelY()) < 0.4) then
      as.state=11
      as:resetTime()
      --UTdebug.log(0,'post-kick time: ',self:getTime(),", avg x: ",kick_request:getAvgAccelX(),", avg y: ",kick_request:getAvgAccelY())
    end
  end
  if (as == body_ToPoseSequence) and (as.state > kickState) and not(self.setOdometry) then
    self.setOdometry = true
    odometry.didKick = true
    odometry.kickHeading = kickData[core.FwdSuperStraightKick].heading
    odometry.kickDistance = kickData[core.FwdSuperStraightKick].distance
  end

  commands.setHeadTilt()
  commands.setHeadPan(0,0.1,false)

  if self.done then
    kick_request.vision_kick_running_ = false
    commands.stand()
    return task_NullTask:set()
  else
    kick_request.vision_kick_running_ = true
    return body_ToPoseSequence:set{poses=poses,times=times,reverseSides=self.rightLeg}
  end
end

function body_LongKick:finished()
  return self.done
end


-- A helper func for reversing the sides of a pose
function getPoseJoint(joint,pose,reversed,reverseRolls)
  if not(reversed) then
    return pose[joint]
  end

  -- because there's only one groin joint, just handle it separately, and don't reverse it
  if (joint == core.LHipYawPitch) or (joint == core.RHipYawPitch) then
    return pose[joint]
  end

  local val
  -- get the joint value for the joint on the opposite side
  if joint >= core.LShoulderPitch and joint <= core.LElbowRoll then
    --left arm
    val = pose[joint+(core.RShoulderPitch-core.LShoulderPitch)]
  elseif joint >= core.LHipYawPitch and joint <= core.LAnkleRoll then
    --left leg
    val = pose[joint+(core.RHipYawPitch-core.LHipYawPitch)]
  elseif joint >= core.RHipYawPitch and joint <= core.RAnkleRoll then
    --right leg
    val = pose[joint-(core.RHipYawPitch-core.LHipYawPitch)]
  elseif joint >= core.RShoulderPitch and joint <= core.RElbowRoll then
    --right arm
    val = pose[joint-(core.RShoulderPitch-core.LShoulderPitch)]
  end

  if reverseRolls then
    -- reverse the roll directions
    local directionReversedJoints = {core.LHipRoll,core.RHipRoll,core.LAnkleRoll,core.RAnkleRoll,core.LShoulderRoll,core.RShoulderRoll,core.LElbowRoll,core.RElbowRoll,core.LElbowYaw,core.RElbowYaw}
    for i = 1,#directionReversedJoints do
      if joint == directionReversedJoints[i] then
        return -val
      end
    end
  end
  return val
end

PrimTask('body_PrintPose',{
    init = true
  })
function body_PrintPose:run()
  UTdebug.log(0,'pose = {}')
  for i = 0,core.NUM_JOINTS-1 do
    UTdebug.log(0,'pose[core.'.. core.getJointName(i) .. '] = ',RAD_T_DEG * joint_angles[i])
  end
end

function body_PrintPose:finished()
  return true
end

CompTask('body_StandHigh',{
    init = true,
    time = 2.0,
    stiffnessOn = true,
    stiffnessOnTime = NIL_INIT,
    startedStiffnessOn = false,
    backToPoseTime = 1.0
  })
function body_StandHigh:choose()
  if self.init then
    self.init = false
    self.stiffnessOn = true
    self.stiffnessOnTime = -100
    self:resetTime()
  end

  walk_request:noWalk()
  local needStiffness = false
  local maxDiff = DEG_T_RAD * 2
  for i = core.LHipYawPitch,core.RAnkleRoll do
    if (i == core.LKneePitch) or (i == core.RKneePitch) then
      if (percepts.joint_angles[i] > maxDiff) then
        needStiffness = true
        break
      end
    elseif math.abs(DEG_T_RAD * standHigh[i] - percepts.joint_angles[i]) > maxDiff then
      needStiffness = true
      break
    end
  end
  if (self:getTime() < self.time) then
    needStiffess = true
  end
  if (self:getTime() < self.stiffnessOnTime + self.backToPoseTime) then
    needStiffness = true
  end

  if needStiffness and not(self.stiffnessOn) then
    commands.setStiffnessCommands(cfgStiffALWalk, 0.1)
    self.stiffnessOn = true
    self.startedStiffnessOn = false
    self.stiffnessOnTime = self:getTime()
  end
  if not(needStiffness) and self.stiffnessOn then
    self.stiffnessOn = false
    commands.setStiffnessCommands(cfgStiffLegsOffRestWalk, 0.1)
  end

  if (self:getTime() < self.stiffnessOnTime + self.backToPoseTime) then
    local init = not(self.startedStiffnessOn)
    self.startedStiffnessOn = true
    commands.setStiffnessCommands(cfgStiffALWalk, 0.1)
    self.stiffnessOn = true
    return body_ToPose:set{pose=standHigh,time=self.backToPoseTime,init=init}
  end
  return body_ToPose:set{pose=standHigh,time=self.time}
end

