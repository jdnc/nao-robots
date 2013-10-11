require 'stateMachine'
require 'strategy'
require 'task'
require 'head'
module(..., package.seeall);

ParaTask('skills_MoveHomeSweepScan',{'body_MoveHome','head_SweepScan'})
ParaTask('skills_MoveHomeBAL',{'body_MoveHome','head_BallActiveLocalize'})

ParaTask('skills_ArcToBallFacingGoalBAL',{'body_ArcToBallFacingGoal', 'head_BallActiveLocalize'})
ParaTask('skills_ArcToBallFacingGoalLFB',{'body_ArcToBallFacingGoal', 'head_LookForBall'})


ParaTask('skills_RotateAroundBallLookBall',{'body_RotateAroundBall', 'head_LookForBall'})
ParaTask('skills_RotateAroundBallBAL',{'body_RotateAroundBall', 'head_BallActiveLocalize'})
ParaTask('skills_RotateAroundBallWalkBAL',{'body_RotateAroundBallWithRotateWalk', 'head_BallActiveLocalize'})
ParaTask('skills_RotateAroundBallWalkLookBall',{'body_RotateAroundBallWithRotateWalk', 'head_LookForBall'})


ParaTask('skills_MoveAndAlignToBallLookBall',{'body_MoveAndAlignToBall', 'head_LookForBall'})
ParaTask('skills_AlignToBallTargetLookBall',{'body_AlignToBallWithTargetWalk', 'head_LookForBall'})
ParaTask('skills_AlignToBallTargetBAL',{'body_AlignToBallWithTargetWalk', 'head_BallActiveLocalize'})

ParaTask('skills_AlignToBallLookBall',{'body_MoveAndAlignToBall', 'head_LookForBall'})
ParaTask('skills_AlignToBallBAL',{'body_MoveAndAlignToBall', 'head_BallActiveLocalize'})

ParaTask('skills_MoveToPointLFB',{'body_MoveToPoint', 'head_LookForBall'})

ParaTask('skills_DribbleBallLookBall',{'body_DribbleBall', 'head_LookForBall'})
ParaTask('skills_DribbleBallBAL',{'body_DribbleBall', 'head_BallActiveLocalize'})

-- keeper stuff
ParaTask('skills_BlockMoveHead', {'body_Block2', 'head_MoveHead'})
ParaTask('skills_KeeperDiveLookForBall',{'body_KeeperDive','head_LookForBall'})
ParaTask('skills_KeeperSafeDiveLookForBall',{'body_SafeDive','head_LookForBall'})
ParaTask('skills_KeeperFakeDiveLookForBall',{'body_FakeDive','head_LookForBall'})
ParaTask('skills_GoalieSquatLookForBall', {'body_GoalieSquat','head_LookForBall'})
ParaTask('skills_MoveToPointBAL',{'body_MoveToPoint', 'head_BallActiveLocalize'})

ParaTask('skills_ToPoseMoveHead',{'body_ToPose','head_MoveHead'})
ParaTask('skills_ToPoseSweepScan',{'body_ToPose','head_SweepScan'})
ParaTask('skills_ToPoseLookForBall',{'body_ToPose','head_LookForBall'})
ParaTask('skills_StandHighBAL',{'body_StandHigh','head_BallActiveLocalize'})


CompTask('skills_PlayAttacker',{
           lastKickOrBall = -1
         })
function skills_PlayAttacker:choose(as)

  local ball = world_objects:getObjPtr(core.WO_BALL)
  if (ball.seen or (as ~= nil and as.kicking)) then
    self.lastKickOrBall = vision_frame_info.seconds_since_start
  end

  UTdebug.log(10, "Time since kick/seen", (vision_frame_info.seconds_since_start - self.lastKickOrBall), "frames since seen", (vision_frame_info.frame_id - ball.frameLastSeen), "ball sd", ball.sd.x, ball.sd.y, "time since completed scan", (vision_frame_info.seconds_since_start - behavior_mem.completeBallSearchTime))

  -- havent seen it in a while, high sd
  local ballLost = (vision_frame_info.seconds_since_start - self.lastKickOrBall) > 1.5 and (vision_frame_info.frame_id - ball.frameLastSeen) > 30 and (ball.sd.x > 350 or ball.sd.y > 350)

  -- recently completed a ball search scan in normal behavior
  local recentSearch = (vision_frame_info.seconds_since_start - behavior_mem.completeBallSearchTime) < 0.2

  UTdebug.log(10, "ballLost", ballLost, "recentSearch", recentSearch)

  if (ballLost and recentSearch and (as == skills_SearchLFB or (as ~= nil and not as.kicking))) then
  -- start search if ball is lost and completed a scan
    return skills_SearchLFB:set()
  elseif (ballLost and as == skills_SearchLFB) then
    -- continue search
    return skills_SearchLFB:set()
  else
    -- normal approach
    return skills_ApproachBall:set()
  end
  
end


CompTask('skills_SearchLFB',{
           searchState = 0,
           init = true,
           spinDir = 1
         })

function skills_SearchLFB:choose(as)
  -- search goes striaght into spinning search behavior
  -- because it shouldn't be called until after a full ball search scan
  -- was run anyway

  local ball = world_objects:getObjPtr(core.WO_BALL)

  local timeBetweenSpins = 30.0
  local turnFrac = 0.5
  -- Todd: at this speed, I think we turn about 60 deg/sec (7.0 s)
  -- Sam, let's go just a hair longer
  local timeToSpin = 8.0

  --local timeToSpin = (6.28 / (turnFrac * commands.walk_max_vel_rot_cw)) + 1.0
  --if (self.spinDir == 1) then  
  --  timeToSpin = (6.28 / (turnFrac * commands.walk_max_vel_rot_ccw)) + 1.0
  --end

  if (self.init) then
    self.init = false
    --commands.endWalk()
    self:resetTime()
    self.searchState = 0
  end

  local me = world_objects:getObjPtr(robot_state.WO_SELF)

  UTdebug.log(10, "Time in search", self:getTime(), "curr time", vision_frame_info.seconds_since_start, " last turn", behavior_mem.lastSearchTurnTime)

  -- turn and scan
  -- if we haven't done it in last 30 seconds
  -- this way we dont keep restarting into search and doing turn
  -- instead of walk to center
  if ((vision_frame_info.seconds_since_start - behavior_mem.lastSearchTurnTime) > timeBetweenSpins) then
    -- keep track of when we did this turn
    behavior_mem.lastSearchTurnTime = vision_frame_info.seconds_since_start
    self.searchState = 1
    self:resetTime()
    if (ball.bearing > 0.0) then
      self.spinDir = 1
    else
      self.spinDir = -1
    end

    commands.setWalkVelocity(0,0,self.spinDir*turnFrac,false)
    return head_MoveHead:set{pan = DEG_T_RAD*self.spinDir*30.0}
    
  end

  -- then walk to center and scan
  -- if state 1 ended, or we skipped state 0 since we had turned recently
  if ((self.searchState == 1 and self:getTime() > timeToSpin) or self.searchState == 0) then
    self.searchState = 2
    self:resetTime()
    -- different positions for each player
    if (robot_state.WO_SELF == 4) then
      return skills_MoveToPointLFB:set{des = core.Point2D(1500,600), maxVel = 0.8, doneDist = 400, doneAngle = 360}
    elseif (robot_state.WO_SELF == 3) then
      return skills_MoveToPointLFB:set{des = core.Point2D(0,0), maxVel = 0.8, doneDist = 400, doneAngle = 360}
    else
      return skills_MoveToPointLFB:set{des = core.Point2D(-1500,-600), maxVel = 0.8, doneDist = 400, doneAngle = 360}
    end
  end

  -- still in state 1, keep spinning
  if (self.searchState == 1) then
    commands.setWalkVelocity(0,0,self.spinDir*turnFrac,false)
    return head_MoveHead:set{pan = DEG_T_RAD*self.spinDir*30.0}
  end

  if (self.searchState == 2 and as ~= nil and as:finished()) then
    self.searchState = 3
    -- spin here forever
    self.spinDir = 1
    commands.setWalkVelocity(0,0,turnFrac,false)
    return head_MoveHead:set{pan = DEG_T_RAD*30.0}
  end
end

function skills_SearchLFB:finished()
  return self.searchState == 3
end

function isSubTaskCompleted(stateMach,as)
  local notFirst = not(stateMach:isFirstFrameInState())
  return (notFirst and as:finished())
end

function skills_updateClusterInfo(as,state)
  -- if we're in rotate or final approach... check for cluster
  local cluster = false
  if (as == skills_RotateAroundBallBAL or state:inState('FinalApproach') or as == skills_DribbleBallLookBall) then

    -- cluster if rotate ends up bumping arm
    --[[ -- dont use arm bumps for clusters right now
    if (as == skills_RotateAroundBallBAL) then
      if ((goalBearing < 0 and processed_sonar.bump_left_) or (goalBearing > 0 and processed_sonar.bump_right_)) then
        cluster = true
        UTdebug.log(10, "bump during rotate, CLUSTER", goalBearing, processed_sonar.bump_left_, processed_sonar.bump_right_)
      end
    end
    --]]

    -- cluster if filtered opp says opponent within 60 cm
    if (not cluster) then
      -- check if we have filtered opponents near us
      for i = core.WO_OPPONENT_FIRST, core.WO_OPPONENT_LAST do
        local opp = world_objects:getObjPtr(i)
        if (opp.sd.x < behavior_params.mainStrategy.maxOpponentSD and opp.sd.y < behavior_params.mainStrategy.maxOpponentSD and opp.distance < 600 and math.abs(opp.bearing) < DEG_T_RAD *70.0) then
          -- cluster
          UTdebug.log(10, "robot nearby in filter, CLUSTER",i,opp.distance,opp.bearing*RAD_T_DEG)
          cluster = true
        end
      end
    end

    -- cluster if sonar fired??
    if (not cluster) then
      if (processed_sonar.on_center_ and processed_sonar.center_distance_ < 0.4) then
        UTdebug.log(10, "sonar center, CLUSTER", processed_sonar.on_center_, processed_sonar.center_distance_)
        cluster = true
      end
    end
  end

  local clusterObj = world_objects:getObjPtr(core.WO_ROBOT_CLUSTER)
  behavior_mem.isInCluster = behaviorC:filterCluster(cluster,clusterObj.seen)
  UTdebug.log(10,'sonarCluster:',cluster,'visionCluster:',clusterObj.seen,'res:',behavior_mem.isInCluster)
  return behavior_mem.isInCluster
end

function skills_executeSetPlay(self)
  local play = behavior_params.setPlayStrategy:getPlayPtr(behavior_mem.setPlayInfo.type)
  if play.active then  -- valid play
    self.kickFound = false
    self.kickChoice = play.kickType
    self.kickData = kicks.kickData[self.kickChoice]
    self.kickHeading = self.kickData.heading
    self.kickDistance = self.kickData.distance
    self.switch = self.kickData.switchable
    self.foot = self.kickData.foot
    
    if self.state:inState('ApproachBall') then
      local desiredGoalBearing = play.desiredGoalBearing
      if play.reversible and behavior_mem.setPlayInfo.reversed then
        desiredGoalBearing = -desiredGoalBearing
      end
      local goalBearing = world_objects:getObjPtr(core.WO_OPP_GOAL).bearing
      local bearingError = desiredGoalBearing - goalBearing
      if math.abs(bearingError) < play.maxBearingError then
        self.state:transition('FinalApproach')
      else
        return skills_RotateAroundBallBAL:set{bearing = -bearingError, minScanDist = 0, desiredFoot=self.foot}
      end
    end
    -- in FinalApproach or Kick if we reach here
    self.kickFound = true
  end
end

CompTask('skills_ApproachBall',{
           init = true,
           state = stateMachine.StateMachine.create('Approach Ball state',{'ApproachBall','FinalApproach','Kick'}),
           kicking = false,
           dribbleFrame = -100,
           kickFound = false,
           kickChoice = core.FwdShortStraightKick,
           bestBlockedKick = -1,
           kickHeading = 0,
           kickDistance = 3000,
           switch = false,
           foot = core.Kick_RIGHT,
           kickData = kicks.kickData[core.FwdShortStraightKick],
         })

function skills_ApproachBall:choose(as)
  if (self.init) then
    self.init = false
    self.state:transition('ApproachBall')
  end

  local ball = world_objects:getObjPtr(core.WO_BALL)
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  self.kicking = false

  local goalBearing = strategy.getKickHeading()
  if (self.kickChoice == core.Dribble) then
    self.kickFound = false
  end

  local cluster = skills_updateClusterInfo(as,self.state)

  -- only request kick selection on rotate or final approach (but not yet incrementing doneCount)
  UTdebug.log(10, "kick choice is ", self.kickChoice, core.Dribble)
  if (as ~= nil and ((as == skills_RotateAroundBallBAL) or (as == body_AlignToBallWithTargetWalk and as.doneCount == 0 and as.xError > 180 ) or (self.kickChoice == core.Dribble) or (self.prevInCluster ~= behavior_mem.isInCluster))) then
    -- early on.. choose only from switchable kicks
    local choice = core.Kick_SWITCHABLE

    -- in final approach... choose only from kicks on that leg
    if (as == body_AlignToBallWithTargetWalk) then --MoveAndAlignToBall) then
      choice = as.currentFoot
    end


    -- Todd: previously we took first valid kick
    self.kickFound, self.kickChoice, self.bestBlockedKick, self.kickData = strategy.selectFirstValidKick(choice)
    -- Todd: now lets try best ranked kick
    --self.kickFound, self.kickChoice, self.bestBlockedKick, self.kickData = strategy.selectBestRankedKick(choice)

    --self.kickChoice = core.FwdSuperStraightKick
    --self.kickData = kicks.kickData[self.kickChoice]
    self.kickHeading = self.kickData.heading
    self.kickDistance = self.kickData.distance
    self.switch = self.kickData.switchable
    self.foot = self.kickData.foot

          --return kicks_ExecuteKick:set{kickChoice = core.WalkKickLeftwardSide, desiredHeading = DEG_T_RAD * 90, desiredDistance = self.kickDistance}

    UTdebug.log(10,"kick choice" , self.kickFound, self.kickChoice, self.bestBlockedKick, RAD_T_DEG*self.kickHeading, self.kickDistance, self.switch, self.foot)
  end

  -- if best kick is a short walk kick and we're not done rotating
  -- see if we have space to rotate more and do a full-on kick at the goal
  -- also dont do this if we're already within shootongoalradius, as the only way a walk kick is valid there is if it would score. then we want to do it
  if (self.kickFound and (self.kickChoice > core.FwdLongStraightKick) and math.abs(goalBearing) > 0.1 and ball.loc:getDistanceTo(core.Point2D(core.FIELD_X/2.0,0)) > behavior_params.mainStrategy.shootOnGoalRadius) then
    local minDistForRotate = behavior_params.mainStrategy.minOppDistForExtraRotateOffset + math.abs(goalBearing) * behavior_params.mainStrategy.minOppDistForExtraRotateFactor
    --local haveRotateSpace = behaviorC:haveKickSpaceFromOpponents(behavior_params.mainStrategy.minOppDistForExtraRotate)
    local haveRotateSpace = behaviorC:haveKickSpaceFromOpponents(minDistForRotate,0)
    UTdebug.log(10, "Chose short kick, see if there's space to rotate for better kick", haveRotateSpace, RAD_T_DEG * goalBearing, behavior_params.mainStrategy.minOppDistForExtraRotateOffset,behavior_params.mainStrategy.minOppDistForExtraRotateFactor,minDistForRotate)
    if (haveRotateSpace) then
      self.kickFound = false
    end
  end

  
  -- see if we changed feet
--  if (as == body_MoveAndAlignToBall) then
--    local footChange = (self.foot ~= as.currentFoot and not self.switch)
--    if (footChange) then
--      UTdebug.log(10,'donecount, new foot, new sitch, old current, old desired, old switchable', as.doneCount, as.xError, self.foot, self.switch, as.currentFoot, as.desiredFoot, as.switchable, footChange)
--    end
--  end


  --UTdebug.log(0,'cluster check:',cluster,'l:',processed_sonar.on_left_,'r:',processed_sonar.on_right_,'c:',processed_sonar.on_center_)
  --UTdebug.log(0,'         dist:',cluster,'l:',processed_sonar.left_distance_,'r:',processed_sonar.right_distance_,'c:',processed_sonar.center_distance_)
  -- if cluster... see if dribble is a valid option
  if (cluster) then
    --UTdebug.log(10, "Filter reports cluster")
    -- if we dribble in clusters
    if (behavior_params.clusterStrategy.behavior == core.Cluster_DRIBBLE) then
      local valid, rank, clear = strategy.isKickChoiceInKickRegion(core.Dribble)
      UTdebug.log(10, "cluster... dribble is", valid, rank, clear)
      
      if (valid == 1) then -- start dribbling for 75 frames (2.5 seconds)
        self.dribbleFrame = vision_frame_info.frame_id
        self.kickFound = true
      end
    end
    -- TODO: if we quick kick or side kick in clusters
  end

  -- set play
  if behavior_mem.setPlayInfo.type ~= core.SetPlay_none then
    local task = skills_executeSetPlay(self)
    if task ~= nil then
      return task
    end
  end
 
  -- continue dribble for 75 frames after cluster
  local continueDribble = false
  if ((vision_frame_info.frame_id - self.dribbleFrame) < 30) then
    self.kickChoice = core.Dribble
    continueDribble = true
  end
 
  if ((ball.distance > 550) and (not self.state:inState('Kick')) and (not self.state:inState('ApproachBall'))) then
    self.state:transition('ApproachBall')
  end

  -- we're incorrectly in final approach (maybe ball moved?)
  -- and now we have no kick and we're not at the right orientation
  -- and no blocked kick
  -- only escape if no kick at all.. not just if kick got blocked by opponent
  if (self.state:inState('FinalApproach') and (not self.kickFound and self.bestBlockedKick == -1) and math.abs(goalBearing) > 0.2) then
    UTdebug.log(10, "In BAD final approach, kickFound, goalBearing", self.kickFound, RAD_T_DEG*goalBearing)
    self.state:transition('ApproachBall')
  end

  if (self.state:inState('ApproachBall')) then

    if (isSubTaskCompleted(self.state,as)) then
      self.state:transition('FinalApproach')
    else
      -- dont skip out of arc while ball is moving fast
      if (ball.distance < 500 and math.abs(ball.relVel.x) < 200 and math.abs(ball.relVel.y) < 200) then
        if (self.kickFound and ball.seen and math.abs(ball.bearing) < 42.0*DEG_T_RAD ) then
          self.state:transition('FinalApproach')
        else
          return skills_RotateAroundBallBAL:set{bearing = goalBearing, minScanDist = 0, desiredFoot=self.foot}
        end
      else
        return skills_ArcToBallFacingGoalBAL:set{heading = goalBearing}
      end
    end
  end

  if (self.state:inState('FinalApproach')) then
    -- no final approach for dribble
    --if ((as == body_MoveAndAlignToBall) and isSubTaskCompleted(self.state,as)) then
    --if (body_MoveAndAlignToBall.done) then
    if (isSubTaskCompleted(self.state,as)) then
    --(self.kickFound and self.kickChoice == core.Dribble)) then
      UTdebug.log(0,"Transition from final approach to kick")
      self.state:transition('Kick')
      --body_MoveAndAlignToBall.doneCount = 0
      --body_MoveAndAlignToBall.done = false
    elseif (self.kickFound and self.kickChoice == core.Dribble) then
      UTdebug.log(0,"Skip final approach to dribble")
      self.state:transition('Kick')
    else
      strategy.setPassInfo(self.kickChoice,self.kickHeading,self.kickDistance,self.kickData.time + strategy.estimateTimeToReachBall(self.kickChoice))
      -- approach with walk to point
      -- putting these here because otherwise they don't seem to override the default values
      return body_AlignToBallWithTargetWalk:set{alignment=self.kickData.alignment,switchable=self.switch,kickChoice=self.kickChoice,desiredFoot=self.foot} -- added kick choice so that we can adjust the desired x based on kick
    end
  end

  if (self.state:inState('Kick')) then
    self.kicking = true
    -- task is over or we were doing dribble and its no longer valid
    UTdebug.log(10, "in kick, check if done", self.kickFound, self.kickChoice)
    if (isSubTaskCompleted(self.state,as) or (as ~= nil and as == skills_DribbleBallLookBall and (not self.kickFound or self.kickChoice ~= core.Dribble))) then
      self.state:transition('ApproachBall')
      self.kicking = false
      UTdebug.log(10,"Transition from kick to approachBall")
      -- only jump to this if we're not keeper
      -- for keeper we'd rather wait before starting to chase again
      if (robot_state.WO_SELF ~= core.KEEPER) then
        return skills_ArcToBallFacingGoalBAL:set{heading = goalBearing}
      else
        return as
      end
    else
      strategy.setPassInfo(self.kickChoice,self.kickHeading,self.kickDistance,self.kickData.time - self.state:timeSinceTransition())
      -- return kick or dribble call
      if (self.state:isFirstFrameInState()) then
        if (self.kickChoice == core.Dribble) then
          self.dribbleFrame = vision_frame_info.frame_id
          --return skills_DribbleBallBAL:set{heading = self.kickHeading}
          return skills_DribbleBallLookBall:set{heading = self.kickHeading}
        else
          return kicks_ExecuteKick:set{kickChoice = self.kickChoice, desiredHeading = self.kickHeading, desiredDistance = self.kickDistance}
          --return kicks_ExecuteKick:set{kickChoice = core.WalkKickFront, desiredHeading = self.kickHeading, desiredDistance = self.kickDistance}
          --return kicks_ExecuteKick:set{kickChoice = core.WalkKickLeftwardSide, desiredHeading = DEG_T_RAD * 90, desiredDistance = self.kickDistance}
          --return kicks_QuickStepKick:set{isLeft = (ball.bearing > 0)}
        end
      --elseif (as == kicks_ExecuteKick) then
        -- possibly change desired kick
        --return kicks_ExecuteKick:set{kickChoice = self.kickChoice, desiredHeading = self.kickHeading, desiredDistance = self.kickDistance}
      else
        return as
      end
    end
  end
end

function skills_ApproachBall:finished()
  return true
end


-- old keeper
CompTask('skills_PlayKeeper',{
           state = 0,
           diving = true, --false,
           clearRange = 800, -- how close the ball needs to be for us to clear
           clearFieldX = -1800, -- how far across the field we'll follow it, if it was within the clearRange
           sweepFieldX = -1800, -- how far across the field we'll go if our mates arent nearby
           practice = false,
           blockCount = 0,
           numFramesToBelieveBlock = 1
         })
function skills_PlayKeeper:choose(as)

  walk_request.keep_arms_out_ = true

  -- normally, we're not clearing
  behavior_mem.keeperClearing = false
  behavior_mem.keeperDiving = core.Dive_NONE

  local maxSide = 450 -- max distance we'll go to the side
  local xInit = -2750 -- home location in x

  local ball = world_objects:getObjPtr(core.WO_BALL)
  local buffer = 250

  -- possibly clear the ball
  -- seen, in range, near us, in penalty box
  local startAttack = (self.state ~= 1 and ball.seen and ball.distance < self.clearRange and ball.loc.x < (self.clearFieldX-buffer) and math.abs(ball.loc.y) < core.PENALTY_Y/2.0)
  local continueAttack = (self.state == 1 and ball.distance < (self.clearRange+buffer) and ball.loc.x < (self.clearFieldX) and math.abs(ball.loc.y) < core.PENALTY_Y/2.0+buffer)
  local isKicking = (as == skills_ApproachBall and as.kicking)

  -- regular block is slow
  local blockTime = 5.0

  -- diving is fast
  if (self.diving) then
    blockTime = 4.4 --2.5
  end

  -- possibly block the ball
  local shouldBlock, dir, yIntercept
  shouldBlock,dir,yIntercept,self.blockCount = skills_shouldBlock(blockTime,self.blockCount,self.numFramesToBelieveBlock)

  UTdebug.log(10, "Keeper", startAttack, continueAttack, isKicking, shouldBlock, dir, yIntercept)

  -- should block and not kicking
  -- if we're diving, continue
  if ((as == skills_BlockMoveHead and not as:finished())
    or (as == skills_KeeperSafeDiveLookForBall and not as:finished())
  or (as == skills_ToPoseLookForBall and as:getTime() < 1.0)) then
    UTdebug.log(10, "still in block, staying with it")
    self.blockCount = 0
    return as
  end

  if ((not isKicking) and shouldBlock) then
    UTdebug.log(10, "execute block")
    --    return skills_BlockLFB:set{dir = dir, yIntercept = yIntercept}
    -- diving keeper
    if (self.diving) then
      local dirbool = false
      local dir  = "LEFT"
      if (yIntercept < 0) then
        dir = "RIGHT"
        dirbool = true
      end
      walk_request:noWalk()

      if (self.practice) then
        --speech:say(dir)
        return skills_ToPoseLookForBall:set{pose = standingPose}
      else 
        return skills_KeeperSafeDiveLookForBall:set{reverseSides = dirbool}
      end
    else
      -- regular blocking keeper
      walk_request:noWalk()

      if (self.practice) then
        ---speech:say('block') 
        return skills_ToPoseLookForBall:set{pose = standingPose}
      else
        return skills_BlockMoveHead:set{pan = 0.0, time = 0.1, head_init = true, yIntercept = yIntercept}
      end
    end
  end

  if (startAttack or continueAttack or isKicking) then
    self.state = 1
    -- make sure we tell others we're clearing for role switching
    UTdebug.log(10,"keeper clearing the ball")
    behavior_mem.keeperClearing = true
    return skills_ApproachBall:set()
  end



  ----------------------------------------------
  -- positioning code if not blocking / clearing

  -- assume ball y of 0 if we haven't seen it in a while
  local bally = ball.loc.y / 2.0
  if ((vision_frame_info.frame_id - ball.frameLastSeen) > 60) then
    bally = 0
  end

  -- just make the points on a line
  local x = xInit
  local y = bally
  y = math.max(math.min(y, maxSide), -maxSide)

  -- lets just try to get to the point
  UTdebug.log(10, "Keeper to pt", x, y, bally, ball.loc.y, (vision_frame_info.frame_id - ball.frameLastSeen))
  self.state = 0
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  local des = core.Point2D(x,y)
  
  -- walk straight there if off by over 1.2m (orientThresh)
  return skills_MoveToPointBAL:set{maxVel = 1.0,des = core.Point2D(x,y),
                                   desTheta = 0,
                                   walkStraight=true, orientThresh = 1200,
                                   scanFreq = 12,
                                   distThresh = 350}

end



-- current keeper
CompTask('skills_PlaySweepingKeeper',{
           state = 0,
           diving = true,
           practice = false, --false,
           isMoving = false,
           errorCount = 0,
           init = true,
           numFramesToBelieveBlock = 3, --1,
           blockCount = 0
         })
function skills_PlaySweepingKeeper:choose(as)
  if self.init then
    self.blockCount = 0
  end

  walk_request.keep_arms_out_ = true

  -- normally, we're not clearing
  behavior_mem.keeperClearing = false
  behavior_mem.keeperDiving = core.Dive_NONE

  local maxSide = 450 -- max distance we'll go to the side
  local xInit = -core.HALF_FIELD_X+350 -- home location in x

  local ball = world_objects:getObjPtr(core.WO_BALL)
  local isKicking = (as == skills_ApproachBall and as.kicking)

  --blockTime = 7.5 --7.0 --5.9

  -- make less jumpy for now
  blockTime = 5.0 -- 4.6 --6.5

  -- possibly block the ball
  local shouldBlock, dir, yIntercept
  shouldBlock,dir,yIntercept,self.blockCount = skills_shouldBlock(blockTime,self.blockCount,self.numFramesToBelieveBlock)

  UTdebug.log(10, "Keeper", isKicking, shouldBlock, dir, yIntercept)

  -- should block and not kicking
  -- if we're diving, continue
  if ((as == skills_BlockMoveHead and not as:finished())
    or (as == skills_KeeperSafeDiveLookForBall and not as:finished())
  or (as == skills_GoalieSquatLookForBall and not as:finished())
or (as == head_BallActiveLocalize and as:getTime() < 1.0)) then
    UTdebug.log(10, "still in block, staying with it")
    self.blockCount = 0
    return as
  end

  if ((not isKicking) and shouldBlock) then
    self.init = false
    UTdebug.log(10, "execute block")
    -- diving keeper
    if (self.diving) then
      local dirbool = false
      local centerBlock = false
      local dir  = "LEFT"
      if (yIntercept < 0) then
        dir = "RIGHT"
        dirbool = true
      end
      walk_request:noWalk()

      -- definitely biased right
      if (yIntercept > -250 and yIntercept < 200) then
        dir = "CENTER"
        centerBlock = true
      end

      if (self.practice) then
        speech:say(dir)
        return skills_KeeperFakeDiveLookForBall:set{reverseSides = dirbool}
      elseif (centerBlock) then
        -- return new katie/sam crouch dive
        return skills_GoalieSquatLookForBall:set()
      else
        return skills_KeeperSafeDiveLookForBall:set{reverseSides = dirbool}
      end
    else
      -- regular blocking keeper
      walk_request:noWalk()

      if (self.practice) then
        speech:say('block') 
        commands.stand()
        return head_BallActiveLocalize:set()
      else
        return skills_GoalieSquatLookForBall:set()
      end
    end
  end

  ----------------------------------------------
  -- positioning code if not blocking / clearing

  -- assume ball y of 0 if we haven't seen it in a while
  local bally = ball.loc.y / 2.0
  if ((vision_frame_info.frame_id - ball.frameLastSeen) > 60) then
    bally = 0
  end

  -- just make the points on a line
  local x = xInit
  local y = bally
  y = math.max(math.min(y, maxSide), -maxSide)

  -- lets just try to get to the point
  UTdebug.log(10, "Keeper to pt", x, y, bally, ball.loc.y, (vision_frame_info.frame_id - ball.frameLastSeen))
  self.state = 0
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  local des = core.Point2D(x,y)
  
  local desOri = me.loc:getAngleTo(ball.loc)
  
  -- cap it to each side
  if (desOri > 60.0*DEG_T_RAD) then
    desOri = 60.0*DEG_T_RAD
  elseif (desOri < -60.0*DEG_T_RAD) then
    desOri = -60.0*DEG_T_RAD
  end

  -- calculate error from target
  local desired = core.Point2D(x,y)
  local distError = me.loc:getDistanceTo(desired)
  local oriError = math.abs(core.normalizeAngle(me.orientation - desOri))

  UTdebug.log(10, "keeper errors", distError, oriError*RAD_T_DEG, self.isMoving)

  -- big error, have to walk
  if (distError > 250 or oriError > DEG_T_RAD * 15.0) then
    self.errorCount = self.errorCount + 1
  elseif (self.errorCount > 0) then
    self.errorCount = self.errorCount - 1
  end

  if (self.errorCount > 8 or self.init) then
    self.isMoving = true
    self.init = false
    -- walk straight there if off by over 1.2m (orientThresh)
    return skills_MoveToPointBAL:set{maxVel = 1.0,des=desired,
                                     desTheta = desOri,
                                     walkStraight=true, orientThresh = 1200,
                                     scanFreq = 12,
                                     distThresh = 350, doneDist = 100, 
                                     doneAngle = 8, doneThresh = 10}
    -- already walking, continue until done
  elseif (self.isMoving) then
    self.init = false
    if (as:finished()) then
      self.isMoving = false
      commands.stand()
      self.errorCount = 0
      return head_BallActiveLocalize:set()
    else
      return as
    end
  else
    self.init = false
    -- not walking, continue
    commands.stand()
    return head_BallActiveLocalize:set()
  end
end



CompTask('skills_PlayPenaltyKickKeeper',{
           init = true,
           isDown  = false,
           practice = false,
           useOppLoc = false,
           oppLocWaitTime = 3.0, -- after opp has been at ball this long, dive
           closeOppTime = -1,
           numFramesToBelieveBlock = 1,
           blockCount = 0
	 })
function skills_PlayPenaltyKickKeeper:choose(as)

  -- on init, walk forward 30 cm
  if (self.init) then
    self.init = false
    walk_request.keep_arms_out_ = true
    commands.setWalkTarget(300,0,0,false)
    self.blockCount = 0
    self:resetTime()
  end

  if ((game_state.secsRemaining < 56) or (self:getTime() > 6.0)) then
    commands.stand()
  end

  local blockTime = 4.5 --7.9 --9.0 --4.6 --4.0 2.5
  local ball = world_objects:getObjPtr(core.WO_BALL)
  local shouldBlock, dir, yIntercept
  shouldBlock,dir,yIntercept,self.blockCount = skills_shouldBlock(blockTime,self.blockCount,self.numFramesToBelieveBlock)

  if (not shouldBlock and self.useOppLoc) then
    -- check if opponent has been close for closeOppTime seconds
    if (behaviorC:getOpponentBallDistance() < 300 and self.closeOppTime == -1 and ball.loc.x < 0) then 
      self.closeOppTime = vision_frame_info.seconds_since_start
      UTdebug.log(10, "opponent detected at ball")
    end
    if (self.closeOppTime > -1 and (vision_frame_info.seconds_since_start - self.closeOppTime) > self.oppLocWaitTime) then
      local oppY = behaviorC:getNearestOpponentY()
      yIntercept = -oppY
      UTdebug.log(10, "opponent has been at ball for ", (vision_frame_info.seconds_since_start - self.closeOppTime), " seconds, y is ", oppY, " dive opposite")
      shouldBlock = true
    end
  end

  -- continue diving if we are
  if ((as == skills_KeeperSafeDiveLookForBall and not as:finished()) or 
      (as == skills_ToPoseLookForBall and as:getTime() < 1.0) or
      (as == skills_GoalieSquatLookForBall and not as:finished())) then
    self.blockCount = 0
    return as
  end

  -- if its time to block
  if (shouldBlock and not self.isDown) then
    self.isDown = true
    local dirbool = false
    local dir  = "LEFT"
    if (yIntercept < 0) then
      dir = "RIGHT"
      dirbool = true
    end
    if (self.practice) then
      --speech:say(dir)
      return skills_ToPoseLookForBall:set{pose = standingPose}
    else
      return skills_KeeperSafeDiveLookForBall:set{reverseSides = dirbool}
    end
  end
  
   -- otherwise, stand and look for ball
  return head_LookForBall:set{}
   
 end



-- determine if its time to execute a blocking behavior
-- i.e. is the ball coming at us and how soon will it be here
function skills_shouldBlock(blockingTime,blockCount,numFramesToBelieveBlock)

  -- notes- dive to side reaches about 635 mm so center is 315 mm to the side
  -- crab pose reaches about 510 mm total, centered on zero
  -- to maximize we'd want to switch at 160, but
  -- since the crab pose is faster, i want to bias towards it more
  -- no more than 250 since that's the edge of its reach either dir

  if ((vision_frame_info.seconds_since_start - behavior_mem.timePlayingStarted) < 5.0) then
    return false, "CENTER", 0, 0
  end

  local reach = 850
  local sideThresh = 200

  local ball = world_objects:getObjPtr(core.WO_BALL)
  local relLoc = ball.relPos 

  -- how long til the ball gets here
  local distFromUs = relLoc.x
  local timeToUs = 100000
  local yIntercept = 0
  if (ball.relVel.x < 0 and relLoc.x > 0) then
    timeToUs = distFromUs / -ball.relVel.x
    yIntercept = relLoc.y + (ball.relVel.y * timeToUs)
  end

  -- rel ball filter time
  local relTimeToUs = 100000
  local relYIntercept = 0
  if (behavior_mem.keeperRelBallVel.x < 0 and behavior_mem.keeperRelBallPos.x > 0) then
    relTimeToUs = behavior_mem.keeperRelBallPos.x / -behavior_mem.keeperRelBallVel.x
    relYIntercept = behavior_mem.keeperRelBallPos.y + (behavior_mem.keeperRelBallVel.y * relTimeToUs)
  end

  local dir = "CENTER"
  if (yIntercept < -sideThresh) then
    dir = "RIGHT"
  elseif (yIntercept > sideThresh) then
    dir = "LEFT"
  end

  local shouldBlock = false

  local yReturn = yIntercept

  -- if ball is coming and within reach, block it
  if (timeToUs < blockingTime and math.abs(yIntercept) < reach and ball.relVel.x < -75 and ball.distance > 700) then
    shouldBlock = true
  end

  -- or rel ball filter says we should block
  --[[
  if (relTimeToUs < blockingTime and math.abs(relYIntercept) < reach and behavior_mem.keeperRelBallVel.x < -75 and behavior_mem.keeperRelBallPos.x > 400) then
    shouldBlock = true
    yReturn = relYIntercept
  end
  --]]

  UTdebug.log(10, "Block", ball.seen, relLoc, ball.relVel, distFromUs, timeToUs, relTimeToUs, yIntercept, shouldBlock, dir)

  if shouldBlock then
    blockCount = blockCount + 1
  elseif blockCount > 0 then
    blockCount = blockCount - 1
  end

  shouldBlock = (blockCount >= numFramesToBelieveBlock)

  -- if ball is coming and within reach, block it
  return shouldBlock, dir, yReturn, blockCount

end


-----------------------------------
-- supporter and defender
-----------------------------------

function calcPositionForPlayer(player,role)
  -- go to a point fwdDist from ball and sideDist over from ball
  local ball = world_objects:getObjPtr(core.WO_BALL)
  local me = world_objects:getObjPtr(player)

  targetPt = core.Point2D(ball.loc.x, ball.loc.y)
  targetPt.x = targetPt.x + role.offsetFromBall.x

  -- go over from ball somewhat, no closer than midfield
  -- if ball is near center, stay on current side
  if (ball.loc.y > 600) then
    targetPt.y = targetPt.y - role.offsetFromBall.y
    -- no closer than midfield
    if (targetPt.y > 0) then
      targetPt.y = -250
    end
  elseif (ball.loc.y < -600) then
    targetPt.y = targetPt.y + role.offsetFromBall.y
    -- no closer than midfield
    if (targetPt.y < 0) then
      targetPt.y = 250
    end
  else
    -- center ball, stay on current side of ball
    if (me.loc.y > ball.loc.y) then
      targetPt.y = targetPt.y + role.offsetFromBall.y
    else
      targetPt.y = targetPt.y - role.offsetFromBall.y
    end
  end

  targetPt.x = core.crop(targetPt.x,role.minX,role.maxX)


  -- splitY -- split the difference between ball y and 0
  if (role.splitY) then
    -- ball angle from own goal
    local ownGoal = world_objects:getObjPtr(core.WO_OWN_GOAL)
    local ballheading = ownGoal.loc:getAngleTo(ball.loc)
    local xDist = targetPt.x - ownGoal.loc.x
    local yOffset = xDist * math.tan(ballheading)
    -- if ball is on the far side, go all the way out to side for throw-in
    if (ball.loc.x > 600) then
      yOffset = ball.loc.y
    end
    if (ball.loc.y > 0 and yOffset < 0) then
      yOffset = -yOffset
    end
    if (ball.loc.y < 0 and yOffset > 0) then
      yOffset = -yOffset
    end
    targetPt.y = yOffset
    UTdebug.log(10, "calc defender y", targetPt.x,ballheading*RAD_T_DEG,xDist, yOffset)
    -- this is the same y as the keeper
    -- lets offset by a robot width
    if (math.abs(targetPt.y) < 150) then
      if (me.loc.y > 0) then
        targetPt.y  = 150
      else
        targetPt.y  = -150
      end
    end
  end

  -- oppositeY
  if (role.oppositeYOfBall) then
    if ball.loc.y > 300 then
      targetPt.y = -math.abs(role.offsetFromBall.y)
    elseif ball.loc.y < -300 then
      targetPt.y = math.abs(role.offsetFromBall.y)
    else
      if me.loc.y > 0 then
        targetPt.y = math.abs(role.offsetFromBall.y)
      else
        targetPt.y = -math.abs(role.offsetFromBall.y)
      end
    end
  end

  -- deal with crossing past teammate robots
  -- if robot is near ball y, and wrong side x
  -- clear out to the side first
  if (math.abs(me.loc.y - ball.loc.y) < 400) then
    if ((targetPt.x > ball.loc.x and (me.loc.x - ball.loc.x) < 300) or
      (targetPt.x < ball.loc.x and (me.loc.x - ball.loc.x) > -300)) then
      -- only partway towards target x
      if (targetPt.x > me.loc.x) then
        targetPt.x = me.loc.x + 150
      else
        targetPt.x = me.loc.x - 150
      end
      if (me.loc.y > ball.loc.y) then
        targetPt.y = targetPt.y + 200
        if (targetPt.y < 800) then
          targetPt.y = 800
        end
      else
        targetPt.y = targetPt.y - 200
        if (targetPt.y > -800) then
          targetPt.y = -800
        end
      end
      UTdebug.log(10, "position might cross ball, changing target to ", targetPt.x, targetPt.y)
    end
  end

  if (math.abs(targetPt.y - ball.loc.y) < 450) then
    if ((targetPt.x > ball.loc.x and (me.loc.x - ball.loc.x) < 300) or
      (targetPt.x < ball.loc.x and (me.loc.x - ball.loc.x) > -300)) then
      -- move target point out more in y until we've past ball
      if (me.loc.y > ball.loc.y) then
        targetPt.y = targetPt.y + 200
        if (targetPt.y < 800) then
          targetPt.y = 800
        end
      else
        targetPt.y = targetPt.y - 200
        if (targetPt.y > -800) then
          targetPt.y = -800
        end
      end
      UTdebug.log(10, "keep target wide of ball until we've past it, changing target to ", targetPt.x, targetPt.y)
    end
  end

  -- max of 2600
  -- min of -2000
  targetPt.x = core.crop(targetPt.x,role.minX,role.maxX)

  -- y max of maxY
  targetPt.y = core.crop(targetPt.y,-role.maxY,role.maxY)
  return targetPt
end

CompTask('skills_PlayPosition',{
           init = true,
           targetPt = core.Point2D(0,0),
           role = NIL_INIT,
           allowStand = false,
           errorCount = 0,
           isMoving = true,
           numFramesMissedSeeingBall = 0,
           useNegativeBallInfo = true
         })
function skills_PlayPosition:choose(as)
  if self.init then
    self.init = false
    self.errorCount = 0
    self.isMoving = true
    self.numFramesMissedSeeingBall = 0
  end

  self.targetPt = calcPositionForPlayer(robot_state.WO_SELF,self.role)

  local ball = world_objects:getObjPtr(core.WO_BALL)
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  local lookTarget = ball.loc

  if not(ball.seen) and (as == skills_RotateInPlace) then
    return as
  end
  
  if ball.seen then
    self.numFramesMissedSeeingBall = 0
  elseif shouldSeeObject(core.WO_BALL,5000) then
    self.numFramesMissedSeeingBall = self.numFramesMissedSeeingBall + 1
  end

  local passInfo = strategy.getExpectedPassInfo()
  local expectingPass = ((passInfo ~= nil) and (passInfo.timeUntilPass < behavior_params.passStrategy.timeToConsider) and (passInfo.target:getDistanceTo(me.loc) < behavior_params.passStrategy.distToConsider) and not(strategy.DEBUGGING_POSITIONING))
  
  if self.useNegativeBallInfo and not(expectingPass) and (self.numFramesMissedSeeingBall > 30.0 * 5) then -- 5 seconds
    self.isMoving = true
    self.numFramesMissedSeeingBall = 0
    UTdebug.log(10,'Should have seen ball, but didn\'t for too long, turning in place')
    return skills_RotateInPlace:set()
  end


  if expectingPass then
    UTdebug.log(50,'Play position, expecting a pass, time, target',passInfo.timeUntilPass,passInfo.target.x,passInfo.target.y)
    speech:say('pass')
    local oppGoal = world_objects:getObjPtr(core.WO_OPP_GOAL)
    local desBearing = oppGoal.bearing
    local angleDiff = core.normalizeAngle(desBearing - ball.bearing)
    if (angleDiff < -behavior_params.passStrategy.maxTurnFromBall) then
      desBearing = core.normalizeAngle(ball.bearing - behavior_params.passStrategy.maxTurnFromBall)
    elseif (angleDiff > behavior_params.passStrategy.maxTurnFromBall) then
      desBearing = core.normalizeAngle(ball.bearing + behavior_params.passStrategy.maxTurnFromBall)
    end

    local targetOffset = core.Point2D_getPointFromPolar(behavior_params.passStrategy.targetOffsetDist,desBearing)
    self.targetPt = passInfo.target - targetOffset
    lookTarget = me.loc + targetOffset
  end

  local desOri = me.loc:getAngleTo(lookTarget)
  local oriError = math.abs(core.normalizeAngle(me.orientation - desOri))
  local distError = me.loc:getDistanceTo(self.targetPt)
  -- big error, have to walk
  if (distError > 250 or oriError > DEG_T_RAD * 15.0) then
    self.errorCount = self.errorCount + 1
  elseif (self.errorCount > 0) then
    self.errorCount = self.errorCount - 1
  end
  
  UTdebug.log(50, "Play Position with fwdDist ", self.role.offsetFromBall.x, " sideDist: ", self.role.offsetFromBall.y, " ball at: ", ball.loc, " target: ", self.targetPt)
  
  
  if (self.errorCount > 8) or self.expectingPass then
    self.isMoving = true
  elseif self.isMoving and (as ~= nil) and as:finished() then
    self.isMoving = false
  end

  if not(self.allowStand) then
    self.isMoving = true
  end

  if ball.distance < 500 then
    self.isMoving = true
  end

  if self.isMoving then
    return skills_MoveToPointBAL:set{des = self.targetPt, lookAtPoint = true,
                                     look = lookTarget, walkStraight = true, orientThresh = 1000,
                                     distThresh = 350, doneDist = 100, doneAngle = 8, doneThresh = 10} -- from keeper
  else
    --print(tostring(robot_state.WO_SELF) .. ' ' .. tostring(robot_state.team_) .. ' in place')
    return skills_StandHighBAL:set()
  end
end

CompTask('skills_PlaySetPlayReceiver',{
          targetPt = core.Point2D()
         })
function skills_PlaySetPlayReceiver:choose(as)
  local oppGoal = world_objects:getObjPtr(core.WO_OPP_GOAL)
  -- if not in set play, just because role hasn't switched yet, stay with prev pt
  if behavior_mem.setPlayInfo.type ~= core.SetPlay_none then
    local ball = world_objects:getObjPtr(core.WO_BALL)
    local play = behavior_params.setPlayStrategy:getPlayPtr(behavior_mem.setPlayInfo.type)
    local goalBearing = play.desiredGoalBearing
    local offset = core.Point2D(play.desiredTargetOffset)
    if behavior_mem.setPlayInfo.reversed then
      goalBearing = -goalBearing
      offset.y = -offset.y
    end
    local dist = kicks.kickData[play.kickType].distance
    local globalKickHeading = core.normalizeAngle(-goalBearing + kicks.kickData[play.kickType].heading)
    self.targetPt.x = ball.loc.x + dist * math.cos(globalKickHeading)
    self.targetPt.y = ball.loc.y + dist * math.sin(globalKickHeading)
  end
  return skills_MoveToPointBAL:set{des = self.targetPt, lookAtPoint = true, look = oppGoal.loc, walkStraight = true, orientThresh = 1000}
end

CompTask('skills_RotateInPlace',{
           init = true,
           spinDir = 1,
           spinTime = 8.0
         })
function skills_RotateInPlace:choose(as)
  if self.init then
    self.init = false
    self:resetTime()
    local ball = world_objects:getObjPtr(core.WO_BALL)
    if ball.bearing > 0.0 then
      self.spinDir = 1
    else
      self.spinDir = -1
    end
  end
  commands.setWalkVelocity(0,0,self.spinDir*0.5,false)
  return head_MoveHead:set{pan = DEG_T_RAD*self.spinDir*30.0}
end

function skills_RotateInPlace:finished()
  return self:getTime() > self.spinTime
end
