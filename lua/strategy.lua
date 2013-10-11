require 'kicks'
require 'cfgstrategy'
module(..., package.seeall)

DEBUGGING_POSITIONING = false

function setStrategy()
  if (game_state.isPenaltyKick) then
    -- dont change anything in penalty kick
    return
  end
  -- normal strategy
  local goalDiff = game_state.ourScore - game_state.opponentScore

  --behavior_params.mainStrategy = cfgstrategy.cfgWalkKickLimitedSlowKickStrategy
  behavior_params.mainStrategy = cfgstrategy.cfgWalkKickLimitedSlowKickFarShotStrategy
  --UTdebug.log(0, "Goal diff", goalDiff, "WALK KICK + LIMITED SLOW. Allow slow kick with ", behavior_params.mainStrategy.minOppDistForSlowKick, " mm space")

  -- use side kicks in clusters for all of these
  behavior_params.clusterStrategy = cfgstrategy.cfgSideKickClusterStrategy
  
  behavior_params.passStrategy = cfgstrategy.cfgPassStrategy
  behavior_params.roleStrategy = cfgstrategy.cfgRoleStrategy
  --behavior_params.roleStrategy = cfgstrategy.cfgAggressiveRoleStrategy
  behavior_params.setPlayStrategy = cfgstrategy.cfgSetPlayStrategy
  behavior_params.cornerStrategy = cfgstrategy.cfgBaseCornerStrategy
--[[
  if goalDiff >= 6 then
    UTdebug.log(0,'STRATEGY: super + long use opps + cautious defender')
    for i = 0,core.NUM_KICKS-1 do
      behavior_params.mainStrategy:setUsableKick(i,false)
    end
    behavior_params.mainStrategy:setUsableKick(core.FwdSuperStraightKick,true)
    behavior_params.mainStrategy:setUsableKick(core.FwdLongStraightKick,true)
    behavior_params.mainStrategy.minOppDistForLongKick = 0 -- distance doesn't matter, need this in addition to SUPER for super to work
    behavior_params.mainStrategy.minOppDistForSuperKick = 250 -- distance matters a little bit
    behavior_params.mainStrategy.orientationErrorFactor = 4.5 -- more confident so we do kick
    behavior_params.roleStrategy = cfgstrategy.cfgRoleStrategy
  elseif goalDiff >= 4 then
    UTdebug.log(0,'STRATEGY: super + long ignore opps')
    for i = 0,core.NUM_KICKS-1 do
      behavior_params.mainStrategy:setUsableKick(i,false)
    end
    behavior_params.mainStrategy:setUsableKick(core.FwdSuperStraightKick,true)
    behavior_params.mainStrategy:setUsableKick(core.FwdLongStraightKick,true)
    behavior_params.mainStrategy.minOppDistForLongKick = 0 -- distance doesn't matter, need this in addition to SUPER for super to work
    behavior_params.mainStrategy.minOppDistForSuperKick = 250 -- distance matters a little bit
    behavior_params.mainStrategy.orientationErrorFactor = 4.5 -- more confident so we do kick
    behavior_params.mainStrategy.maxOpponentSD = -1
    behavior_params.mainStrategy.minOpponentDist = -1


  elseif goalDiff >= 2 then
    UTdebug.log(0,'STRATEGY: super')
    for i = 0,core.NUM_KICKS-1 do
      behavior_params.mainStrategy:setUsableKick(i,false)
    end
    behavior_params.mainStrategy:setUsableKick(core.FwdSuperStraightKick,true)
    behavior_params.mainStrategy.minOppDistForLongKick = 0 -- distance doesn't matter, need this in addition to SUPER for super to work
    behavior_params.mainStrategy.minOppDistForSuperKick = 0 -- distance doesn't matter
    behavior_params.mainStrategy.orientationErrorFactor = 5.5 -- more confident so we do kick
  end
--]]
--[[  
  if goalDiff >= 6 then
    -- only super kicks
    UTdebug.log(0,'STRATEGY: super')
    for i = 0,core.NUM_KICKS-1 do
      behavior_params.mainStrategy:setUsableKick(i,false)
    end
    behavior_params.mainStrategy:setUsableKick(core.FwdSuperStraightKick,true)
    behavior_params.mainStrategy.minOppDistForLongKick = 0 -- distance doesn't matter, need this in addition to SUPER for super to work
    behavior_params.mainStrategy.minOppDistForSuperKick = 0 -- distance doesn't matter
    behavior_params.mainStrategy.orientationErrorFactor = 5.5 -- more confident so we do kick
  elseif goalDiff >= 4 then
    -- only walk kicks
    UTdebug.log(0,'STRATEGY: Walk kick')
    for i = 0,core.NUM_KICKS-1 do
      behavior_params.mainStrategy:setUsableKick(i,false)
    end
    behavior_params.mainStrategy:setUsableKick(core.WalkKickFront,true)
    behavior_params.mainStrategy:setUsableKick(core.WalkKickLeftwardSide,true)
    behavior_params.mainStrategy:setUsableKick(core.WalkKickRightwardSide,true)
    behavior_params.mainStrategy:setUsableKick(core.WalkKickLeftward,true)
    behavior_params.mainStrategy:setUsableKick(core.WalkKickRightward,true)
  elseif goalDiff >= 2 then
    -- farther robots
    UTdebug.log(0,'STRATEGY: farther forwards and back')
    -- forward farther forwards
    local role = behavior_params.roleStrategy:getRolePositionConfigPtr(core.FORWARD)
    role.offsetFromBall.x = 4500
    -- defender farther back
    local role = behavior_params.roleStrategy:getRolePositionConfigPtr(core.DEFENDER)
    role.offsetFromBall.x = -3000
  end
--]]
  -- different for penalty kick
  if (game_state.isPenaltyKick) then
    behavior_params.mainStrategy = cfgstrategy.cfgPenaltyKickStrategy
  end
  -- let's do a different strategy for the goalie
  if (robot_state.role_ == core.KEEPER) then
    behavior_params.mainStrategy = cfgstrategy.cfgFwdGoalieStrategy
    behavior_params.clusterStrategy = cfgstrategy.cfgNoClusterStrategy
  end

  strategy.initKickRegion()
end


function getKickHeading()
  local heading
  if (behavior_mem.ballIsInCorner) then
    heading = getPenBoxHeading()
  else
    --heading = getBestGoalHeading()
    heading = behavior_mem.largeGapHeading
  end
  return heading
end


function getPenBoxHeading()
  local ball = world_objects:getObjPtr(core.WO_BALL)
  local targetPt = behavior_params.cornerStrategy.targetPt
  local newTargetPt = core.Point2D(targetPt.x, targetPt.y)

  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  if (me.loc.y < 0) then
    newTargetPt = core.Point2D(targetPt.x, -targetPt.y)
  end

  local heading = (ball.loc):getBearingTo(newTargetPt, me.orientation)

  return heading
end


-- right on goal line, check if dribble is valid first
function checkGoalLineDribble()

  local ball = world_objects:getObjPtr(core.WO_BALL)

  local lineDist = 200

  -- not in dribble region, nothing happens here
  if (ball.loc.x < (core.FIELD_X/2.0 - lineDist) or math.abs(ball.loc.y) > (core.GOAL_Y/2.0 - 200)) then
    return -1
  end

  local valid, rank
  valid, rank = isKickChoiceInKickRegion(core.Dribble)

  if (valid == 1) then
    return core.Dribble
  else
    return -1
  end

end

function chooseFootForBallOnPost()

  -- check if the ball is on the post
  local ball = world_objects:getObjPtr(core.WO_BALL)
  local leftPost = world_objects:getObjPtr(core.WO_OPP_LEFT_GOALPOST)
  local rightPost = world_objects:getObjPtr(core.WO_OPP_RIGHT_GOALPOST)

  if (ball.loc:getDistanceTo(leftPost.loc) < 300) then
    -- left post, have to use left foot
    UTdebug.log(10, "ball on left post")
    return true, core.Kick_LEFT
  elseif (ball.loc:getDistanceTo(rightPost.loc) < 300) then
    -- right post, must use right foot
    UTdebug.log(10, "ball on right post")
    return true, core.Kick_RIGHT
  end
  return false, core.Kick_SWITCHABLE
end


-- currently used kick selection
function selectFirstValidKick(foot_choice)

  UTdebug.log(10, "foot choice is", foot_choice)

  -- set the heading and foot of the goalward kicks and passing kicks
  if (behavior_mem.ballIsInCorner) then
    setCornerShots()
  else
    setGoalGapKicks()
  end

  local forcedFoot, forcedFootChoice = chooseFootForBallOnPost()
  
  -- if we force a foot based on post, set foot_choice to it
  if (forcedFoot) then
    foot_choice = forcedFootChoice
    UTdebug.log(10, "on post, only use kicks with foot", foot_choice)
  end

  setPassingKicks()

  -- loop through all kicks, take first one that is valid
  -- none valid, take first one that would have been valid without opponents
  -- or just return false

  -- index of best kicks
  local bestClearKick = -1
  local bestBlockedKick = -1

  -- get current strategy
  local strategy
  if (behavior_mem.ballIsInCorner) then
    strategy = behavior_params.cornerStrategy
  else
    strategy = behavior_params.mainStrategy
  end

  -- if on goal line, possibly do dribble first
  bestClearKick  = checkGoalLineDribble()

  if (bestClearKick == -1) and behavior_mem.isInCluster and not forcedFoot then
    bestClearKick = checkClusterSideKicks(strategy,behavior_params.clusterStrategy,foot_choice)
  end
  
  if (bestClearKick == -1) and behavior_mem.isInCluster and not forcedFoot then
    bestClearKick = checkClusterQuickKicks(strategy,behavior_params.clusterStrategy,foot_choice)
  end

  -- do we have space for slow kicks?
  local slowKicksValid = behaviorC:haveKickSpaceFromOpponents(behavior_params.mainStrategy.minOppDistForSlowKick, behavior_params.mainStrategy.ignoreBehindAngleForSlowKick)
  local superKicksValid = behaviorC:haveKickSpaceFromOpponents(behavior_params.mainStrategy.minOppDistForSuperKick, behavior_params.mainStrategy.ignoreBehindAngleForSlowKick)
  if robot_state.WO_SELF == core.KEEPER then
    superKicksValid = false
  end
  -- don't super kick near the goal, just put it in with something faster
  local ball = world_objects:getObjPtr(core.WO_BALL)
  if ball.loc.x > core.HALF_FIELD_X - 3000 then
    superKicksValid = false
  end

  -- loop through kicks, if we didn't already find one
  if (bestClearKick == -1) then
    for i = 0, core.NUM_KICKS-1 do

      UTdebug.log(80,"Kick",i,luaC:getString(core.kickNames,i),"distance",kicks.kickData[i].distance,"heading",RAD_T_DEG*kicks.kickData[i].heading,"set",kicks.kickData[i].isSet,"slow",kicks.kickData[i].isSlow)
      UTdebug.log(80,slowKicksValid,superKicksValid)
      -- only if fast kick or slow kicks are ok
      if ((not kicks.kickData[i].isSlow or slowKicksValid) and 
          ((i ~= core.FwdSuperStraightKick) or superKicksValid)) then
        bestClearKick, bestBlockedKick = checkKick(i,strategy,foot_choice,bestClearKick,bestBlockedKick,false)
      end

      -- break on best kick
      if bestClearKick ~= -1 then
        break
      end
    end -- kick loop
  end -- kick not already found

  local kickFound = (bestClearKick ~= -1)
  local bestKick = bestClearKick

  -- no kick, return possibly blocked kick
  if (not kickFound) then
    --UTdebug.log(0, "No clear kick, going with best blocked kick", bestClearKick, bestBlockedKick)
    bestKick = bestBlockedKick
  end

  -- in case there were no usable kicks
  if (bestKick == -1) then
    -- default kick
    bestKick = behavior_params.mainStrategy.defaultKick
    bestKickRanking = 10000
  end

  local data = kicks.kickData[bestKick]

   UTdebug.log(80, "Selected kick, heading, distance, cluster", kickFound, luaC:getString(core.kickNames,bestKick), data.heading, data.distance,behavior_mem.isInCluster)

  behavior_mem.kickChoice = bestKick
  behavior_mem.kickDist = data.distance
  behavior_mem.kickRank = 0--bestKickRanking
  behavior_mem.kickHeading = data.heading
  behavior_mem.chooseKick = kickFound

  -- return if we found a best kick, its index, heading, distance, y, switchable
  --return kickFound, bestKick, bestBlockedKick, data.heading, data.distance, data.yOffset, data.switchable, data.foot
  return kickFound, bestKick, bestBlockedKick, data

end



function selectBestRankedKick(foot_choice)

  UTdebug.log(10, "foot choice is", foot_choice)

  -- set the heading and foot of the goalward kicks and passing kicks
  if (behavior_mem.ballIsInCorner) then
    setCornerShots()
  else
    setGoalGapKicks()
  end

  setPassingKicks()

  -- loop through all kicks, take first one that is valid
  -- none valid, take first one that would have been valid without opponents
  -- or just return false

  -- index of best kicks
  local bestClearKick = -1
  local bestBlockedKick = -1
  -- lower is better
  local bestClearRank = 1000000
  local bestBlockedRank = 1000000

  -- get current strategy
  local strategy
  if (behavior_mem.ballIsInCorner) then
    strategy = behavior_params.cornerStrategy
  else
    strategy = behavior_params.mainStrategy
  end

  -- if on goal line, possibly do dribble first
  bestClearKick  = checkGoalLineDribble()
  
  if (bestClearKick == -1) and behavior_mem.isInCluster then
    bestClearKick = checkClusterSideKicks(strategy,behavior_params.clusterStrategy,foot_choice)
  end
  
  if (bestClearKick == -1) and behavior_mem.isInCluster then
    bestClearKick = checkClusterQuickKicks(strategy,behavior_params.clusterStrategy,foot_choice)
  end

  -- loop through kicks, if we didn't already find one
  if (bestClearKick == -1) then
    for i = 0, core.NUM_KICKS-1 do

      UTdebug.log(80,"Kick",i,"distance",kicks.kickData[i].distance,"heading",RAD_T_DEG*kicks.kickData[i].heading,"set",kicks.kickData[i].isSet,"usable",strategy:getUsableKick(i))

      -- ignore kicks we're not using
      -- and they're not slow or slow kicks are ok
      if (strategy:getUsableKick(i) and kicks.kickData[i].isSet) then

        local valid, rank, clear = isKickChoiceInKickRegion(i)

        if (localization_mem.oppositeModels) then
          valid = false
          UTdebug.log(10, "Models predicting opposite directions!, no valid kick")
        end

        UTdebug.log(30, "Kick choice, valid, rank, clear", i, valid, rank, clear)

        -- check if its on this foot
        if (valid) then
          --if (foot_choice == core.Kick_SWITCHABLE and not kicks.kickData[i].switchable) then
            --valid = 0
            --UTdebug.log(10, "Kick ", i, "not valid because its not a switchable kick")
          --elseif (foot_choice ~= core.Kick_SWITCHABLE and kicks.kickData[i].foot ~= foot_choice and not kicks.kickData[i].switchable) then

          local relBall = core.Point2D(localizationC.filtered_close_ball_.x,localizationC.filtered_close_ball_.y)
          if (foot_choice ~= core.Kick_SWITCHABLE and kicks.kickData[kickInd].foot ~= foot_choice and not kicks.kickData[kickInd].switchable and (relBall.x < 150) and (math.abs(relBall.y) > 20)) then
            valid = 0
            UTdebug.log(10, "Kick ", i, "not valid because it a ", kicks.kickData[i].foot , "kick and approach wants a ", foot_choice)
          end
        end

        -- best clear kick
        if (valid == 1 and clear == 1 and rank < bestClearRank) then
          bestClearKick = i
          bestClearRank = rank
        end

        -- best blocked
        if (valid == 1 and rank < bestBlockedRank) then
          bestBlockedKick = i
          bestBlockedRank = rank
        end

      end -- usable kick
    end -- kick loop
  end -- kick not already found

  local kickFound = (bestClearKick ~= -1)
  local bestKick = bestClearKick
  local bestRank = bestClearRank

  -- no kick, return possibly blocked kick
  if (not kickFound) then
    --UTdebug.log(0, "No clear kick, going with best blocked kick", bestClearKick, bestBlockedKick)
    bestKick = bestBlockedKick
    bestRank = bestBlockedRank
  end

  -- in case there were no usable kicks
  if (bestKick == -1) then
    -- default kick
    bestKick = behavior_params.mainStrategy.defaultKick
  end

  local data = kicks.kickData[bestKick]

  --  --UTdebug.log(0, "Selected kick, heading, distance", kickFound, robot:getString(core.kickNames,bestKick), data.heading, data.distance)

  behavior_mem.kickChoice = bestKick
  behavior_mem.kickDist = data.distance
  behavior_mem.kickRank = bestRank
  behavior_mem.kickHeading = data.heading
  behavior_mem.chooseKick = kickFound

  -- return if we found a best kick, its index, heading, distance, y, switchable
  --return kickFound, bestKick, bestBlockedKick, data.heading, data.distance, data.yOffset, data.switchable, data.foot
  return kickFound, bestKick, bestBlockedKick, data

end




function isKickInKickRegion(location,orientation, distance, angle, whichVariation, kickIndex)
  local globalKickHeading = core.normalizeAngle(angle+orientation)
  local newX = location.x + (distance*math.cos(globalKickHeading));
  local newY = location.y + (distance*math.sin(globalKickHeading));
  local valid = 0
  local goal = 0
  local output = behaviorC:isInKickRegion(newX,newY,globalKickHeading)

  if (output > 0) then
    valid = 1
  end

  if (output > 1) then
    goal = 1
  end

  -- get dist to goal, teammates and opponents
  local rank = 0
  local finalPt = core.Point2D(newX, newY)

  -- possibly get rank values for the kick
  --rank = getTargetPointRank(finalPt)

  if (whichVariation == 1) then
    behavior_mem:setKickRightEval(kickIndex, finalPt, globalKickHeading, output)
  elseif (whichVariation == 0) then
    behavior_mem:setKickLeftEval(kickIndex, finalPt, globalKickHeading, output)
  elseif (whichVariation == 2) then
    behavior_mem:setKickStrongEval(kickIndex, finalPt, globalKickHeading, output)
  elseif (whichVariation == 3) then
    behavior_mem:setKickWeakEval(kickIndex, finalPt, globalKickHeading, output)
  end

  return valid, rank
end


function getTargetPointRank(finalPt)
  local opp_goal = world_objects:getObjPtr(core.WO_OPP_GOAL)
  local own_goal = world_objects:getObjPtr(core.WO_OWN_GOAL)

  -- if not goal, rank is distance to goal
  if (goal == 1) then
    rank = -10000000
  else
    -- dist to opponent goal
    rank = finalPt:getDistanceTo(opp_goal.loc)

    -- subtract distance to own goal (important at near end)
    rank = rank - finalPt:getDistanceTo(own_goal.loc)

    -- distance to other robots matters 1/2 as much as distance to goal
    local mateFactor = 0.5

    -- also add distance to nearest teammate (with some factor)
    rank = rank + getDistanceToClosestMate(finalPt) * mateFactor

    -- subtract distance to nearest opponent (with some factor)
    rank = rank - behaviorC:getOpponentDistance(finalPt) * mateFactor
  end

  return rank
end

function isKickChoiceInKickRegion(kickChoice)
  local ball = world_objects:getObjPtr(core.WO_BALL)
  local me = world_objects:getObjPtr(robot_state.WO_SELF)

  local occupancyGridThreshold = 0.2;

  local sdFactor
  if (behavior_mem.ballIsInCorner) then
    sdFactor = behavior_params.cornerStrategy.orientationErrorFactor
  else
    sdFactor = behavior_params.mainStrategy.orientationErrorFactor
  end

  local sdError = me.sdOrientation/sdFactor
  -- what pct error to add on to kick distance
  local pctDistError = 0.225


  -- min sdError of 0.05
  if (sdError < 0.05) then
    sdError = 0.05
  end

  -- larger min error for non-straight walk kicks
  if (kickChoice == core.WalkKickLeftward or kickChoice == core.WalkKickRightward or kickChoice == core.WalkKickLeftwardSide or kickChoice == core.WalkKickRightwardSide) then
    if (sdError < 0.1) then
      sdError = 0.1
    end
  end

    --UTdebug.log(10, "sdOrient", RAD_T_DEG*me.sdOrientation, "sdFactor", sdFactor, "sdError", RAD_T_DEG*sdError)


  local opp = 0
  local left = 1
  local right = 1
  local strong = 1
  local weak = 1
  local rankLeft = 0
  local rankRight = 0
  local rankStrong = 0
  local rankWeak = 0
  local totalRank = 0
  local totalValid = 0

  -- check left valid and rank
  left, rankLeft = isKickInKickRegion(ball.loc, core.normalizeAngle(me.orientation+sdError), kicks.kickData[kickChoice].distance, kicks.kickData[kickChoice].heading, 0,kickChoice)
  totalRank = rankLeft*2
  totalValid = left

  -- only check right if left was ok
  -- or in sim, for debug purposes
  if (left == 1 or vision_frame_info.source == core.MEMORY_SIM) then
    right, rankRight = isKickInKickRegion(ball.loc, core.normalizeAngle(me.orientation-sdError), kicks.kickData[kickChoice].distance, kicks.kickData[kickChoice].heading, 1,kickChoice)
    totalRank = rankLeft + rankRight
    totalValid = totalValid + right
  else
    behavior_mem:setKickRightEval(kickChoice, core.Point2D(0, 0), 0, -1)
  end

  -- only check strong kick if left and right were ok
  if (totalValid == 2 or vision_frame_info.source == core.MEMORY_SIM) then
    strong, rankStrong = isKickInKickRegion(ball.loc, me.orientation, (1+pctDistError)*kicks.kickData[kickChoice].distance, kicks.kickData[kickChoice].heading, 2,kickChoice)
    totalRank = totalRank + rankStrong
    totalValid = totalValid + strong
  else
    behavior_mem:setKickStrongEval(kickChoice, core.Point2D(0, 0), 0, -1)
  end
  

  -- only check weak if others were valid
  if (totalValid == 3 or vision_frame_info.source == core.MEMORY_SIM) then
    weak, rankWeak = isKickInKickRegion(ball.loc, me.orientation, (1-pctDistError)*kicks.kickData[kickChoice].distance, kicks.kickData[kickChoice].heading, 3,kickChoice)
    totalRank = totalRank + rankWeak
    totalValid = totalValid + weak
  else
    behavior_mem:setKickWeakEval(kickChoice, core.Point2D(0, 0), 0, -1)
  end


  -- if both were ok, then check opponents on middle
  if (totalValid == 4 or vision_frame_info.source == core.MEMORY_SIM) then
    local globalKickHeading = core.normalizeAngle(kicks.kickData[kickChoice].heading+me.orientation)
    local newX = ball.loc.x + (kicks.kickData[kickChoice].distance*math.cos(globalKickHeading));
    local newY = ball.loc.y + (kicks.kickData[kickChoice].distance*math.sin(globalKickHeading));
    local target = core.Point2D(newX, newY)

    -- opponent check based on world objects
    local oppCheck = false
    -- allow dribble into opponents
    if (kickChoice ~= core.Dribble) then
      oppCheck = behaviorC:opponentWorldObjectCheck(kicks.kickData[kickChoice].distance, kicks.kickData[kickChoice].heading, target)
    end

    behavior_mem:setKickOpponent(kickChoice,oppCheck)

    if (not oppCheck) then
      opp = 1
    else
      UTdebug.log(70, "Kick invalidated by opponents", kickChoice, oppCheck)
      -- add a penalty if it goes through an opponent
      -- assuming no opponent block gets us 2.5 m closer to goal
      totalRank = totalRank + 2500
    end
  end

  behavior_mem:setEvaluated(kickChoice, true)

  UTdebug.log(70, "Kick in Region: left, right, weak, strong, finalAnswer, totalRank ",left,right,weak,strong,(totalValid == 4), totalRank)

  -- return valid, rank, clear from opponents
  if (totalValid==4) then
    return 1, totalRank, opp
  else
    return 0, totalRank, opp
  end
end

-- find distance from kick pt to closest active teammate (including self)
function getDistanceToClosestMate(kickPt)
  local minDist = 10000
  local closestMate = -1

  for i=core.WO_TEAM_FIRST,core.WO_TEAM_LAST do
    if isTeammateActive(i,true,true) then
      -- get distance
      local mate = world_objects:getObjPtr(i)
      local dist = kickPt:getDistanceTo(mate.loc)
      if (dist < minDist) then
        minDist = dist
        closestMate = i
      end
    end
  end

  UTdebug.log(90, "Closest mate to kick point is ", closestMate, " with distance ", minDist)
  return minDist
end

function isTeammateActive(i,allowSelf,checkState)
  if (not allowSelf) and (i == robot_state.WO_SELF) then
    return false
  end
  local tp = team_packets:getPktPtr(i)
  local frameRecv = team_packets:getFrameReceived(i)
  if checkState and (tp.bvrData.state ~= core.PLAYING) then
    return false
  end
  return (not tp.bvrData.fallen) and ((vision_frame_info.frame_id - frameRecv) <= 30)
end

function setPassingKicks()
  local ball = world_objects:getObjPtr(core.WO_BALL)
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  local opp_goal = world_objects:getObjPtr(core.WO_OPP_GOAL)

  -- go through teammates
  for i=core.WO_TEAM_FIELD_FIRST,core.WO_TEAM_LAST do
    local kickIndex = kicks.Passes[i]
    local tp = team_packets:getPktPtr(i)
    local frameRecv = team_packets:getFrameReceived(i)
    
    if isTeammateActive(i,false,true) then
      -- lets target a point in front of teammate
      -- it's pass distance from mate towards opp goal
      local mate = world_objects:getObjPtr(i)
      local targetPt = mate.loc
      local relPoint = core.Point2D(behavior_params.mainStrategy.passDistance, mate.loc:getAngleTo(opp_goal.loc), core.POLAR)
      local newTargetPt = targetPt + relPoint

      local distance = ball.loc:getDistanceTo(newTargetPt)
      local heading = ball.loc:getBearingTo(newTargetPt,me.orientation)
      UTdebug.log(80, "Pass to mate", i, "at loc", mate.loc, " target:",newTargetPt,distance,heading*RAD_T_DEG,mate.loc:getAngleTo(opp_goal.loc)*RAD_T_DEG)

      setKick(kickIndex, distance, heading, false)
    else
      UTdebug.log(10, "No pass to mate ", i, " because self, not playing, fallen, or no messages", tp.bvrData.state, tp.bvrData.fallen, (vision_frame_info.frame_id - frameRecv))
      kicks.kickData[kickIndex].isSet = false
      --kicks.kickData[kickIndex] = kicks.KickInfo.create(true, 0, 3000, 40, 2.0, core.Kick_RIGHT, false)
    end
  end
end


function setCornerShots()
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  local ball = world_objects:getObjPtr(core.WO_BALL)
  local target = behavior_params.cornerStrategy.targetPt

  local distance = ball.loc:getDistanceTo(target)
  local heading = ball.loc:getBearingTo(target,me.orientation)

  UTdebug.log(10, "small gap corner target", target, distance, heading*RAD_T_DEG)

  -- to point on top of box
  setKick(core.FwdLongSmallGapKick, distance, heading, false)

  -- nearer point
  local nearTarget = core.Point2D(target.x, target.y)
  if (ball.loc.y > 0) then
    nearTarget.y = (core.GOAL_Y/2.0) - 200
  else
    nearTarget.y = -(core.GOAL_Y/2.0) + 200
  end
  distance = ball.loc:getDistanceTo(nearTarget)
  heading = ball.loc:getBearingTo(nearTarget,me.orientation)
  UTdebug.log(10, "large gap corner target", nearTarget, distance, heading*RAD_T_DEG)

  setKick(core.FwdLongLargeGapKick, distance, heading, false)

end


function setGoalGapKicks()
  local ball = world_objects:getObjPtr(core.WO_BALL)
  local goal = world_objects:getObjPtr(core.WO_OPP_GOAL)

  local goalDist = ball.loc:getDistanceTo(goal.loc)

  -- if goal is farther than 3.5 m, set that as distance
  -- most likely making kick invalid (farther than we can kick)
  local kickDist = 7000
  if ((goalDist+200) > kickDist) then
    kickDist = goalDist+200
  end

  if (math.abs(behavior_mem.largeGapHeading) < 7.0) then
    setKick(core.FwdLongLargeGapKick, kickDist, 0, true)
  else
    setKick(core.FwdLongLargeGapKick, kickDist, behavior_mem.largeGapHeading, true)
  end

  if (math.abs(behavior_mem.smallGapHeading) < 7.0) then
    setKick(core.FwdLongSmallGapKick, kickDist, 0, true)

  else
    setKick(core.FwdLongSmallGapKick, kickDist, behavior_mem.smallGapHeading, true)
  end

end


function setKick(index, distance, heading, maxDist)

  -- left foot for rigtward kick
  local foot = core.Kick_LEFT
  -- make it right foot for leftward kick
  if (heading > 0) then
    foot = core.Kick_RIGHT
  end

  -- if its a small angle, still switchable
  local switch = false
  if (math.abs(heading) < DEG_T_RAD * 5.0) then
    switch = true
  end


  -- is this kick feasible (within maxKickAngle deg, less than 3.5m)
  local validKick = (math.abs(heading) <= (behavior_params.mainStrategy.maxKickAngle+0.001) and distance < 7001 and distance > 849)

  -- even though we only planned on 3500, for goal kicks, lets request 5000
  if (maxDist) then
    distance = 7000
  end

  -- set (switch,heading,distance,desY,time) for goal ward kick
  kicks.kickData[index].switchable = switch
  kicks.kickData[index].heading = heading
  kicks.kickData[index].distance = distance
  kicks.kickData[index].foot = foot
  kicks.kickData[index].time = 2.0
  kicks.kickData[index].isSet = validKick

  --kicks.kickData[index] = kicks.KickInfo.create(switch, heading, distance, 50, 2.0, foot, validKick)
end



function initKickRegion()
  local xInd, yInd, x, y
  for xInd = 0,core.NUM_KICK_REGIONS_X-1 do
    x = xInd * core.KICK_REGION_SIZE_X - (core.FIELD_X / 2.0)
    for yInd = 0,core.NUM_KICK_REGIONS_Y-1 do
      y = yInd * core.KICK_REGION_SIZE_Y - (core.FIELD_Y / 2.0)

      -- center point of region is x,y
      behavior_mem:setValidKickRegion(xInd,yInd,isXYInKickRegion(x,y))
    end
  end
end

function isXYInKickRegion(x,y)
  local strategy = behavior_params.mainStrategy
  -- angle from offensive posts
  local post = world_objects:getObjPtr(core.WO_OPP_LEFT_GOALPOST)
  local insidePostLoc = core.Point2D(post.loc.x, post.loc.y - strategy.insidePostBuffer)
  local m = calculateSlopeFromPointAngle(insidePostLoc,math.pi - strategy.postAngle)
  local dx = (math.abs(y) - insidePostLoc.y) / m
  --  local linePost = core.Line2D(insidePostLoc,math.pi - strategy.postAngle)
  -- ball in corner
  if (x > insidePostLoc.x + dx) then
    return false
  end

  -- off back edge of field (in case behindAngle is ever negative)
  if (x < (-core.FIELD_X / 2.0)) then
    return false
  end

  -- not near sidelines
  if (math.abs(y) > (core.FIELD_Y / 2.0 - strategy.edgeBuffer)) then
    return false
  end

  -- not near own goal
  local ownGoal = world_objects:getObjPtr(core.WO_OWN_GOAL)
  if (ownGoal.loc:getDistanceTo(core.Point2D(x,y)) < strategy.ownGoalRadius) then
    return false
  end
  return true
end

function calculateSlope(pt1,pt2)
  local dx = pt2.x - pt1.x
  if math.abs(dx) < 0.01 then
    dx = 0.01
  end
  local slope = (pt2.y - pt1.y) / dx
  return slope
end

function calculateSlopeFromPointAngle(pt,angle)
  local pt2 = pt + core.Point2D(100,angle,core.POLAR)
  return calculateSlope(pt,pt2)
end

function setKickAngles()
  -- set angle kicks lower (but not side kicks
  local maxAngle = behavior_params.mainStrategy.maxKickAngle
  local eps = 0.001
  for i = 0, core.NUM_KICKS-1 do
    if (kicks.kickData[i].heading > maxAngle + eps and kicks.kickData[i].heading < DEG_T_RAD*40.0) then
      UTdebug.log(0, "set kick with heading", kicks.kickData[i].heading*RAD_T_DEG, " to heading ", maxAngle*RAD_T_DEG)
      kicks.kickData[i].heading = maxAngle
    elseif (kicks.kickData[i].heading < -maxAngle - eps and kicks.kickData[i].heading > DEG_T_RAD*-40.0) then
      UTdebug.log(0, "set kick with heading", kicks.kickData[i].heading*RAD_T_DEG, " to heading ", -maxAngle*RAD_T_DEG)
      kicks.kickData[i].heading = -maxAngle
    end
  end
end

function checkKick(kickInd,strategy,foot_choice,bestClearKick,bestBlockedKick,ignoreUsability)
  -- ignore kicks we're not using
  -- and they're not slow or slow kicks are ok
  local usable = ignoreUsability or strategy:getUsableKick(kickInd)
  if (usable and kicks.kickData[kickInd].isSet) then
    
    local valid, rank, clear = isKickChoiceInKickRegion(kickInd)

    if (localization_mem.oppositeModels) then
      valid = false
      UTdebug.log(10, "Models predicting opposite directions!, no valid kick")
    end

    UTdebug.log(90, "Kick choice, valid, rank", kickInd, valid, rank)

    -- check if its on this foot
    if (valid) then
      -- sbarrett, why can't we switch from switchable to other foot?
      --if (foot_choice == core.Kick_SWITCHABLE and not kicks.kickData[kickInd].switchable) then
        --valid = 0
        --UTdebug.log(10, "Kick ", kickInd, "not valid because its not a switchable kick")

      local relBall = core.Point2D(localizationC.filtered_close_ball_.x,localizationC.filtered_close_ball_.y)
      -- sbarrett, still allow a switch if we're far from the ball, or if the y is fairly centered
      if (foot_choice ~= core.Kick_SWITCHABLE and kicks.kickData[kickInd].foot ~= foot_choice and not kicks.kickData[kickInd].switchable and (relBall.x < 150) and (math.abs(relBall.y) > 20)) then
        valid = 0
        UTdebug.log(10, "Kick ", kickInd, "not valid because it a ", kicks.kickData[kickInd].foot , "kick and approach wants a ", foot_choice)
      end
    end

    -- best clear kick
    if (valid == 1 and clear == 1) then
      bestClearKick = kickInd
    end

    -- return on first valid clear kick
    --if (valid == 1 and clear == 1) then break end

    -- valid but not clear from opponents
    if (valid == 1 and bestBlockedKick == -1) then
      bestBlockedKick = kickInd
    end

  end -- usable kick

  return bestClearKick,bestBlockedKick
end

function checkClusterQuickKicks(strategy,clusterStrategy,foot_choice)
  if (clusterStrategy.behavior ~= core.Cluster_QUICK) then
    return -1
  end
  
  local clearKick, blockedKick
  clearKick,blockedKick = checkKick(core.WalkKickFront,strategy,foot_choice,-1,-1,true)
  if (clearKick == -1) then
    clearKick = blockedKick
  end
  return clearKick
end

function setTeammateSide()
  -- go through teammates
  for i=core.WO_TEAM_FIELD_FIRST,core.WO_TEAM_LAST do
    local tp = team_packets:getPktPtr(i)
    local frameRecv = team_packets:getFrameReceived(i)
    
    -- only teammates, and playing, not fallen, sent messages recently
    if (i ~= robot_state.WO_SELF and tp.bvrData.state == core.PLAYING and (not tp.bvrData.fallen) and (vision_frame_info.frame_id - frameRecv) < 30) then
      -- if within 1.25 meters of ball
      local mate = world_objects:getObjPtr(i)
      if (mate.distance < 1250) then
        -- get bearing
        if (math.abs(math.pi/2.0 - mate.bearing) < math.pi/4.0) then
          UTdebug.log(10, "teammate ", i, " is on our LEFT at distance ", mate.distance, " and bearing", mate.bearing * RAD_T_DEG)
          return 1
        elseif (math.abs(-math.pi/2.0 - mate.bearing) < math.pi/4.0) then
          UTdebug.log(10, "teammate ", i, " is on our RIGHT at distance ", mate.distance, " and bearing", mate.bearing * RAD_T_DEG)
          return -1
        end
      else
        UTdebug.log(10, "teammate", i, " at distance ", mate.distance, " and bearing ", mate.bearing *RAD_T_DEG)
      end
      
    end
  end
  return 0
end


function checkClusterSideKicks(strategy,clusterStrategy,foot_choice)
  if (clusterStrategy.behavior ~= core.Cluster_SIDEKICK) then
    return -1
  end
  behavior_mem.kickTestUsingCluster = true
  UTdebug.log(10,'checking cluster kicks')
  local clearKick, temp
  local first = core.WalkKickLeftwardSide
  local second = core.WalkKickRightwardSide
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  -- decide whether to prefer left or right, first based on teammate side
  local mateSide = setTeammateSide()
  if (mateSide == 1) then
    -- do nothing, already prefer leftward
  elseif (mateSide == -1) then
    first = core.WalkKickRightwardSide
    second = core.WalkKickLeftwardSide
  else
    -- no mate, decide based on y location
    if me.loc.y > 0 then
      first = core.WalkKickRightwardSide
      second = core.WalkKickLeftwardSide
    end
  end

  clearKick, temp = checkKick(first,strategy,foot_choice,-1,-1,true)
  --UTdebug.log(10,luaC:getString(core.kickNames,first),clearKick)
  if clearKick ~= -1 then
    --if first == core.WalkKickLeftwardSide then
      --UTdebug.log(0,'left one')
      ----speech:say('left one')
    --else
      --UTdebug.log(0,'right one')
      ----speech:say('right one')
    --end
  else
    clearKick, temp = checkKick(second,strategy,foot_choice,-1,-1,true)
    --UTdebug.log(10,luaC:getString(core.kickNames,second),clearKick)
    --if clearKick ~= -1 then
      --if second == core.WalkKickLeftwardSide then
        --UTdebug.log(0,'left two')
        ----speech:say('left two')
      --else
        --UTdebug.log(0,'right two')
        ----speech:say('right two')
      --end
    --end
  end
  behavior_mem.kickTestUsingCluster = false
  return clearKick
end

function resetPassInfo()
  behavior_mem.passInfo.executingPass = false
  behavior_mem.passInfo.targetPlayer = -1
end

function setPassInfo(kickChoice,kickHeading,kickDistance,time)
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  local ball = world_objects:getObjPtr(core.WO_BALL)
  local globalKickHeading = core.normalizeAngle(me.orientation + kickHeading)
  behavior_mem.passInfo.target.x = ball.loc.x + kickDistance * math.cos(globalKickHeading)
  behavior_mem.passInfo.target.y = ball.loc.y + kickDistance * math.sin(globalKickHeading)
  behavior_mem.passInfo.kickChoice = kickChoice
  behavior_mem.timeUntilPass = time
  behavior_mem.passInfo.executingPass = true

  local minDist = 999999
  local targetPlayer = -1
  for i=core.WO_TEAM_FIELD_FIRST,core.WO_TEAM_LAST do
    if isTeammateActive(i,true,true) then
      local mate = world_objects:getObjPtr(i)
      local dist = behavior_mem.passInfo.target:getDistanceTo(mate.loc)
      if mate.loc.x > behavior_mem.passInfo.target.x then
        dist = dist * behavior_params.passStrategy.upfieldPenaltyFactor -- prefer somebody behind the ball
      end
      if dist < minDist then
        minDist = dist
        targetPlayer = i
      end
    end
  end

  behavior_mem.passInfo.targetPlayer = targetPlayer
  --local team = 'blue'
  --if robot_state.team_ == 1 then
    --team = 'red'
  --end
  --print('pass: ' .. tostring(robot_state.WO_SELF) .. ' ' .. team .. ' ' .. tostring(time) .. ' ' .. tostring(targetPlayer) .. ' ' .. tostring(minDist))
end

function estimateTimeToReachBall(kickChoice)
  --local ball = world_objects:getObjPtr(core.WO_BALL)
  local speedMax = walk_param.bh_params_.speedMax * 2.0 -- *2 because 2 steps per sec
  local xTime = behavior_mem.targetPt.x / speedMax.translation.x
  local yTime = behavior_mem.targetPt.y / speedMax.translation.y
  
  local time
  if xTime > yTime then
    time = xTime
  else
    time = yTime
  end

  if kickChoice == core.WalkKickFront then
    time = time - 0.20
  end

  return time
end

function getExpectedPassInfo()
  for i = core.WO_TEAM_FIRST,core.WO_TEAM_LAST do
    if isTeammateActive(i,false,true) then
      local passInfo = team_packets:getPktPtr(i).bvrData.passInfo
      if passInfo.executingPass and (passInfo.targetPlayer == robot_state.WO_SELF) then
        return passInfo
      end
    end
  end
  return nil
end

function selectSetPlay()
  if not(game_state.ourKickOff) then
    behavior_mem.setPlayInfo.type = core.SetPlay_none
    return
  end

  local timeInPlaying = vision_frame_info.seconds_since_start - behavior_mem.timePlayingStarted
  -- only choose set play in set
  if (game_state.state ~= core.SET) then
    -- reset set play if not in set or beginning of playing
    if (game_state.state ~= core.PLAYING) or (timeInPlaying > behavior_params.setPlayStrategy.maxTimeInPlaying) or not(behavior_mem.isKickOffShot) then
      behavior_mem.setPlayInfo.type = core.SetPlay_none
    end
    return
  end
  
  

  if robot_state.role_ == core.CHASER then
    local prevSetPlay = behavior_mem.setPlayInfo.type
    local prevReversed = behavior_mem.setPlayInfo.reversed
    behavior_mem.setPlayInfo.type = core.SetPlay_none
    -- chaser in SET
    local me = world_objects:getObjPtr(robot_state.WO_SELF)
    local ball = world_objects:getObjPtr(core.WO_BALL)
    if (me.loc:getDistanceTo(ball.loc) > behavior_params.setPlayStrategy.maxDistFromBall) then 
      -- too far for set play
      return
    end
    -- look through the set plays
    local maxScore = 0
    for playNum = 0,core.SetPlayStrategy_NUM_SET_PLAYS-1 do
      local play = behavior_params.setPlayStrategy:getPlayPtr(playNum)
      if play.active then
        local score
        local targetPlayer
        for i = 0,1 do
          local reversed = (i == 1)
          if not(reversed) or play.reversible then
            local preference = math.random()
            if (playNum == prevSetPlay) and (reversed == prevReversed) then
              preference = 1.1
            end
            score,targetPlayer = evalSetPlay(play,reversed,preference)
            if score > maxScore then
              maxScore = score
              behavior_mem.setPlayInfo.type = playNum
              behavior_mem.setPlayInfo.reversed = reversed
              behavior_mem.setPlayInfo.targetPlayer = targetPlayer
            end
          end
        end
      end
    end
  else
    -- use the set play the chaser sent out or none otherwise
    behavior_mem.setPlayInfo.type = core.SetPlay_none
    for i = core.WO_TEAM_FIRST,core.WO_TEAM_LAST do
      local bvrData = team_packets:getPktPtr(i).bvrData
      if isTeammateActive(i,false,false) and (bvrData.role == core.CHASER) then
        behavior_mem.setPlayInfo = bvrData.setPlayInfo
        return
      end
    end 
  end
end



function evalSetPlay(play,reversed,pref)
  local score = -1
  -- currently all the same if the target exists
  local desiredGoalBearing = play.desiredGoalBearing
  if reversed then
    desiredGoalBearing = -desiredGoalBearing
  end  

  local target = -1
  local dist = behavior_params.setPlayStrategy.maxTargetDistFromLine
  for i = core.WO_TEAM_FIRST,core.WO_TEAM_LAST do
    local mate = world_objects:getObjPtr(i)
    if isTeammateActive(i,false,false) and  -- active
        (mate.loc.x > -behavior_params.setPlayStrategy.maxTargetDistFromLine) and -- close to center line
        (mate.loc.y * -desiredGoalBearing > 0) then -- on the correct side
      local d = math.abs(mate.loc.x) -- dist from line
      if (d < dist) then
        target = i
        dist = d
      end
    end
  end
  
  local ball = world_objects:getObjPtr(core.WO_BALL)
  local globalKickHeading = core.normalizeAngle(kicks.kickData[play.kickType].heading - desiredGoalBearing)
  local newX = ball.loc.x + (kicks.kickData[play.kickType].distance*math.cos(globalKickHeading));
  local newY = ball.loc.y + (kicks.kickData[play.kickType].distance*math.sin(globalKickHeading));
  local kickTarget = core.Point2D(newX, newY)
  local oppBlockingKick = behaviorC:opponentWorldObjectCheck(kicks.kickData[play.kickType].distance, globalKickHeading, kickTarget)

  pref = pref * 25 -- scale between 0 and 25

  if play.requiresTargetPlayer then
    -- pass
    if target < 0 then
      return -1,-1
    end

    local mate = world_objects:getObjPtr(target)
    local receiverDist = mate.loc:getDistanceTo(kickTarget)
    local receiverBearing = mate.loc:getAngleTo(kickTarget)
    local oppBlockingReceiver = behaviorC:opponentWorldObjectCheckGivenStartLoc(receiverDist, receiverBearing, kickTarget,mate.loc)

    local distFactor = 25 * (1 - dist / behavior_params.setPlayStrategy.maxTargetDistFromLine)
    local score = 125 + distFactor + pref -- prefer pass to bury deep, prefer closer receiver, prefer randomness
    
    if oppBlockingKick then
      score = score - 50
    end
    if oppBlockingReceiver then
      score = score - 50
    end
    return score,target
  else
    -- not pass
    if oppBlockingKick then
      return 50 + pref,target
    else
      return 100 + pref,target
    end
  end
end
