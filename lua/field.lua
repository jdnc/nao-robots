require 'strategy'

module(..., package.seeall);


function processFrame()

  ballIsInCorner()
  robotIsInCorner()
  isKickOffShot()

  if (ballIsInCorner()) then
    calcCornerShots()
  else
    calcGoalGaps()
  end

  UTdebug.log(10, "Ball in corner:", behavior_mem.ballIsInCorner, "robot in corner:", behavior_mem.robotIsInCorner, "kickoffshot:", behavior_mem.isKickOffShot, "largeGap:", RAD_T_DEG*behavior_mem.largeGapHeading, "smallGap:", RAD_T_DEG*behavior_mem.smallGapHeading)
  if (localization_mem.blueSide == core.Sides_NOSIDES) then
    UTdebug.log(10, "No blue/white side info", localization_mem.blueSide)
  elseif (localization_mem.blueSide == core.Sides_RIGHTSIDE) then
    UTdebug.log(10, "Blue on RIGHT side of field", localization_mem.blueSide)
  elseif (localization_mem.blueSide == core.Sides_LEFTSIDE) then
    UTdebug.log(10, "Blue on LEFT side of field", localization_mem.blueSide)
  else
    UTdebug.log(10, "Blue side set to unknown value", localization_mem.blueSide)
  end

end


function ballIsInCorner()
  -- angle from offensive posts
  local ball = world_objects:getObjPtr(core.WO_BALL)
  local post = world_objects:getObjPtr(core.WO_OPP_LEFT_GOALPOST)
  local postLoc = core.Point2D(post.loc.x, post.loc.y)

  -- near post does not count as corner
  local rightPost = world_objects:getObjPtr(core.WO_OPP_RIGHT_GOALPOST)
  if (ball.loc:getDistanceTo(post.loc) < 250 or ball.loc:getDistanceTo(rightPost.loc) < 250) then
    UTdebug.log(10, "ball near post, not corner")
    behavior_mem.ballIsInCorner = false
    return
  end


  -- for hysterisis... if we were in corner last time, lets move threshold a bit
  -- to be more likely to say we're still in
  if (behavior_mem.ballIsInCorner) then
    postLoc.y = postLoc.y - 300
  end

  -- can't be corner if its past before 2 meters
  if (behavior_mem.ballIsInCorner and ball.loc.x < 1800) then
    behavior_mem.ballIsInCorner = false
    return
  end
  if (not behavior_mem.ballIsInCorner and ball.loc.x < 2000) then
    behavior_mem.ballIsInCorner = false
    return
  end

  local linePost = core.Line2D(postLoc,math.pi - behavior_params.cornerStrategy.cornerAngle)

  behavior_mem.ballIsInCorner = ball.loc.x > linePost:getXGivenY(math.abs(ball.loc.y))
end

function robotIsInCorner()
  -- angle from offensive posts
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  local post = world_objects:getObjPtr(core.WO_OPP_LEFT_GOALPOST)
  local linePost = core.Line2D(post.loc,math.pi - behavior_params.cornerStrategy.cornerAngle)
  if(behavior_params.cornerStrategy.cornerAngle < .000001) then
    return -- This function just errors out if the corner angle is too small
  end
  behavior_mem.robotIsInCorner = math.abs(me.loc.x) > linePost:getXGivenY(math.abs(me.loc.y))
end

function isKickOffShot()
  local inCircle = false

  if game_state.state == core.SET then
    behavior_mem.numFramesNotKickOff = 0
  end

  if (game_state.ourKickOff and not game_state.isPenaltyKick) then
    local ball = world_objects:getObjPtr(core.WO_BALL)

    if ((ball.loc):getMagnitude() < (core.CIRCLE_RADIUS + 200)) then
      -- still in circle
      --UTdebug.log(10, "P3, ball still in circle", ball.loc)
      inCircle = true
    end
  end

  if not inCircle then
    behavior_mem.numFramesNotKickOff = behavior_mem.numFramesNotKickOff + 1
    -- don't go back to thinking it's the kickOffShot if it's been more than 10 frames where we thought it wasn't
  end
  if behavior_mem.numFramesNotKickOff > 10 then
    inCircle = false
  end

  behavior_mem.isKickOffShot = inCircle

end


-- from corner, instead of goal gaps, we want kicks to top of box
-- maybe near and middle
function calcCornerShots()

  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  local ball = world_objects:getObjPtr(core.WO_BALL)

  local target = behavior_params.cornerStrategy.targetPt

  -- to point on top of box
  behavior_mem.smallGapHeading = ball.loc:getBearingTo(target, me.orientation)

  -- nearer point
  local nearTarget = core.Point2D(target.x, target.y)
  if (ball.loc.y > 0) then
    nearTarget.y = (core.GOAL_Y/2.0) - 200
  else
    nearTarget.y = -(core.GOAL_Y/2.0) + 200
  end
  behavior_mem.largeGapHeading = ball.loc:getBearingTo(nearTarget, me.orientation)

end


-- set kicks for near and far gap of goal if there's an opponent
-- if not, then do middle and near post
function calcGoalGaps()

  local lastLargeGap = behavior_mem.largeGapHeading
  local lastSmallGap = behavior_mem.smallGapHeading

  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  local ball = world_objects:getObjPtr(core.WO_BALL)

  -- make sure we don't have backward headings, so keep ball pt on field
  local originPt = core.Point2D(ball.loc.x, ball.loc.y)
  if (originPt.x > (core.FIELD_X / 2.0 - 450) and originPt.y < (core.FIELD_Y/2.0)) then
    originPt.x = (core.FIELD_X / 2.0 - 450)
  end

  local leftPost = world_objects:getObjPtr(core.WO_OPP_LEFT_GOALPOST)
  local rightPost = world_objects:getObjPtr(core.WO_OPP_RIGHT_GOALPOST)

  local leftPostBearing = originPt:getBearingTo(leftPost.loc, me.orientation)
  local rightPostBearing = originPt:getBearingTo(rightPost.loc, me.orientation)

  -- try to find an opponent between the two posts
  local foundKeeper = false
  local leftKeeperBearing = 0.0
  local rightKeeperBearing = 0.0
  for j=0,core.MAX_OPP_MODELS_IN_MEM-1 do

    -- if standard deviation is reasonable
    local oppSD = opponent_mem:getSD(j)

    if (opponent_mem:getAlpha(j) > 0 and oppSD.x < behavior_params.mainStrategy.maxOpponentSD and oppSD.y < behavior_params.mainStrategy.maxOpponentSD) then
      local oppLoc = opponent_mem:getLocation(j)
      local oppLoc1 = core.Point2D(oppLoc.x, oppLoc.y + behavior_params.mainStrategy.opponentWidth)
      local oppLoc2 = core.Point2D(oppLoc.x, oppLoc.y - behavior_params.mainStrategy.opponentWidth)

      local oppBearing1 = originPt:getBearingTo(oppLoc1, me.orientation)
      local oppBearing2 = originPt:getBearingTo(oppLoc2, me.orientation)

      if (oppBearing1 < leftPostBearing and oppBearing1 > rightPostBearing) then
        if (not foundKeeper) then
          leftKeeperBearing = oppBearing1
          rightKeeperBearing = oppBearing1
        else
          -- in case we see multiple opponents in the way, save the left most
          -- and right most bearings
          if (oppBearing1 > leftKeeperBearing) then
            leftKeeperBearing = oppBearing1
          end
          if (oppBearing1 < rightKeeperBearing) then
            rightKeeperBearing = oppBearing1
          end
        end
        foundKeeper = true
      end

      if (oppBearing2 < leftPostBearing and oppBearing2 > rightPostBearing) then
        if (not foundKeeper) then
          leftKeeperBearing = oppBearing2
          rightKeeperBearing = oppBearing2
        else
          -- in case we see multiple opponents in the way, save the left most
          -- and right most bearings
          if (oppBearing2 > leftKeeperBearing) then
            leftKeeperBearing = oppBearing2
          end
          if (oppBearing2 < rightKeeperBearing) then
            rightKeeperBearing = oppBearing2
          end
        end
        foundKeeper = true
      end

    end
  end

  -- set near and far gap kicks (only if up field
  if (foundKeeper and ball.loc.x > 900) then
    --UTdebug.log(0, "Found keeper at bearing", keeperBearing*RAD_T_DEG)

    local leftGapBearing = (leftKeeperBearing + leftPostBearing) / 2.0
    local rightGapBearing = (rightKeeperBearing + rightPostBearing) / 2.0

    local largeGapBearing = 0
    local smallGapBearing = 0

    -- in goal mouth, possible use 0 heading if its between the two by enough
    if (ball.loc.x > 2400 and math.abs(ball.loc.y) < (core.GOAL_Y / 2.0) - 200) then
      local leftThird = (leftPostBearing - leftKeeperBearing) / 3.0
      if ((leftPostBearing - leftThird) > 0 and (leftKeeperBearing+leftThird) < 0) then
        UTdebug.log(10, "0 is close enough to center of left gap, setting heading 0 instead of",leftGapBearing*RAD_T_DEG)
        leftGapBearing = 0
      end
      local rightThird = (rightKeeperBearing - rightPostBearing) / 3.0
      if ((rightPostBearing + rightThird) < 0 and (rightKeeperBearing-rightThird) > 0) then
        UTdebug.log(10, "0 is close enough to center of right gap, setting heading 0 instead of",rightGapBearing*RAD_T_DEG)
        rightGapBearing = 0
      end
    end

    -- is one of these gaps similar to last large gap
    local similarGap = false
    local similarIsLeft = false
    if (behavior_mem.foundKeeper and math.abs(leftGapBearing - lastLargeGap) < DEG_T_RAD * 5) then
      similarGap = true
      similarIsLeft = true
    elseif 
      (behavior_mem.foundKeeper and math.abs(rightGapBearing - lastLargeGap) < DEG_T_RAD * 5) then
      similarGap = true
      similarIsLeft = false
    end

    -- if we have a similar one from last time, make it harder to switch gaps
    local buffer = 0
    if (similarGap) then
      buffer = 12.0*DEG_T_RAD
    end

    UTdebug.log(10, "left gap size", RAD_T_DEG*(leftPostBearing - leftKeeperBearing), " riight gap size", RAD_T_DEG* (rightKeeperBearing - rightPostBearing), " large to left diff", RAD_T_DEG*math.abs(leftGapBearing - lastLargeGap), "large to right diff",  RAD_T_DEG*math.abs(rightGapBearing - lastLargeGap))

    if (math.abs((leftPostBearing - leftKeeperBearing) - (rightKeeperBearing - rightPostBearing)) < buffer) then
      if ((similarIsLeft and (leftPostBearing - leftKeeperBearing) < (rightKeeperBearing - rightPostBearing)) or ((not similarIsLeft) and (leftPostBearing - leftKeeperBearing) > (rightKeeperBearing - rightPostBearing))) then
        UTdebug.log(6, "Similar gap to last time, and small diff in gap size, use hysterisis", buffer*RAD_T_DEG, RAD_T_DEG* ((leftPostBearing - leftKeeperBearing) - (rightKeeperBearing - rightPostBearing)))
      end
    end


    if ((leftPostBearing - leftKeeperBearing) > (rightKeeperBearing - rightPostBearing + buffer)) then
      largeGapBearing = leftGapBearing
      smallGapBearing = rightGapBearing
    elseif ((leftPostBearing - leftKeeperBearing) < (rightKeeperBearing - rightPostBearing - buffer)) then
      largeGapBearing = rightGapBearing
      smallGapBearing = leftGapBearing
    else
      -- not big enough to change, keep using similar
      if (similarIsLeft) then
        largeGapBearing = leftGapBearing
        smallGapBearing = rightGapBearing
      else
        largeGapBearing = rightGapBearing
        smallGapBearing = leftGapBearing
      end
    end
      

    -- set these in memory
    behavior_mem.largeGapHeading = largeGapBearing
    behavior_mem.smallGapHeading = smallGapBearing
    behavior_mem.foundKeeper = true

    UTdebug.log(20, "large gap, small gap, left post, right post, keeper",largeGapBearing*RAD_T_DEG, smallGapBearing*RAD_T_DEG, leftPostBearing*RAD_T_DEG,rightPostBearing*RAD_T_DEG, leftKeeperBearing*RAD_T_DEG, rightKeeperBearing*RAD_T_DEG)

  else -- set center and near post
    
    local oppGoal = world_objects:getObjPtr(core.WO_OPP_GOAL)
    local centerBearing = originPt:getBearingTo(oppGoal.loc, me.orientation)

    -- in goal mouth, possible use 0 heading if its between the two by enough
    if (ball.loc.x > 2400 and math.abs(ball.loc.y) < (core.GOAL_Y / 2.0) - 200) then
      local third = (leftPostBearing - rightPostBearing) / 3.0
      if ((leftPostBearing - third) > 0 and (rightPostBearing+third) < 0) then
        UTdebug.log(10, "0 is close enough to center of goal, setting heading 0 instead of",centerBearing*RAD_T_DEG)
        centerBearing = 0
      end
    end

    local nearCorner
    if (ball.loc.y > 0) then
      nearCorner = core.Point2D(core.FIELD_X/2.0, core.GOAL_Y/2.0 - 400)
    else
      nearCorner = core.Point2D(core.FIELD_X/2.0, -core.GOAL_Y/2.0 + 400)
    end

    local nearCornerBearing = originPt:getBearingTo(nearCorner, me.orientation)

    behavior_mem.largeGapHeading = centerBearing
    behavior_mem.smallGapHeading = nearCornerBearing
    behavior_mem.foundKeeper = false

    UTdebug.log(20, "Set center and near corner", centerBearing*RAD_T_DEG, nearCornerBearing*RAD_T_DEG)
  end

end

