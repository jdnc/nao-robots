module(...,package.seeall)

-- Todd: just putting these params here for now
cfgSwitching = {}

-- how many recently must a robot have seen the ball to be considered for chasing
cfgSwitching.max_missed_frames = 90

-- how recent do we have to have seen a packet from teammate?
cfgSwitching.max_missed_packet_frames = 150 --90 --30

-- distance where we can't be called off the chase
cfgSwitching.max_calloff_distance = 100

-- how close players are before we'll prioritize on # rather than distance
-- (since distance may be noisy and we could end up repeatedly switching off)
cfgSwitching.close_distance = 150


function getBidsAndCommInfo(switchParams,tp,bid,dist,frameRecv)
  local hearSomeone = false
  local someoneChasing = false
  for i = core.WO_TEAM_FIRST, core.WO_TEAM_LAST do
    tp[i] = team_packets:getPktPtr(i)
    bid[i] = tp[i].bvrData.ballBid 
    dist[i] = tp[i].bvrData.ballDistance
    frameRecv[i] = team_packets:getFrameReceived(i)

    -- ready.. bid is just which # you are
    if (game_state.state == core.READY) then
      bid[i] = 5000 - i * 1000
      tp[i].bvrData.ballMissed = 1
      tp[i].bvrData.fallen = false
    end

    if (i == robot_state.WO_SELF) then
      frameRecv[i] = vision_frame_info.frame_id
    end

    if (i ~= robot_state.WO_SELF and (vision_frame_info.frame_id - frameRecv[i]) < switchParams.max_missed_packet_frames) then
      hearSomeone = true
    end

    -- dont count them if they haven't seen it in 150 frames
    -- or if they're not in playing state
    -- or if they're fallen
    -- or if we haven't heard from them in a while
    if ((tp[i].bvrData.ballMissed > switchParams.max_missed_frames)
        or (tp[i].bvrData.state ~= core.READY and tp[i].bvrData.state ~= core.PLAYING and tp[i].bvrData.state ~= core.SET)
        or tp[i].bvrData.fallen
        or (vision_frame_info.frame_id - frameRecv[i]) > switchParams.max_missed_packet_frames) then
      --print ('  here',i)
      --print ('    ',(game_state.state ~= core.READY) and (tp[i].bvrData.ballMissed > switchParams.max_missed_frames),tp[i].bvrData.state ~= core.READY and tp[i].bvrData.state ~= core.PLAYING and tp[i].bvrData.state ~= core.SET,tp[i].bvrData.fallen,(vision_frame_info.frame_id - frameRecv[i]) > switchParams.max_missed_packet_frames)
      bid[i] = 6000
      if (i == robot_state.WO_SELF) then
        bid[i] = 5999
      end
      if (i == robot_state.WO_SELF and i == core.KEEPER) then
        -- not keeper if he's having these issues.. he wont go into search
        bid[i] = 6001
      end
    end

    if (tp[i].bvrData.role == core.CHASER and i ~= robot_state.WO_SELF and bid[i] < 5900) then
      someoneChasing = true
    end
  end
  return hearSomeone,someoneChasing
end

function noCommRoles(switchParams,tp,bid)
  local ball = world_objects:getObjPtr(core.WO_BALL)

  if (robot_state.WO_SELF == core.KEEPER) then 
    -- if we see ball, and it's in the box or very close, chase
    if (tp[robot_state.WO_SELF].bvrData.ballMissed < switchParams.max_missed_frames) and ((bid[robot_state.WO_SELF] < 5) or (ball.distance < 500)) and (game_state.state == core.PLAYING) then
      robot_state.role_ = core.CHASER
      UTdebug.log(10, "Keeper: Could not hear ANYONE, but near ball, CHASE")
    else
      robot_state.role_ = robot_state.WO_SELF
      UTdebug.log(10, "Could not hear ANYONE, go to static role", robot_state.role_)
    end
    return
  end

  -- close, go for it
  if (tp[robot_state.WO_SELF].bvrData.ballMissed < switchParams.max_missed_frames) and (ball.distance  < 2000) and (game_state.state == core.PLAYING) then -- NOTE, probably should keep this above the difference between the starting positions and the ball
    robot_state.role_ = core.CHASER
    UTdebug.log(10, "Could not hear ANYONE, but near ball, CHASE")
  else 
    robot_state.role_ = robot_state.WO_SELF
    --if robot_state.WO_SELF == core.SUPPORTER then
      --robot_state.role_ = core.CAUTIOUS_DEFENDER
    --end
    UTdebug.log(10, "Could not hear ANYONE, go to static role", robot_state.role_)
  end
end

function roleSwitch()
  -- set which params we're using
  local switchParams = cfgSwitching
  -- setup some vars for getting all of the players' comms
  local tp = {}
  local bid = {}
  local dist = {}
  local frameRecv = {}
  local hearSomeone = false
  local someoneChasing = false
  
  -- get bid and other info for each player
  hearSomeone,someoneChasing = getBidsAndCommInfo(switchParams,tp,bid,dist,frameRecv)
  if (robot_state.ignore_comms_) then
    hearSomeone = false
  end

  -- if we didnt hear anyone... do static roles
  if (not hearSomeone) then
    noCommRoles(switchParams,tp,bid)
    return
  end
  
  -- figure out who is closest to ball
  local closest = getMinInd(bid)

  UTdebug.log(10,"we have bid ", bid[robot_state.WO_SELF], " dist: ", dist[robot_state.WO_SELF], " closest is ", closest, " with bid ", bid[closest], " dist: ", dist[closest], " missed: ", tp[closest].bvrData.ballMissed, " state: ",  tp[closest].bvrData.state, " fallen: ", tp[closest].bvrData.fallen, " since last pkt: ", (vision_frame_info.frame_id - frameRecv[closest]))

  -- if we are closest, we're probably chaser
  if (closest == robot_state.WO_SELF) then
    -- if a higher numbered player is within close_distance, and is already chasing, let them chase
    for i=core.WO_TEAM_LAST,robot_state.WO_SELF+1,-1 do
      if (tp[i].bvrData.role == core.CHASER and (bid[i] - bid[robot_state.WO_SELF]) < switchParams.close_distance) then
        UTdebug.log(10,"Player",i,"is already chasing, and at similar distance, let them chase", (bid[i] - bid[robot_state.WO_SELF]))
        closest = i
      end
    end
  end
   
  -- if we're keeper with a high bid, then be keeper
  if (bid[robot_state.WO_SELF] > 3500 and robot_state.WO_SELF == core.KEEPER) then
    robot_state.role_ = core.KEEPER
    UTdebug.log(10, "keeper has high bid" ,  bid[robot_state.WO_SELF], " be keeper")
    return
  end

  -- if we're still closest, then its us
  if (closest == robot_state.WO_SELF) then
    UTdebug.log(10,"we're closest, we CHASE!")
    robot_state.role_ = core.CHASER
    return
  end

  -- special case if keeper should be clearing, and is already clearing
  if (closest == core.KEEPER) then
    -- make sure keeper actually started clearing
    if (tp[core.KEEPER].bvrData.role == core.CHASER) then
      UTdebug.log(10, "keeper is clearing, special role switch")
      local activeInds = getActiveNonChaserInds(switchParams,tp,frameRecv,closest)
      roleSwitchKeeperIsClearing(tp,activeInds,frameRecv)
      return
    else
      bid[core.KEEPER] = 6001
      closest = getMinInd(bid)
      UTdebug.log(10, "keeper is closest, but not clearing yet, find new chaser", closest)
      if (closest == robot_state.WO_SELF) then
        UTdebug.log(10,"we're closest, we CHASE!")
        robot_state.role_ = core.CHASER
        return
      end
    end
  end

  -- keeper remains keeper if not clearing
  if (robot_state.WO_SELF == core.KEEPER and closest ~= core.KEEPER) then
    UTdebug.log(10, "am keeper, not clearing, stay keeper")
    robot_state.role_ = core.KEEPER
    return
  end

  -- if none of our teammates are chasing, we chase
  if (not someoneChasing) then
    UTdebug.log(10,"no teammates are chasing, we CHASE!")
    robot_state.role_ = core.CHASER
    return
  end

  -- if we're within max calloff dist cm of ball, chase
  if (bid[robot_state.WO_SELF] < switchParams.max_calloff_distance) then
    UTdebug.log(10,"within max call-off distance, we CHASE!")
    robot_state.role_ = core.CHASER
    return
  end

  -- no one else has seen it recently or no else is sending messages
  if (bid[closest] > 5900) then
    UTdebug.log(10,"no one active/seeing ball, we CHASE!")
    robot_state.role_ = core.CHASER
    return
  end
  
  -- figure out indices of other non-chasing players other than the goalie
  local activeInds = getActiveNonChaserInds(switchParams,tp,frameRecv,closest)
  --UTdebug.log(10,"active inds:",unpack(activeInds))
  
  -- otherwise, we're either defending, supporting, or playing forward
  local ball = world_objects:getObjPtr(core.WO_BALL)
  -- set our role options
  --   given as {role,x,imp}
  --   importance is used when robots are missing
  --   like bids, importance being lower means more important
  --   x: lower means closer to own goal - used for assigning
  if ((tp[robot_state.WO_SELF].bvrData.state == core.READY) or (tp[robot_state.WO_SELF].bvrData.state == core.SET)) then -- special case in ready and set
    -- always make 4 the supporter
    if (robot_state.WO_SELF == core.WO_TEAM4) then
      robot_state.role_ = core.SUPPORTER
      return
    end
    -- forward is more important than defender on kickoff
    -- NOTE, we're only assigning to two robots, since 4 will always be supporter
    roles = {
      {role=core.DEFENDER,x=0,imp=1},
      {role=core.FORWARD, x=1,imp=0}
    }
  elseif behavior_mem.isKickOffShot and (behavior_mem.setPlayInfo.type ~= core.SetPlay_none) then
    if (robot_state.WO_SELF == behavior_mem.setPlayInfo.targetPlayer) then
      robot_state.role_ = core.SET_PLAY_RECEIVER
      return
    end
    -- NOTE, we're only assigning to two robots, but I'm leaving in the third just in case
    roles = {
      {role=core.DEFENDER,x=0,imp=1},
      {role=core.SUPPORTER,x=1,imp=0},
      {role=core.FORWARD,x=2,imp=2} -- should never be used
    }
  else
    local area
    if behavior_mem.isKickOffShot then
      area = behavior_params.roleStrategy.kickoffArea
    else
      for i = 0,core.MAX_NUM_FIELD_AREAS-1 do
        area = behavior_params.roleStrategy:getFieldAreaRoleConfigPtr(i)
        if ball.loc.x > area.minX then
          --print('in area ' .. tostring(i))
          break
        end
      end
    end
    roles = getRolesForArea(area)
  end

  assignRole(tp,activeInds,roles)
end

function roleSwitchKeeperIsClearing(tp,activeInds,frameRecv)
  -- keeper is clearing, what other four roles are we going to play?
  local roles = getRolesForArea(behavior_params.roleStrategy.clearingKeeperArea)
  assignRole(tp,activeInds,roles)
end

function getRolesForArea(area)
  local roles = {}
  local ball = world_objects:getObjPtr(core.WO_BALL)
  for i = 1,core.NUM_FIELD_PLAYERS do
    local role = luaC:getInt(area.rolesOrderedByImportance,i-1)
    if role < 0 then
      break
    end
    local cfg = behavior_params.roleStrategy:getRolePositionConfigPtr(role)
    local x = core.crop(ball.loc.x + cfg.offsetFromBall.x,cfg.minX,cfg.maxX)
    roles[i] = {role=role, x=x, imp=i}
    --print('roles[' .. luaC:getString(core.roleNames,role) .. '] = ' .. tostring(role) .. ' ' .. tostring(x) .. ' ' .. tostring(i))
  end
  return roles
end

function assignRole(tp,activeInds,roles)
  local assignRolesToClosest = false
  -- order roles by imp
  function compare(a,b)
    return a.imp < b.imp
  end
  table.sort(roles,compare)

  -- remove roles that we can't fill
  for i=#activeInds+1,#roles do
    roles[i] = nil
  end
  
  if assignRolesToClosest then
    for _,roleInfo in ipairs(roles) do
      local minDistToRole = 999999
      local roleCfg = behavior_params.roleStrategy:getRolePositionConfigPtr(roleInfo.role)
      local minActiveInd = nil
      for activeInd,player in pairs(activeInds) do
        local targetPt = skills.calcPositionForPlayer(player,roleCfg)
        local dist = world_objects:getObjPtr(player).loc:getDistanceTo(targetPt)
        if dist < minDistToRole then
          minDistToRole = dist
          minActiveInd = activeInd
        end
      end
      if activeInds[minActiveInd] == robot_state.WO_SELF then
        UTdebug.log(10,'assigning role',luaC:getString(core.roleNames,roleInfo.role))
        robot_state.role_ = roleInfo.role
        return
      end
      activeInds[minActiveInd] = nil
    end
  else
    local playersOrderedByXLoc = getIndsSortedByXLoc(tp,activeInds)
    ---- order roles by x loc
    function compare(a,b)
      return a.x < b.x
    end
    table.sort(roles,compare)

    ---- assign to remaining roles
    local i = 1
    for _,v in ipairs(roles) do
      local ind = playersOrderedByXLoc[i]
      if ind == robot_state.WO_SELF then
        UTdebug.log(10,'assigning role',luaC:getString(core.roleNames,v.role))
        robot_state.role_ = v.role
        return
      end
      i = i + 1
    end
  end
end

function getActiveNonChaserInds(switchParams,tp,frameRecv,closest)
  --UTdebug.log(10,'getActiveNonChaserInds')
  local inds = {}
  for i = core.WO_TEAM_FIRST,core.WO_TEAM_LAST do
    --UTdebug.log(10,'   considering',i)
    if (i ~= core.KEEPER) and (i ~= closest) and isPlayerActive(switchParams,tp,frameRecv,i) then
      table.insert(inds,i)
    end
  end
  --UTdebug.log(10,'final:',unpack(inds))
  return inds
end

function isPlayerActive(switchParams,tp,frameRecv,ind)
  local heardRecently = (vision_frame_info.frame_id - frameRecv[ind]) <= switchParams.max_missed_packet_frames -- communicating
  if (tp[robot_state.WO_SELF].bvrData.state == core.READY) then
    return ((tp[ind].bvrData.state == core.READY) or (tp[ind].bvrData.state == core.SET) or (tp[ind].bvrData.state == core.PLAYING)) and heardRecently and (ind ~= core.WO_TEAM4) -- don't assign player 4 in ready
  end
  --UTdebug.log(10,'    ',(tp[ind].bvrData.state == core.PLAYING or tp[ind].bvrData.state == core.SET),not(tp[ind].bvrData.fallen),(vision_frame_info.frame_id - frameRecv[ind]) <= switchParams.max_missed_packet_frames)
  --UTdebug.log(10,'      ',vision_frame_info.frame_id,frameRecv[ind],switchParams.max_missed_packet_frames)
  return ((tp[ind].bvrData.state == core.PLAYING or tp[ind].bvrData.state == core.SET) -- check state
          and not(tp[ind].bvrData.fallen) -- fallen
          and heardRecently)
end

--function getMin(value)
  --if (value[1] <= value[2] and value[1] <= value[3] and value[1] <= value[4]) then
    --return 1
  --elseif (value[2] < value[1] and value[2] <= value[3] and value[2] <= value[4]) then
    --return 2
  --elseif (value[3] < value[1] and value[3] <= value[2] and value[3] <= value[4]) then
    --return 3
  --else
    --return 4
  --end
--end

function getMinInd(arr)
  local minVal = math.huge
  local ind = nil
  for k,v in pairs(arr) do
    if v < minVal then
      minVal = v
      ind = k
    end
  end
  return ind
end

function getIndsSortedByXLoc(tp,activeInds)
  --UTdebug.log(10,'getIndsSortedByXLoc:')
  local arr = {}
  for _,ind in ipairs(activeInds) do
    table.insert(arr,{ind,tp[ind].locData.robotX})
    --UTdebug.log(10,'  arr[',#arr,'] = ',unpack(arr[#arr]))
  end
  function compare(a,b)
    return (a[2] < b[2]) or ((a[2] == b[2]) and (a[1] < b[1]))
  end
  table.sort(arr,compare)
  for k,v in ipairs(arr) do
    --UTdebug.log(10,'  sorted arr[',k,'] = ',unpack(v))
  end
  local inds = {}
  for k,v in ipairs(arr) do
    table.insert(inds,v[1])
  end
  return inds
end

function checkKeeper()
  -- if ball is coming at us, we have to be keeper
  local ball = world_objects:getObjPtr(core.WO_BALL)
  if (ball.relPos.x < 3000 and ball.relVel.x < -40) then
    robot_state.role_ = core.KEEPER
  end
  -- stay keeper during dives
  if (behavior_mem.keeperDiving ~= core.Dive_NONE) then
    robot_state.role_ = core.KEEPER
  end
end

