module(..., package.seeall)

local function createOffsets(fwd,side,turn)
  local tab = {}
  tab.fwd = fwd
  tab.side = side
  tab.turn = turn
  return tab
end

function initWalk()
  local id = robot_state.robot_id_

  initWalkOffsets(id)
  initWalkParams(id)
end

function initWalkOffsets(id)
  local offset = cfgHTWKWalkOffsets[id]
  if offset == nil then
    --UTdebug.log(0,'NO ROBOT SPECIFIC WALK OFFSETS for',id)
    offset = cfgHTWKWalkOffsets[0]
  end
  --UTdebug.log(0,'Walk Offsets:',offset.fwd,offset.side,offset.turn)
  walk_request:setOdometryOffsets(offset.fwd,offset.side,offset.turn)
end

function initWalkParams(id)
  walk_param.send_params_ = true
  -- use default parameters for most, only overwrite what we need
  local params = walk_param.htwk_params_
  -- WALK
  --params.maxVel = core.Pose2D(0.4,360 * 0.75,90)
  params.maxVel = core.Pose2D(1.381,190,113)
  params.maxBack = -60
  params.maxVec = core.Pose2D(0.12,0.12,0.04)
  params.bodyTiltStop = 0.14
  params.bodyTiltNoAccel = 0.07
  params.maxVelAccel = core.Pose2D(0.008,1,1)
  params.maxVelDecel = core.Pose2D(0.08,20,5)
  params.odometryFactors = core.Pose2D(1.923,1.9,3.226) -- rot,fwd,side
  --params.startLength = 1
  local factor = 1.0
  params.balanceKneePitch = factor * -0.5
  params.balanceHipPitch = factor * 0.265

  params.bodyTiltTarget = DEG_T_RAD * 0-- -2.5 -- -2.5 -- -2.5 -- negative goes forwards???
  params.bodyTiltTargetFullFwd = DEG_T_RAD * 0-- -2.5 -- 2.5 -- -2.5 -- negative goes forwards???
  params.bodyTiltTargetFullBack = DEG_T_RAD * 0-- -2.5 -- -2.5 -- -2.5 -- negative goes forwards???
  params.bodyTiltTargetUpdate = 0.003

  if (id == 27) then
    --non-silly params
    --UTdebug.log(0,'Colby Jack Walk Params')
    --params.maxVel = core.Pose2D(1.381,190*0.6,113)
    --params.maxBack = -30
    --params.maxVec = core.Pose2D(0.12,0.12*0.6,0.04)
    --params.maxVelDecel = core.Pose2D(0.01,6,2)

    UTdebug.log(0,'Colby Jack Walk Params')
    local g = 0.3
    params.maxVel = core.Pose2D(1.381,g*190,113)
    params.maxBack = -15
    params.maxVec = core.Pose2D(0.12,g*0.12,0.04)
    params.bodyTiltNoAccel = 0.11
    params.useAnkleTiltHack = false
    params.ankleTiltHackSwingAmount = -2.0 * DEG_T_RAD
    params.ankleTiltHackStanceAmount = 2.0 * DEG_T_RAD
    local factor = 1.0
    params.balanceKneePitch = factor * -0.5
    params.balanceHipPitch = factor * 0.265
    params.bodyTiltTarget = DEG_T_RAD * -2.5 -- -2.5 -- -2.5 -- negative goes forwards???
    params.bodyTiltTargetFullFwd = DEG_T_RAD * -1.0 -- 2.5 -- -2.5 -- negative goes forwards???
    params.bodyTiltTargetFullBack = DEG_T_RAD * -1.0 -- -2.5 -- -2.5 -- negative goes forwards???
    params.bodyTiltTargetUpdate = 0.003
  elseif id == 28 then
    -- michael should be confident
    params.bodyTiltStop = 0.14
    params.bodyTiltNoAccel = 0.07
    local g = 0.7
    params.maxVel = core.Pose2D(1.381,g*190,113)
    params.maxBack = -15
    params.maxVec = core.Pose2D(0.12,g*0.12,0.04)
    local factor = 0.8
    params.balanceKneePitch = factor * -0.5
    params.balanceHipPitch = factor * 0.265
    params.maxVelAccel = core.Pose2D(0.005,0.7,0.7)
  elseif id == 29 then
    local g = 0.3
    params.maxVel = core.Pose2D(1.381,g*190,113)
    params.maxBack = -15
    params.maxVec = core.Pose2D(0.12,g*0.12,0.04)
    params.bodyTiltNoAccel = 0.11
    params.useAnkleTiltHack = true
    params.ankleTiltHackSwingAmount = -3 * DEG_T_RAD
    params.ankleTiltHackStanceAmount = 3 * DEG_T_RAD
    local factor = 0.6
    params.balanceKneePitch = factor * -0.5
    params.balanceHipPitch = factor * 0.265
    params.bodyTiltTarget = DEG_T_RAD * -2.5 -- -2.5 -- -2.5 -- negative goes forwards???
    params.bodyTiltTargetFullFwd = DEG_T_RAD * -2.5 -- 2.5 -- -2.5 -- negative goes forwards???
    params.bodyTiltTargetFullBack = DEG_T_RAD * -2.5 -- -2.5 -- -2.5 -- negative goes forwards???
  elseif (id == 30) then
    local g = 0.6
    params.maxVel = core.Pose2D(1.381,g*190,113)
    params.maxBack = -15
    params.maxVec = core.Pose2D(0.12,g*0.12,0.04)

    params.bodyTiltTarget = DEG_T_RAD * -8 -- -2.5 -- -2.5 -- negative goes forwards???
    params.bodyTiltTargetFullFwd = DEG_T_RAD * -8 -- 2.5 -- -2.5 -- negative goes forwards???
    params.bodyTiltTargetFullBack = DEG_T_RAD * -8 -- -2.5 -- -2.5 -- negative goes forwards???
    params.maxVelAccel = core.Pose2D(0.008,1,1)
    params.maxVelDecel = core.Pose2D(0.04,10,2.5)
    local factor = 0.6
    params.balanceKneePitch = factor * -0.5
    params.balanceHipPitch = factor * 0.265
    params.useAnkleTiltHack = true
    params.ankleTiltHackSwingAmount = -2.5 * DEG_T_RAD
    params.ankleTiltHackStanceAmount = 2.5 * DEG_T_RAD
  elseif (id == 31 or id == 32) then
    params.bodyTiltTarget = DEG_T_RAD * -2.5 -- -2.5 -- -2.5 -- negative goes forwards???
    params.bodyTiltTargetFullFwd = DEG_T_RAD * -2.5 -- 2.5 -- -2.5 -- negative goes forwards???
    params.bodyTiltTargetFullBack = DEG_T_RAD * -2.5 -- -2.5 -- -2.5 -- negative goes forwards???
    local g = 0.7
    params.maxVel = core.Pose2D(1.381,g*190,113)
    params.maxBack = -15
    params.maxVec = core.Pose2D(0.12,g*0.12,0.04)
    local factor = 0.7
    params.balanceKneePitch = factor * -0.5
    params.balanceHipPitch = factor * 0.265
    params.maxVelDecel = core.Pose2D(0.04,10,2.5)
  elseif false then --id == 32 then
    params.bodyTiltStop = 0.18
    params.bodyTiltNoAccel = 0.11
    local g = 0.5
    params.maxVel = core.Pose2D(1.381,g*190,113)
    params.maxBack = -15
    params.maxVec = core.Pose2D(0.12,g*0.12,0.04)
    local factor = 0.7
    params.balanceKneePitch = factor * -0.5
    params.balanceHipPitch = factor * 0.265
    params.useAnkleTiltHack = false--true
    params.ankleTiltHackSwingAmount = -2.5 * DEG_T_RAD
    params.ankleTiltHackStanceAmount = 2.5 * DEG_T_RAD
    local angle = 4.0
    params.bodyTiltTarget = DEG_T_RAD * angle -- -2.5 -- -2.5 -- negative goes forwards???
    params.bodyTiltTargetFullFwd = DEG_T_RAD * angle -- 2.5 -- -2.5 -- negative goes forwards???
    params.bodyTiltTargetFullBack = DEG_T_RAD * angle -- -2.5 -- -2.5 -- negative goes forwards???
  end


  -- KICK stuff
  initFrontKick(params,id)
  initSideKick(params,id)

  -- SIM stuff
  if vision_frame_info.source == core.MEMORY_SIM then
    --walk_param.htwk_params_.frameInc = 2
    params.frameInc = 2
    params.sensorDelayProp = 8 / params.frameInc
    params.sensorDelayFsr = 12 / params.frameInc
  end
end

function initFrontKick(params,id)
  local frames = 0.5 * params.numFrames
  local swingReachPoint = {70,0,0}
  local stanceReachPoint = {0,0,0}
  local reachPointFrame = frames - 5
  local times = {0,1,reachPointFrame-4,reachPointFrame,frames-1,frames}
  local swing = {}
  local stance = {}

  -- remember that lua does 1 indexing by default
  swing[0] =  {0,0,0, swingReachPoint[1],0,0}
  swing[1] =  {0,0,0, swingReachPoint[2],0,0}
  swing[2] =  {0,0,0, swingReachPoint[3],0,0}

  stance[0] = {0,0,0,stanceReachPoint[1],0,0}
  stance[1] = {0,0,0,stanceReachPoint[2],0,0}
  stance[2] = {0,0,0,stanceReachPoint[3],0,0}
  setKickSpline(params.frontKick,times,swing,stance)
end

function initSideKick(params,id)
  local frames = 0.5 * params.numFrames
  local swingReachPoint = {20,-30,10}
  local swingExtendPoint = {20,10,10}

  local reachPointFrame = 0.3 * frames
  local extendPointFrame = 0.6 * frames
  local times = {0,1,reachPointFrame,extendPointFrame,frames-1,frames}
  local swing = {}
  local stance = {}

  -- remember that lua does 1 indexing by default
  swing[0] =  {0,0,swingReachPoint[1], swingExtendPoint[1],0,0}
  swing[1] =  {0,0,swingReachPoint[2], swingExtendPoint[2],0,0}
  swing[2] =  {0,0,swingReachPoint[3], swingExtendPoint[3],0,0}

  stance[0] = {0,0,0,0,0,0}
  stance[1] = {0,0,0,0,0,0}
  stance[2] = {0,0,0,0,0,0}
  setKickSpline(params.sideKick,times,swing,stance)
end

function setKickSpline(kick,times,swing,stance)
  kick.numPts = table.getn(times)
  local count
  setArray(kick.numPts,kick.times,times)
  for dim = 0,2 do
    count = table.getn(swing[dim])
    if count ~= kick.numPts then
      UTdebug.log(0,'*** INCORRECT NUMBER OF POINTS FOR SWING, Expected:',kick.numPts,'Got:',count)
    end
    count = table.getn(stance[dim])
    if count ~= kick.numPts then
      UTdebug.log(0,'*** INCORRECT NUMBER OF POINTS FOR STANCE, Expected:',kick.numPts,'Got:',count)
    end
  end
  setArray(kick.numPts,kick.xswing,swing[0])
  setArray(kick.numPts,kick.yswing,swing[1])
  setArray(kick.numPts,kick.zswing,swing[2])
  setArray(kick.numPts,kick.xstance,stance[0])
  setArray(kick.numPts,kick.ystance,stance[1])
  setArray(kick.numPts,kick.zstance,stance[2])
end

function setArray(num,dst,src)
  for i = 1,num do
    luaC:setDouble(dst,i-1,src[i])
  end
end

cfgHTWKWalkOffsets = {}
-- format is fwd,side,turn in absolute velocities (not related to max vel)
cfgHTWKWalkOffsets[0] = createOffsets(-45,0,DEG_T_RAD * 0)
cfgHTWKWalkOffsets[-1] = createOffsets(0,0,DEG_T_RAD * 0) -- simulator
--cfgHTWKWalkOffsets[27] = createOffsets(-55,0,DEG_T_RAD * 0.18)
cfgHTWKWalkOffsets[27] = createOffsets(-55*0.6,0,DEG_T_RAD * 1)
cfgHTWKWalkOffsets[28] = createOffsets(-40,0,DEG_T_RAD * -3)
cfgHTWKWalkOffsets[29] = createOffsets(-45 * 0.5,0,DEG_T_RAD * -0.18)
cfgHTWKWalkOffsets[30] = createOffsets(-75,20,DEG_T_RAD * -4)
cfgHTWKWalkOffsets[31] = createOffsets(-30,0,DEG_T_RAD * 1)
--cfgHTWKWalkOffsets[32] = createOffsets(-35,0,DEG_T_RAD * 0) --orig 32 H2

cfgHTWKWalkOffsets[32] = createOffsets(-25,0,DEG_T_RAD * 4) --bowdoin robot

