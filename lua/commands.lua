require 'cfgstiff'
module(..., package.seeall)

local walkTiltOffset

last_arm_command = 0
function setArmsBehind()
  if ((vision_frame_info.seconds_since_start - last_arm_command) > 5.0) then
    -- havent sent commands in a while, sent the side arm pose first
    last_arm_command = vision_frame_info.seconds_since_start
    setArmPose(armSidePose,800)
    return 
  end

  if vision_frame_info.seconds_since_start - last_arm_command < 1.0 then
    return
  end
  last_arm_command = vision_frame_info.seconds_since_start

  
  if (robot_state.role_ == core.KEEPER) then
    -- keep arms out more for diving
    setArmPose(armBehindKeeperPose,1000)
    --setArmPose(armBehindPose,1000)
  else
    setArmPose(armBehindPose,1000)
  end
end

function stand()
  walk_request:stand()
  --setArmsBehind()
end

function setWalkVelocity(velX, velY, velTheta, sprint)
  UTdebug.log(80,'setWalkVelocity, requested vels:',velX,velY,velTheta)
  --if sprint then
    --setWalkMode(core.WalkMode_SPRINT)
    ----setWalkMode(core.WalkMode_SLOW)
  --else
    --setWalkMode(core.WalkMode_SLOW)
  --end
  
  --velX,velY,velTheta = chooseWalkVels(velX,velY,velTheta,sprint)

  UTdebug.log(80,'setWalkVelocity, final vels:',velX,velY,velTheta)
  walk_request:setWalk(velX,velY,velTheta)
  
  --commands.setStiffnessCommands(cfgStiffALWalk,0.1)
  --setArmsBehind()
end

function resetTiltOffset()
  walkTiltOffset = 0
end

function handleTiltOffset()
  if joint_commands.send_head_pitch_angle_ and not(joint_commands.head_pitch_angle_change_) then
    local val = luaC:getFloat(joint_commands.angles_,core.HeadPitch) + DEG_T_RAD * walkTiltOffset
    luaC:setFloat(joint_commands.angles_,core.HeadPitch,val)
  end
end

function getWalkOffsets()
  local offset = cfgWalkOffsets[robot_state.robot_id_]
  if offset == nil then
    UTdebug.log(80,'NO OFFSETS:(')
    offset = cfgWalkOffsets[0]
  end
  UTdebug.log(80,'offset:',offset[3])
  return offset
end

function chooseWalkVels(velX,velY,velTheta,sprint)
  local maxFwdIncrease = 0.33
  
  --local stepFreq = al_walk_param.walk_step_min_period_ / 50.0
  --local maxPossibleFwd = al_walk_param.walk_max_step_x_ * 1000.0 / stepFreq
  --local currFwdFrac = walk_info.robot_velocity_.translation.x / maxPossibleFwd
  local currFwdFrac = walk_info.robot_velocity_frac_.translation.x
  local maxFwdFromAccel = core.crop(currFwdFrac + maxFwdIncrease,0,1.0)
 
  local maxFwd = 1.0 
  local maxSide = 1.0
  local maxTheta = 1.0

  local nonSprintMaxFwd = 0.8
  local nonSprintMaxBack = -0.6
  local offsets = getWalkOffsets()
  local maxSprintTheta = 0.2
  local turnOffset = 0

  if sprint then
    maxFwd = math.min(maxFwd,maxFwdFromAccel)
    velX = core.crop(velX,0,maxFwd)
    velY = 0
    turnOffset = getTurnOffset(offsets,velX,velY,sprint)
    velTheta = core.crop(velTheta + turnOffset,-maxSprintTheta,maxSprintTheta)
  else
    -- get x's ranging from -1.0 to 1.0, we'll convert the back later
    if velX < 0 then
      velX = velX / math.abs(nonSprintMaxBack)
    else
      velX = velX / nonSprintMaxFwd
    end
    velX = core.crop(velX, -1.0, 1.0)
    velY = core.crop(velY,-1.0,1.0)
    velTheta = core.crop(velTheta,-1.0,1.0)

    -- compromise between forwards and side
    maxSide = math.min(maxSide,1.0 - 0.5 * math.abs(velX))
    maxFwd = math.min(maxFwd,1.0 - 0.5 * math.abs(velY))
    -- compromise between forwards and turning -- can cheat the diamond slightly
    maxTheta = math.min(maxTheta,1.0 - 0.4 * math.abs(velX))
    maxFwd = math.min(maxFwd,1.0 - 0.4 * math.abs(velTheta))
    -- compromise between side and turning
    maxSide = math.min(maxSide,1.0 - 0.5 * math.abs(velTheta))
    maxTheta = math.min(maxTheta,1.0 - 0.5 * math.abs(velY))

    local minSide = -maxSide
    local minFwd = -maxFwd
    local minTheta = -maxTheta
    maxFwd = math.min(maxFwd,maxFwdFromAccel)

    -- don't accelerate sideways too quickly
    local maxSideIncrease = 0.5
    minSide = math.min(0.0,math.max(minSide,walk_info.robot_velocity_frac_.translation.y - maxSideIncrease))
    maxSide = math.max(0.0,math.min(maxSide,walk_info.robot_velocity_frac_.translation.y + maxSideIncrease))

    ---- don't switch side too quickly
    --if walk_info.robot_velocity_frac_.translation.y > 0.1 then
      --minSide = -0.01
    --elseif walk_info.robot_velocity_frac_.translation.y < -0.1 then
      --maxSide = 0.01
    --end
    -- or fwd
    if walk_info.robot_velocity_frac_.translation.x > 0.5  then
      minFwd = -0.01
    end
    -- other fwd change handled already by max change
    -- or turn
    if walk_info.robot_velocity_frac_.rotation > 0.1 then
      minTheta = -0.01
    elseif walk_info.robot_velocity_frac_.rotation < -0.1 then
      maxTheta = 0.01
    end

    --print ('side bounds',minSide,maxSide,velTheta,math.abs(velTheta))
    velX = core.crop(velX, minFwd, maxFwd)
    velY = core.crop(velY, minSide, maxSide)
  
    if velX < 0 then
      velX = velX * math.abs(nonSprintMaxBack)
    else
      velX = velX * nonSprintMaxFwd
    end

    turnOffset = getTurnOffset(offsets,velX,velY,sprint)
    velTheta = core.crop(velTheta + turnOffset, minTheta, maxTheta)
  end
  
  --walk_request:setOdometryOffsets(0,0,turnOffsetFrac) -- REMOVED by sbarrett on 4/11/12 as we're not using this system now
  
  if sprint then
    UTdebug.log(80,'SPRINT, vels:',velX,velY,velTheta)
  else
    UTdebug.log(80,'stroll, vels:',velX,velY,velTheta)
  end

  return velX,velY,velTheta
end

function getTurnOffset(offsets,velX,velY,sprint)
  local turn = offsets.turnInPlace
  if velX > 0 then
    if sprint then
      turn = turn + math.abs(velX) * (offsets.turnSprint  - offsets.turnInPlace)
    else
      turn = turn + math.abs(velX) * (offsets.turnFwd  - offsets.turnInPlace)
    end
  else
    turn = turn + math.abs(velX) * (offsets.turnBack  - offsets.turnInPlace)
  end
  if velY > 0 then
    turn = turn + math.abs(velY) * (offsets.turnLeft - offsets.turnInPlace)
  else
    turn = turn + math.abs(velY) * (offsets.turnRight - offsets.turnInPlace)
  end
  return turn
end


function avoidBumpObstacles(velX, velY, velTheta, sprint, relTargetPt)

  -- no bumps -- all clear
  if (not processed_sonar.bump_left_ and not processed_sonar.bump_right_) then
    return velX,velY,velTheta,sprint,false
  end

  -- we're in sprint, just turn a bit away
  if (sprint) then
    if (processed_sonar.bump_left_) then
      if (velTheta > -0.05) then
        velTheta = -0.05
      end
      return velX,velY,velTheta,sprint,true
    end
    if (processed_sonar.bump_right_) then
      if (velTheta < 0.05) then
        velTheta = 0.05
      end
      return velX,velY,velTheta,sprint,true
    end
  else

  -- otherwise, let's strafe a bit away
    if (processed_sonar.bump_left_) then
      if (velY > -0.2) then
        velY = -0.2
      end
      return velX,velY,velTheta,sprint,true
    end
    if (processed_sonar.bump_right_) then
      if (velY < 0.2) then
        velY = 0.2
      end
      return velX,velY,velTheta,sprint,true
    end
  end
end

function avoidSonarObstacles(velX,velY,velTheta,sprint,relTargetPt)
  local sonarCenter = processed_sonar.center_distance_ * 1000
  local sonarLeft = processed_sonar.left_distance_ * 1000
  local sonarRight = processed_sonar.right_distance_ * 1000
  local ball = world_objects:getObjPtr(core.WO_BALL)
  local maxAvoidDist = 750

  ---- colby jack's sonars are broken
  --if (robot_state.robot_id_ == 96) then
  --  -- ball's close enough, just ignore
  --  return velX,velY,velTheta,sprint,false
  --end

  -- if keeper and in box
  if (robot_state.WO_SELF == core.KEEPER) then
    local me = world_objects:getObjPtr(robot_state.WO_SELF)
    if (me.loc.x < (-core.FIELD_X/2.0 + core.PENALTY_X) and math.abs(me.loc.y) < core.PENALTY_Y/2.0) then
      -- keeper cant be called for pushing inside box
      return velX,velY,velTheta,sprint,false
    end
  end

  if behavior_mem.keeperRelBallPos:getMagnitude() < 200 then
    -- ball's close enough, just ignore
    return velX,velY,velTheta,sprint,false
  end

  -- not sure if this is used anywhere
  -- set sonar obstacle time
  if (processed_sonar.on_center_ or processed_sonar.on_right_ or processed_sonar.on_left_) then
    behavior_mem.sonarObstacleTime = vision_frame_info.seconds_since_start
  end

  if relTargetPt:getMagnitude() < 200 then
    -- desPt's close enough, just ignore
    return velX,velY,velTheta,sprint,false
  end

  -- if we're not moving forward (rotating in place, etc) - do nothing
  if (velX < 0.01) then
    return velX,velY,velTheta,sprint,false
  end

  if not(processed_sonar.on_center_) then
    sonarCenter = 2550
  end
  if not(processed_sonar.on_left_) then
    sonarLeft = 2550
  end
  if not(processed_sonar.on_right_) then
    sonarRight = 2550
  end

  -- only worry about the closer obstacle
  if sonarLeft < sonarRight then
    sonarRight = 2550
  else
    sonarLeft = 2550
  end

  UTdebug.log(10, "sonar l,c,r", sonarLeft, sonarCenter, sonarRight)

  -- do something about it
  if sonarCenter < maxAvoidDist then -- if obstacle is in center
    UTdebug.log(10,'center',processed_sonar.on_center_,processed_sonar.center_distance_ * 1000,sonarCenter, processed_sonar.left_distance_, processed_sonar.right_distance_)
    velX = -0.1
    velTheta = 0.0
    sprint = false

    -- if point is near straight ahead, choose based on direction we were traveling
    if (math.abs(relTargetPt.y) < 350) then
      -- if we were doing this in last second
      if ((vision_frame_info.seconds_since_start - behavior_mem.avoidSonarTime) < 1.0) then
        UTdebug.log(10, "point near straight", relTargetPt.y, " last avoid was ",behavior_mem.avoidSonarTime, "continue left?", behavior_mem.avoidSonarDirIsLeft)
        -- continue in dir we were going
        if (behavior_mem.avoidSonarDirIsLeft) then
          velY = 0.8
        else
          velY = -0.8
        end
      else
        -- no recent avoid, choose based on target point
        if relTargetPt.y > 0 then
          velY = 0.8
        else
          velY = -0.8
        end
      end
      
    else
      -- can reliably choose a side? (far enough to side)
      if relTargetPt.y > 0 then
        velY = 0.8
      else
        velY = -0.8
      end
    end
  else -- obstacle on left or right 
    ---- Todd: lets not do anything if its only left or only right
    --if (true) then 
    --  return velX,velY,velTheta,sprint,false 
    --end
    ----------------------------------
    local sonarObstacle
    local dir
    local sideAvoidDist = 250
    if sonarLeft < maxAvoidDist then
      sonarObstacle = sonarLeft
      dir = -1
    elseif sonarRight < maxAvoidDist then
      sonarObstacle = sonarRight
      dir = 1
    end
    if sonarObstacle == nil then
      -- no obstacle
      return velX,velY,velTheta,sprint,false
    end

    local timeToContact = sonarObstacle / (velX * walk_max_vel_x)
    if timeToContact > 0 then
      if timeToContact < 1.0 then
        velX = 0.0
      end
      velY = dir * (sideAvoidDist / timeToContact) / walk_max_vel_y
      velTheta = 0.0
      sprint = false
    end
  end

  behavior_mem.avoidSonarTime = vision_frame_info.seconds_since_start
  if (velY > 0) then
    behavior_mem.avoidSonarDirIsLeft = true
  else
    behavior_mem.avoidSonarDirIsLeft = false
  end

  --speech:say("sonar")
  return velX,velY,velTheta,sprint,true
end

function setWalkTarget(targetx, targety)
  local target, turnOffsetFrac

  target,turnOffsetFrac = calcOffsetTargetPoint(targetx,targety)
  --if math.abs(targetx) < 80 then
  --setWalkMode(core.WalkMode_TARGET_CLOSE)
  --else
  setWalkMode(core.WalkMode_TARGET)
  --end
  
  --walk_request:setOdometryOffsets(0,0,turnOffsetFrac) -- REMOVED by sbarrett on 4/11/12 as we're not using this system now  
  walk_request:setWalkTarget(target.translation.x,target.translation.y,target.rotation)
  
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  local relTarget = core.Point2D(targetx, targety)
  behavior_mem.absTargetPt = relTarget:relativeToGlobal(me.loc, me.orientation)
  
  commands.setStiffnessCommands(cfgStiffALWalk,0.1)
  --setArmsBehind()

  --UTdebug.log(10, "set walk target", targetx, targety, "relative next robot pos is ", walk_info.robot_relative_next_position_.translation.x, walk_info.robot_relative_next_position_.translation.y, walk_info.robot_relative_next_position_.rotation * RAD_T_DEG, "new target",walk_request.target_point_.x,walk_request.target_point_.y)
end

function calcOffsetTargetPoint(targetx,targety)
  -- estimate the time to the target
  local x_time = targetx / walk_max_vel_x
  local y_time = targety / walk_max_vel_y
  local time = math.max(x_time,y_time)
  -- estimate the vels to target
  local x_vel = targetx / (walk_max_vel_x * time)
  local y_vel = targety / (walk_max_vel_y * time)
  -- get the turn offset for these vels
  local offsets = getWalkOffsets()
  local turnOffsetFrac = getTurnOffset(offsets,x_vel,y_vel,false) -- fraction of turn, [-1.0,1.0]
  local turn_offset = 0
  if (turnOffsetFrac > 0) then
    turn_offset = turnOffsetFrac * walk_max_vel_rot_ccw * time -- total turn over the time
  else
    turn_offset = turnOffsetFrac * walk_max_vel_rot_cw * time -- total turn over the time
  end
  
  --local target = core.vector2_float(targetx / al_walk_param.fwd_odometry_factor_ ,targety / al_walk_param.side_odometry_factor_)
  local target = core.vector2_float(targetx / 1.2 ,targety / al_walk_param.side_odometry_factor_)

  UTdebug.log(10,'orig_target:',target.x,target.y)

  --target:rotate(turn_offset)
  UTdebug.log(10,'turn_offset:',turn_offset,'new_target',target.x,target.y)
  walk_request:setWalkTarget(target.x,target.y,turn_offset)

  return core.Pose2D(turn_offset,target.x,target.y),turnOffsetFrac
end

function setWalkRotate(targetx, targety, distance, heading)
  walk_request.walk_to_target_ = false
  walk_request.rotate_around_target_ = true
  walk_request.target_point_.x = targetx
  walk_request.target_point_.y = targety
  walk_request.rotate_distance_ = distance
  walk_request.rotate_heading_ = heading
  commands.setStiffnessCommands(cfgStiffALWalk,0.1)

  walk_request.motion_ = core.WalkRequestBlock_WALK
end

function setWalkVelocityAbs(velX,velY,velTheta)
  commands.setStiffnessCommands(cfgStiffALWalk,0.1)

  walk_request:setWalk(velX,velY,velTheta)
  walk_request.percentage_speed_ = false
end

function setHeadPan(target_pos, target_time, isChange)
  if (isChange == nil) then isChange = false end
  -- make sure tilt is correct
  commands.setHeadTilt()
  
  joint_commands:setHeadPan(target_pos, target_time*1000.0, isChange)
end

function setHeadTilt()
  -- Todd: with 2 cameras, head tilt should ALWAYS be -21 deg
  local a = -21
  local id = robot_state.robot_id_
  if id == 29 then
    a = -23
  elseif id == 32 then
    a = -18.5
  elseif id == 38 then
    a = -19.5
  end
  joint_commands:setHeadTilt(DEG_T_RAD * a, 200.0, false)
end

function setHeadPanTilt(pan_target,tilt_target,target_time,isChange)
  setHeadTilt()
  setHeadPan(pan_target,target_time,isChange)
end

function setCameraParams(which_cam,camConfig)
  if which_cam == core.Camera_BOTTOM then
    camera_block.set_bottom_params_ = true
    targetParams = camera_block.params_bottom_camera_
  else
    camera_block.set_top_params_ = true
    targetParams = camera_block.params_top_camera_
  end

  -- set the params in commands struct
  targetParams.kCameraAutoWhiteBalance = camConfig.kCameraAutoWhiteBalance
  targetParams.kCameraExposureAuto = camConfig.kCameraExposureAuto
  targetParams.kCameraBacklightCompensation = camConfig.kCameraBacklightCompensation

  targetParams.kCameraBrightness = camConfig.kCameraBrightness
  targetParams.kCameraContrast = camConfig.kCameraContrast
  targetParams.kCameraSaturation = camConfig.kCameraSaturation
  targetParams.kCameraHue = camConfig.kCameraHue
  targetParams.kCameraExposure = camConfig.kCameraExposure
  targetParams.kCameraGain = camConfig.kCameraGain
  targetParams.kCameraSharpness = camConfig.kCameraSharpness

end


function setWalkMode(mode, init)
  if (init ~= true) and (mode == behavior_mem.walk_mode_) then
    return
  end

  --if mode == core.WalkMode_SLOW then
    --setALWalkParameters(cfgWalkSlow)
  --elseif mode == core.WalkMode_MID then
    --setALWalkParameters(cfgWalkMid)
  --elseif mode == core.WalkMode_SPRINT then
    --setALWalkParameters(cfgWalkSprint)
  --elseif mode == core.WalkMode_TARGET then
    --setALWalkParameters(cfgWalkTarget)
  --elseif mode == core.WalkMode_TARGET_CLOSE then
    --setALWalkParameters(cfgWalkTargetClose)
  --elseif mode == core.WalkMode_KICK then
    --setALWalkParameters(cfgWalkKick)
  --else
    --UTdebug.log(10,'Unknown walk mode requested, ignoring:',mode)
    --return
  --end
  --UTdebug.log(10,'Switching walk mode',core.WalkMode_getName(behavior_mem.walk_mode_),'->',core.WalkMode_getName(mode))
  UTdebug.log(0,'Ignoring walk mode, only using base params')
  setALWalkParameters(cfgALWalk2012)
  behavior_mem.walk_mode_ = mode
end

function setALWalkParameters(params)
  al_walk_param.send_params_ = true
  
  
  al_walk_param.maxStepX = params.maxStepX
  al_walk_param.maxStepY = params.maxStepY
  al_walk_param.maxStepTheta = params.maxStepTheta
  al_walk_param.maxStepFrequency = params.maxStepFrequency
  al_walk_param.stepHeight = params.stepHeight
  al_walk_param.torsoWx = params.torsoWx
  al_walk_param.torsoWy = params.torsoWy

  --al_walk_param.walk_max_trapezoid_ = params.walk_max_trapezoid_
  --al_walk_param.walk_min_trapezoid_ = params.walk_min_trapezoid_
  --al_walk_param.walk_step_max_period_ = params.walk_step_max_period_
  --al_walk_param.walk_step_min_period_ = params.walk_step_min_period_
  --al_walk_param.walk_max_step_x_ = params.walk_max_step_x_
  --al_walk_param.walk_max_step_y_ = params.walk_max_step_y_ 
  --al_walk_param.walk_max_step_theta_ = params.walk_max_step_theta_ 
  --al_walk_param.walk_step_height_ = params.walk_step_height_ 
  --al_walk_param.walk_foot_separation_ = params.walk_foot_separation_ 
  --al_walk_param.walk_foot_orientation_ = params.walk_foot_orientation_
  --al_walk_param.walk_torso_height_ = params.walk_torso_height_
  --al_walk_param.walk_inverted_pendulum_height_ = params.walk_inverted_pendulum_height_ 
  --al_walk_param.torso_orientation_x_ = params.torso_orientation_x_ 
  --al_walk_param.torso_orientation_y_ = params.torso_orientation_y_ 

  al_walk_param.fwd_odometry_factor_ = params.fwd_odometry_factor_
  al_walk_param.side_odometry_factor_ = params.side_odometry_factor_
  al_walk_param.turn_cw_odometry_factor_ = params.turn_cw_odometry_factor_
  al_walk_param.turn_ccw_odometry_factor_ = params.turn_ccw_odometry_factor_

  
  --if (vision_frame_info.source == core.MEMORY_SIM and robot_state.team_ == core.TEAM_BLUE) then
    ---- sim with blue team 1.75x faster
    --al_walk_param.fwd_odometry_factor_ = al_walk_param.fwd_odometry_factor_ * 1.75
    --al_walk_param.side_odometry_factor_ = al_walk_param.side_odometry_factor_ * 1.75
    --al_walk_param.turn_cw_odometry_factor_ = al_walk_param.turn_cw_odometry_factor_ * 1.75
    --al_walk_param.turn_ccw_odometry_factor_ = al_walk_param.turn_ccw_odometry_factor_ * 1.75

  --end

  -- TODO WALK ODOMETRY STUFF
  --saveCurrentALWalkParams()
end

function setKickSpline(params,spline)
  if spline == nil then
    return
  end

  params.num_swing_spline_pts = spline.swingNumPts
  local time = 0
  for i = 0,spline.swingNumPts-1 do
    time = time + spline.swingTimes[i+1]
    luaC:setDouble(params.spline_swing_times,i,time)
    luaC:setDouble(params.spline_swing_xs,i,spline.swingXs[i+1])
    luaC:setDouble(params.spline_swing_ys,i,spline.swingYs[i+1])
    luaC:setDouble(params.spline_swing_zs,i,spline.swingZs[i+1])
  end
    
  params.num_stance_spline_pts = spline.stanceNumPts
  time = 0
  for i = 0,spline.stanceNumPts-1 do
    time = time + spline.stanceTimes[i+1]
    luaC:setDouble(params.spline_stance_times,i,time)
    luaC:setDouble(params.spline_stance_xs,i,spline.stanceXs[i+1])
    luaC:setDouble(params.spline_stance_ys,i,spline.stanceYs[i+1])
    luaC:setDouble(params.spline_stance_zs,i,spline.stanceZs[i+1])
  end
end

function setKickParameters(params, paramsSuper)
  kick_params.send_params_ = true
  kick_params.params_ = params
  kick_params.params_super_ = paramsSuper
end

function setWalkParameters(params)
  walk_param.send_params_ = true
  
  -- max speeds
  walk_param.params_.max_step_size_ = createPose2D(params.max_step_size_)
  -- dimensions of walk
  walk_param.params_.foot_separation_ = params.foot_separation_
  walk_param.params_.walk_height_ = params.walk_height_
  walk_param.params_.pendulum_height_ = params.pendulum_height_

  walk_param.params_.step_height_ = params.step_height_
  walk_param.params_.phase_length_ = params.phase_length_ -- seconds for 1 step by 1 foot
  walk_param.params_.double_support_frac_ = params.double_support_frac_
  walk_param.params_.left_foot_zmp_offset_.x = params.left_foot_zmp_offset_.x
  walk_param.params_.left_foot_zmp_offset_.y = params.left_foot_zmp_offset_.y
  walk_param.params_.right_foot_zmp_offset_.x = params.right_foot_zmp_offset_.x
  walk_param.params_.right_foot_zmp_offset_.y = params.right_foot_zmp_offset_.y

  walk_param.params_.accel_sensor_delay_frames_ = params.accel_sensor_delay_frames_
  walk_param.params_.pen_sensor_delay_frames_ = params.pen_sensor_delay_frames_

  walk_param.params_.num_averaged_sensor_zmp_frames_ = params.num_averaged_sensor_zmp_frames_
  walk_param.params_.num_averaged_sensor_pen_frames_ = params.num_averaged_sensor_pen_frames_

  walk_param.params_.closed_loop_zmp_ = params.closed_loop_zmp_
  walk_param.params_.closed_loop_pen_ = params.closed_loop_pen_
  walk_param.params_.interp_zmp_forward_ = params.interp_zmp_forward_


  walk_param.params_.zmp_sensor_control_ratio_ = params.zmp_sensor_control_ratio_ 
  walk_param.params_.pen_sensor_control_ratio_ = params.pen_sensor_control_ratio_

  walk_param.params_.tilt_roll_factor_ = params.tilt_roll_factor_

  walk_param.params_.min_step_change_time_ = params.min_step_change_time_
  walk_param.params_.lift_start_time_ = params.lift_start_time_
  walk_param.params_.lift_stop_time_ = params.lift_stop_time_
  walk_param.params_.step_start_time_ = params.step_start_time_
  walk_param.params_.step_stop_time_ = params.step_stop_time_
  walk_param.params_.step_speed_factor_ = params.step_speed_factor_

  walk_param.params_.hip_roll_offset_amount_ = params.hip_roll_offset_amount_
  walk_param.params_.hip_roll_offset_rise_frac_ = params.hip_roll_offset_rise_frac_
  walk_param.params_.hip_roll_offset_fall_frac_ = params.hip_roll_offset_fall_frac_
  walk_param.params_.hip_roll_offset_start_frac_ = params.hip_roll_offset_start_frac_
  walk_param.params_.hip_roll_offset_stop_frac_ = params.hip_roll_offset_stop_frac_

  walk_param.params_.swing_tilt_amount_ = params.swing_tilt_amount_
  walk_param.params_.swing_tilt_start_frac_ = params.swing_tilt_start_frac_
  walk_param.params_.swing_tilt_stop_frac = params.swing_tilt_stop_frac_

  saveCurrentWalkParams(params)
end

function saveCurrentHTWKWalkParams()
  -- currently just copied from HTWKWalkModule.cpp
  walk_max_vel_x = 360-- * 0.75
  walk_max_vel_y = 90
  walk_max_vel_rot_cw = 0.4
  walk_max_vel_rot_ccw = 0.4
  walk_max_vel_rot = 0.4
end

function saveCurrentALWalkParams()
  --current_walk_cfg = params
  walk_max_vel_rot_cw = 0.5 * al_walk_param.turn_cw_odometry_factor_* DEG_T_RAD*al_walk_param.walk_max_step_theta_ / (al_walk_param.walk_step_min_period_ / 50.0) 
  walk_max_vel_rot_ccw = 0.5 * al_walk_param.turn_ccw_odometry_factor_* DEG_T_RAD*al_walk_param.walk_max_step_theta_ / (al_walk_param.walk_step_min_period_ / 50.0)
  walk_max_vel_rot = 0.5 * (walk_max_vel_rot_cw + walk_max_vel_rot_ccw)
  walk_max_vel_x = al_walk_param.fwd_odometry_factor_*1000.0 *al_walk_param.walk_max_step_x_ / (al_walk_param.walk_step_min_period_ / 50.0) 
  walk_max_vel_y = al_walk_param.side_odometry_factor_*1000.0 *al_walk_param.walk_max_step_y_ / (al_walk_param.walk_step_min_period_ / 50.0) 
  --UTdebug.log(0,'Max Vel (fwd, side, turn): ',walk_max_vel_x, walk_max_vel_y, RAD_T_DEG*walk_max_vel_rot)
end



function saveCurrentWalkParams()
  --current_walk_cfg = params
  -- Todd: 0.5* because we have to take 2 steps for each foot to move one step_size
  walk_max_vel_rot_cw = 0.5 * walk_param.params_.max_step_size_.rotation / walk_param.params_.phase_length_
  walk_max_vel_rot_ccw = 0.5 * walk_param.params_.max_step_size_.rotation / walk_param.params_.phase_length_
  walk_max_vel_rot = 0.5 * (walk_max_vel_rot_cw + walk_max_vel_rot_ccw)

  walk_max_vel_x = 0.5 * walk_param.params_.max_step_size_.translation.x / walk_param.params_.phase_length_
  walk_max_vel_y = 0.5 * walk_param.params_.max_step_size_.translation.y / walk_param.params_.phase_length_
  --UTdebug.log(0,'Max Fwd Vel: ',walk_max_vel_x)
end

function createVector3(intable)
  return core.vector3_float(intable[1],intable[2],intable[3])
end

function createVector2(intable)
  return core.vector2_float(intable[1],intable[2])
end

function createPose2D(intable)
  return core.Pose2D(intable[1],intable[2],intable[3])
end


function setStiffnessCommands(cfgStiff, time)

  -- only set if we're not already at this stiffness
  if (isAtStiffness(cfgStiff)) then
    return
  end

  -- set in the joint commands block
  for i=0, core.NUM_JOINTS-1 do
    joint_commands:setJointStiffness(i, cfgStiff[i])
  end
  
  -- set time and send_stiffness_
  joint_commands.send_stiffness_ = true
  joint_commands.stiffness_time_ = time * 1000.0
  
end

function isAtStiffness(cfgStiff)
  for i = 0, core.NUM_JOINTS-1 do
    local stiff = luaC:getFloat(luaC.joint_angles_.stiffness_,i)
    local error = math.abs(stiff - cfgStiff[i])
    -- return false if error > 0.05
    if (error > 0.05) then 
      return false 
    end
  end

  return true
end

function setArmPose(armPose,time)
  --joint_commands.send_arm_angles_ = true
  joint_commands.send_body_angles_ = true
  joint_commands.arm_command_time_ = time
  for joint,val in pairs(armPose) do
    joint_commands:setJointCommandDeg(joint,val)
  end
end
