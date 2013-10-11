DEG_T_RAD = math.pi / 180.0
RAD_T_DEG = 180.0 / math.pi

require 'behavior'
require 'percepts'
require 'cfgcam'
require 'cfgwalk'
require 'commands'
require 'config'
require 'field'
require 'cfgalwalk'
require 'cfghtwkwalk'
require 'cfgstiff'
require 'cfgkick'
require 'robotPositions'
require 'roleSwitch'
require 'task'

-- This function is run to initialise everything
function init()
  if core then
    print "Creating Lua link to C++ Core"  
    robot = core.VisionCore_inst_
  else
    print "***ERROR*** Can't create Lua link to C++ Core" 
  end

  visionC = robot.vision_
  localizationC = robot.localization_
  perfectC = robot.perfect_localization_
  offFieldLocalizationC = robot.off_field_localization_
  opponentsC = robot.opponents_ 
  behaviorC = robot.behavior_
  ledsC = robot.leds_

  initMemory()
  initNonMemory()
end

function initNonMemory()
  -- init localization params
  config.configLocalization()
  localizationC:reInit()
  opponentsC:reInit()
  world_objects:init(robot_state.team_)

  -- init which side of the field blue is one
  localization_mem.blueSide = core.Sides_NOSIDES;
  --localization_mem.blueSide = core.Sides_RIGHTSIDE;
  --localization_mem.blueSide = core.Sides_LEFTSIDE;

  kicks.checkKicks() -- must be before setKickAngles
  strategy.initKickRegion()
  --strategy.setKickAngles()
  strategy.setStrategy()
  
  cfghtwkwalk.initWalk()
  cfgwalk.initWalk()
  -- set these here so it works for behavior simulation
  setRobotSpecificParams() -- accomplishes the sadness of setting the robot specific parameters
  commands.setKickParameters(cfgStraightKick, cfgStraightSuperKick)
  --commands.setKickParameters(cfgStraightKick,cfgStraightKickSpline)
  -- al walk
  commands.setWalkMode(core.WalkMode_SLOW,true)
  commands.saveCurrentHTWKWalkParams()

  -- setup robot positions
  robotPositions.initializePositions()

  first_frame = true
  announced_vision = false
end

function initMemory()
  luaC = robot.lua_
  
  behavior_mem = luaC.behavior_
  camera_block = luaC.camera_block_
  game_state = luaC.game_state_
  kick_request = luaC.kick_request_
  odometry = luaC.odometry_
  robot_state = luaC.robot_state_
  sensors = luaC.sensors_
  vision_frame_info = luaC.vision_frame_info_
  walk_param = luaC.walk_param_
  kick_params = luaC.kick_params_
  walk_request = luaC.walk_request_
  world_objects = luaC.world_objects_
  team_packets = luaC.team_packets_
  opponent_mem = luaC.opponents_  
  behavior_params = luaC.behavior_params_
  joint_commands = luaC.joint_commands_
  processed_sonar = luaC.vision_processed_sonar_
  al_walk_param = luaC.al_walk_param_
  walk_info = luaC.walk_info_
  robot_vision = luaC.robot_vision_
  body_model = luaC.body_model_
  robot_info = luaC.robot_info_
  speech = luaC.speech_
  localization_mem = luaC.localization_

  camera_offset = luaC:getFloat(robot_info.dimensions_.values_, core.RobotDimensions_tiltOffsetToBottomCamera) - luaC:getFloat(robot_info.dimensions_.values_, core.RobotDimensions_tiltOffsetToTopCamera)

  text_logger = robot.textlog_

  --joint_angles = luaC.joint_angles_
  joint_angles = percepts.joint_angles

end

function errHandler(x)
  UTdebug.log(0,'LUA ERROR:')
  UTdebug.log(0,x)
  UTdebug.log(0,debug.traceback())
  speech:say("lua")
end

-- This is where lua gets called from the vision thread
function processFrame()
  robot:preVision() -- get the vision lock and copy over necessary data from both the motion process and the image capture thread  

  if first_frame then
    first_frame = false
    world_objects:init(robot_state.team_)

    -- init camera parameters
    commands.setCameraParams(core.Camera_BOTTOM,cfgCamRoboCup)
    commands.setCameraParams(core.Camera_TOP,cfgCamRoboCup)

    setRobotSpecificParams() -- accomplishes the sadness of setting the robot specific parameters

    commands.setKickParameters(cfgStraightKick, cfgStraightSuperKick)
    --commands.setKickParameters(cfgStraightKick,cfgStraightKickSpline)

    -- init walk parameters
    -- al walk
    commands.setWalkMode(core.WalkMode_SLOW,true)
    cfghtwkwalk.initWalk()
    cfgwalk.initWalk()

    -- vision stuff
    if (visionC ~= nil) then
      visionC:initSpecificModule()
    end

    --[[
    if (processed_sonar.sonar_module_enabled_) then
      speech:say("Sonar Enabled")
    else
      speech:say("Sonar Disabled")
    end
    ]]--
  end

  if not(announced_vision) and camera_block.cameras_tested_ then
    speech:say("Vision")
    announced_vision = true
  end


  UTdebug.log(10, "odom, dive, walk_req", odometry.getting_up_side_, odometry.fall_direction_, behavior_mem.keeperDiving, walk_request.motion_)
  -- not getting up, not falling, and not diving
  if (odometry.getting_up_side_ == core.Getup_NONE and odometry.fall_direction_ == core.Fall_NONE and behavior_mem.keeperDiving == core.Dive_NONE and game_state.state ~= core.TEST_ODOMETRY) then 
    if (visionC ~= nil) then
      visionC:processFrame()
    end
  else
    -- reset vision of everything
    world_objects:reset()
  end
  
  robot:postVision() -- release the lock

  percepts.populateLocalCopy()
  checkArmBumpers() -- fill in arm bump contacts
  commands.resetTiltOffset()

  if (offFieldLocalizationC ~= nil) then
    offFieldLocalizationC.processFrame() -- NOTE: this should happen before localization's process frame because it fills out information for localization
  end
  --perfectC:processFrame()
  localizationC:processFrame()
  opponentsC:processFrame()
  local camOffsets = getCamOffsets()
  localizationC:filterCloseBallPosition(camOffsets.rel_ball_fwd,camOffsets.rel_ball_side,camOffsets.scale_side)

  -- do this after setting other stuff
  behavior.processFrame()

  commands.handleTiltOffset()

  --checkSonarStatus();
  --walk_request:setWalk(1.0,0.0,0)

  doLights()

  robot:publishData() -- copy commands back to the motion process and copy camera info back to the image capture thread
  --robot:logMemory()
end


function doLights()
  doFootLights()
  doStateLights()
  doEyeLights()
  doEarLights() 
end

function doEarLights()
  if (game_state.state == core.INITIAL or
      game_state.state == core.FINISHED) then
    doInitialEarLights()
  else
    doPlayingEarLights()
  end
end

function doInitialEarLights()
  -- display last time we heard from each teammate
  local p1frames = (vision_frame_info.frame_id - team_packets:getFrameReceived(1))
  local p2frames = (vision_frame_info.frame_id - team_packets:getFrameReceived(2))
  local p3frames = (vision_frame_info.frame_id - team_packets:getFrameReceived(3))  
  local p4frames = (vision_frame_info.frame_id - team_packets:getFrameReceived(4))
  local p5frames = (vision_frame_info.frame_id - team_packets:getFrameReceived(5))

  if (p1frames < 30) then
    ledsC:partLeftEar(1, 0, 4)
  else
    ledsC:partLeftEar(0, 0, 4)
  end

  if (p2frames < 30) then
    ledsC:partLeftEar(1, 5, 4)
  else
    ledsC:partLeftEar(0, 5, 4)
  end
 
  -- front top
  if (p3frames < 30) then
    ledsC:partRightEar(1, 0, 3)
  else
    ledsC:partRightEar(0, 0, 3)
  end

  -- bottom
  if (p4frames < 30) then
    ledsC:partRightEar(1, 4, 2)
  else
    ledsC:partRightEar(0, 4, 2)
  end
 
  -- back top
  if (p5frames < 30) then
    ledsC:partRightEar(1, 7, 2)
  else
    ledsC:partRightEar(0, 7, 2)
  end

end


function doPlayingEarLights()
  -- chaser
  if (robot_state.role_ == core.CHASER) then
    ledsC:frontRightEar(1)
  else
    ledsC:frontRightEar(0)
  end

  -- I'd like not to send this every frame, rather
  -- only send when seen changes
  if (world_objects:getObjPtr(core.WO_BALL).seen) then
    ledsC:frontLeftEar(1)
  else
    ledsC:frontLeftEar(0)
  end

  -- default to off
  ledsC:backLeftEar(0)
  ledsC:backRightEar(0)

  -- left arm bumper
  if (processed_sonar.bump_left_) then
    ledsC:backLeftEar(1)
  end
  if (processed_sonar.bump_right_) then
    ledsC:backRightEar(1)
  end
  
  -- sonar avoidance
  --ledsC:frontLeftEar(0)
  -- turn some on
  --local minSonarDist = 0.75
  --if processed_sonar.on_center_ and (processed_sonar.center_distance_ < minSonarDist) then
  --  ledsC:backLeftEar(1)
  --  ledsC:backRightEar(1)
  --  --ledsC:frontLeftEar(1)
  --end
  --if processed_sonar.on_left_ and (processed_sonar.left_distance_ < minSonarDist) then
  --  ledsC:backLeftEar(1)
  --end
  --if processed_sonar.on_right_ and (processed_sonar.right_distance_ < minSonarDist) then
  --  ledsC:backRightEar(1)
  --  --ledsC:frontLeftEar(1)
  --end
end


function doStateLights()
  -- do chest led based on state
  state = game_state.state
  if (state == core.INITIAL) then 
    ledsC:chest(0,0,0)
  elseif (state == core.READY) then
    ledsC:chest(0,0,1)
  elseif (state == core.SET) then
    ledsC:chest(1,1,0)
  elseif (state == core.PLAYING) then
    ledsC:chest(0,1,0)
  elseif (state == core.PENALISED) then
    ledsC:chest(1,0,0)
  elseif (state == core.TESTING) then
    ledsC:chest(1,0,1)
  elseif (state == core.FINISHED) then
    ledsC:chest(1,1,1)
  end
end


function doFootLights()
  -- do feet leds
  if (robot_state.team_== 0) then --blue
    ledsC:rightFoot(0,0,1)
  elseif (robot_state.team_==1) then --red
    ledsC:rightFoot(1,0,0)
  end
  
  if (game_state.ourKickOff) then
    ledsC:leftFoot(1,1,1)
  else
    ledsC:leftFoot(0,0,0)
  end
end


function doEyeLights()
  if (game_state.state == core.INITIAL) then
    doInitialEyeLights()
  elseif (game_state.state == core.FINISHED) then
    doFinishedEyeLights()
  else
    doPlayingEyeLights()
  end
  if (robot_state.ignore_comms_) then
    ledsC:allLeftEye(1,0,0)
  end
end

function doInitialEyeLights() 
  doEyeBalls()
  doEyePower()
  doEyeHeat()
end

function doFinishedEyeLights()
  doEyeBalls()
  doEyePower()
  doEyeHeat()
end

function doPlayingEyeLights()
  --doLeftEyeObjects()
  --doRightEyeSonar()
  doLeftEyeKicks()
  doRightEyeKicks()
end

function doLeftEyeObjects()
  local goal = world_objects:getObjPtr(core.WO_UNKNOWN_GOAL)
  local circle = world_objects:getObjPtr(core.WO_CENTER_CIRCLE)
  if goal.seen then
    ledsC:allLeftEye(0,0,1)
  elseif circle.seen then
    ledsC:allLeftEye(1,0,0)
  else
    ledsC:allLeftEye(0,0,0)
  end
end

function doRightEyeSonar()
  if not(processed_sonar.sonar_module_enabled_) then
    ledsC:allRightEye(0,0,0)
  else
    if processed_sonar.on_left_ then
      ledsC:allRightEye(1,0,0)
    elseif processed_sonar.on_center_ then
      ledsC:allRightEye(0,1,0)
    elseif processed_sonar.on_right_ then
      ledsC:allRightEye(0,0,1)
    else
      ledsC:allRightEye(1,1,1)
    end
  end
end

function doEyePower()
  local battery = sensors:getValue(core.battery)
  if (battery > 0.95) then
    ledsC:allBottomLeftEye(1,1,1)
    return false
  elseif (battery > 0.8) then
    ledsC:allBottomLeftEye(1,1,0)
    return false
  elseif (battery > 0.65) then
    ledsC:allBottomLeftEye(1,.5,1)
    return false
  else
    ledsC:allBottomLeftEye(1,0,0)
    return true
  end
end

function doEyeHeat()
  local max_temp = getMaxTemp()
  if (max_temp > 74) then
    ledsC:allTopLeftEye(1,0,0)
    return true
  elseif (max_temp > 64) then
    ledsC:allTopLeftEye(1,.5,0)
    return true
  elseif (max_temp > 54) then
    ledsC:allTopLeftEye(1,1,0)
    return true
  else
    ledsC:allTopLeftEye(1,1,1)
    return false
  end
end

function doEyeBalls()
  local ball = world_objects:getObjPtr(core.WO_BALL)
  if (ball.seen) then
    if ball.fromTopCamera then
      ledsC:allTopRightEye(0,0,1)
      ledsC:allBottomRightEye(0,1,0)
    else
      ledsC:allTopRightEye(0,1,0)
      ledsC:allBottomRightEye(0,0,1)
    end
  else
    ledsC:allTopRightEye(0,1,0)
    ledsC:allBottomRightEye(0,1,0)
  end
end


function doLeftEyeKicks()
  ledsC:allLeftEye(0,0,0)
  if (behavior_mem.chooseKick) then
    if (behavior_mem.kickChoice == core.FwdLongStraightKick or
        behavior_mem.kickChoice == core.FwdLongLeftwardOutKick or
        behavior_mem.kickChoice == core.FwdLongLeftwardinKick) then
      ledsC:allLeftEye(1,0,0)
    elseif (behavior_mem.kickChoice == core.FwdMediumStraightKick or
        behavior_mem.kickChoice == core.FwdMediumLeftwardOutKick or
        behavior_mem.kickChoice == core.FwdMediumLeftwardinKick) then
      ledsC:allLeftEye(0,1,0)
    elseif (behavior_mem.kickChoice == core.WalkKickFront or
            behavior_mem.kickChoice == core.WalkKickLeftward) then
      ledsC:allLeftEye(0,0,1)
    elseif (behavior_mem.kickChoice == core.WalkKickLeftwardSide) then
      ledsC:allLeftEye(1,1,1)
    end
  end
end

function doRightEyeKicks()
  ledsC:allRightEye(0,0,0)
  if (behavior_mem.chooseKick) then
    if (behavior_mem.kickChoice == core.FwdLongStraightKick or
        behavior_mem.kickChoice == core.FwdLongRightwardOutKick or
        behavior_mem.kickChoice == core.FwdLongRightwardinKick) then
      ledsC:allRightEye(1,0,0)
    elseif (behavior_mem.kickChoice == core.FwdMediumStraightKick or
        behavior_mem.kickChoice == core.FwdMediumRightwardOutKick or
        behavior_mem.kickChoice == core.FwdMediumRightwardinKick) then
      ledsC:allRightEye(0,1,0)
    elseif (behavior_mem.kickChoice == core.WalkKickFront or
            behavior_mem.kickChoice == core.WalkKickRightward) then
      ledsC:allRightEye(0,0,1)
    elseif (behavior_mem.kickChoice == core.WalkKickRightwardSide) then
      ledsC:allRightEye(1,1,1)
    end
  end
end

function getMaxTemp()
  local max_temp = 0
  for i = 0, core.NUM_JOINTS-1 do
    local temp = sensors:getJointTemperature(i)
    if (temp > max_temp) then
      max_temp = temp
    end
  end
  return max_temp
end

function setRobotSpecificParams()
  setRobotSpecificWalkParams()
  setRobotSpecificKickParams()
  setRobotSpecificSittingPose()
end

function checkArmBumpers()

  -- if we're not walking, no bumps

  -- Removed extraneous checks for left/right bumps - JM 6/19/12
  --if (odometry.standing or odometry.getting_up_side_ ~= core.Getup_NONE or odometry.fall_direction_ ~= core.Fall_NONE) then
  if (odometry.standing) then
    processed_sonar.bump_left_ = false
    processed_sonar.bump_right_ = false
    return
  end

  -- not walking, no bumps
  --if (walk_info.robot_velocity_frac_.translation.x == 0 and walk_info.robot_velocity_frac_.translation.y == 0 and walk_info.robot_velocity_frac_.rotation == 0) then
  --  processed_sonar.bump_left_ = false
  --  processed_sonar.bump_right_ = false
  -- print("throw 2")
  --  return
  --end

  -- not walking, no bumps
  --if (odometry.displacement.translation.x == 0 and odometry.displacement.translation.y == 0 and odometry.displacement.rotation == 0) then
  --  processed_sonar.bump_left_ = false
  --  processed_sonar.bump_right_ = false
  -- print("throw 3")
  --  return
  -- end

  processed_sonar.bump_left_ = checkArmBumper(core.LShoulderPitch)
  processed_sonar.bump_right_ = checkArmBumper(core.RShoulderPitch)

  if (processed_sonar.bump_left_) then
    UTdebug.log(10, "bump left")
    --speech:say("left")
  end
  if (processed_sonar.bump_right_) then
    UTdebug.log(10, "bump right")
    --speech:say("right")
  end
end

function checkSonarStatus()
  --UTdebug.log(0, "checking sonar status: ",processed_sonar.sonar_module_update_)
  if (processed_sonar.sonar_module_update_) then
    if (processed_sonar.sonar_module_enabled_) then
      --speech:say("sonar enabled")
    else
      --speech:say("sonar disabled")
    end
  end
end

function checkArmBumper(start)
  if (game_state.state ~= core.READY) and (game_state.state ~= core.PLAYING) then
    return false
  end
  -- shoulder pitch, shoulder roll, elbow yaw, elbow roll
  local maxAngles = {}
  local minAngles = {}
  --if (walk_request.keep_arms_out) then -- this should work but it doesn't
  if (robot_state.WO_SELF == 1) then -- goal keeper keeps his arms out
    maxAngles[1] = -116 + 10
    maxAngles[2] = 12 + 1
    maxAngles[3] = -85
    maxAngles[4] = 0

    minAngles[1] = -116 - 10
    minAngles[2] = 12 - 1
    minAngles[3] = -85
    minAngles[4] = 0
  else
    maxAngles[1] = -116 + 5
    maxAngles[2] = 8
    maxAngles[3] = 25 + 25
    maxAngles[4] = -53 + 25
  
    minAngles[1] = -116 - 5
    minAngles[2] = 8
    minAngles[3] = 25 - 25
    minAngles[4] = -53 -25
  end 

  local errorThresh = 3.0

  --for i = start, start+3 do
  for i = start+1, start+1 do -- only care about shoulder roll
    local value = percepts.joint_angles[i] * RAD_T_DEG
    local minValue = minAngles[i-start+1]
    local maxValue = maxAngles[i-start+1]
    if (value > maxValue+errorThresh or value < minValue-errorThresh) then
      UTdebug.log(10, "Arm bump from joint",luaC:getString(core.JointNames,i), "value", value, "min", minValue-errorThresh, "max",maxValue+errorThresh)
      return true
    end

  end

  return false

end

function shouldSeeObject(WO,maxDistToSee)
  local me = world_objects:getObjPtr(robot_state.WO_SELF)
  local obj = world_objects:getObjPtr(WO)
  local relLoc = obj.loc:globalToRelative(me.loc,me.orientation)
  if relLoc:getMagnitude() > maxDistToSee then
    return false
  end

  local FOVfactor = 0.75 -- we might not see it on the edges
  local FOV = (core.FOVx / 2.0) * FOVfactor

  local bearing = me.loc:getBearingTo(obj.loc,me.orientation + joint_angles[core.HeadPan])
  --local relVector = core.vector3_float(relLoc.x,relLoc.y,0) -- z doesn't matter
  --local bearing = topCameraMatrix.bearing(relVector)
  return math.abs(bearing) < FOV
end

function rerequire(modules)
  --print('WARNING: not reloading init.lua')
  package.loaded.task = nil
  require 'task'
  RELOADING_LUA = true
  for _,module in ipairs(modules) do
    --print ('MODULE: ',module)
    package.loaded[module] = nil
    require(module)
  end
  RELOADING_LUA = false
  --print('before initNonMemory')
  initNonMemory()
  --print('DONE WITH REREQUIRE')
end



-- code that runs on require
init()

