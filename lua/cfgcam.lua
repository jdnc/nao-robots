-- params at robocup (bottom camera), set using Gene
-- i just reduced exposure on both top and bottom camera from the lab params

cfgCamRoboCup = {}

cfgCamRoboCup.kCameraAutoWhiteBalance = 0
cfgCamRoboCup.kCameraExposureAuto = 0 
cfgCamRoboCup.kCameraBacklightCompensation = 1
cfgCamRoboCup.kCameraBrightness = 220
cfgCamRoboCup.kCameraContrast = 53 -- your choice (20-53)
cfgCamRoboCup.kCameraSaturation = 128 -- your choice (0-255)
cfgCamRoboCup.kCameraHue = 0
cfgCamRoboCup.kCameraExposure = 0 -- leave at 0 otherwise frame rate gets hit
cfgCamRoboCup.kCameraGain = 32 -- about 200 is good
cfgCamRoboCup.kCameraSharpness = 0

-- params at TTU lab...
cfgCamTTUBot = {}

cfgCamTTUBot.kCameraAutoWhiteBalance = 0 
cfgCamTTUBot.kCameraExposureAuto = 0 
cfgCamTTUBot.kCameraBacklightCompensation = 0 
cfgCamTTUBot.kCameraBrightness = 115 
cfgCamTTUBot.kCameraContrast = 63
cfgCamTTUBot.kCameraSaturation = 120 
cfgCamTTUBot.kCameraHue = 0 
cfgCamTTUBot.kCameraExposure = 101  
cfgCamTTUBot.kCameraGain = 24 
cfgCamTTUBot.kCameraSharpness = 0 

cfgCamTTUTop = {}

cfgCamTTUTop.kCameraAutoWhiteBalance = 0 
cfgCamTTUTop.kCameraExposureAuto = 0
cfgCamTTUTop.kCameraBacklightCompensation = 0 
cfgCamTTUTop.kCameraBrightness = 120 --0:255
cfgCamTTUTop.kCameraContrast = 43 --0:127
cfgCamTTUTop.kCameraSaturation = 90 --0:256
cfgCamTTUTop.kCameraHue = 0 -- -180:180
cfgCamTTUTop.kCameraExposure = 101
cfgCamTTUTop.kCameraGain = 26
cfgCamTTUTop.kCameraSharpness = 0

CamOffset = {}
CamOffset.__index = CamOffset
function CamOffset.create(rel_ball_side,rel_ball_fwd)
  local tab = {}
  setmetatable(tab,CamOffset)
  tab.rel_ball_side = rel_ball_side
  tab.rel_ball_fwd = rel_ball_fwd
  return tab
end

-- camera offsets (only all now)
cfgCamOffsets = {} -- pixels
-- rel_ball_side, rel_ball_fwd
cfgCamOffsets[0] = CamOffset.create(0,0) -- default
cfgCamOffsets[27] = CamOffset.create(-3, -92) -- 6/26/12 at 9:28am by sbarrett
cfgCamOffsets[28] = CamOffset.create(14.0, -107) -- 6/29/13 by katie
cfgCamOffsets[29] = CamOffset.create(-6.8, -99) -- 6/28/13 1:02pm by sbarrett
cfgCamOffsets[30] = CamOffset.create(-7.3, -120) -- 6/19/13 by katie
cfgCamOffsets[31] = CamOffset.create(0,-108) -- 5/30/13 by sbarrett
cfgCamOffsets[32] = CamOffset.create(6,-110) -- 6/19/13 by katie
cfgCamOffsets[33] = CamOffset.create(-7.4, -78) -- 6/29/13 at 5:19pm by sbarrett
cfgCamOffsets[34] = CamOffset.create(-2.7, -99) -- 6/29/13 by katie
cfgCamOffsets[35] = CamOffset.create(-5.2, -102) -- 6/29/13 by katie
cfgCamOffsets[36] = CamOffset.create(-7.9, -116) -- 6/29/13 by katie
cfgCamOffsets[37] = CamOffset.create(-9.5, -126) -- 6/29/13 by katie 
cfgCamOffsets[38] = CamOffset.create(0.2, -126) -- 6/29/13 at 9:08pm by sbarrett

function getCamOffsets()
  local offsets = cfgCamOffsets[robot_state.robot_id_]
  if offsets == nil then
    offsets = cfgCamOffsets[0]
  end
  --offsets.scale_side = 0.772 -- everyone uses the same for now
  offsets.scale_side = 1.0 -- everyone uses the same for now
  return offsets
end
