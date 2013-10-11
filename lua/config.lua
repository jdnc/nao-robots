-- Change which files are called to load a different configuration

require 'cfgukf'
require 'cfgopploc'

--require 'walkLearning'
--require 'kickLearning'

module(..., package.seeall);

function fillParamStruct(struct, tbl)
  local fields = getmetatable(struct)['.set']
  for k, v in pairs(tbl) do
    if fields[k] == nil then
      error('Attempted to set unknown field: ' .. tostring(k))
    else
      struct[k] = v
    end
  end
  -- Now make sure everything was set
  for k, v in pairs(fields) do
    if tbl[k] == nil then
      print('Warning: field "' .. tostring(k) ..
          '" did not get set')
    end
  end
end

function configLocalization()
  print ('Lua:Loading Localization Configuration Files')

  -- config localization ukf
  ukfParams = localizationC.ukfParams
  --fillParamStruct(ukfParams, cfgUKFLearned6)
  fillParamStruct(ukfParams, cfgUKFHandTuned2012)

  -- config opp tracker
  oppParams = opponentsC.ukfParams
  fillParamStruct(oppParams, cfgOppHandTuned)
end

--[[
-- call class to set parameters
function configAll()

  -- get param struct
  logParams = memoryC.lp
  configLog()

  configLocalization()

  -- fill struct
  fillParamStruct(logParams, cfgLog)

  -- fill vision params
  visionParams = visionC.visionParams
  fillParamStruct(visionParams, cfgVision)

end
--]]


