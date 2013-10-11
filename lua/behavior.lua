require 'soccerBvr'
require 'field'
module(..., package.seeall)

function processFrame()

  -- Todd: these are here so they get called before behavior
  -- even when we're ONLY running behavior in world window sim
  if (vision_frame_info.source == core.MEMORY_SIM) then
    percepts.populateLocalCopy()
  end

  field.processFrame()
  -- for debug, set all kicks to unevaluated
  behavior_mem:setAllUnevaluated()
  strategy.resetPassInfo()

  --print "lua behavior process frame"

  --- Edit this line to change the behaviour being run
  --print (game_state)
  --print (game_state.state)
  --commands.stand()
  --walk_request:noWalk()
  --kick_request:setFwdKick(core.Kick_RIGHT,3000)

  root = SoccerBehavior:set()
  
  --Perform task
  if root.__time == nil then
    root.__time = timer()
  end
  local msg = root:run()

  resetDirty()

  -- Generates a trace of all function calls and variables
  -- and logs them with loglevel 25
  -- so we can print them to screen or to file

  -- in sim, or if we're logging behavior trace
  if behavior_mem.log_behavior_trace_ or vision_frame_info.source == core.MEMORY_SIM then
    -- TODO only doing in sim now, should do in tool
    root:trace()
  end

end


