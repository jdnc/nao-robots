require 'io'
require 'UTdebug'

module(..., package.seeall);

-- output
-- 0 - none
-- 1 - errors - default
-- 2 - all transitions

-- Generic state machine
StateMachine = {}
StateMachine.__index = StateMachine

function StateMachine.create(name,stateNames)
  local ste = {} 
  setmetatable(ste,StateMachine)
  ste.name = name
  ste.states = {}
  if (stateNames ~= nil) then
    for i=1,#stateNames do
      ste.states[stateNames[i]] = stateNames[i]
    end
    ste.current = stateNames[i]
  else
    ste.current = 'INVALID'
  end

  ste.transition_frame = 0
  ste.transition_time = 0
  ste.output = 1
  return ste
end

function StateMachine:transition(newState)
  if (self.states[newState] == nil) then
    if (self.output > 0) then
      io.write('Lua StateMachine(',name,'):transition: attempt to transition to unknown state, from ',self.current,' -> ',newState,'\n')
    end
    return
  end
  if (self.output > 1) then
    UTdebug.log(0,self.name,' transitioned from ',self.current,' -> ',newState)
  end
  self.transition_frame = vision_frame_info.frame_id
  self.transition_time = vision_frame_info.seconds_since_start
  self.current = newState
end

function StateMachine:numStates()
  return #self.states
end

function StateMachine:printStates()
  io.write ("Printing states in : ",self.name,"\n")
  for k,v in pairs(self.states) do io.write("  ",k,"\n") end
end

function StateMachine:setOutput(output)  -- set the ouput level
  self.output=output
end

function StateMachine:inState(newState)
  return self.current == newState
end

function StateMachine:isFirstFrameInState()
  return self.transition_frame == vision_frame_info.frame_id
end

-- for oldkicks.lua

function StateMachine:currentName()
  return self.current
end

function StateMachine:transitionName(name)
  self:transition(name)
end

function StateMachine:checkFirstTimeInState()
  return self:isFirstFrameInState()
end

function StateMachine:timeSinceTransition()
  return vision_frame_info.seconds_since_start - self.transition_time
end
