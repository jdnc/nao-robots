-- Infrastructure code for tasks

require 'util'
require 'UTdebug'

-- Value used to set initial field value to nil
NIL_INIT = 'NIL_INIT'

RELOADING_LUA = false

-- List of Tasks with at least one field's dirty bit set
__dirtyList = {}

-- Composite Tasks

__CompTaskMT = {}

function __CompTaskMT:__init(parent)
  self.__time = timer()
  self.__parent = parent
  if self.__as ~= nil then self.__as.__parent = nil end
  self.__as = NullTask
  for k, v in pairs(self.__initTable) do
    if not self.__dirty[k] then
      if v == NIL_INIT then
        self[k] = nil
      else
        self[k] = v
      end
    end
  end
end

function __CompTaskMT:run()
  if self.choose == nil then
    error('Cannot run ' .. self.__name .. '.  "choose" method undefined')
  end
  local time
  if self.__as then
    time = self.__as:getTime()
  end
  -- we'll initialize whatever child we'll choose if we don't have one
  local initSubtask = false
  if self.__as == nil then
    initSubtask = true
  end
  local retTask = self:choose(self.__as, time)
  if retTask == nil then
    if self.__as == nil then
      error(self.__name .. ':choose() returned nil with nil activeSubtask')
      --print ("bottom of tree ", self.__name)
    else
      retTask = self.__as
    end
  end
  if self.__as ~= nil and self.__as ~= retTask then
    -- if we're returning something different, set old as's parent to nil
    -- so that its time will be reset if/when this subtask is restarted later
    self.__as.__parent = nil
  end
  self.__as = retTask
  if self.__as.__parent ~= self or initSubtask then
    self.__as:__init(self)
  end
  return self.__as:run()
  --return self.__as:choose()
end

function __CompTaskMT:trace(level)
  level = level or 0
  addTaskLabel(level, self.__name .. ' (Comp) Time: ' .. self:getTime())
  for k, v in pairs(self.__initTable) do
    addTaskLabel(level+1, '.' .. k .. ' = ' .. tostring(self[k]))
  end
  if self.__as then
    self.__as:trace(level+2)
  else
    addTaskLabel(level+2, '[None]')
  end
end

function __CompTaskMT:set(tbl)
  if tbl then
    for k, v in pairs(tbl) do
      self[k] = v
      self.__dirty[k] = true
      __dirtyList[self] = true
    end
  end
  return self
end


function __CompTaskMT:getTime()
  if self.__time == nil then
    return 0
  end
  return self:__time()
end

function __CompTaskMT:resetTime()
  self.__time = timer()
end

function __CompTaskMT:getActiveSubtask()
  return self.__as
end

function __CompTaskMT:finished()
  return false
end

-- Warning this should only be called using self:setMe
function __CompTaskMT:setMe(tbl)
  __CompTaskMT.set(self,tbl)
end

function __CompTaskMT:__checkField(field, value)
  if self:__hasField(field) then
    rawset(self, field, value)
  else
    error('Cannot add new field "' .. tostring(field) .. '" to ' ..
        self.__name .. '. Make sure that the field is not initialized to nil. ' ..
        'Use NIL_INIT instead.')
  end
end

function __CompTaskMT:__hasField(field)
  return contains(self.__members, field)
end

function CompTask(taskName, initTable)
  if taskName == nil then
    error('Task name nil in CompTask constructor')
  end
  
  if RELOADING_LUA then
    _G[taskName] = nil
  end

  if _G[taskName] ~= nil then
    error('Task "' .. taskName .. '" already defined')
  end

  -- Create an empty set of fields of none was specified
  initTable = initTable or {}

  -- Create task and add it to global namespace
  local task = {}
  _G[taskName] = task

  -- Set up the task's initial values
  task.__initTable = initTable
  task.__name = taskName

  -- Set the metatable
  setmetatable(task, { __index = __CompTaskMT })

  -- initial task fields
  task.__members = {'__parent', '__time', '__as', '__dirty', 'choose', 'finished', 'set'}
  for k, v in pairs(task.__initTable) do
    table.insert(task.__members, k)
    if v ~= NIL_INIT then
      task[k] = v
    end
  end

  -- Create dirty list
  task.__dirty = {}

  -- Don't allow any more fields to be added
  getmetatable(task).__newindex = __CompTaskMT.__checkField

end


-- Primitive Tasks

__PrimTaskMT = {}

function __PrimTaskMT:__init(parent)
  self.__time = timer()
  self.__parent = parent
  for k, v in pairs(self.__initTable) do
    if not self.__dirty[k] then
      if v == NIL_INIT then
        self[k] = nil
      else
        self[k] = v
      end
    end
  end
end

function __PrimTaskMT:set(tbl)
  if tbl then
    for k, v in pairs(tbl) do
      self[k] = v
      self.__dirty[k] = true
      __dirtyList[self] = true
    end
  end
  return self
end

function __PrimTaskMT:trace(level)
  level = level or 0
  addTaskLabel(level, self.__name .. ' (Prim) Time: ' .. self:getTime())
  for k, v in pairs(self.__initTable) do
    addTaskLabel(level+1, '.' .. k .. ' = ' .. tostring(self[k]))
  end
end

function __PrimTaskMT:getTime()
  if self.__time == nil then
    return 0
  end
  return self:__time()
end

function __PrimTaskMT:resetTime()
  self.__time = timer()
end

function __PrimTaskMT:finished()
  return false
end


-- Warning this should only be called using self:setMe
function __PrimTaskMT:setMe(tbl)
  __PrimTaskMT.set(self,tbl)
end

function __PrimTaskMT:__checkField(field, value)
  if self:__hasField(field) then
    rawset(self, field, value)
    -- if field is a public member field then
    -- set the dirty bit
    if self.__initTable[field] ~= nil then
      self.__dirty[field] = true
      __dirtyList[self] = true
    end
  else
    error('Cannot add new field "' .. tostring(field) .. '" to ' ..
        self.__name .. '. Make sure that the field is not initialized nil. ' ..
        'Use NIL_INIT instead.')
  end
end

function __PrimTaskMT:__hasField(field)
  return contains(self.__members, field)
end

function PrimTask(taskName, initTable)
  if taskName == nil then
    error('Task name nil in PrimTask constructor')
  end
  
  if RELOADING_LUA or (taskName == 'task_NullTask') then
    _G[taskName] = nil
  end

  if _G[taskName] ~= nil then
    error('Task "' .. taskName .. '" already defined')
  end

  -- Create an empty set of fields of none was specified
  initTable = initTable or {}

  -- Create task and add it to global namespace
  local task = {}
  _G[taskName] = task

  -- Set up the task's initial values
  task.__initTable = initTable
  task.__name = taskName

  -- Set the metatable
  setmetatable(task, { __index = __PrimTaskMT })

  -- initial task fields
  task.__members = {'__parent', '__time', '__dirty', '__init', 'run', 'finished'}
  for k, v in pairs(task.__initTable) do
    table.insert(task.__members, k)
    if v ~= NIL_INIT then
      task[k] = v
    end
  end

  -- Create dirty list
  task.__dirty = {}

  -- Don't allow any more fields to be added
  getmetatable(task).__newindex = __PrimTaskMT.__checkField
end


-- Parallel tasks

__ParaTaskMT = {}

function __ParaTaskMT:__init(parent)
  self.__time = timer()
  self.__parent = parent
  self.__initSubtasks = true
end

function __ParaTaskMT:run()
  local t = {}
  for st in listIter(self.__subs) do
    local subTask = _G[st]
    if subTask == nil then
      error('Parallel task: ' .. self.__name ..
            ' attempted to call invalid child task: ' .. st)
    end
    if subTask.__parent ~= self or self.__initSubtasks == true then
      subTask:__init(self)
    end
    -- don't try to combine outputs, we're not returning anything anyway
    --for k, v in pairs(subTask:run()) do
    -- t[k] = v
    --end
    subTask:run()
  end
  self.__initSubtasks = false
  return t
end


function __ParaTaskMT:getTime()
  if self.__time == nil then
    return 0
  end
  return self:__time()
end

function __ParaTaskMT:finished()
  -- return true if all sub tasks are true
  for st in listIter(self.__subs) do
    local subTask = _G[st]
    if subTask == nil then
      error('Parallel task: ' .. self.__name ..
            ' attempted to call invalid child task: ' .. st)
    end
    if (not subTask:finished()) then
      return false
    end
  end
  return true
end


function __ParaTaskMT:__checkField(field, value)
  if contains(self.__members, field) then
    rawset(self, field, value)
  else
    error('Cannot add new field "' .. tostring(field) .. '" to ' ..
        self.__name .. '. Make sure that the field is not initialized to nil. ' ..
        'Use NIL_INIT instead.')
  end
end

function __ParaTaskMT:__hasField(field)
  for st in listIter(self.__subs) do
    local subTask = _G[st]
    if subTask == nil then
      error('Parallel task: ' .. self.__name ..
            ' attempted to call __hasField on invalid child task: ' .. st)
    end
    if subTask:__hasField(field) then
      return true
    end
  end
  return false
end

function __ParaTaskMT:set(tbl)
  if tbl then
    local taskKeyTbl = {}
    for st in listIter(self.__subs) do
      taskKeyTbl[st] = {}
    end
    for k, v in pairs(tbl) do
      local found = false
      for st in listIter(self.__subs) do
        local subTask = _G[st]
        if subTask == nil then
          error('Parallel task: ' .. self.__name ..
                ' attempted to set invalid child task: ' .. st)
        end
        if subTask:__hasField(k) then
          if found then
--            print('Warning: Multiple child tasks with same field name, setting both: ' ..
--                  tostring(k) .. '. Ambiguous set in ' .. self.__name)
          end
          found = true
          taskKeyTbl[st][k] = v
        end
      end
    end
    for st, params in pairs(taskKeyTbl) do
      _G[st]:set(params)
    end
  end
  return self
end

function __ParaTaskMT:trace(level)
  level = level or 0
  addTaskLabel(level, self.__name .. ' (Para) Time: ' .. self:getTime())
  for st in listIter(self.__subs) do
    local subTask = _G[st]
    if subTask == nil then
      error('Parallel task: ' .. self.__name ..
            ' attempted to trace invalid child task: ' .. st)
    end
    subTask:trace(level+2)
  end
end


function ParaTask(taskName, subTasks)
  if taskName == nil then
    error('Task name nil in ParaTask constructor')
  end

  if RELOADING_LUA then
    _G[taskName] = nil
  end

  if _G[taskName] ~= nil then
    error('Task "' .. taskName .. '" already defined')
  end

  -- Create task and add it to global namespace
  local task = {}
  _G[taskName] = task

  task.__name = taskName
  task.__subs = subTasks
  task.__initSubtasks = true

  -- Set the metatable
  setmetatable(task, { __index = __ParaTaskMT })

  -- initial task fields
  task.__members = {'__parent', '__time', '__init', 'run', 'finished', 'set'}

  -- Don't allow any more fields to be added
  getmetatable(task).__newindex = __ParaTaskMT.__checkField

end

function resetDirty()
  for task, v in pairs(__dirtyList) do
    task.__dirty = {}
  end
  __dirtyList = {}
end


function addTaskLabel(level, ...)
  local debugstring = ""

  -- add space for indent
  for i = 0, level do
    debugstring = "  " .. debugstring
  end

  -- add print statement afterward
  for i,v in ipairs(arg) do
    debugstring = debugstring .. tostring(v) .. " "
  end
  --debugstring = debugstring .. "\n"

  -- log it
  UTdebug.log(25, debugstring)

end



PrimTask('task_NullTask')
function task_NullTask:run()
  return {}
end
