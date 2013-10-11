-- Utilities

-- Does table t contain value val?

function contains(t,val)
  if t == nil then
    return false
  end
  for v in listIter(t) do
    if v == val then
      return true
    end
  end
  return false
end

-- Get just the values from a table (array)

function listIter(t)
  local i = 0
  local n = table.getn(t)
  return function ()
           i = i + 1
           if i <= n then return t[i] end
         end
end

function printTable(t)
  for k, v in pairs(t) do
    print(tostring(k) .. " => " .. tostring(v))
  end
end

-- deep copy an object
function deepcopy(object)
  local lookup_table = {}
  local function _copy(object)
    if type(object) ~= "table" then
      return object
    elseif lookup_table[object] then
      return lookup_table[object]
    end
    local new_table = {}
    lookup_table[object] = new_table
    for index, value in pairs(object) do
      new_table[_copy(index)] = _copy(value)
    end
    return setmetatable(new_table, getmetatable(object))
  end
  return _copy(object)
end

-- Create a timer that when called will return the 
-- elapsed time in seconds since the timer was created

function timer()
   local startTime = vision_frame_info.seconds_since_start
   return function ()
       return vision_frame_info.seconds_since_start - startTime
    end
end
