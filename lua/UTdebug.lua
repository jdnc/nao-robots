module(..., package.seeall);

-- Print to screen and file if logging is enabled and "loglevel" is less than or equal to the logging level
-- If loglevel is 0, always print to the screen (but not necessarily the file)

function log (loglevel, ...)
   local debugstring = ""

   for i,v in ipairs(arg) do
      debugstring = debugstring .. tostring(v) .. " "
   end
   --debugstring = debugstring .. "\n"
   
   text_logger:log(loglevel, core.BehaviorModuleLog, debugstring)
 end
