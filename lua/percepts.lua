module(..., package.seeall)

joint_angles = {}

function populateLocalCopy()
  for i=0,core.NUM_JOINTS-1 do 
    joint_angles[i] = luaC:getFloat(luaC.joint_angles_.values_,i) 
   end
end
