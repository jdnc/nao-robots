require 'stateMachine'
module(..., package.seeall);


function setFieldArea(fieldArea,minX,rolesOrderedByImportance)
  fieldArea.used = true
  fieldArea.minX = minX
  for i=1,#rolesOrderedByImportance do
    fieldArea:setRole(i-1,rolesOrderedByImportance[i]) -- -1 because c is 0 indexed and lua is 1 indexed
  end
end

local FIELD_ALWAYS = -999999

------------------
--- MAIN FIELD ---
------------------

cfgBaseStrategy = core.KickStrategy()
cfgBaseStrategy.edgeBuffer = 500
cfgBaseStrategy.postAngle = 45 * DEG_T_RAD
cfgBaseStrategy.forwardOpeningAngle = 80 * DEG_T_RAD
cfgBaseStrategy.insidePostBuffer = 100
cfgBaseStrategy.maxArcAngle = 40 * DEG_T_RAD
cfgBaseStrategy.shootOnGoalRadius = 1500
cfgBaseStrategy.ownGoalRadius = 1500
cfgBaseStrategy.maxOpponentSD = 850.0
cfgBaseStrategy.minOpponentDist = 500
cfgBaseStrategy.allowOpponentSideDist = -1
cfgBaseStrategy.opponentWidth = 75
cfgBaseStrategy.orientationErrorFactor = 4.0
cfgBaseStrategy.passDistance = 750
cfgBaseStrategy.maxKickAngle = 30.0 * DEG_T_RAD
--cfgBaseStrategy.supportFwdDist = -500
--cfgBaseStrategy.supportSideDist = 1200--1500
--cfgBaseStrategy.defendBackDist = 2000
--cfgBaseStrategy.forwardFwdDist = 1800--2000
--cfgBaseStrategy.forwardSideDist = 600
cfgBaseStrategy.minOppDistForSlowKick = 0 -- allow slow kicks even with 0 dist opp
cfgBaseStrategy.minOppDistForExtraRotate = 1200 -- if we have space, rotate for better kick
cfgBaseStrategy.defaultKick = core.WalkKickFront
--cfgBaseStrategy.useSupporterAtMidfield = false
for i = 0,core.NUM_KICKS-1 do
  cfgBaseStrategy:setUsableKick(i,true)
end


-- More aggressive strategy (more forward required)
cfgFwdStrategy = core.KickStrategy()
cfgFwdStrategy.edgeBuffer = 700
cfgFwdStrategy.postAngle = 30 * DEG_T_RAD
cfgFwdStrategy.forwardOpeningAngle = 60 * DEG_T_RAD
cfgFwdStrategy.insidePostBuffer = 220
cfgFwdStrategy.maxArcAngle = 25 * DEG_T_RAD --10 * DEG_T_RAD --10 * DEG_T_RAD
cfgFwdStrategy.shootOnGoalRadius = 1500
cfgFwdStrategy.ownGoalRadius = 1500
cfgFwdStrategy.maxOpponentSD = 850.0
cfgFwdStrategy.minOpponentDist = 650 --500
cfgFwdStrategy.allowOpponentSideDist = -1
cfgFwdStrategy.opponentWidth = 150
cfgFwdStrategy.orientationErrorFactor = 2.5 -- less confident so we dont kick out of bounds
cfgFwdStrategy.passDistance = 850
cfgFwdStrategy.maxKickAngle = 15.0 * DEG_T_RAD
--cfgFwdStrategy.supportFwdDist = -550
--cfgFwdStrategy.supportSideDist = 650 --1200--1500
--cfgFwdStrategy.defendBackDist = 1800--1700 --2000
--cfgFwdStrategy.forwardFwdDist = 3000--2000
--cfgFwdStrategy.forwardSideDist = 700
cfgFwdStrategy.minOppDistForSlowKick = 0 -- allow slow kicks always
cfgFwdStrategy.minOppDistForExtraRotate = 1500 -- space to rotate for better kick
cfgFwdStrategy.defaultKick = core.WalkKickFront
--cfgFwdStrategy.useSupporterAtMidfield = true
for i = 0,core.NUM_KICKS-1 do
  cfgFwdStrategy:setUsableKick(i,true)
end
cfgFwdStrategy:setUsableKick(core.Dribble,false)
cfgFwdStrategy:setUsableKick(core.FwdShortStraightKick,false)
cfgFwdStrategy:setUsableKick(core.FwdLongLargeGapKick,false)
cfgFwdStrategy:setUsableKick(core.FwdLongSmallGapKick,false)
cfgFwdStrategy:setUsableKick(core.FwdPass5Kick,false)
cfgFwdStrategy:setUsableKick(core.FwdPass4Kick,false)
cfgFwdStrategy:setUsableKick(core.FwdPass3Kick,false)
cfgFwdStrategy:setUsableKick(core.FwdPass2Kick,false)
-- no side kicks except in cluster
--cfgFwdStrategy:setUsableKick(core.WalkKickLeftwardSide,false)
--cfgFwdStrategy:setUsableKick(core.WalkKickRightwardSide,false)




cfgFwdGoalieStrategy = core.KickStrategy(cfgFwdStrategy)
for i = 0,core.NUM_KICKS-1 do
  cfgFwdGoalieStrategy:setUsableKick(i,false)
end
-- keeper uses just long and medium kicks
cfgFwdGoalieStrategy:setUsableKick(core.FwdSuperStraightKick,true)
cfgFwdGoalieStrategy:setUsableKick(core.FwdLongStraightKick,true)
cfgFwdGoalieStrategy:setUsableKick(core.FwdMediumStraightKick,true)



-- More aggressive strategy (more forward required)
cfgWalkKickStrategy = core.KickStrategy()
cfgWalkKickStrategy.edgeBuffer = 700
cfgWalkKickStrategy.postAngle = 30 * DEG_T_RAD
cfgWalkKickStrategy.forwardOpeningAngle = 60 * DEG_T_RAD
cfgWalkKickStrategy.insidePostBuffer = 220
cfgWalkKickStrategy.maxArcAngle = 25 * DEG_T_RAD --10 * DEG_T_RAD --10 * DEG_T_RAD
cfgWalkKickStrategy.shootOnGoalRadius = 700 --600--1500
cfgWalkKickStrategy.ownGoalRadius = 1500
cfgWalkKickStrategy.maxOpponentSD = 850.0
cfgWalkKickStrategy.minOpponentDist = 650 --500
cfgWalkKickStrategy.allowOpponentSideDist = -1
cfgWalkKickStrategy.opponentWidth = 150
cfgWalkKickStrategy.orientationErrorFactor = 2.5 -- less confident so we dont kick out of bounds
cfgWalkKickStrategy.passDistance = 850
cfgWalkKickStrategy.maxKickAngle = 15.0 * DEG_T_RAD
--cfgWalkKickStrategy.supportFwdDist = -550
--cfgWalkKickStrategy.supportSideDist = 650 --1200--1500
--cfgWalkKickStrategy.defendBackDist = 1800--1700 --2000
--cfgWalkKickStrategy.forwardFwdDist = 1800--2000
--cfgWalkKickStrategy.forwardSideDist = 700
cfgWalkKickStrategy.minOppDistForSlowKick = 7500 -- never do slow kicks
cfgWalkKickStrategy.minOppDistForExtraRotate = 7500 -- never do extra rotate
cfgWalkKickStrategy.defaultKick = core.WalkKickFront
--cfgWalkKickStrategy.useSupporterAtMidfield = true
for i = 0,core.NUM_KICKS-1 do
  cfgWalkKickStrategy:setUsableKick(i,false)
end

cfgWalkKickStrategy:setUsableKick(core.WalkKickFront,true)
cfgWalkKickStrategy:setUsableKick(core.WalkKickLeftwardSide,true)
cfgWalkKickStrategy:setUsableKick(core.WalkKickRightwardSide,true)
cfgWalkKickStrategy:setUsableKick(core.WalkKickLeftward,true)
cfgWalkKickStrategy:setUsableKick(core.WalkKickRightward,true)


-- More aggressive strategy (more forward required)
cfgWalkKickLimitedSlowKickStrategy = core.KickStrategy()
cfgWalkKickLimitedSlowKickStrategy.edgeBuffer = 700
cfgWalkKickLimitedSlowKickStrategy.postAngle = 30 * DEG_T_RAD
cfgWalkKickLimitedSlowKickStrategy.forwardOpeningAngle = 60 * DEG_T_RAD
cfgWalkKickLimitedSlowKickStrategy.insidePostBuffer = 220
cfgWalkKickLimitedSlowKickStrategy.maxArcAngle = 25 * DEG_T_RAD --10 * DEG_T_RAD --10 * DEG_T_RAD
cfgWalkKickLimitedSlowKickStrategy.shootOnGoalRadius = 700--1500
cfgWalkKickLimitedSlowKickStrategy.ownGoalRadius = 1500
cfgWalkKickLimitedSlowKickStrategy.maxOpponentSD = 850.0
cfgWalkKickLimitedSlowKickStrategy.minOpponentDist = 650 --500
cfgWalkKickLimitedSlowKickStrategy.allowOpponentSideDist = -1
cfgWalkKickLimitedSlowKickStrategy.opponentWidth = 150
cfgWalkKickLimitedSlowKickStrategy.orientationErrorFactor = 3.5 -- less confident so we dont kick out of bounds
cfgWalkKickLimitedSlowKickStrategy.passDistance = 850
cfgWalkKickLimitedSlowKickStrategy.maxKickAngle = 15.0 * DEG_T_RAD
--cfgWalkKickLimitedSlowKickStrategy.supportFwdDist = -550
--cfgWalkKickLimitedSlowKickStrategy.supportSideDist = 650 --1200--1500
--cfgWalkKickLimitedSlowKickStrategy.defendBackDist = 1800--1700 --2000
--cfgWalkKickLimitedSlowKickStrategy.forwardFwdDist = 1800--2000
--cfgWalkKickLimitedSlowKickStrategy.forwardSideDist = 700
cfgWalkKickLimitedSlowKickStrategy.minOppDistForSlowKick = 250 -- only do slow kick with 1/4 m of space
cfgWalkKickLimitedSlowKickStrategy.minOppDistForSuperKick = 500 -- only do super kick with 1/2 m of space
cfgWalkKickLimitedSlowKickStrategy.minOppDistForExtraRotateOffset = 500
cfgWalkKickLimitedSlowKickStrategy.minOppDistForExtraRotateFactor = 500 / (math.pi * 0.5) -- 500mm if 90 degrees off
cfgWalkKickLimitedSlowKickStrategy.defaultKick = core.WalkKickFront
--cfgWalkKickLimitedSlowKickStrategy.useSupporterAtMidfield = true
cfgWalkKickLimitedSlowKickStrategy.ignoreBehindAngleForSlowKick = 60 * DEG_T_RAD
for i = 0,core.NUM_KICKS-1 do
  cfgWalkKickLimitedSlowKickStrategy:setUsableKick(i,false)
end

cfgWalkKickLimitedSlowKickStrategy:setUsableKick(core.WalkKickFront,true)
cfgWalkKickLimitedSlowKickStrategy:setUsableKick(core.WalkKickLeftwardSide,true)
cfgWalkKickLimitedSlowKickStrategy:setUsableKick(core.WalkKickRightwardSide,true)
cfgWalkKickLimitedSlowKickStrategy:setUsableKick(core.WalkKickLeftward,true)
cfgWalkKickLimitedSlowKickStrategy:setUsableKick(core.WalkKickRightward,true)
cfgWalkKickLimitedSlowKickStrategy:setUsableKick(core.FwdSuperStraightKick,true)
cfgWalkKickLimitedSlowKickStrategy:setUsableKick(core.FwdLongStraightKick,true)
cfgWalkKickLimitedSlowKickStrategy:setUsableKick(core.FwdMediumStraightKick,true)


cfgWalkKickLimitedSlowKickFarShotStrategy = core.KickStrategy(cfgWalkKickLimitedSlowKickStrategy)
cfgWalkKickLimitedSlowKickFarShotStrategy.shootOnGoalRadius = 1500

-- Side kick only strategy (hold ball, dont really score)
cfgSideKickStrategy = core.KickStrategy()
cfgSideKickStrategy.edgeBuffer = 700
cfgSideKickStrategy.postAngle = 30 * DEG_T_RAD
cfgSideKickStrategy.forwardOpeningAngle = 90 * DEG_T_RAD
cfgSideKickStrategy.insidePostBuffer = 220
cfgSideKickStrategy.maxArcAngle = 5 * DEG_T_RAD --10 * DEG_T_RAD --10 * DEG_T_RAD
cfgSideKickStrategy.shootOnGoalRadius = 10
cfgSideKickStrategy.ownGoalRadius = 1500
cfgSideKickStrategy.maxOpponentSD = 850.0
cfgSideKickStrategy.minOpponentDist = 650 --500
cfgSideKickStrategy.allowOpponentSideDist = -1
cfgSideKickStrategy.opponentWidth = 150
cfgSideKickStrategy.orientationErrorFactor = 2.5 -- less confident so we dont kick out of bounds
cfgSideKickStrategy.passDistance = 850
cfgSideKickStrategy.maxKickAngle = 15.0 * DEG_T_RAD
--cfgSideKickStrategy.supportFwdDist = -600
--cfgSideKickStrategy.supportSideDist = 680 --1200--1500
--cfgSideKickStrategy.defendBackDist = 1800--1700 --2000
--cfgSideKickStrategy.forwardFwdDist = 1800--2000
--cfgSideKickStrategy.forwardSideDist = 700
cfgSideKickStrategy.minOppDistForSlowKick = 7500 -- no slow kicks
cfgSideKickStrategy.minOppDistForExtraRotate = 7500 -- no extra rotate
cfgSideKickStrategy.defaultKick = core.WalkKickFront
--cfgSideKickStrategy.useSupporterAtMidfield = true

for i = 0,core.NUM_KICKS-1 do
  cfgSideKickStrategy:setUsableKick(i,false)
end
-- only side kicks 
cfgSideKickStrategy:setUsableKick(core.WalkKickLeftwardSide,true)
cfgSideKickStrategy:setUsableKick(core.WalkKickRightwardSide,true)


-- strategy for end of half... one last shot on goal
cfgDesperationStrategy = core.KickStrategy(cfgWalkKickLimitedSlowKickStrategy)
cfgDesperationStrategy.minOppDistForSlowKick = 0 -- always can do slow kicks
cfgDesperationStrategy.minOppDistForSuperKick = 0 -- always can do slow kicks
cfgDesperationStrategy.minOppDistForExtraRotate = 0 -- can always rotate
cfgDesperationStrategy.orientationErrorFactor = 7.5 -- more confident so we do kick
cfgDesperationStrategy.defaultKick = core.FwdLongStraightKick
for i = 0,core.NUM_KICKS-1 do
  cfgDesperationStrategy:setUsableKick(i,false)
end
cfgDesperationStrategy:setUsableKick(core.FwdSuperStraightKick,true)




-- for b-human... keep ball farther away, try more shots on goal
cfgBHumanStrategy = core.KickStrategy()
cfgBHumanStrategy.edgeBuffer = 700
cfgBHumanStrategy.postAngle = 30 * DEG_T_RAD
cfgBHumanStrategy.forwardOpeningAngle = 60 * DEG_T_RAD
cfgBHumanStrategy.insidePostBuffer = 150
cfgBHumanStrategy.maxArcAngle = 25 * DEG_T_RAD --10 * DEG_T_RAD
cfgBHumanStrategy.shootOnGoalRadius = 6500 --5200
cfgBHumanStrategy.ownGoalRadius = 1500
cfgBHumanStrategy.maxOpponentSD = 750.0
cfgBHumanStrategy.minOpponentDist = 1000 --500
cfgBHumanStrategy.allowOpponentSideDist = -1
cfgBHumanStrategy.opponentWidth = 35
cfgBHumanStrategy.orientationErrorFactor = 6 --8 --4
cfgBHumanStrategy.passDistance = 750
cfgBHumanStrategy.maxKickAngle = 15.0 * DEG_T_RAD
--cfgBHumanStrategy.supportFwdDist = -800
--cfgBHumanStrategy.supportSideDist = 1200--1500
--cfgBHumanStrategy.defendBackDist = 1800--1700 --2000
--cfgBHumanStrategy.forwardFwdDist = 1200--2000
--cfgBHumanStrategy.forwardSideDist = 700
cfgBHumanStrategy.minOppDistForSlowKick = 1500 -- only do slow kick with lots of space
cfgBHumanStrategy.minOppDistForExtraRotate = 2500 -- only do extra rotate with lots of space
cfgBHumanStrategy.defaultKick = core.WalkKickFront
--cfgBHumanStrategy.useSupporterAtMidfield = false


for i = 0,core.NUM_KICKS-1 do
  cfgBHumanStrategy:setUsableKick(i,true)
end
cfgBHumanStrategy:setUsableKick(core.Dribble,false)



 
-- allow dribbles or long kicks
cfgDribbleStrategy = core.KickStrategy()
cfgDribbleStrategy.edgeBuffer = 500
cfgDribbleStrategy.postAngle = 30 * DEG_T_RAD
cfgDribbleStrategy.forwardOpeningAngle = 50 * DEG_T_RAD
cfgDribbleStrategy.insidePostBuffer = 220
cfgDribbleStrategy.maxArcAngle = 30 * DEG_T_RAD
cfgDribbleStrategy.shootOnGoalRadius = 1200
cfgDribbleStrategy.ownGoalRadius = 1500
cfgDribbleStrategy.maxOpponentSD = 850.0
cfgDribbleStrategy.minOpponentDist = 700
cfgDribbleStrategy.allowOpponentSideDist = -1
cfgDribbleStrategy.opponentWidth = 150
cfgDribbleStrategy.orientationErrorFactor = 2.5
cfgDribbleStrategy.passDistance = 750
cfgDribbleStrategy.maxKickAngle = 10.0 * DEG_T_RAD
--cfgDribbleStrategy.supportFwdDist = -650
--cfgDribbleStrategy.supportSideDist = 900--1500
--cfgDribbleStrategy.defendBackDist = 1800
--cfgDribbleStrategy.forwardFwdDist = 1800--2000
--cfgDribbleStrategy.forwardSideDist = 600
cfgDribbleStrategy.minOppDistForSlowKick = 250 -- only slow kick with some space
cfgDribbleStrategy.minOppDistForExtraRotate = 0 -- no extra rotate
cfgDribbleStrategy.defaultKick = core.Dribble
--cfgDribbleStrategy.useSupporterAtMidfield = true

for i = 0,core.NUM_KICKS-1 do
  cfgDribbleStrategy:setUsableKick(i,false)
end
cfgDribbleStrategy:setUsableKick(core.Dribble,true)
cfgDribbleStrategy:setUsableKick(core.FwdLongStraightKick,true)
cfgDribbleStrategy:setUsableKick(core.FwdSuperStraightKick,true)


-- Penalty kick early - super kick, but more conservative
cfgPenaltyKickEarlyStrategy = core.KickStrategy(cfgWalkKickLimitedSlowKickStrategy)
cfgPenaltyKickEarlyStrategy.minOppDistForSlowKick = 0
cfgPenaltyKickEarlyStrategy.minOppDistForSuperKick = 0
cfgPenaltyKickEarlyStrategy.minOppDistForExtraRotate = 0 -- always allow extra rotate
cfgPenaltyKickEarlyStrategy.defaultKick = core.FwdSuperStraightKick
for i = 0,core.NUM_KICKS-1 do
  cfgPenaltyKickEarlyStrategy:setUsableKick(i,false)
end
cfgPenaltyKickEarlyStrategy:setUsableKick(core.FwdSuperStraightKick,true)
cfgPenaltyKickEarlyStrategy.orientationErrorFactor = 2.5

-- desparation part
cfgPenaltyKickStrategy = core.KickStrategy(cfgWalkKickLimitedSlowKickStrategy)
cfgPenaltyKickStrategy.orientationErrorFactor = 7.5


------------------
----- CORNER -----
------------------

-- basic corner
cfgBaseCornerStrategy = core.CornerKickStrategy()
cfgBaseCornerStrategy.cornerAngle = 45.0  * DEG_T_RAD
cfgBaseCornerStrategy.endlineBuffer = 300.0
cfgBaseCornerStrategy.distFromEndline = 1100.0
cfgBaseCornerStrategy.distFromFarPost = 200.0
cfgBaseCornerStrategy.targetPt = core.Point2D(core.FIELD_X / 2.0 - 650, 0)
cfgBaseCornerStrategy.orientationErrorFactor = 2.0
for i = 0,core.NUM_KICKS-1 do
  cfgBaseCornerStrategy:setUsableKick(i,false)
end
-- overwrite gap kicks with kicks to top of box for corner
--cfgBaseCornerStrategy:setUsableKick(core.FwdLongLargeGapKick,true)
--cfgBaseCornerStrategy:setUsableKick(core.FwdLongSmallGapKick,true)
cfgBaseCornerStrategy:setUsableKick(core.WalkKickFront,true)
cfgBaseCornerStrategy:setUsableKick(core.WalkKickLeftward,true)
cfgBaseCornerStrategy:setUsableKick(core.WalkKickRightward,true)
--cfgBaseCornerStrategy:setUsableKick(core.FwdShortStraightKick,true)
--cfgBaseCornerStrategy:setUsableKick(core.FwdPass5Kick,true)
--cfgBaseCornerStrategy:setUsableKick(core.FwdPass4Kick,true)
--cfgBaseCornerStrategy:setUsableKick(core.FwdPass3Kick,true)
--cfgBaseCornerStrategy:setUsableKick(core.FwdPass2Kick,true)


-- dribble only corner
cfgDribbleCornerStrategy = core.CornerKickStrategy()
cfgDribbleCornerStrategy.cornerAngle = 45.0  * DEG_T_RAD
cfgDribbleCornerStrategy.endlineBuffer = 300.0
cfgDribbleCornerStrategy.distFromEndline = 800.0
cfgDribbleCornerStrategy.distFromFarPost = 200.0
cfgDribbleCornerStrategy.targetPt = core.Point2D(core.FIELD_X / 2.0 - 400, 0)
cfgDribbleCornerStrategy.orientationErrorFactor = 2.0
for i = 0,core.NUM_KICKS-1 do
  cfgDribbleCornerStrategy:setUsableKick(i,false)
end
cfgDribbleCornerStrategy:setUsableKick(core.Dribble, true)

------------------
---- CLUSTER -----
------------------

cfgSideKickClusterStrategy = core.ClusterKickStrategy()
cfgSideKickClusterStrategy.behavior = core.Cluster_SIDEKICK
cfgSideKickClusterStrategy.forwardOpeningAngle = 115 * DEG_T_RAD
cfgSideKickClusterStrategy.allowOpponentSideDist = 300
cfgSideKickClusterStrategy.shootOnGoalRadius = 100 -- at 10 cm out, just hope it bounces our way

cfgNoClusterStrategy = core.ClusterKickStrategy()
cfgNoClusterStrategy.behavior = core.Cluster_NONE
cfgNoClusterStrategy.forwardOpeningAngle = -1
cfgNoClusterStrategy.allowOpponentSideDist = -1
cfgNoClusterStrategy.shootOnGoalRadius = -1

cfgDribbleClusterStrategy = core.ClusterKickStrategy()
cfgDribbleClusterStrategy.behavior = core.Cluster_DRIBBLE
cfgDribbleClusterStrategy.forwardOpeningAngle = -1
cfgDribbleClusterStrategy.allowOpponentSideDist = -1
cfgDribbleClusterStrategy.shootOnGoalRadius = -1

cfgQuickClusterStrategy = core.ClusterKickStrategy()
cfgQuickClusterStrategy.behavior = core.Cluster_QUICK
cfgQuickClusterStrategy.forwardOpeningAngle = -1
cfgQuickClusterStrategy.allowOpponentSideDist = -1
cfgQuickClusterStrategy.shootOnGoalRadius = -1

------------------
------ PASS ------
------------------

cfgPassStrategy = core.PassStrategy()
cfgPassStrategy.timeToConsider = 2.0
cfgPassStrategy.distToConsider = 1750
cfgPassStrategy.targetOffsetDist = 150
cfgPassStrategy.upfieldPenaltyFactor = 1.5
cfgPassStrategy.maxTurnFromBall = DEG_T_RAD * 45

------------------
--- POSITIONS ----
------------------

cfgRoleStrategy = core.RoleStrategy()
local role
-- DEFENDER
role = cfgRoleStrategy:getRolePositionConfigPtr(core.DEFENDER)
role.offsetFromBall.x = -1800
role.offsetFromBall.y = 1500
role.minX = -core.HALF_FIELD_X + 1500
role.maxX = -600
role.maxY = 1400
role.splitY = true
role.oppositeYOfBall = false
-- SUPPORTER
role = cfgRoleStrategy:getRolePositionConfigPtr(core.SUPPORTER)
role.offsetFromBall.x = -350
role.offsetFromBall.y = 650
role.minX = -1500
role.maxX = core.HALF_FIELD_X - 1000
role.maxY = 1600
role.splitY = false
role.oppositeYOfBall = false
-- MIDFIELD
role = cfgRoleStrategy:getRolePositionConfigPtr(core.MIDFIELD)
role.offsetFromBall.x = 2000
role.offsetFromBall.y = 700
role.minX = -2000
role.maxX = 1000
role.maxY = 1600
role.splitY = true
role.oppositeYOfBall = false
-- FORWARD
role = cfgRoleStrategy:getRolePositionConfigPtr(core.FORWARD)
role.offsetFromBall.x = 3000
role.offsetFromBall.y = 700
role.minX = 0
role.maxX = core.HALF_FIELD_X - 1000
role.maxY = 1600
role.splitY = false
role.oppositeYOfBall = false
-- CAUTIOUS_DEFENDER
role = cfgRoleStrategy:getRolePositionConfigPtr(core.CAUTIOUS_DEFENDER)
role.offsetFromBall.x = -5000
role.offsetFromBall.y = 0.5 * core.HALF_GOAL_Y
role.minX = -core.HALF_FIELD_X + 1000
role.maxX = -core.HALF_FIELD_X + 2500
role.maxY = 1400
role.splitY = false
role.oppositeYOfBall = true

-- AREAS
local area
-- super offensive end
setFieldArea(cfgRoleStrategy:getFieldAreaRoleConfigPtr(0),0.5 * core.HALF_FIELD_X,{core.DEFENDER,core.SUPPORTER,core.CAUTIOUS_DEFENDER}) -- halfway on their side, we don't want both a forward and supporter
--setFieldArea(cfgRoleStrategy:getFieldAreaRoleConfigPtr(0),0.5 * core.HALF_FIELD_X,{core.DEFENDER,core.FORWARD,core.SUPPORTER}) -- halfway on their side, we don't want both a forward and supporter
-- offensive end
setFieldArea(cfgRoleStrategy:getFieldAreaRoleConfigPtr(1),500,{core.DEFENDER,core.FORWARD,core.CAUTIOUS_DEFENDER})
-- midfield
setFieldArea(cfgRoleStrategy:getFieldAreaRoleConfigPtr(2),-1000,{core.DEFENDER,core.FORWARD,core.CAUTIOUS_DEFENDER})
-- defensive
setFieldArea(cfgRoleStrategy:getFieldAreaRoleConfigPtr(3),-core.HALF_FIELD_X + 1500,{core.CAUTIOUS_DEFENDER,core.FORWARD,core.DEFENDER})
-- super defensive
setFieldArea(cfgRoleStrategy:getFieldAreaRoleConfigPtr(4),FIELD_ALWAYS,{core.CAUTIOUS_DEFENDER,core.FORWARD,core.SUPPORTER})
-- kickoff
-- forward is more important than defender on kickoff
setFieldArea(cfgRoleStrategy.kickoffArea,FIELD_ALWAYS,{core.FORWARD,core.DEFENDER,core.SUPPORTER})
-- goalie is clearing
setFieldArea(cfgRoleStrategy.clearingKeeperArea,FIELD_ALWAYS,{core.CAUTIOUS_DEFENDER,core.DEFENDER,core.SUPPORTER,core.FORWARD})

-- aggressive
cfgAggressiveRoleStrategy = core.RoleStrategy(cfgRoleStrategy)
-- super offensive end
setFieldArea(cfgAggressiveRoleStrategy:getFieldAreaRoleConfigPtr(0),0.5 * core.HALF_FIELD_X,{core.DEFENDER,core.SUPPORTER,core.MIDFIELD}) -- halfway on their side, we don't want both a forward and supporter
--setFieldArea(cfgRoleStrategy:getFieldAreaRoleConfigPtr(0),0.5 * core.HALF_FIELD_X,{core.DEFENDER,core.FORWARD,core.SUPPORTER}) -- halfway on their side, we don't want both a forward and supporter
-- offensive end
setFieldArea(cfgAggressiveRoleStrategy:getFieldAreaRoleConfigPtr(1),500,{core.DEFENDER,core.FORWARD,core.SUPPORTER})
-- midfield
setFieldArea(cfgAggressiveRoleStrategy:getFieldAreaRoleConfigPtr(2),-1000,{core.DEFENDER,core.FORWARD,core.SUPPORTER})
-- defensive
setFieldArea(cfgAggressiveRoleStrategy:getFieldAreaRoleConfigPtr(3),-core.HALF_FIELD_X + 1500,{core.SUPPORTER,core.FORWARD,core.DEFENDER})
-- super defensive
setFieldArea(cfgAggressiveRoleStrategy:getFieldAreaRoleConfigPtr(4),FIELD_ALWAYS,{core.CAUTIOUS_DEFENDER,core.FORWARD,core.SUPPORTER})
-- kickoff
-- forward is more important than defender on kickoff
setFieldArea(cfgAggressiveRoleStrategy.kickoffArea,FIELD_ALWAYS,{core.FORWARD,core.DEFENDER,core.SUPPORTER})
-- goalie is clearing
setFieldArea(cfgAggressiveRoleStrategy.clearingKeeperArea,FIELD_ALWAYS,{core.CAUTIOUS_DEFENDER,core.DEFENDER,core.MIDFIELD,core.FORWARD})

------------------
--- SET PLAYS ----
------------------
local play
cfgSetPlayStrategy = core.SetPlayStrategy()
cfgSetPlayStrategy.maxTimeInPlaying = 10.0 -- seconds since starting playing to still try to do the set play
cfgSetPlayStrategy.maxDistFromBall = 500.0 -- only do set play if within a half meter of the ball
cfgSetPlayStrategy.maxTargetDistFromLine = 500.0 -- only do set play if target is within a half meter of the center line
-- kickoffDiagonalPass
play = cfgSetPlayStrategy:getPlayPtr(core.SetPlay_kickoffDiagonalPass)
play.active = true
play.kickType = core.WalkKickFront
play.desiredGoalBearing = DEG_T_RAD * 45
play.maxBearingError = DEG_T_RAD * 10
play.requiresTargetPlayer = true
play.desiredTargetOffset = core.Point2D(-150,0) -- 15 cm back from the pass location
play.reversible = true -- when reversible is false, goes to right player
-- kickoffBuryDeep
play = cfgSetPlayStrategy:getPlayPtr(core.SetPlay_kickoffBuryDeep)
play.active = true
play.kickType = core.FwdLongStraightKick
play.desiredGoalBearing = DEG_T_RAD * 30
play.maxBearingError = DEG_T_RAD * 5
play.requiresTargetPlayer = false
play.desiredTargetOffset = core.Point2D(-150,0) --unused
play.reversible = true -- when reversible is false, goes to right corner

-- vim: ts=2:sw=2:expandtab:softtabstop=2
