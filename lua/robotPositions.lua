module(...,package.seeall)

-- define positions

local HALF_FIELD_X = core.HALF_FIELD_X
local HALF_FIELD_Y = core.HALF_FIELD_Y
local PENALTY_X = core.PENALTY_X
local PENALTY_Y = core.PENALTY_Y
local GOAL_Y = core.GOAL_Y
local HALF_PENALTY_Y = 0.5 * PENALTY_Y
local HALF_GOAL_Y = 0.5 * GOAL_Y
local PENALTY_CROSS_X = core.PENALTY_CROSS_X
local CIRCLE_RADIUS = core.CIRCLE_RADIUS
local METER = 1000
local EPS = 20


-- *** REMEMBER POSES ARE ANGLE,X,Y ***

local penaltyPoses = {
  core.Pose2D( math.pi/2.0,-PENALTY_CROSS_X,-HALF_FIELD_Y),
  core.Pose2D(-math.pi/2.0,-PENALTY_CROSS_X, HALF_FIELD_Y)
}

local startingSidelinePoses = {
  -- default
  core.Pose2D(-math.pi/2.0,-2 * METER,HALF_FIELD_Y),
  -- keeper starts on left sideline, 4 meters from midline
  core.Pose2D(-math.pi/2.0,-4 * METER,HALF_FIELD_Y),
  -- player 2 starts on left sideline, 2.5 meters from midline
  core.Pose2D(-math.pi/2.0,-2.5 * METER,HALF_FIELD_Y),
  -- player 3 starts on left side line, 1 meter from midline
  core.Pose2D(-math.pi/2.0,-1 * METER,HALF_FIELD_Y),
  -- player 4 starts on right sideline, 2.5 meters from midline
  core.Pose2D(math.pi/2.0,-2.5 * METER,-HALF_FIELD_Y),
  -- player 5 starts on right side line, 1 meter from midline
  core.Pose2D(math.pi/2.0,-1 * METER,-HALF_FIELD_Y)
}

local ourKickoffPosesDesired = {
  -- default
  core.Pose2D(0,40,-0.5 * HALF_FIELD_Y),
  -- keeper:   200 mm in front of goal line
  core.Pose2D(0,-HALF_FIELD_X + 200, 0),
  -- player 2: left defender
  core.Pose2D(0,-1600,400),
  -- player 3: left wing
  core.Pose2D(0,-300,1400),
  -- player 4: right wing
  core.Pose2D(0,-300,-1400),
  -- player 5: near center, straight on now
  core.Pose2D(0,-300,0)
}

local ourKickoffPosesManual = {
  -- default
  core.Pose2D(0,40,-0.5 * HALF_FIELD_Y),
  -- keeper starts in center of goal
  core.Pose2D(0,-HALF_FIELD_X + EPS,0),
  -- player 2 starts on corner of penalty box
  core.Pose2D(0,-1 * (HALF_FIELD_X - PENALTY_X) + EPS,HALF_PENALTY_Y),
  -- player 3 starts on other corner of penalty box
  core.Pose2D(0,-1 * (HALF_FIELD_X - PENALTY_X) + EPS,-1 * HALF_PENALTY_Y),
  -- player 4 starts even with cross, straight up from post
  core.Pose2D(0,-1 * PENALTY_CROSS_X, -1 * HALF_GOAL_Y),
  -- player 5 starts just behind center circle
  core.Pose2D(0,-1 * CIRCLE_RADIUS - 2 * EPS,0)
}

local ourKickoffPosesManualReversible = {
  false,false,false,false,true,false -- only player 4 is likely to be reversed
}

local theirKickoffPosesDesired = {
  -- default
  core.Pose2D(0,40,-0.5 * HALF_FIELD_Y),
  -- keeper:   200 mm in front of goal line
  core.Pose2D(0,-HALF_FIELD_X + 200, 0),
  -- player 2: a little in front of the penalty box in front of post
  core.Pose2D(0,-(HALF_FIELD_X - PENALTY_X) + 200,HALF_GOAL_Y),
  -- player 3
  core.Pose2D(0,-1900,800),
  -- player 4
  core.Pose2D(0,-1900,-800),
  -- player 5: 250 mm back from center circle
  core.Pose2D(0,-CIRCLE_RADIUS - 350,0)
}

local theirKickoffPosesManual = {
  -- default
  core.Pose2D(0,40,-0.5 * HALF_FIELD_Y),
  -- keeper starts in center of goal
  core.Pose2D(0,-HALF_FIELD_X+EPS,0),
  -- player 2 starts halfway between corner of penalty box and edge of the field
  core.Pose2D(0,-1 * (HALF_FIELD_X - PENALTY_X),0.5 * (HALF_PENALTY_Y + HALF_FIELD_Y)),
  -- player 3 starts halfway between other corner of penalty box and edge of the field
  core.Pose2D(0,-1 * (HALF_FIELD_X - PENALTY_X),-0.5 * (HALF_PENALTY_Y + HALF_FIELD_Y)),
  -- player 4 starts halfway between corner of penalty box and center of field
  core.Pose2D(0,-1 * (HALF_FIELD_X - PENALTY_X) + EPS,0.5 * HALF_PENALTY_Y),
  -- player 5 starts halfway between other corner of penalty box and center of field
  core.Pose2D(0,-1 * (HALF_FIELD_X - PENALTY_X) + EPS,-0.5 * HALF_PENALTY_Y)
}

local theirKickoffPosesManualReversible = {
  false,false,false,false,false,false -- none are reversible since there's already a model in that spot
}

-- functions
function initializePositions()
  -- penalty
  setArray(core.penaltyPoses,penaltyPoses,true,core.NUM_PENALTY_POSES-1)
  -- sideline
  setArray(core.startingSidelinePoses,startingSidelinePoses,true)
  -- our kickoff
  setArray(core.ourKickoffPosesDesired,ourKickoffPosesDesired,true)
  setArray(core.ourKickoffPosesManual,ourKickoffPosesManual,true)
  setArray(core.ourKickoffPosesManualReversible,ourKickoffPosesManualReversible,false)
  -- their kickoff
  setArray(core.theirKickoffPosesDesired,theirKickoffPosesDesired,true)
  setArray(core.theirKickoffPosesManual,theirKickoffPosesManual,true)
  setArray(core.theirKickoffPosesManualReversible,theirKickoffPosesManualReversible,false)
end

function setArray(cArr,luaArr,isPose2D,maxInd)
  if maxInd == nil then
    maxInd = core.NUM_TEAM_POSES-1
  end
  for cInd = 0,maxInd do
    local luaInd = cInd + 1 -- stupid 1 indexing
    if isPose2D then
      luaC:setPose2D(cArr,cInd,luaArr[luaInd])
    else
      luaC:setBool(cArr,cInd,luaArr[luaInd])
    end
  end
end

