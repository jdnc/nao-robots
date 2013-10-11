require 'math'

cfgOpp = {}

-- R values for measurements of different types
cfgOpp.R_vision_theta = 3.6940425745183485E-4 -- Todd: should be similar to goal bearing readings
cfgOpp.R_vision_range_offset = 168.0253073526829           -- Todd: distance similar to line projection distances??
cfgOpp.R_vision_range_relative = 0.37326177209179223
cfgOpp.ignore_vision_dist = 700  -- in cm, since ukf is

cfgOpp.R_sonar_theta = 0.0084                -- Todd: no idea for these sonar ones
cfgOpp.R_sonar_range_offset = 1260
cfgOpp.R_sonar_range_relative = 0.7
cfgOpp.ignore_sonar_dist = 500  -- in cm, since ukf is

cfgOpp.R_bump_theta = 0.02
cfgOpp.R_bump_range = 200.0
cfgOpp.R_bump_distance_estimate = 25.0 -- in cm

cfgOpp.kappa = 1.0
cfgOpp.vel_decay_rate = 0.616 -- Todd: should decay quickly
cfgOpp.outlier_rejection_thresh = 0.9 --14.95 --15.0

-- process noise
cfgOpp.robot_pos_noise = 4.02 --4.0
cfgOpp.robot_vel_noise = 6.32
cfgOpp.team_sd_factor = 5 -- assume teammates sd are 10x worse than they say

-- std's used when resetting things
cfgOpp.init_sd_x = 50.0
cfgOpp.init_sd_y = 50.0
cfgOpp.init_sd_vel = 10.0

-- num models
cfgOpp.max_models_after_merge = 6

cfgOpp.USE_VISION = true
cfgOpp.USE_SONAR  = false
cfgOpp.USE_BUMP   = true
cfgOpp.USE_TEAM = true



cfgOppLearned = {}

-- R values for measurements of different types
cfgOppLearned.R_vision_theta = 1e-5
cfgOppLearned.R_vision_range_offset = 237.31
cfgOppLearned.R_vision_range_relative = 0.3055
cfgOppLearned.ignore_vision_dist = 700  -- in cm, since ukf is

cfgOppLearned.R_sonar_theta = 0.0084  -- Todd: no idea for these sonar ones
cfgOppLearned.R_sonar_range_offset = 1260
cfgOppLearned.R_sonar_range_relative = 0.7
cfgOppLearned.ignore_sonar_dist = 500  -- in cm, since ukf is

cfgOppLearned.R_bump_theta = 0.02
cfgOppLearned.R_bump_range = 200.0
cfgOppLearned.R_bump_distance_estimate = 25.0 -- in cm

cfgOppLearned.kappa = 1.0
cfgOppLearned.vel_decay_rate = 0.616 -- Todd: should decay quickly
cfgOppLearned.outlier_rejection_thresh = 3.947 --0.9 --14.95 --15.0

-- process noise
cfgOppLearned.robot_pos_noise = 12 --4 --8 --4.02 --4.0
cfgOppLearned.robot_vel_noise = 18 --6.32 --12 --6.32
cfgOppLearned.team_sd_factor = 5 -- assume teammates sd are 10x worse than they say

-- std's used when resetting things
cfgOppLearned.init_sd_x = 50.0
cfgOppLearned.init_sd_y = 50.0
cfgOppLearned.init_sd_vel = 10.0

-- num models
cfgOppLearned.max_models_after_merge = 4 --6

cfgOppLearned.USE_VISION = true
cfgOppLearned.USE_SONAR  = false
cfgOppLearned.USE_BUMP   = true
cfgOppLearned.USE_TEAM = true



cfgOppHandTuned = {}

-- R values for measurements of different types
cfgOppHandTuned.R_vision_theta = 1e-5
cfgOppHandTuned.R_vision_range_offset = 237.31
cfgOppHandTuned.R_vision_range_relative = 0.3055
cfgOppHandTuned.ignore_vision_dist = 700  -- in cm, since ukf is

cfgOppHandTuned.R_sonar_theta = 0.0084  -- Todd: no idea for these sonar ones
cfgOppHandTuned.R_sonar_range_offset = 1260
cfgOppHandTuned.R_sonar_range_relative = 0.7
cfgOppHandTuned.ignore_sonar_dist = 500  -- in cm, since ukf is

cfgOppHandTuned.R_bump_theta = 0.03
cfgOppHandTuned.R_bump_range = 800.0
cfgOppHandTuned.bump_distance_estimate = 25.0 -- in cm


cfgOppHandTuned.kappa = 1.0
cfgOppHandTuned.vel_decay_rate = 0.616 -- Todd: should decay quickly
cfgOppHandTuned.outlier_rejection_thresh = 3.947 --0.9 --14.95 --15.0

-- process noise
cfgOppHandTuned.robot_pos_noise = 6 --4 --8 --4.02 --4.0
cfgOppHandTuned.robot_vel_noise = 18 --6.32 --12 --6.32
cfgOppHandTuned.team_sd_factor = 1.0 --3 -- assume teammates sd are factor times worse than they say

-- std's used when resetting things
cfgOppHandTuned.init_sd_x = 250.0
cfgOppHandTuned.init_sd_y = 200.0
cfgOppHandTuned.init_sd_vel = 20.0

-- num models
cfgOppHandTuned.max_models_after_merge = 4 --6

cfgOppHandTuned.USE_VISION = true
cfgOppHandTuned.USE_SONAR  = false
cfgOppHandTuned.USE_BUMP   = true
cfgOppHandTuned.USE_TEAM   = true
