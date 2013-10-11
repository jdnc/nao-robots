require 'math'

-- less outlier reset
-- smaller outlier reset
-- less pos noise / more ori noise

cfgUKFHandTuned2012 = {}

-- R values for measurements of different types
cfgUKFHandTuned2012.R_goal_theta = 0.00003
cfgUKFHandTuned2012.R_goal_range_offset = 300 --11369.486376104707
cfgUKFHandTuned2012.R_goal_range_relative = 0.10 --08187923444388568
cfgUKFHandTuned2012.ignore_goal_dist = 1200  -- in cm, since ukf is

cfgUKFHandTuned2012.R_circle_theta = 0.002382484472701167
cfgUKFHandTuned2012.R_circle_range_offset = 1.0E-5
cfgUKFHandTuned2012.R_circle_range_relative = 0.02116007917680214
cfgUKFHandTuned2012.ignore_circle_dist = 350

cfgUKFHandTuned2012.R_line_theta = 0.5189340444563318
cfgUKFHandTuned2012.R_line_range_offset = 168.0253073526829
cfgUKFHandTuned2012.R_line_range_relative = 0.37326177209179223
cfgUKFHandTuned2012.ignore_line_dist = 350

cfgUKFHandTuned2012.R_int_theta = 0.4228196514773248
cfgUKFHandTuned2012.R_int_range_offset = 2312.14198399771
cfgUKFHandTuned2012.R_int_range_relative = 0.29531612641418414
cfgUKFHandTuned2012.ignore_int_dist = 350

-- sbarrett - jake says we should trust the cross (in a non-religious way)
cfgUKFHandTuned2012.R_cross_theta = 0.01 --cfgUKFHandTuned2012.R_int_theta
cfgUKFHandTuned2012.R_cross_range_offset = 30 --cfgUKFHandTuned2012.R_int_range_offset
cfgUKFHandTuned2012.R_cross_range_relative = 0.05 -- cfgUKFHandTuned2012.R_int_range_relative
cfgUKFHandTuned2012.ignore_cross_dist = 250

cfgUKFHandTuned2012.R_ball_theta = 1.0E-5
cfgUKFHandTuned2012.R_ball_range_offset = 28.099394128595872
cfgUKFHandTuned2012.R_ball_range_relative = 0.015 --0.017340047102684532
cfgUKFHandTuned2012.ignore_ball_dist = 800

cfgUKFHandTuned2012.R_kick_vel =  75 --15 --75.97275522076296
cfgUKFHandTuned2012.R_kick_bearing = 0.25 --0.1 --0.24236990556426424
cfgUKFHandTuned2012.kick_ball_vel_reset_noise = 250.0

cfgUKFHandTuned2012.shared_ball_sd_factor = 2.0 --1.5 --2.0 --5.0

cfgUKFHandTuned2012.R_crop_factor = 5.0

-- reset thresholds
cfgUKFHandTuned2012.large_angle_sd = 1.55 --0.1 --1.55 --0.001 --1e-5 --1.55
cfgUKFHandTuned2012.object_error_thresh = 0.13974642163121867
cfgUKFHandTuned2012.object_error_decay = 0.83 --0.7396740477426499
cfgUKFHandTuned2012.reset_sum_thresh = 4.5 --3.6 --2.1 --3.673797565122742
cfgUKFHandTuned2012.reset_num_thresh = 2 --1

-- ambig object param
cfgUKFHandTuned2012.outlier_alpha_degrade = 1.0E-5
cfgUKFHandTuned2012.t_l_mismatch_degrade = 1.0

cfgUKFHandTuned2012.kappa = 1.0
cfgUKFHandTuned2012.ball_decay_rate = 0.97 -- Todd: as measured by 3m 7 sec kick --0.95 --0.85 --0.6156808728836556
cfgUKFHandTuned2012.outlier_rejection_thresh = 3.0 --46.058925711593886
cfgUKFHandTuned2012.kick_outlier_rejection_thresh = 30.0
cfgUKFHandTuned2012.seen_ball_outlier_rejection_thresh = 10.0
cfgUKFHandTuned2012.shared_ball_outlier_rejection_thresh = 5.0
cfgUKFHandTuned2012.keeper_outlier_rejection_thresh_diff = 1.0

-- process noise
cfgUKFHandTuned2012.robot_pos_noise = 4 --1.5 --3 --1.5 --0.8662409414768735
cfgUKFHandTuned2012.robot_orient_noise = 0.3 --0.14 --0.15 --0.2 --0.2 --0.10628031718950162
cfgUKFHandTuned2012.ball_pos_noise = 18.0 --4.0 --3.0 --2.65 --1.75 --2.25 --15 --5.649540015152918
cfgUKFHandTuned2012.ball_vel_noise = 10 --15 --10 --15 --10 --30 --70 -- 2.2375723953936175
cfgUKFHandTuned2012.keeper_ball_vel_noise = 20 --15 --10 --15 --10 --30 --70 -- 2.2375723953936175

cfgUKFHandTuned2012.robot_bump_sd_factor = 3.5 --2.0
cfgUKFHandTuned2012.fallen_robot_sd_factor = 8.0
cfgUKFHandTuned2012.fallen_ball_sd_factor = 4.0
cfgUKFHandTuned2012.standing_robot_sd_factor = 1.5 --2.0
cfgUKFHandTuned2012.moving_ball_sd_factor = 8.0


-- std's used when resetting things
cfgUKFHandTuned2012.init_sd_x = 25
cfgUKFHandTuned2012.init_sd_y = 20
cfgUKFHandTuned2012.init_sd_theta = 0.5
cfgUKFHandTuned2012.init_sd_vel = 100 --23.3
cfgUKFHandTuned2012.reset_ball_sd_pos = 200 --23.3

cfgUKFHandTuned2012.fallen_sd_theta = 2.5 --3--2.1091366865527785
cfgUKFHandTuned2012.reset_sd_pos = 35 --120 --64.0996853178977
cfgUKFHandTuned2012.reset_sd_theta = 2.5 --2.117683451012757
cfgUKFHandTuned2012.reset_sd_ball = 5 --133.69868846560908

-- num models
cfgUKFHandTuned2012.max_models_after_merge = 4

-- seeding
cfgUKFHandTuned2012.seed_model_alpha = 0.2
cfgUKFHandTuned2012.min_reseed_bearing_diff = DEG_T_RAD * 2.0

-- keeping models after getup/pen
cfgUKFHandTuned2012.ambig_min_model_alpha = 0.035
cfgUKFHandTuned2012.ambig_model_min_nframes = 5
cfgUKFHandTuned2012.ambig_model_min_ball_frames = 3

-- keeper ball spawning
cfgUKFHandTuned2012.keeper_ball_spawn_freq = 15
cfgUKFHandTuned2012.keeper_ball_spawn_likelihood = 0.05
cfgUKFHandTuned2012.keeper_ball_spawn_velocity = 32 -- cm/s

-- flipped robot threshold (must be greater than this)
cfgUKFHandTuned2012.mate_flipped_ball_thresh = 1

-- which things to use
cfgUKFHandTuned2012.USE_CIRCLE = true
cfgUKFHandTuned2012.USE_GOAL = true
cfgUKFHandTuned2012.USE_WHOLE_GOAL = false   -- normally false, so we just use posts
cfgUKFHandTuned2012.USE_AMBIG = true

cfgUKFHandTuned2012.SPLIT_ON_POSTS = true
cfgUKFHandTuned2012.SPLIT_ON_LINES = false
cfgUKFHandTuned2012.SPLIT_LINES_ON_HIGH_SD = false --true
cfgUKFHandTuned2012.SPLIT_ON_INTS = false

cfgUKFHandTuned2012.SPLIT_LINE_SD_POS_THRESH = 70.0
cfgUKFHandTuned2012.SPLIT_LINE_SD_THETA_THRESH = 3.2 -- to be higher than fallen sd_theta


cfgUKFHandTuned2012.USE_INTERSECTIONS = false--true
cfgUKFHandTuned2012.USE_CROSSES = true --true
cfgUKFHandTuned2012.USE_BALL_IN_SET = true
cfgUKFHandTuned2012.USE_LINES = true
cfgUKFHandTuned2012.RESEED_ON_OUTLIERS = false
cfgUKFHandTuned2012.USE_SHARED_BALL = true
cfgUKFHandTuned2012.CHECK_LINES_IN_FOV = true
cfgUKFHandTuned2012.DO_RESEED = false--true


