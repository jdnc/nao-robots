import sys
import pythonswig_module
import math

DEG_T_RAD = math.pi / 180.0
RAD_T_DEG = 180.0 / math.pi

def init():
  global instance, swig
  global visionC, pythonC, ledsC, localizationC, opponentsC
  global BehaviorModuleLog
  global text_logger
  global instance

  swig = pythonswig_module
  instance = pythonswig_module.PythonInterface().CORE_INSTANCE
  BehaviorModuleLog = swig.BehaviorModuleLog
  visionC = instance.vision_
  pythonC = instance.python_
  ledsC = instance.leds_
  localizationC = instance.localization_
  opponentsC = instance.opponents_
  text_logger = instance.textlog_

  this = sys.modules[__name__]
  for item in dir(swig):
    if item.startswith("__"): continue
    setattr(this, item, getattr(swig, item))
  initMemory()

def initMemory():

  global behavior_mem, camera_block, game_state, kick_request, odometry, robot_state, sensors
  global vision_frame_info, walk_param, kick_params, walk_request, world_objects, team_packets, opponent_mem
  global behavior_params, joint_commands, processed_sonar, al_walk_param, walk_info, robot_vision, body_model
  global robot_info, speech, localization_mem, image
  global joint_angles

  behavior_mem = pythonC.behavior_
  camera_block = pythonC.camera_block_
  game_state = pythonC.game_state_
  kick_request = pythonC.kick_request_
  odometry = pythonC.odometry_
  robot_state = pythonC.robot_state_
  sensors = pythonC.sensors_
  vision_frame_info = pythonC.vision_frame_info_
  walk_param = pythonC.walk_param_
  kick_params = pythonC.kick_params_
  walk_request = pythonC.walk_request_
  world_objects = pythonC.world_objects_
  team_packets = pythonC.team_packets_
  opponent_mem = pythonC.opponents_  
  behavior_params = pythonC.behavior_params_
  joint_commands = pythonC.joint_commands_
  processed_sonar = pythonC.vision_processed_sonar_
  al_walk_param = pythonC.al_walk_param_
  walk_info = pythonC.walk_info_
  robot_vision = pythonC.robot_vision_
  body_model = pythonC.body_model_
  robot_info = pythonC.robot_info_
  speech = pythonC.speech_
  localization_mem = pythonC.localization_
  joint_angles = pythonC.joint_angles_
  image = pythonC.image_
