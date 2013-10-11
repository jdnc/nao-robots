#!/usr/bin/env python

import core

joint_angles = [0] * core.NUM_JOINTS
sensors = [0] * core.NUM_SENSORS
rel_parts = [0] * core.BodyPart.NUM_PARTS
abs_parts = [0] * core.BodyPart.NUM_PARTS

def update():
  for i in range(core.NUM_JOINTS):
    joint_angles[i] = core.pythonC.getFloat(core.joint_angles.values_, i)

  for i in range(core.NUM_SENSORS):
    sensors[i] = core.pythonC.getFloat(core.sensors.values_, i)

  for i in range(core.BodyPart.NUM_PARTS):
    # body parts in the torso-centered/rotated coordinate frame
    rel_parts[i] = core.pythonC.getPose3D(core.body_model.rel_parts_, i)
    # body parts relative to the ground directly beneath the torso
    abs_parts[i] = core.pythonC.getPose3D(core.body_model.abs_parts_, i)
