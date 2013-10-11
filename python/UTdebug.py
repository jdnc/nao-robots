#!/usr/bin/env python
import core

def log(loglevel, message):
  core.text_logger.log(loglevel, core.BehaviorModuleLog, message)
  
