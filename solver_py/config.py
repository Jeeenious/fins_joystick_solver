# -*- coding: utf-8 -*-
"""从 robot/config.yaml 加载速度解算参数"""
import os
import yaml

_cfg_path = os.path.join(os.path.dirname(__file__), "../config/config.yaml")
with open(_cfg_path, "r") as f:
    _cfg = yaml.safe_load(f)

DIRECTION_THRESHOLD = _cfg["control"]["direction_threshold"]
DEPTH_THRESHOLD    = _cfg["control"]["depth_threshold"]
ROTATE_THRESHOLD   = _cfg["control"]["rotate_threshold"]
MAX_SPEED          = _cfg["control"]["max_speed"]
PWM_MIN            = _cfg["servo"]["pwm_min"]
PWM_MAX            = _cfg["servo"]["pwm_max"]
SERVO_STEP         = _cfg["servo"]["step"]
