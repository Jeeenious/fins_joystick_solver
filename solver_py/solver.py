# -*- coding: utf-8 -*-
"""速度解算：摇杆值 → 推力百分比 / 舵机 PWM / 夹爪方向"""
from .config import (
    DIRECTION_THRESHOLD, DEPTH_THRESHOLD, ROTATE_THRESHOLD,
    MAX_SPEED, PWM_MIN, PWM_MAX, SERVO_STEP,
)


def calculate_speed(axis_value, threshold):
    speed = 0
    if not -threshold < axis_value < threshold:
        if axis_value > 0:
            speed = int((axis_value - threshold) * MAX_SPEED / (1 - threshold))
        else:
            speed = -int((-axis_value - threshold) * MAX_SPEED / (1 - threshold))
    return speed


def solve_motion(leftH, leftV, rightH, rightV):
    return {
        "x_pct": calculate_speed(leftH, DIRECTION_THRESHOLD),
        "y_pct": -calculate_speed(leftV, DIRECTION_THRESHOLD),
        "yaw_rate_cmd": calculate_speed(rightH, ROTATE_THRESHOLD)
        if not (-ROTATE_THRESHOLD < rightH < ROTATE_THRESHOLD) else 0,
        "depth_force_cmd": calculate_speed(rightV, DEPTH_THRESHOLD)
        if not (-DEPTH_THRESHOLD < rightV < DEPTH_THRESHOLD) else 0,
    }


def solve_servo(hat_x, hat_y, pwm_x, pwm_y):
    changed = False
    if hat_y == 1:
        pwm_x = min(pwm_x + SERVO_STEP, PWM_MAX)
        changed = True
    elif hat_y == -1:
        pwm_x = max(pwm_x - SERVO_STEP, PWM_MIN)
        changed = True
    if hat_x == 1:
        pwm_y = min(pwm_y + SERVO_STEP, PWM_MAX)
        changed = True
    elif hat_x == -1:
        pwm_y = max(pwm_y - SERVO_STEP, PWM_MIN)
        changed = True
    return pwm_x, pwm_y, changed


def solve_gripper(close_pressed, open_pressed):
    if close_pressed and not open_pressed:
        return 1
    if open_pressed and not close_pressed:
        return -1
    return 0
