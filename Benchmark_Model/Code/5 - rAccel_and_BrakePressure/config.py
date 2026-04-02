# steering_control_config.py
from dataclasses import dataclass
import pandas as pd
import numpy as np

@dataclass
class SteeringControlParams:
    tracks_df:  pd.DataFrame
    Vx: float
    Vy: float
    yaw_angle: float
    x: float
    y: float
    e_phi_gain: float
    timestep: float
    n_lap: int
    new_sim: any
    e_l_gains: list
    e_l_limits: list
    total_e_l_limits: float
    steer_limits: list
    dil_mode_enabled: bool

@dataclass
class PedalControlParams:
    tracks_df: pd.DataFrame
    velocity_profile_df: pd.DataFrame
    Vx: float
    Vy: float
    time_step: float
    ref_distance: float
    n_lap: int
    throttle_Kp: float
    throttle_Ki: float
    throttle_Kd: float
    throttle_integral: float
    throttle_prev_error: float
    brake_Kp: float
    brake_Ki: float
    brake_Kd: float
    brake_integral: float
    brake_prev_error: float
    n_MGU: int
    wheel_velocity_RL: float
    wheel_velocity_RR: float
    slip_FL: float
    slip_FR: float
    slip_RL: float
    slip_RR: float
    max_engine_brake_torque: float
    differential_gain_1: float
    driving_differential_gain: float
    overrun_differential_gain: float
    front_wheel_coefficient: float
    rear_wheel_coefficient: float
    max_drive_torque: float
    max_brake_torque: float
    max_long_slip: float
    slip_constant: float
    torque_injection: float
    dil_mode_enabled: bool