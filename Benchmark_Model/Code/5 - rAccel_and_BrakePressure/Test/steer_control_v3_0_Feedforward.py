import numpy as np
import math
from scipy.interpolate import interp1d
import pandas as pd

sdistance = 0
# nLap = 0
last_e_l = 0
corner = False


def refStateNow(trajectory_matrix, Vx, Vy, orientation, timestep, newSim, nLap, max_sDistance_Lap1, LapCompleted):
    global sdistance

    refDistance = 0

    if newSim is True:
        sdistance = sdistance - sdistance
        refDistance = refDistance - refDistance
        newSim = False

    refDistances = []

    V_magnitude = np.sqrt(Vx**2 + Vy**2)
    max_sdistance = max_sDistance_Lap1
    phi_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['Orientation'], kind='linear', fill_value="extrapolate")
    phi = phi_interpolator(sdistance)
    sdistance += V_magnitude * timestep * np.cos(orientation - phi)
    if sdistance >= max_sdistance:
        sdistance -= max_sdistance
        LapCompleted = True
        nLap += 1
        print("Lap Completed")

    refDistance = sdistance

    return refDistance, newSim, nLap

def compute_feedforward(ref_distance, vProfiledf, DiLMode, nLap):
    """ Compute feedforward throttle and brake values using the reference distance. """
    
    target_steer = []
    global corner
    steer_interpolator = interp1d(vProfiledf['sDistance'], vProfiledf['xSteerRack'], kind='linear', fill_value="extrapolate")
    
    # Compute feedforward throttle and brake values at the reference distance
    ff_steer = steer_interpolator(ref_distance)

    if abs(ff_steer) > 2:
        corner = True
    else:
        corner = False

    if DiLMode == True:
        ff_steer = ff_steer/1000
    ff_steer = -ff_steer
    target_steer.append(ff_steer)
    if DiLMode == True:
        ff_steer = ff_steer * 1000
    # ff_steer = -ff_steer
    
    return ff_steer, target_steer, corner

def AngularOffset(trajectory_matrix, refDistance, orientation, e_phi_gain, nLap):
    # Computes the angular offset from the ideal orientation by comparing the current orientation with the reference orientation (trajectory orientation)

    refLine_orientation = []
    phi_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['Orientation'], kind='linear', fill_value="extrapolate") # Equation 4.25
    phi = phi_interpolator(refDistance)
    refLine_orientation.append(phi)
    e_phi = phi - orientation # Equation 4.26
    e_phi *= e_phi_gain

    return e_phi, refLine_orientation

def compute_lateral_offset(trajectory_matrix, x, y, orientation, ref_distance, nLap, e_l_gain, sat_e_l):
    global last_e_l

    """ Compute current lateral offset with smooth sign transitions """
    xr_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['x'], kind='linear', fill_value="extrapolate")
    yr_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['y'], kind='linear', fill_value="extrapolate")
    
    xr = xr_interpolator(ref_distance)
    yr = yr_interpolator(ref_distance)
    r = [xr, yr]
    
    lateral_offset = (r[1] - y) * math.cos(orientation) - (r[0] - x) * math.sin(orientation)

    e_l = lateral_offset * e_l_gain
    saturated_total_e_l = saturation(e_l, -sat_e_l, sat_e_l)

    return saturated_total_e_l


def compute_pid(error, dt, Kp, Ki, Kd, integral, prev_error):
    """ Compute the PID correction """
    P = Kp * error
    
    integral += error * dt
    I = Ki * integral
    
    derivative = (error - prev_error) / dt if dt > 0 else 0
    D = Kd * derivative
    
    output = P + I + D
    
    prev_error = error
    
    return output, integral, prev_error,  P, I, D

def saturation(value, min_value, max_value):
    saturated_values = np.clip(value, min_value, max_value)
    return saturated_values

def calculate_steering_FF(tracks_df, vProfiledf, Vx, Vy, yawAngle, x, y, e_phi_gain, timestep, nLap, newSim, 
                           Kp, Ki, Kd, integral, prev_error, e_l_gain, sat_e_l, DiLMode, max_sDistance_Lap1, clockwise):
    """ PID-based Steering Control without path preview points """
    trajectory_matrix = tracks_df
    global corner
    
    ref_distance, newSim, nLap  = refStateNow(trajectory_matrix, Vx, Vy, yawAngle, timestep, newSim, nLap, max_sDistance_Lap1, clockwise)

    steering_angle, target_steer, corner = compute_feedforward(ref_distance, vProfiledf, DiLMode, nLap)
    
    e_phi, refLineOrientation = AngularOffset(trajectory_matrix, ref_distance, yawAngle, e_phi_gain, nLap)
    
    e_l = compute_lateral_offset(trajectory_matrix, x, y, yawAngle, ref_distance, nLap, e_l_gain, sat_e_l)

    
    steering_angle_PID, integral, prev_error, P, I, D = compute_pid(e_l + e_phi, timestep, Kp, Ki, Kd, integral, prev_error)
    steering_angle_PID = saturation(steering_angle_PID, -1, 1)

    if corner == True:
        steering_angle_PID = steering_angle_PID * 0.1
    else:
        steering_angle_PID = steering_angle_PID * 0.01

    if DiLMode == True:
        steering_angle = steering_angle / 1000
        steering_angle_PID = steering_angle_PID / 1000
        P = P / 1000
        I = I / 1000
        D = D / 1000

    # if invert_steering == True:
    #     # P = -P
    #     # I = -I
    #     # D = -D
    #     # steering_angle_PID = -steering_angle_PID
    #     steering_angle = -steering_angle

    steering_angle += steering_angle_PID

    if DiLMode == True:
        steering_angle = saturation(steering_angle, -0.028, 0.028)
    else:
        steering_angle = saturation(steering_angle, -28, 28)
    
    
    return steering_angle, e_phi, e_l, refLineOrientation, ref_distance, integral, prev_error, newSim, P, I, D, target_steer

if __name__ == "__main__":
    calculate_steering_FF()
