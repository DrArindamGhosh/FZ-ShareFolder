import numpy as np
import math
from scipy.interpolate import interp1d
import pandas as pd

sdistance = 0
# nLap = 0
last_e_l = 0


def refStateNow(trajectory_matrix, Vx, Vy, orientation, timestep, newSim, nLap):
    global sdistance
    # global nLap

    refDistance = 0

    if newSim is True:
        sdistance = sdistance - sdistance
        refDistance = refDistance - refDistance
        newSim = False

    refDistances = []
    max_sdistance = np.max(trajectory_matrix['sDistance'])
    max_sdistance2 = np.max(trajectory_matrix['sDistance2'])

    V_magnitude = np.sqrt(Vx**2 + Vy**2)
    if nLap == 0:
        phi_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['Orientation'], kind='linear', fill_value="extrapolate")
        phi = phi_interpolator(sdistance)
    if nLap > 0:
        phi_interpolator = interp1d(trajectory_matrix['sDistance2'], trajectory_matrix['Orientation2'], kind='linear', fill_value="extrapolate")
        phi = phi_interpolator(sdistance)


    # print(f"V_Mag: {V_magnitude}, Orientation: {orientation}, phi: {phi}")
    sdistance += V_magnitude * timestep * np.cos(orientation - phi)

    if max_sdistance - sdistance < 0.5:
        if nLap == 1:
            sdistance -= max_sdistance
        if nLap > 1:
            sdistance -= max_sdistance2
        phi = phi_interpolator(sdistance)

    # print(f"sdistance: {sdistance}")

    refDistance = sdistance

    return refDistance, refDistances, newSim

def compute_feedforward(ref_distance, vProfiledf, DiLMode, nLap):
    """ Compute feedforward throttle and brake values using the reference distance. """
    
    target_steer = []
    # Interpolators for rAccel (Throttle) and pBrake (Brake Pressure)
    steer_interpolator = interp1d(vProfiledf['sDistance'], vProfiledf['xSteerRack'], kind='linear', fill_value="extrapolate")
    if nLap > 0:
        steer_interpolator = interp1d(vProfiledf['sDistance2'], vProfiledf['xSteerRack2'], kind='linear', fill_value="extrapolate")
    
    # Compute feedforward throttle and brake values at the reference distance
    ff_steer = steer_interpolator(ref_distance)

    if DiLMode == True:
        ff_steer = ff_steer/1000
    ff_steer = -ff_steer
    target_steer.append(ff_steer)
    if DiLMode == True:
        ff_steer = ff_steer * 1000
    ff_steer = -ff_steer
    
    return ff_steer, target_steer

def AngularOffset(trajectory_matrix, refDistance, orientation, e_phi_gain, nLap):
    # Computes the angular offset from the ideal orientation by comparing the current orientation with the reference orientation (trajectory orientation)

    refLine_orientation = []
    if nLap == 0:
        phi_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['Orientation'], kind='linear', fill_value="extrapolate") # Equation 4.25
    if nLap > 0:
        phi_interpolator = interp1d(trajectory_matrix['sDistance2'], trajectory_matrix['Orientation2'], kind='linear', fill_value="extrapolate") # Equation 4.25
    phi = phi_interpolator(refDistance)
    

    refLine_orientation.append(phi)


    e_phi = phi - orientation # Equation 4.26

    e_phi *= e_phi_gain

    return e_phi, refLine_orientation

# def compute_lateral_offset(trajectory_matrix, x, y, ref_distance, nLap):
#     """ Compute current lateral offset without preview points """
#     if nLap == 0:
#         max_sdistance = np.max(trajectory_matrix['sDistance'])
#         xr_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['x'], kind='linear', fill_value="extrapolate")
#         yr_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['y'], kind='linear', fill_value="extrapolate")
#     else:
#         max_sdistance = np.max(trajectory_matrix['sDistance2'])
#         xr_interpolator = interp1d(trajectory_matrix['sDistance2'], trajectory_matrix['x2'], kind='linear', fill_value="extrapolate")
#         yr_interpolator = interp1d(trajectory_matrix['sDistance2'], trajectory_matrix['y2'], kind='linear', fill_value="extrapolate")
    
#     xr = xr_interpolator(ref_distance)
#     yr = yr_interpolator(ref_distance)
    
#     lateral_offset = ((x - xr) ** 2 + (y - yr) ** 2) ** 0.5
#     lateral_offset = -lateral_offset
    
#     return lateral_offset

def compute_lateral_offset(trajectory_matrix, x, y, orientation, ref_distance, nLap, e_l_gain, sat_e_l):
    global last_e_l

    """ Compute current lateral offset with smooth sign transitions """
    if nLap == 0:
        xr_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['x'], kind='linear', fill_value="extrapolate")
        yr_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['y'], kind='linear', fill_value="extrapolate")
    if nLap > 0:
        xr_interpolator = interp1d(trajectory_matrix['sDistance2'], trajectory_matrix['x2'], kind='linear', fill_value="extrapolate")
        yr_interpolator = interp1d(trajectory_matrix['sDistance2'], trajectory_matrix['y2'], kind='linear', fill_value="extrapolate")
    
    xr = xr_interpolator(ref_distance)
    yr = yr_interpolator(ref_distance)
    r = [xr, yr]
    
    # Compute lateral error magnitude
    # xL = x * math.cos(orientation)
    # yL = y * math.sin(orientation)
    # l = [xL, yL]
    # print(f"X: {x}, Y: {y}, R: {r}, Orientation: {orientation}, nLap: {nLap}")
    lateral_offset = (r[1] - y) * math.cos(orientation) - (r[0] - x) * math.sin(orientation)
    # lateral_offset = (r[1] - l[1]) * math.cos(orientation) - (r[0] - l[0]) * math.sin(orientation)
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

def calculate_steering_pid(tracks_df, vProfiledf, Vx, Vy, yawAngle, x, y, e_phi_gain, timestep, nLap, newSim, 
                           Kp, Ki, Kd, integral, prev_error, e_l_gain, sat_e_l, DiLMode, invert_steering):
    """ PID-based Steering Control without path preview points """
    trajectory_matrix = tracks_df
    
    ref_distance, _, newSim = refStateNow(trajectory_matrix, Vx, Vy, yawAngle, timestep, newSim, nLap)

    steering_angle, target_steer = compute_feedforward(ref_distance, vProfiledf, DiLMode, nLap)
    
    e_phi, refLineOrientation = AngularOffset(trajectory_matrix, ref_distance, yawAngle, e_phi_gain, nLap)
    
    e_l = compute_lateral_offset(trajectory_matrix, x, y, yawAngle, ref_distance, nLap, e_l_gain, sat_e_l)

    
    steering_angle_PID, integral, prev_error, P, I, D = compute_pid(e_l + e_phi, timestep, Kp, Ki, Kd, integral, prev_error)

    if DiLMode == True:
        steering_angle = steering_angle / 1000
        steering_angle_PID = steering_angle_PID / 1000
        P = P / 1000
        I = I / 1000
        D = D / 1000

    if invert_steering == True:
        # P = -P
        # I = -I
        # D = -D
        # steering_angle_PID = -steering_angle_PID
        steering_angle = -steering_angle

    steering_angle += steering_angle_PID

    if DiLMode == True:
        steering_angle = saturation(steering_angle, -0.028, 0.028)
    else:
        steering_angle = saturation(steering_angle, -28, 28)

    
    
  
    return steering_angle, e_phi, e_l, refLineOrientation, ref_distance, integral, prev_error, newSim, P, I, D, target_steer

if __name__ == "__main__":
    calculate_steering_pid()
