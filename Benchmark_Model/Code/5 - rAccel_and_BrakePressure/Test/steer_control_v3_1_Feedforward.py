# === Improved Steering Controller with Lookahead Projection and Adaptive Gains ===
# This code implements a feedforward + feedback steering controller for high-performance path tracking.
# It uses the Fiala tire model for feedforward steering and multi-point preview control for feedback.
# Key ideas from Kapania & Gerdes (2015) and multi-preview control literature are used.

import numpy as np
import math
from scipy.interpolate import interp1d

# --- Feedforward Steering Calculation using the Fiala Tire Model ---
def compute_steering_fiala(Ux, kappa, L=2.8, lf=1.4, lr=1.4, Caf=80000, Car=80000, 
                           mf=420, mr=420, mu=1.7):
    # Calculates ideal steering angle based on curvature and lateral force balance
    m = mf + mr  # total vehicle mass
    a_y = Ux**2 * kappa  # lateral acceleration from curvature

    # Approximate lateral force distribution front/rear
    Fyf = mf / m * m * a_y
    Fyr = mr / m * m * a_y

    # Estimate slip angles using inverse of friction-limited lateral force
    alpha_f = np.arcsin(np.clip(Fyf / (mu * mf * 9.81), -1, 1))
    alpha_r = np.arcsin(np.clip(Fyr / (mu * mr * 9.81), -1, 1))

    # Feedforward steering from bicycle model + slip compensation
    delta_rad = (L + ((lr * Car - lf * Caf) / (Caf + Car))) * kappa
    return delta_rad - alpha_f + alpha_r

# --- Convert Steering Wheel Angle to Rack Travel (mm) ---
def steering_angle_to_rack(angle_deg):
    return 0.1154 * angle_deg - 0.0178  # empirically fit linear relation

# --- Generic Saturation Function ---
def saturation(value, min_value, max_value):
    return np.clip(value, min_value, max_value)

# --- Main Steering Controller ---
def calculate_steering_control_3_1(tracks_df, Vx, Vy, yawAngle, x, y, e_phi_gain, timestep, nLap, newSim, 
                               e_l_gains, e_l_limits, total_e_l_limits, steer_limits, Dil_Mode, max_sDistance_Lap1, clockwise):

    global sdistance

    # --- Update Simulation Distance Along Path ---
    V_magnitude = np.sqrt(Vx**2 + Vy**2)
    if newSim:
        sdistance = 0
        newSim = False

    # Interpolate desired path orientation at current location
    phi_interpolator = interp1d(tracks_df['sDistance'], tracks_df['Orientation'], kind='linear', fill_value="extrapolate")
    phi = phi_interpolator(sdistance)

    # Advance distance along the path using vehicle progress in world frame
    sdistance += V_magnitude * timestep * np.cos(yawAngle - phi)
    if sdistance >= max_sDistance_Lap1:
        sdistance -= max_sDistance_Lap1
        nLap += 1
        print("Lap Completed")

    ref_distance = sdistance
    current_position = x, y

    # --- Multi-Point Preview Distances (Velocity-Scaled) ---
    L = np.array([
        0,
        Vx * 0.1,
        Vx * 0.2,
        Vx * 0.3,
        Vx * 0.4,
        Vx * 0.6,
        Vx * 0.8,
        Vx * 1.0
    ])

    # --- Compute Reference Points Along Track for Each Preview ---
    r = np.zeros((8, 2))
    x_interp = interp1d(tracks_df['sDistance'], tracks_df['x'], kind='linear', fill_value="extrapolate")
    y_interp = interp1d(tracks_df['sDistance'], tracks_df['y'], kind='linear', fill_value="extrapolate")
    max_s = tracks_df['sDistance'].max()
    for i in range(8):
        s_target = ref_distance + L[i]
        if s_target > max_s:
            s_target -= max_s  # wrap around lap
        r[i, :] = [x_interp(s_target), y_interp(s_target)]

    # --- Compute Lookahead Points from Current Vehicle Position ---
    l = np.zeros((8, 2))
    for i in range(8):
        l[i, 0] = current_position[0] + L[i] * np.cos(yawAngle)
        l[i, 1] = current_position[1] + L[i] * np.sin(yawAngle)

    # --- Lateral Error Between Vehicle and Reference for Each Preview Point ---
    e_l = np.zeros(8)
    for i in range(8):
        offset = (r[i, 1] - l[i, 1]) * np.cos(yawAngle) - (r[i, 0] - l[i, 0]) * np.sin(yawAngle)
        e_l[i] = offset * e_l_gains[i]  # scale per-preview point

    # --- Clip Lateral Errors ---
    e_l_upper = np.array(e_l_limits)
    e_l_lower = -e_l_upper
    sat_e_l = saturation(e_l, e_l_lower, e_l_upper)
    total_e_l = np.sum(sat_e_l)
    total_sat_e_l = saturation(total_e_l, -total_e_l_limits, total_e_l_limits)

    # --- Angular Heading Error ---
    lookahead_distance = 1.0  # fixed lookahead for angular projection
    e_phi = phi - yawAngle  # orientation error
    lookahead_projection = total_sat_e_l + lookahead_distance * e_phi

    # --- Velocity-Adaptive Feedback Gain ---
    adaptive_gain = e_phi_gain * (1 + 0.1 * Vx)
    delta_fb = -adaptive_gain * lookahead_projection  # feedback correction

    # --- Feedforward Steering via Fiala Tire Model ---
    curvature_interp = interp1d(tracks_df['sDistance'], tracks_df['curvature'], fill_value="extrapolate")
    kappa = curvature_interp(ref_distance)  # desired curvature
    delta_rad = compute_steering_fiala(Vx, kappa)  # desired angle in radians
    delta_deg = np.degrees(delta_rad)
    xSteerRack_ff = steering_angle_to_rack(delta_deg)  # mm

    # --- Combine Feedforward + Feedback ---
    xSteerRack = xSteerRack_ff + delta_fb

    # --- Apply Optional DiL Unit Conversion and Saturation ---
    if Dil_Mode:
        xSteerRack = xSteerRack / 1000  # convert mm → m
        xSteerRack = saturation(xSteerRack, -steer_limits/1000, steer_limits/1000)
    else:
        xSteerRack = saturation(xSteerRack, -steer_limits, steer_limits)
        xSteerRack = -xSteerRack  # flip direction for simulation model

    return xSteerRack, e_phi, sat_e_l, l, r, e_l, [phi], ref_distance, newSim, nLap