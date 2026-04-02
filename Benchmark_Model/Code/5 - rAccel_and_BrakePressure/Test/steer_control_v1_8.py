# steer_control_v16_1_DiL.py Compatible with Model_Application_v16_1, v16_2 and v16_3

import numpy as np
import math
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy.io import loadmat
import pandas as pd


sdistance = 0
nLap = 0


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

    return refDistance, newSim, nLap, LapCompleted, phi


def LeverDistances(longitudinal_velocity):
    # Returns a list of distances from a base point, scaled by the long. velocity component.
    # Creates an array of distances based on the longitudinal velocity, which represent the preview points mentioned in the paper

    longitudinal_velocity = abs(longitudinal_velocity)
    L = np.zeros(8)

    L[0] = 0
    L[1] = longitudinal_velocity * 0.1
    L[2] = longitudinal_velocity * 0.2
    L[3] = longitudinal_velocity * 0.3
    L[4] = longitudinal_velocity * 0.4
    L[5] = longitudinal_velocity * 0.6
    L[6] = longitudinal_velocity * 0.8
    L[7] = longitudinal_velocity * 1.0 # Equation 4.20

    return L


def CorrespondingIdealPathPoints(trajectory_matrix, lever_distances, refDistance, nLap):
    # Computes points on the ideal path based on lever distances and the reference distance.
    # Uses interpolation to find corresponding points on the trajectory.
    ideal_coordinates = []

    # Initialize the result matrix r with zeros
    r = np.zeros((8, 2))
    
    max_sdistance = np.max(trajectory_matrix['sDistance'])
    xr_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['x'], kind='linear', fill_value="extrapolate")
    yr_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['y'], kind='linear', fill_value="extrapolate")
    
    for i in range(8):
        targetDistance = refDistance + lever_distances[i] # Equation 4.22
        # print("Target Distance: ", targetDistance)
        if targetDistance > max_sdistance:
            targetDistance -= max_sdistance
        
        # Interpolate the x and y coordinates at the target distance
        xr = xr_interpolator(targetDistance)
        yr = yr_interpolator(targetDistance) # Equation 4.23
        r[i, :] = [xr, yr]

    # print(f"Ideal path points: " , r)
    ideal_coordinates.append(r)

    return r, ideal_coordinates


def LeverAbsoluteCoordinates(lever_distances, start_x, start_y, orientation):
    # Computes absolute coordinates of points relative to a base position (start_x, start_y) using lever distances and orientation

    lever_coordinates = []

    # Initialize the result array with zeros
    l = np.zeros((8, 2))
    
    for i in range(8):
        # Calculate the coordinates based on the orientation and lever distances
        xL = start_x + lever_distances[i] * math.cos(orientation)
        yL = start_y + lever_distances[i] * math.sin(orientation) # Equation 4.21
        # xL = start_x + lever_distances[i]
        # yL = start_y + lever_distances[i]
        l[i, :] = [xL, yL]

    # print(f"Lever Coordinates: " , l)
    lever_coordinates.append(l)
    # print(l)

    return l, lever_coordinates


def AngularOffset(trajectory_matrix, refDistance, orientation, e_phi_gain, nLap, LapCompleted, clockwise, phi):
    # Computes the angular offset from the ideal orientation by comparing the current orientation with the reference orientation (trajectory orientation)

    refLine_orientation = []
    # if LapCompleted == True and nLap == 1:
    #     if clockwise == True:
    #         phi = phi - 2*np.pi
    #     else:
    #         phi =+ 2 * np.pi
        # LapCompleted = False
    

    refLine_orientation.append(phi)

    # Calculate the angular offset
    e_phi = phi - orientation # Equation 4.26


    e_phi *= e_phi_gain


    # print(e_phi)



    return e_phi, refLine_orientation


def LateralOffset(r, l, orientation, gains, lower_limits, upper_limits, lower_total_e_l_limits, upper_total_e_l_limits):
    # Calculates the lateral offsets between ideal path points and actual lever points based on orientation

    # Initialize the result array for lateral offsets
    e_l = np.zeros(8)
    saturated_e_l = []
    error_values_gains_array = []

    # print(f"r: {r}, \nl: {l}")

    for i in range(8):
        # Calculate the lateral offset based on the orientation
        offset = (r[i, 1] - l[i, 1]) * math.cos(orientation) - (r[i, 0] - l[i, 0]) * math.sin(orientation) # Equation 4.24
        # print(offset)

        e_l[i] = offset * gains[i]
        # print(e_l[i])
        # error_values_gains_array.append(e_l)

    

    saturated_e_l = np.array([saturation(e_l[i], lower_limits[i], upper_limits[i]) for i in range(8)])
    # print(saturated_e_l)

    # saturated_e_l.append(saturated_e_l)

    total_e_l = np.sum(saturated_e_l)
    # print(total_e_l)
    saturated_total_e_l = saturation(total_e_l, lower_total_e_l_limits, upper_total_e_l_limits)
    # print(saturated_total_e_l)
    # saturated_total_e_l = saturation(total_e_l, -10, 10)

    return e_l, saturated_e_l, saturated_total_e_l, error_values_gains_array


def saturation(value, min_value, max_value):
    saturated_values = np.clip(value, min_value, max_value)
    return saturated_values

def steering_control_output(e_phi, saturated_total_e_l, steer_limits, Dil_Mode):
    unsaturated_steering_controls = e_phi + saturated_total_e_l
    if Dil_Mode == True:
        unsaturated_steering_controls = unsaturated_steering_controls / 1000
        steering_control_input = saturation(unsaturated_steering_controls, -steer_limits/1000, steer_limits/1000)
    else:
        steering_control_input = saturation(unsaturated_steering_controls, -steer_limits, steer_limits)
    # steering_control_input = -steering_control_input
    return steering_control_input



##################################################################################################################
# MAIN FUNCTION



def calculate_steering_control(tracks_df, Vx, Vy, yawAngle, x, y, e_phi_gain, timestep, nLap, newSim, 
                               e_l_gains, e_l_limits, total_e_l_limits, steer_limits, Dil_Mode, max_sDistance_Lap1, clockwise):
                        

    trajectory_matrix = tracks_df
    trajectory_matrix.head(), trajectory_matrix.columns

    steering_outputs = []
    
    current_position = x, y

    # Reference State Now
    LapComplete = False
    ref_distance, newSim, nLap, LapComplete, phi = refStateNow(trajectory_matrix, Vx, Vy, yawAngle, timestep, newSim, nLap, max_sDistance_Lap1, LapComplete)
    # ref_distance, ref_orientation, refDistances = refStateNow(trajectory_matrix, Vx, Vy, yawAngle, timestep, sdistance, nLap)

    # Calculate Lever Distances
    lever_distances = LeverDistances(Vx)

    # Corresponding Ideal Path Points
    ideal_path_points, ideal_coordinates_plot = CorrespondingIdealPathPoints(trajectory_matrix, lever_distances, ref_distance, nLap)

    # Lever Absolute Coordinates
    lever_absolute_coordinates, lever_coords_plot = LeverAbsoluteCoordinates(lever_distances, current_position[0], current_position[1], yawAngle)

    e_phi, refLine_orientation = AngularOffset(trajectory_matrix, ref_distance, yawAngle, e_phi_gain, nLap, LapComplete, clockwise, phi)

    e_l_gains = np.array(e_l_gains)
    e_l_upper_limits = np.array(e_l_limits)
    e_l_lower_limits = np.array([-limit for limit in e_l_limits])

    upper_total_e_l_limits = total_e_l_limits
    lower_total_e_l_limits = - total_e_l_limits



    # Lateral Offset
    e_l, saturated_e_l, total_saturated_e_l, error_values_gains_array = LateralOffset(ideal_path_points, lever_absolute_coordinates, yawAngle, 
                                                                                      e_l_gains, e_l_lower_limits, e_l_upper_limits, 
                                                                                    lower_total_e_l_limits, upper_total_e_l_limits)

    # Steering Control
    steering_control_input = steering_control_output(e_phi, total_saturated_e_l, steer_limits, Dil_Mode)
    steering_outputs.append(steering_control_input)
    
    return steering_control_input, e_phi, saturated_e_l, lever_coords_plot, ideal_coordinates_plot, e_l, refLine_orientation, ref_distance, newSim, nLap


if __name__ == "__main__":
    calculate_steering_control()
    