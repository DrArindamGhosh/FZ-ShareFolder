import numpy as np
import math
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy.io import loadmat
import pandas as pd

file_path = 'London_Traj_Mat.xlsx'
trajectory_matrix = pd.read_excel(file_path)
trajectory_matrix.head(), trajectory_matrix.columns

# file_path = 'Shanghai_Traj_Mat.xlsx'
# trajectory_matrix = pd.read_excel(file_path)
# trajectory_matrix.head(), trajectory_matrix.columns

sdistance = 0
nLap = 0


def refStateNow(trajectory_matrix, Vx, Vy, orientation, timestep):
    # Calculates the current reference state of the vehicle given the trajectory T, velocity V, orientation, and a timestep.
    # The code calculates the reference state, taking into account the vehicle’s orientation and updating the reference distance based on velocity and orientation
    # Corresponds with the paper’s approach of continuously updating the vehicle's reference position along the trajectory

    global sdistance
    global nLap

    max_sdistance = np.max(trajectory_matrix['sDistance'])

    V_magnitude = np.sqrt(Vx**2 + Vy**2)
    
    # Create an interpolation function based on trajectory data
    phi_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['Orientation2'], kind='linear', fill_value="extrapolate")
    # print(f"sLap: ", trajectory_matrix [:, 0])
    phi = phi_interpolator(sdistance)
    # phi = phi - np.pi
    # phi = phi + np.pi

    sdistance += V_magnitude * timestep  * max(0, np.cos(orientation - phi))
    # sdistance += V_magnitude * timestep * np.cos(orientation - phi)

    if sdistance >= max_sdistance:
        sdistance -= max_sdistance
        phi = phi_interpolator(sdistance)

        # # Determine if we are going clockwise or anticlockwise
        # if trajectory_matrix[0, 2] > trajectory_matrix[-1, 2]:
        #     # Anticlockwise
        #     nLap -= 1
        # else:
        #     # Clockwise
        #     nLap += 1

    # print(f"sdistance: " , sdistance)

    refDistance = sdistance
    # print(f"Reference Orientation Make Up: " , phi, nLap)
    refOrientation = phi + nLap * 2 * np.pi

    # print(f"Reference Distance: " , refDistance)
    # print(f"Reference Orientation: " , refOrientation)

    return refDistance, refOrientation


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


def CorrespondingIdealPathPoints(trajectory_matrix, lever_distances, refDistance):
    # Computes points on the ideal path based on lever distances and the reference distance.
    # Uses interpolation to find corresponding points on the trajectory.
    ideal_coordinates = []

    # Initialize the result matrix r with zeros
    r = np.zeros((8, 2))
    
    # Extract the maximum distance from the trajectory data
    max_sdistance = np.max(trajectory_matrix['sDistance'])
    # print("Max Distance: ", max_sdistance)
    
    # Create interpolation functions for the x and y coordinates
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


def AngularOffset(trajectory_matrix, refDistance, orientation, e_phi_gain):
    # Computes the angular offset from the ideal orientation by comparing the current orientation with the reference orientation (trajectory orientation)

    refLine_orientation = []
    phi_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['Orientation2'], kind='linear', fill_value="extrapolate") # Equation 4.25
    phi = phi_interpolator(refDistance)
    # print("phi: ", phi)
    # print("Reference Distance: ", refDistance)
    # phi = phi - np.pi
    refLine_orientation.append(phi)

    # Calculate the angular offset
    e_phi = phi - orientation # Equation 4.26
    # e_phi = orientation - phi  # Equation 4.26

    e_phi *= e_phi_gain

    # print(e_phi)



    return e_phi, refLine_orientation


def LateralOffset(r, l, orientation, gains, lower_limits, upper_limits):
    # Calculates the lateral offsets between ideal path points and actual lever points based on orientation

    # Initialize the result array for lateral offsets
    # orientation = -orientation
    e_l = np.zeros(8)
    saturated_e_l = []
    error_values_gains_array = []

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
    saturated_total_e_l = saturation(total_e_l, -18, 18)
    # print(saturated_total_e_l)
    # saturated_total_e_l = saturation(total_e_l, -10, 10)

    return e_l, saturated_e_l, saturated_total_e_l, error_values_gains_array


def saturation(value, min_value, max_value):
    # Apply saturation limits to an array of values.

    saturated_values = np.clip(value, min_value, max_value)
    return saturated_values

def steering_control_output(e_phi, saturated_total_e_l):
    unsaturated_steering_controls = e_phi + saturated_total_e_l
    steering_control_input = saturation(unsaturated_steering_controls, -28, 28)
    steering_control_input = -steering_control_input
    return steering_control_input



##################################################################################################################
# MAIN FUNCTION



def calculate_steering_control(Vx, Vy, yawAngle, x, y, timestep):
    
    # Calculate Lever Distances
    lever_distances = LeverDistances(Vx)

    steering_outputs = []
    


    current_position = x, y


    # Reference State Now
    ref_distance, ref_orientation = refStateNow(trajectory_matrix, Vx, Vy, yawAngle, timestep)

    # Corresponding Ideal Path Points
    ideal_path_points, ideal_coordinates_plot = CorrespondingIdealPathPoints(trajectory_matrix, lever_distances, ref_distance)

    # Lever Absolute Coordinates
    lever_absolute_coordinates, lever_coords_plot = LeverAbsoluteCoordinates(lever_distances, current_position[0], current_position[1], yawAngle)

    # Angular Offset
    e_phi_gain = 2
    # e_phi_gain = 0.5235987756
    e_phi, refLine_orientation = AngularOffset(trajectory_matrix, ref_distance, yawAngle, e_phi_gain)
    # e_phi = AngularOffset(current_orientation, ref_orientation, e_phi_gain)

    # Angular Offset Test
    # e_phi_gain = 0.5
    # e_phi = AngularOffset(current_orientation, ref_orientation, e_phi_gain)

    # Matlab Values
    e_l_gains = np.array([3, 2, 1.3, 0.8, 0.55, 0.32, 0.12, 0.04])
    e_l_upper_limits = np.array([1.5, 1.5, 2, 2.75, 3, 2.75, 2.25, 1.7])
    e_l_lower_limits = np.array([-1.5, -1.5, -2, -2.75, -3, -2.75, -2.25, -1.7])

    # Test Values
    # e_l_gains = np.array([0.5,   0.25,  0.15,  0.1,   0.075, 0.05,  0.025, 0.01])
    # e_l_upper_limits = np.array([0.75,  0.75,  1,    1.375, 1.5,   1.375, 1.125, 0.85])
    # e_l_lower_limits = np.array([-0.75,  -0.75,  -1,    -1.375, -1.5,   -1.375, -1.125, -0.85])

    # Paper Values
    # e_l_gains = np.array([10, 10, 6, 2, 0.8, 0.16, 0.04, 0.01])
    # e_l_upper_limits = np.array([1, 2, 2, 2, 2, 1, 1, 1])
    # e_l_lower_limits = np.array([-1, -2, -2, -2, -2, -1, -1, -1])

    # Lateral Offset
    e_l, saturated_e_l, total_saturated_e_l, error_values_gains_array = LateralOffset(ideal_path_points, lever_absolute_coordinates, yawAngle, e_l_gains, e_l_lower_limits, e_l_upper_limits)

    # print("Angular Offset (e_phi):", e_phi)
    # print("Lateral Offsets (e_l):", e_l)
    # print("Total Saturated Lateral Offset (total_saturated_e_l):", total_saturated_e_l)

    # Steering Control
    steering_control_input = steering_control_output(e_phi, total_saturated_e_l)
    steering_outputs.append(steering_control_input)



    # print("Steering Control:", steering_control_input)
    
    return steering_control_input, e_phi, saturated_e_l, lever_coords_plot, ideal_coordinates_plot, e_l, refLine_orientation, ref_distance






if __name__ == "__main__":
    calculate_steering_control()
    