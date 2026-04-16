# Compatible with v16_4 only using a single trajectory

import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
plt.switch_backend('agg')
import numpy as np
import pandas as pd
import os

# Ensure the directory exists
output_dir = 'SimulationGraphs'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

def plotTraj(base_name, tracks_df, sim_x, sim_y, lever_coordinates, ideal_coordinates, e_l_values, current_time):
    tracks_df.head(), tracks_df.columns
    plt.figure(figsize=(10, 6))
    plt.plot(tracks_df['x'], tracks_df['y'], label='Ideal Path', marker='o', linestyle='-', markersize=2)
    # plt.plot(tracks_df['x2'], tracks_df['y2'], label='Ideal Path Lap 2', marker='o', linestyle='-', markersize=2, color='purple')
    plt.title('Plot of X, Y Coordinates')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.grid(True)

    plt.plot(sim_x, sim_y, label='Actual Path', linestyle='--')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')

    for i, (lever_coords, ideal_coords) in enumerate(zip(lever_coordinates, ideal_coordinates)):
        x_values = lever_coords[:, 0]
        y_values = lever_coords[:, 1]
        plt.plot(x_values, y_values, marker='x', linestyle='--', label='Lever Positions' if i == 0 else "")
        x_ideal = ideal_coords[:, 0]
        y_ideal = ideal_coords[:, 1]
        plt.plot(x_ideal, y_ideal, marker='o', linestyle='--', label='Ideal Points Corresponding to Lever Positions' if i == 0 else "")

        for j, ((x_lever, y_lever), (x_ideal, y_ideal)) in enumerate(zip(zip(x_values, y_values), zip(x_ideal, y_ideal))):
            plt.plot([x_lever, x_ideal], [y_lever, y_ideal], 'k-', linewidth=0.5)
            midpoint = ((x_lever + x_ideal) / 2, (y_lever + y_ideal) / 2)
            distance = np.sqrt((x_lever - x_ideal)**2 + (y_lever - y_ideal)**2)
            plt.annotate(f"Dist = {distance:.2f}", midpoint, textcoords="offset points", xytext=(-15, -15), ha='center')
            e_l_value = e_l_values[j]
            plt.annotate(f"Err = {e_l_value:.2f}", midpoint, textcoords="offset points", xytext=(-15, 15), ha='center')

    plt.title(f'{base_name} Vehicle Trajectory')
    plt.legend()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_Vehicle_Trajectory.png'))
    plt.show()

def plot_e_phi_Values(base_name, e_phi_values, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(e_phi_values, label='e_phi', linestyle='-', marker='o')
    plt.title(f'{base_name} Angular Offset over Iterations', fontsize=26)
    plt.xlabel('Iteration', fontsize=26)
    plt.ylabel('Error (e_phi)', fontsize=26)
    plt.legend(fontsize=26)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_Angular_Offset.png'))
    plt.show()

def plot_e_l_Values(base_name, saturated_e_l_values, current_time):
    plt.figure(figsize=(15, 10))
    plt.title(f'{base_name} Saturated Preview Inputs over Iterations')
    for i in range(8):
        plt.subplot(8, 1, i + 1)
        plt.plot(saturated_e_l_values[i], label=f'Saturated Preview Inputs[{i}]')
        plt.xlabel('Iteration over SimTime')
        plt.ylabel('Lateral Offset')
        plt.legend(fontsize=18)
        plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_Lateral_Offsets.png'))
    plt.show()

def plot_balance_Values(base_name, steering_input, line_orientation, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(steering_input, label='Steering Input', linestyle='-', marker='o')
    # plt.plot(line_orientation, label='Orientation of Ref Line (Radians)')
    plt.title(f'{base_name} Steering Input over Iterations', fontsize=26)
    plt.xlabel('Iteration', fontsize=26)
    plt.ylabel('Steering Input', fontsize=26)
    plt.legend(fontsize=26)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_Steering_Input_vs_RefLine_Orientation.png'))
    plt.show()

def plot_Angles(base_name, yaw_angle, line_orientation, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(yaw_angle, label='Orientation of Vehicle (Radians)', linestyle='-', marker='o')
    plt.plot(line_orientation, label='Orientation of Ref Line (Radians)', linestyle='--', marker='s')
    plt.title(f'{base_name} Car and Trajectory Orientation over Iterations', fontsize=22)
    plt.xlabel('Iteration', fontsize=26)
    plt.ylabel('Yaw Angle', fontsize=26)
    plt.legend(fontsize=26)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_Car_and_Trajectory_Orientation.png'))
    plt.show()

def plot_Torque(base_name, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(MDriveRL, label='MDriveRL', linestyle='-', marker='o')
    plt.plot(MDriveRR, label='MDriveRR', linestyle='--', marker='s')
    plt.plot(MBrakeFL, label='MBrakeFL', linestyle='-.', marker='D')
    plt.plot(MBrakeFR, label='MBrakeFR', linestyle=':', marker='^')
    plt.plot(MBrakeRL, label='MBrakeRL', linestyle='-', marker='v')
    plt.plot(MBrakeRR, label='MBrakeRR', linestyle='--', marker='<')
    plt.title(f'{base_name} Car Torque Inputs', fontsize=26)
    plt.xlabel('Iteration', fontsize=26)
    plt.ylabel('Torque', fontsize=26)
    plt.legend(fontsize=26)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_Car_Torque_Inputs.png'))
    plt.show()

def plot_TargetValues(base_name, targetVelocityArray, longAccelerationArray, current_Velocity, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(targetVelocityArray, label='Target Velocity (kph)', linestyle='-', marker='o')
    plt.plot(longAccelerationArray, label='Longitudinal Acceleration (m/s**2)', linestyle='--', marker='s')
    plt.plot(current_Velocity, label='Current Velocity (kph)', linestyle='-.', marker='D')
    plt.title(f'{base_name} Target Velocity Values', fontsize=26)
    plt.xlabel('Iteration', fontsize=26)
    plt.ylabel('Velocity and Acceleration', fontsize=26)
    plt.legend(fontsize=26)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_Target_Velocity_Values.png'))
    plt.show()

def plot_longThrustValues(longThrust, slipThrust):
    plt.figure(figsize=(15, 10))
    plt.plot(longThrust, label='Long Thrust', linestyle='-', marker='o')
    plt.plot(slipThrust, label='Slip Thrust', linestyle='--', marker='s')
    plt.title('Thrust Change After Slip', fontsize=16)
    plt.xlabel('Iteration', fontsize=14)
    plt.ylabel('Thrust (N)', fontsize=14)
    plt.legend(fontsize=12)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'Thrust_Change_After_Slip.png'))
    plt.show()

def plot_Scf_Values(base_name, scaling_factor, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(scaling_factor, label='Scaling Factor', linestyle='-', marker='o')
    plt.title(f'{base_name} Scaling Factor over Iterations', fontsize=26)
    plt.xlabel('Iteration', fontsize=26)
    plt.ylabel('Scaling Factor', fontsize=26)
    plt.legend(fontsize=26)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_Scaling_Factor.png'))
    plt.show()
