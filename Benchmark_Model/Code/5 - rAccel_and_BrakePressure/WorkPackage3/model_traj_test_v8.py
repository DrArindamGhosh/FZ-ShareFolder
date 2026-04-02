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

def plotTraj(base_name, tracks_df, sim_x, sim_y, current_time):
    tracks_df.head(), tracks_df.columns
    plt.figure(figsize=(10, 6))
    plt.plot(tracks_df['x'], tracks_df['y'], label='Ideal Path', marker='o', linestyle='-', markersize=2)
    plt.plot(tracks_df['x2'], tracks_df['y2'], label='Ideal Path Lap 2', marker='o', linestyle='-', markersize=2, color='purple')
    plt.title('Plot of X, Y Coordinates')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.grid(True)

    plt.plot(sim_x, sim_y, label='Actual Path', linestyle='--')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')

    plt.title(f'Vehicle Trajectory')
    plt.legend()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_a_Vehicle_Trajectory.png'))
    plt.show()

def plot_e_phi_Values(base_name, e_phi_values, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(e_phi_values, label='e_phi')
    plt.title(f'Angular Offset over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('Error (e_phi)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_e_Angular_Offset.png'))
    plt.show()

def plot_e_l_Values(base_name, saturated_e_l_values, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(saturated_e_l_values, label='Saturated Preview Input')
    plt.title(f'Saturated Preview Input over Iterations')
    plt.xlabel('Iteration over SimTime')
    plt.ylabel('Lateral Offset')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_f_Lateral_Offset.png'))
    plt.show()

def plot_balance_Values(base_name, steering_input, target_steering_angles, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(steering_input, label='Steering Input')
    plt.plot(target_steering_angles, label='Target Steer')
    plt.title(f'Steering Input and Target Steer over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('Values')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_b_Steering_Input_and_Target.png'))
    plt.show()

def plot_Angles(base_name, yaw_angle, line_orientation, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(yaw_angle, label='Orientation of Vehicle (Radians)')
    plt.plot(line_orientation, label='Orientation of Ref Line (Radians)')
    plt.title(f'Car and Trajectory Orientation over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('Yaw Angle')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_c_Car_and_Trajectory_Orientation.png'))
    plt.show()

def plot_Torque(base_name, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(MDriveRL, label='MDriveRL')
    plt.plot(MDriveRR, label='MDriveRR')
    plt.plot(MBrakeFL, label='MBrakeFL')
    plt.plot(MBrakeFR, label='MBrakeFR')
    plt.plot(MBrakeRL, label='MBrakeRL')
    plt.plot(MBrakeRR, label='MBrakeRR')
    plt.title(f'Car Torque Inputs')
    plt.xlabel('Iteration')
    plt.ylabel('Torque')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_l_Car_Torques.png'))
    plt.show()

def plot_TargetValues(base_name, targetVelocityArray, longAccelerationArray, bPressure, current_Velocity, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(targetVelocityArray, label='Target Velocity (kph)')
    plt.plot(current_Velocity, label='Current Velocity (kph)')
    plt.plot(longAccelerationArray, label='Accelerator Pedal (%)')
    plt.plot(bPressure, label='Brake Pressure (bar)')
    plt.title(f'Target Velocity Values')
    plt.xlabel('Iteration')
    plt.ylabel('Velocity, Accelerator Pedal, and Brake Pressure')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_g_Target_Velocity_Values.png'))
    plt.show()

def plot_PedalValues(base_name, accPedal, bPressure, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(accPedal, label='Throttle (%)')
    plt.plot(bPressure, label='Brake Pressure (bar)')
    plt.title(f'Pedal Values over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('Throttle (%) and Brake Pressure (bar)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_h_Pedal_Values.png'))
    plt.show()

def plot_nMGU_Values(base_name, nMGU, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(nMGU, label='nMGU')
    plt.title(f'nMGU over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('nMGU')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_m_nMGU.png'))
    plt.show()

def plot_Throttle_PID_Values(base_name, t_P, t_I, t_D, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(t_P, label='Throttle P')
    plt.plot(t_I, label='Throttle I')
    plt.plot(t_D, label='Throttle D')
    plt.title(f'Throttle PID Values over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('PID Values')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_i_Throttle_PID_Values.png'))
    plt.show()

def plot_Brake_PID_Values(base_name, b_P, b_I, b_D, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(b_P, label='Brake P')
    plt.plot(b_I, label='Brake I')
    plt.plot(b_D, label='Brake D')
    plt.title(f'Brake PID Values over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('PID Values')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_j_Brake_PID_Values.png'))
    plt.show()

def plot_Steer_PID_Values(base_name, s_P, s_I, s_D, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(s_P, label='Steer P')
    plt.plot(s_I, label='Steer I')
    plt.plot(s_D, label='Steer D')
    plt.title(f'Steer PID Values over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('PID Values')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_d_Steer_PID_Values.png'))
    plt.show()

def plot_vCar_error_Values(base_name, vCar_error, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(vCar_error, label='vCar_error')
    plt.title(f'vCar_error over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('vCar_error (kph)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_k_vCar_error.png'))
    plt.show()