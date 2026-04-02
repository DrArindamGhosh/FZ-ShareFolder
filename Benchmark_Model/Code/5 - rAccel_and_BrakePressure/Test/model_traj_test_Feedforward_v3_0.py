# Compatible with v16_4 only using a single trajectory

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import dill
# plt.switch_backend('agg')
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
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_A_Vehicle_Trajectory.png'))
    # with open(os.path.join(output_dir, f'{current_time}_{base_name}_A_Vehicle_Trajectory.dill'), 'wb') as f:
    #     dill.dump(plt, f)
    # plt.show()

def plotTrajFF(base_name, tracks_df, sim_x, sim_y, current_time):
    tracks_df.head(), tracks_df.columns
    plt.figure(figsize=(10, 6))
    plt.plot(tracks_df['x'], tracks_df['y'], label='Ideal Path', marker='o', linestyle='-', markersize=2)
    plt.title('Plot of X, Y Coordinates')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.grid(True)

    plt.plot(sim_x, sim_y, label='Actual Path', linestyle='--')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')

    plt.title(f'{base_name} Vehicle Trajectory')
    plt.legend()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_A_Vehicle_Trajectory.png'))

def plot_e_phi_Values(base_name, e_phi_values, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(e_phi_values, label='e_phi')
    plt.title(f'{base_name} Angular Offset over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('Error (e_phi)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_E_Angular_Offset.png'))
    # with open(os.path.join(output_dir, f'{current_time}_{base_name}_E_Angular_Offset.dill'), 'wb') as f:
    #     dill.dump(plt, f)
    # plt.show()

def plot_e_l_Values(base_name, saturated_e_l_values, current_time):
    plt.figure(figsize=(15, 10))
    for i in range(8):
        plt.subplot(8, 1, i + 1)
        plt.plot(saturated_e_l_values[i], label=f'Saturated Preview Inputs[{i}]')
        plt.title(f'{base_name} Saturated Preview Inputs[{i}] over Iterations')
        plt.xlabel('Iteration over SimTime')
        plt.ylabel('Lateral Offset')
        plt.legend()
        plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_F_Lateral_Offset.png'))
    # with open(os.path.join(output_dir, f'{current_time}_{base_name}_F_Lateral_Offset.dill'), 'wb') as f:
    #     dill.dump(plt, f)
    # plt.show()

def plot_e_l_Values_FF(base_name, saturated_e_l_values, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(saturated_e_l_values, label='Saturated Preview Input')
    plt.title(f'Saturated Preview Input over Iterations')
    plt.xlabel('Iteration over SimTime')
    plt.ylabel('Lateral Offset')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_F_Lateral_Offset.png'))

def plot_balance_Values(base_name, steering_input, target_steering_angles, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(steering_input, label='Steering Input')
    plt.plot(target_steering_angles, label='Target Steer')
    plt.title(f'{base_name} Steering Input over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('Steering Input')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_B_Steering_Input_and_Target.png'))
    # with open(os.path.join(output_dir, f'{current_time}_{base_name}_B_Steering_Input_and_Target.dill'), 'wb') as f:
    #     dill.dump(plt, f)
    # plt.show()

def plot_Angles(base_name, yaw_angle, line_orientation, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(yaw_angle, label='Orientation of Vehicle (Radians)')
    plt.plot(line_orientation, label='Orientation of Ref Line (Radians)')
    plt.title(f'{base_name} Car and Trajectory Orientation over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('Yaw Angle')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_C_Car_and_Trajectory_Orientation.png'))
    # with open(os.path.join(output_dir, f'{current_time}_{base_name}_C_Car_and_Trajectory_Orientation.dill'), 'wb') as f:
    #     dill.dump(plt, f)
    # plt.show()

def plot_Torque(base_name, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(MDriveRL, label='MDriveRL')
    plt.plot(MDriveRR, label='MDriveRR')
    plt.plot(MBrakeFL, label='MBrakeFL')
    plt.plot(MBrakeFR, label='MBrakeFR')
    plt.plot(MBrakeRL, label='MBrakeRL')
    plt.plot(MBrakeRR, label='MBrakeRR')
    plt.title(f'{base_name} Car Torque Inputs')
    plt.xlabel('Iteration')
    plt.ylabel('Torque')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_L_Car_Torques.png'))
    # with open(os.path.join(output_dir, f'{current_time}_{base_name}_L_Car_Torques.dill'), 'wb') as f:
    #     dill.dump(plt, f)
    # plt.show()

def plot_TargetValues(base_name, targetVelocityArray, longAccelerationArray, bPressure, current_Velocity, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(targetVelocityArray, label='Target Velocity (kph)')
    plt.plot(longAccelerationArray, label='Accelerator Pedal (%)')
    plt.plot(current_Velocity, label='Current Velocity (kph)')
    plt.plot(bPressure, label='Brake Pressure (bar)')
    plt.title(f'{base_name} Target Velocity Values')
    plt.xlabel('Iteration')
    plt.ylabel('Velocity and Accelerator Pedal')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_G_Target_Velocity_Values.png'))
    # with open(os.path.join(output_dir, f'{current_time}_{base_name}_G_Target_Velocity_Values.dill'), 'wb') as f:
    #     dill.dump(plt, f)
    # plt.show()

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
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_H_Pedal_Values.png'))
    # with open(os.path.join(output_dir, f'{current_time}_{base_name}_H_Pedal_Values.dill'), 'wb') as f:
    #     dill.dump(plt, f)
    # plt.show()

def plot_nMGU_Values(base_name, nMGU, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(nMGU, label='nMGU')
    plt.title(f'{base_name} nMGU over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('nMGU')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_M_nMGU.png'))
    # with open(os.path.join(output_dir, f'{current_time}_{base_name}_M_nMGU.dill'), 'wb') as f:
    #     dill.dump(plt, f)
    # plt.show()

def plot_Throttle_PID_Values(base_name, t_P, t_I, t_D, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(t_P, label='Throttle P')
    plt.plot(t_I, label='Throttle I')
    plt.plot(t_D, label='Throttle D')
    plt.title(f'{base_name} Throttle PID Values over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('PID Values')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_I_Throttle_PID_Values.png'))
    # with open(os.path.join(output_dir, f'{current_time}_{base_name}_I_Throttle_PID_Values.dill'), 'wb') as f:
    #     dill.dump(plt, f)
    # plt.show()

def plot_Brake_PID_Values(base_name, b_P, b_I, b_D, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(b_P, label='Brake P')
    plt.plot(b_I, label='Brake I')
    plt.plot(b_D, label='Brake D')
    plt.title(f'{base_name} Brake PID Values over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('PID Values')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_J_Brake_PID_Values.png'))
    # with open(os.path.join(output_dir, f'{current_time}_{base_name}_J_Brake_PID_Values.dill'), 'wb') as f:
    #     dill.dump(plt, f)
    # plt.show()

def plot_vCar_error_Values(base_name, vCar_error, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(vCar_error, label='vCar_error')
    plt.title(f'{base_name} vCar_error over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('vCar_error (kph)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_K_vCar_error.png'))
    # with open(os.path.join(output_dir, f'{current_time}_{base_name}_K_vCar_error.dill'), 'wb') as f:
    #     dill.dump(plt, f)
    # plt.show()

def plot_refDistances(base_name, refDistances, current_time):
    plt.figure(figsize=(15, 10))
    plt.plot(refDistances, label='refDistances')
    plt.title(f'{base_name} refDistances over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('refDistances (m)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_D_refDistances.png'))
    # with open(os.path.join(output_dir, f'{current_time}_{base_name}_D_refDistances.dill'), 'wb') as f:
    #     dill.dump(plt, f)
    # plt.show()

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
    plt.savefig(os.path.join(output_dir, f'{current_time}_{base_name}_N_Steer_PID_Values.png'))
