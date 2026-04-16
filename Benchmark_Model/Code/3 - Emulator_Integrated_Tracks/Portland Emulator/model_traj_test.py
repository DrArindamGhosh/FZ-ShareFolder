import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import numpy as np
import pandas as pd


# file_path = 'MEX_driving_line.xlsx'
# data = pd.read_excel(file_path)
# data.head(), data.columns

# file_path = 'Berlin_2023_Traj_Mat.xlsx'
# data = pd.read_excel(file_path)
# data.head(), data.columns

# file_path = 'Shanghai_Traj_Mat.xlsx'
# data = pd.read_excel(file_path)
# data.head(), data.columns

file_path = 'Portland_Traj_Mat.xlsx'
data = pd.read_excel(file_path)
data.head(), data.columns

# file_path = 'Portland_Traj_Mat_Emulator.xlsx'
# data = pd.read_excel(file_path)
# data.head(), data.columns

def plotTraj(sim_x, sim_y, lever_coordinates, ideal_coordinates, e_l_values):
    plt.figure(figsize=(10, 6))
    plt.plot(data['x'], data['y'], label='Ideal Path', marker='o', linestyle='-', markersize=2)
    plt.title('Plot of X, Y Coordinates')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.grid(True)
    # plt.plot(x, y, label='Intended Trajectory')

    # If you can calculate actual positions from your model
    plt.plot(sim_x, sim_y, label='Actual Path', linestyle='--')

    plt.xlabel('X Position')
    plt.ylabel('Y Position')

    # Plotting racing car image
    # car_image = plt.imread("JTRT_Gen3_RacingCar_image.png")
    # trans = mtransforms.Affine2D().rotate_around(sim_x[len(sim_x)-1] + 0.5, sim_y[len(sim_y)-1] + 0.5, -yaw_angle) + plt.gca().transData
    # plt.imshow(car_image, extent=[sim_x[len(sim_x)-1], sim_x[len(sim_x)-1] + 1, sim_y[len(sim_y)-1], sim_y[len(sim_y)-1] + 1], transform=trans, aspect='auto')

    # plt.imshow(car_image, extent=[sim_x[999], sim_x[999] + 1, sim_y[999], sim_y[999] + 1])


    for i, (lever_coords, ideal_coords) in enumerate(zip(lever_coordinates, ideal_coordinates)):
        x_values = lever_coords[:, 0]
        y_values = lever_coords[:, 1]
        plt.plot(x_values, y_values, marker='x', linestyle='--', label='Lever Positions' if i == 0 else "")
        x_ideal = ideal_coords[:, 0]
        y_ideal = ideal_coords[:, 1]
        plt.plot(x_ideal, y_ideal, marker='o', linestyle='--', label='Ideal Points Corresponding to Lever Positions' if i == 0 else "")

        for j, ((x_lever, y_lever), (x_ideal, y_ideal)) in enumerate(zip(zip(x_values, y_values), zip(x_ideal, y_ideal))):
            plt.plot([x_lever, x_ideal], [y_lever, y_ideal], 'k-', linewidth=0.5)  # Draw the connecting line
            midpoint = ((x_lever + x_ideal) / 2, (y_lever + y_ideal) / 2)
            
            # Calculate the distance
            distance = np.sqrt((x_lever - x_ideal)**2 + (y_lever - y_ideal)**2)
            plt.annotate(f"Dist = {distance:.2f}", midpoint, textcoords="offset points", xytext=(-15, -15), ha='center')

            # Annotate the error value
            e_l_value = e_l_values[j]  # Access the specific e_l value for this pair
            plt.annotate(f"Err = {e_l_value:.2f}", midpoint, textcoords="offset points", xytext=(-15, 15), ha='center')

    
    plt.xlabel('X Position')
    plt.ylabel('Y Position')


    plt.title('Vehicle Trajectory')
    plt.legend()
    plt.show()


def plot_e_phi_Values(e_phi_values):
    # Plotting e_phi
    plt.figure(figsize=(15, 10))
    # plt.subplot(1, 2, 1)
    plt.plot(e_phi_values, label='e_phi')
    plt.title('Angular Offset over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('Error (e_phi)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


def plot_e_l_Values(saturated_e_l_values):  
    plt.figure(figsize=(15, 10))  

    # Plotting saturated_e_l
    for i in range(8):
        plt.subplot(8, 1, i + 1)
        plt.plot(saturated_e_l_values[i], label=f'Saturated Preview Inputs[{i}]')
        plt.title(f'Saturated Preview Inputs[{i}] over Iterations')
        plt.xlabel('Iteration over SimTime')
        plt.ylabel(f'Lateral Offset')
        plt.legend()
        plt.grid(True)


    plt.tight_layout()
    plt.show()

def plot_balance_Values(steering_input, line_orientation):

    plt.figure(figsize=(15, 10))
    # plt.plot(x, label='X (m)')
    # plt.plot(y, label='Y (m)')
    # plt.plot(Vx, label='Velocity X (kph)')
    # plt.plot(Vy, label='Velocity Y (kph)')
    plt.plot(steering_input, label='Steering Input')
    plt.plot(line_orientation, label='Orientation of Ref Line (Radians)')
    # plt.plot(steering_input, label='Steering Input Value')
    

    plt.title('Steering Input against RefLine Orientation')
    plt.xlabel('Iteration')
    plt.ylabel('Steering Input against RefLine Orientation')
    plt.legend()
    plt.grid(True)    

    plt.tight_layout()
    plt.show()


def plot_Angles(yaw_angle, line_orientation):

    plt.figure(figsize=(15, 10))
    plt.plot(yaw_angle, label='Orientation of Vehicle (Radians)')
    plt.plot(line_orientation, label='Orientation of Ref Line (Radians)')
    # plt.plot(steering_input, label='Steering Input Value')
    

    plt.title('Car and Trajectory Orientation over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('Yaw Angle')
    plt.legend()
    plt.grid(True)    

    plt.tight_layout()
    plt.show()


def plot_Torque(MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR):

    plt.figure(figsize=(15, 10))
    plt.plot(MDriveRL, label='MDriveRL')
    plt.plot(MDriveRR, label='MDriveRR')
    plt.plot(MBrakeFL, label='MBrakeFL')
    plt.plot(MBrakeFR, label='MBrakeFR')
    plt.plot(MBrakeRL, label='MBrakeRL')
    plt.plot(MBrakeRR, label='MBrakeRR')


    plt.title('Car Torque Inputs')
    plt.xlabel('Iteration')
    plt.ylabel('Torque')
    plt.legend()
    plt.grid(True)    

    plt.tight_layout()
    plt.show()

def plot_TargetValues(targetVelocityArray, longAccelerationArray, current_Velocity):

    plt.figure(figsize=(15, 10))
    plt.plot(targetVelocityArray, label='Target Velocity (mps)')
    plt.plot(longAccelerationArray, label='Longitudinal Acceleration (m/s**2)')
    plt.plot(current_Velocity, label='Current Velocity (mps)')

    plt.title('Target Values')
    plt.xlabel('Iteration')
    plt.ylabel('Velocity and Acceleration')
    plt.legend()
    plt.grid(True)    

    plt.tight_layout()
    plt.show()

def plot_longThrustValues(longThrust, slipThrust):

    plt.figure(figsize=(15, 10))
    plt.plot(longThrust, label='Long Thrust')
    plt.plot(slipThrust, label='Slip Thrust')

    plt.title('Thrust Change After Slip')
    plt.xlabel('Iteration')
    plt.ylabel('Thrust (N)')
    plt.legend()
    plt.grid(True)    

    plt.tight_layout()
    plt.show()


def plot_AntiLockSpin(slip_values, antiLock_outputs, antiSpinAntiLock_outputs):
    plt.figure(figsize=(10, 8))


    # Plot for Anti-Lock function
    plt.subplot(2, 1, 1)
    plt.plot(slip_values * 100, antiLock_outputs, label='Anti-lock Function')
    plt.title('Anti-lock Function for Front Wheels')
    plt.xlabel('Longitudinal Slip (%)')
    plt.ylabel('Output')
    plt.grid(True)

    # Plot for Anti-Spin/Anti-Lock function
    plt.subplot(2, 1, 2)
    plt.plot(slip_values * 100, antiSpinAntiLock_outputs, label='Anti-spin/Anti-lock Function')
    plt.title('Anti-spin/Anti-lock Function for Rear Wheels')
    plt.xlabel('Longitudinal Slip (%)')
    plt.ylabel('Output')
    plt.grid(True)

    plt.tight_layout()
    plt.show()