import win32com.client
from steer_control_v3_4 import calculate_steering_control
from model_traj_test import plotTraj
from model_traj_test import plot_e_phi_Values
from model_traj_test import plot_e_l_Values
from model_traj_test import plot_balance_Values
from model_traj_test import plot_Angles
import matplotlib.pyplot as plt
import numpy as np

def main():
    balance = win32com.client.Dispatch("{5C7A18B0-6AF5-456D-9CFB-F6F651FD31B5}")
 
    print("F9 requested")
 
    nFail = 0
    localDatabaseHandle = 0
    localSetupRecord = 0
 
    assert balance is not None, "Balance object is null"
    balance.compat_SetFailValue(-1)
 
    localDatabaseHandle = balance.OpenLocalDatabase()
    assert localDatabaseHandle != 0, "Failed to open local database"
 
    dbHandle = balance.OpenEDBConnection("HHDev", "HHDev", "https://hhdm-api.hh-dev.com")

    localSetupRecord = balance.OpenSetupWarehouse(dbHandle, "SetupNameIsID", "4889ecf8-0320-472c-a520-d9ab97f64cc2")
 
    carHandle = balance.OpenCarFromSetup(localSetupRecord)

    # Initial Velocity in x, y and z directions
    balance.DynamicsConvertCar_Vinit(carHandle, 0, 0, 0)
    # balance.DynamicsConvertCar_Vinit(carHandle, -100, 100, 0)
    steering_input = 0
    iterations = 90000

    x_positions = []
    y_positions = []
    yawAngles = []
    velocity_X = []
    velocity_Y = []

    e_phi_values = []
    refLine_orientations = []
    saturated_e_l_values = [[] for _ in range(8)]

    lever_coordinates = []
    ideal_coordinates = []

    steering_inputs = []

    errors = [[] for _ in range(8)]

 
    for _ in range(iterations):
        
        # DynamicsSimExtDrive2() simulates one timestep, where "carHandle" is the setup, the second parameter is steering input, the four "i"s are mechanical torque
        # to each of the Front Left, Front Right, Rear Left, Rear Right tyres, and the last four parameters represent braking torque for the Front Left, Front Right, Rear Left, Rear Right tyres
        balance.DynamicsSimExtDrive2(carHandle, steering_input, 0, 0, 100, 100, 0, 0, 0, 0)
        # balance.DynamicsSimExtDrive2(carHandle, steering_input, 0, 0, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR)
        

        simTime = balance.ReadChannelScalar(carHandle, "Chassis", "", "Sim Time")
        print(f"Time Elapsed: {simTime}\n")


        # Chassis Position

        x, y, z = balance.ReadChannelVector(carHandle, "Chassis", "", "Position")
        print(f"Iteration {_+1}:") 
        print(f"Positions: X Output = {x}")
        print(f"           Y Output = {y}\n")
        x_positions.append(x)
        y_positions.append(y)

        # Yaw Rates

        yawAngle = -balance.ReadChannelScalar(carHandle, "Chassis", "", "Vehicle Yaw")
        yawAngle = yawAngle + np.pi
        if yawAngle < np.pi:
            yawAngle = yawAngle + 2*np.pi
        # else:
        #     yawAngle =
            
        print(f"                 Yaw Angle = {yawAngle}\n")
        yawAngles.append(yawAngle)

        # Velocities
        Vx, Vy, Vz = balance.ReadChannelVector(carHandle, "Chassis", "", "Vehicle Velocity")

        # print(f"Velocities:      X Output = {Vx}")
        # print(f"                 Y Output = {Vy}\n")
        velocity_X.append(Vx)
        velocity_Y.append(Vy)

        # handWheel = balance.ReadChannelScalar(carHandle, "Steering Rack", "Front RHS", "RackPos")
        # handWheel = handWheel * 57
        # print(f"                 Steer Rack = {handWheel}\n")

        steering_inputs.append(steering_input)
        steering_input, e_phi, saturated_e_l, levercoordsplot, idealcoordsplot, error_values, refLine_orientation = calculate_steering_control(Vx, Vy, yawAngle, x, y)
        # steering_input = steering_input/9
        e_phi_values.append(e_phi)
        for i in range(8):
            saturated_e_l_values[i].append(saturated_e_l[i])
        lever_coordinates.append(levercoordsplot)
        ideal_coordinates.append(idealcoordsplot)
        refLine_orientations.append(refLine_orientation)
        
        # errors.append(error_values)
        # print(ideal_coordinates)
        # print(errors)
        
    # yawAngle = 4.7 # Manual input Yaw Angle: 4.7, however this should be coming from balance - from radians to 270 degrees
    plotTraj(x_positions, y_positions, levercoordsplot, idealcoordsplot, error_values)
    errors.clear()

    plot_e_phi_Values(e_phi_values)

    plot_e_l_Values(saturated_e_l_values)

    plot_balance_Values(x_positions, y_positions, velocity_X, velocity_Y, steering_inputs)

    plot_Angles(yawAngles, refLine_orientations)
 
if __name__ == "__main__":
    main()