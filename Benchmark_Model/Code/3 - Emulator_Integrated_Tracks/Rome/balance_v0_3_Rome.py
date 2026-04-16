import win32com.client
from steer_control_v3_4_Rome import calculate_steering_control
from speed_control_v2_3_Rome import calculateTorqueControl
from model_traj_test import plotTraj
from model_traj_test import plot_e_phi_Values
from model_traj_test import plot_e_l_Values
from model_traj_test import plot_balance_Values
from model_traj_test import plot_Angles
from model_traj_test import plot_Torque
from model_traj_test import plot_TargetValues
from model_traj_test import plot_longThrustValues
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

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

    iterations = 1000000

    steering_input = 0
    MDriveRL = 0 
    MDriveRR = 0
    MBrakeFL = 0
    MBrakeFR = 0
    MBrakeRL = 0
    MBrakeRR = 0

    MDriveRL_array = []
    MDriveRR_array = []
    MBrakeFL_array = []
    MBrakeFR_array = []
    MBrakeRL_array = [] 
    MBrakeRR_array = []
    tVelocity_array = []
    acc_array = []
    slipThrust_array = []
    oldThrust_array = []

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
    refDistances = []
    tVCs = []

    errors = [[] for _ in range(8)]
    previous_targetVelocity = None
 
    for _ in range(iterations):
        
        # DynamicsSimExtDrive2() simulates one timestep, where "carHandle" is the setup, the second parameter is steering input, the four "i"s are mechanical torque
        # # to each of the Front Left, Front Right, Rear Left, Rear Right tyres, and the last four parameters represent braking torque for the Front Left, Front Right, Rear Left, Rear Right tyres
        # balance.DynamicsSimExtDrive2(carHandle, steering_input, 0, 0, 50, 50, 5, 5, 5, 5)
        balance.DynamicsSimExtDrive2(carHandle, steering_input, 0, 0, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR)
        

        simTime = balance.ReadChannelScalar(carHandle, "Chassis", "", "Sim Time")
        # print(f"Time Elapsed: {simTime}\n")


        # Chassis Position

        x, y, z = balance.ReadChannelVector(carHandle, "Chassis", "", "Position")
        print(f"Iteration {_+1}:") 
        print(f"Positions: X Output = {x}")
        print(f"           Y Output = {y}\n")
        x_positions.append(x)
        y_positions.append(y)

        # Yaw Rates

        yawAngle = balance.ReadChannelScalar(carHandle, "Chassis", "", "Vehicle Yaw")
        yawAngle = yawAngle + np.pi
        
        if yawAngle < np.pi/2:
            yawAngle = yawAngle + 2*np.pi
            print(f"                 Yaw Angle = {yawAngle}\n")       
        
        if yawAngles and yawAngles[-1] > 7:
            if yawAngle < 2*np.pi: 
                yawAngle = yawAngle + 2*np.pi
            print(f"                 Yaw Angle = {yawAngle}\n")

        # if yawAngle > 5:
        #     yawAngle = yawAngle - 2*np.pi

        # if yawAngles and yawAngles[-1] < -1:
        #     if yawAngle > -1: 
        #         yawAngle = yawAngle - 2*np.pi

        yawAngles.append(yawAngle)

        # Velocities
        Vx, Vy, Vz = balance.ReadChannelVector(carHandle, "Chassis", "", "Vehicle Velocity")

        Vx = abs(Vx)
        Vy = abs(Vy)
        print(f"Velocities:      X Output = {Vx}")
        print(f"                 Y Output = {Vy}\n")
        velocity_X.append(Vx)
        velocity_Y.append(Vy)

        # Wheel Velocities
        rearLHSWheelVelocity = balance.ReadChannelScalar(carHandle, "Tyre", "Rear LHS", "Omega")
        rearRHSWheelVelocity = balance.ReadChannelScalar(carHandle, "Tyre", "Rear RHS", "Omega")
        # print(f"                 Rear LHS Wheel Velocity = {rearLHSWheelVelocity}")
        # print(f"                 Rear RHS Wheel Velocity = {rearRHSWheelVelocity}\n")

        # Slip Ratios
        frontLHSWheelSlipRatio = balance.ReadChannelScalar(carHandle, "Tyre", "Front LHS", "Slip Ratio")
        frontRHSWheelSlipRatio = balance.ReadChannelScalar(carHandle, "Tyre", "Front RHS", "Slip Ratio")
        rearLHSWheelSlipRatio = balance.ReadChannelScalar(carHandle, "Tyre", "Rear LHS", "Slip Ratio")
        rearRHSWheelSlipRatio = balance.ReadChannelScalar(carHandle, "Tyre", "Rear RHS", "Slip Ratio")
        # print(f"                 Front LHS Wheel Slip Ratio = {frontLHSWheelSlipRatio}")
        # print(f"                 Front RHS Wheel Slip Ratio = {frontRHSWheelSlipRatio}")
        # print(f"                 Rear LHS Wheel Slip Ratio = {rearLHSWheelSlipRatio}")
        # print(f"                 Rear RHS Wheel Slip Ratio = {rearRHSWheelSlipRatio}\n")

        ###########################################################################################################################################################
        # Steering Control
        steering_inputs.append(steering_input)
        steering_input, e_phi, saturated_e_l, levercoordsplot, idealcoordsplot, error_values, refLine_orientation  = calculate_steering_control(Vx, Vy, yawAngle, x, y)
        

        
        e_phi_values.append(e_phi)
        for i in range(8):
            saturated_e_l_values[i].append(saturated_e_l[i])
        lever_coordinates.append(levercoordsplot)
        ideal_coordinates.append(idealcoordsplot)
        refLine_orientations.append(refLine_orientation)

        ###########################################################################################################################################################
        # Speed Control
        MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, targetVelocity, longAcceleration, slipThrust, oldLongThrust = calculateTorqueControl(Vx, 
                                                                                                                                                            Vy, 
                                                                                                                                                            yawAngle, 
                                                                                                                                                            x, 
                                                                                                                                                            y, 
                                                                                                                                                            rearLHSWheelVelocity, 
                                                                                                                                                            rearRHSWheelVelocity, 
                                                                                                                                                            frontLHSWheelSlipRatio, 
                                                                                                                                                            frontRHSWheelSlipRatio, 
                                                                                                                                                            rearLHSWheelSlipRatio,
                                                                                                                                                            rearRHSWheelSlipRatio,
                                                                                                                                                            steering_input)

        MDriveRL_array.append(MDriveRL)
        MDriveRR_array.append(MDriveRR)
        MBrakeFL_array.append(MBrakeFL)
        MBrakeFR_array.append(MBrakeFR)
        MBrakeRL_array.append(MBrakeRL)
        MBrakeRR_array.append(MBrakeRR)
        tVelocity_array.append(targetVelocity)
        if previous_targetVelocity is not None and abs(targetVelocity - previous_targetVelocity) > 20:
            print(f"Breaking loop at iteration {_+1} due to Target Velocity difference > 20")
            break
        
        previous_targetVelocity = targetVelocity
        acc_array.append(longAcceleration)
        slipThrust_array.append(slipThrust)
        oldThrust_array.append(oldLongThrust)

        ###########################################################################################################################################################

            
    plotTraj(x_positions, y_positions, levercoordsplot, idealcoordsplot, error_values)
    errors.clear()

    plot_e_phi_Values(e_phi_values)

    plot_e_l_Values(saturated_e_l_values)

    plot_balance_Values(steering_inputs, refLine_orientations)

    plot_Angles(yawAngles, refLine_orientations)

    # plot_corners(refLine_orientations, refDistances, tVCs)

    plot_Torque(MDriveRL_array, MDriveRR_array, MBrakeFL_array, MBrakeFR_array, MBrakeRL_array, MBrakeRR_array)

    plot_TargetValues(tVelocity_array, acc_array, velocity_X)

    # plot_longThrustValues(oldThrust_array, slipThrust_array)

if __name__ == "__main__":
    main()