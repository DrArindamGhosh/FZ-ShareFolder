from steer_control_v3_4_Shanghai import calculate_steering_control
from speed_control_v2_3_Shanghai import calculateTorqueControl
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
import socket
import time
import struct

def adjust_numbers(last_number, new_number):
    
    yawAngle = new_number
    print(f"Last number: {last_number}, New number: {new_number}")
    
    if new_number - last_number > 1:

        yawAngle = (new_number - np.pi)
        print(f"{yawAngle} = {new_number} - {np.pi}") 
    
    if last_number - new_number < -5:

        # yawAngle = ((np.pi - new_number) * -1)
        # print(f"{yawAngle} = {np.pi} - {new_number}")
        yawAngle = (new_number - 2*np.pi)
        print(f"{yawAngle} = {new_number} - {2*np.pi}")

    if last_number - new_number > 5:

        yawAngle = (new_number + 2*np.pi)
        print(f"{yawAngle} = {new_number} + {2*np.pi}")

    return yawAngle

def main():

    iterations = 100000

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

    # Define the server address and port
    SERVER_IP = '127.0.0.1'
    SERVER_PORT = 6014


 
    for _ in range(iterations):

        data_to_send = struct.pack('fffffff', steering_input, 0, 0, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR)
        # data_to_send = struct.pack('fffffff', 0, 0, 0, 0, 0, 0, 0)

        # Create a socket object
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Connect to the server
        client_socket.connect((SERVER_IP, SERVER_PORT))

        

        # print(f"Connected to server at {SERVER_IP}:{SERVER_PORT}")
        # Send the data
        client_socket.sendall(data_to_send)
        try:
            data, addr = client_socket.recvfrom(1024)  # Buffer size is 1024 bytes
            received_length = len(data)
            expected_length = 16 * 4  # 16 floats, each 4 bytes
            
            print(f"Received data length: {received_length} bytes")
            print(f"Expected data length: {expected_length} bytes")

            if received_length == expected_length:
                num_floats = 16  # The number of floats in the byte stream (same as in the server)
                float_data = struct.unpack('f' * num_floats, data[:num_floats * 4])  # Unpack as 4-byte floats
                
                # Assign the received float data to variables
                (x, y, z, Vx, Vy, Vz, yawAngle, frontLHSWheelSlipRatio, frontRHSWheelSlipRatio, 
                rearLHSWheelSlipRatio, rearRHSWheelSlipRatio, frontLHSWheelVelocity, frontRHSWheelVelocity, 
                rearLHSWheelVelocity, rearRHSWheelVelocity, simTime) = float_data

                # print("Received float data:")
                print(f"x = {x}")
                print(f"y = {y}")
                print(f"z = {z}")
                print(f"Vx = {Vx}")
                print(f"Vy = {Vy}")
                print(f"Vz = {Vz}")
                print(f"yawAngle = {yawAngle}")
                print(f"frontLHSWheelSlipRatio = {frontLHSWheelSlipRatio}")
                print(f"frontRHSWheelSlipRatio = {frontRHSWheelSlipRatio}")
                print(f"rearLHSWheelSlipRatio = {rearLHSWheelSlipRatio}")
                print(f"rearRHSWheelSlipRatio = {rearRHSWheelSlipRatio}")
                print(f"frontLHSWheelVelocity = {frontLHSWheelVelocity}")
                print(f"frontRHSWheelVelocity = {frontRHSWheelVelocity}")
                print(f"rearLHSWheelVelocity = {rearLHSWheelVelocity}")
                print(f"rearRHSWheelVelocity = {rearRHSWheelVelocity}")
                print(f"simTime = {simTime}")
            else:
                print("Error: Received data length does not match the expected length.")
        except socket.timeout:
            print("No response from server")

        client_socket.close()
                #print("Connection closed")
        
        print(f"Iteration {_+1}:") 

        x_positions.append(x)
        y_positions.append(y)


        yawAngle = yawAngle + np.pi

        if yawAngles:
            last_yawAngle = yawAngles[-1]
        else:
            last_yawAngle = yawAngle
        
        # yawAngle = adjust_yawAngles(last_yawAngle, yawAngle)
        yawAngle = adjust_numbers(last_yawAngle, yawAngle)
        yawAngles.append(yawAngle)

        # yawAngles.append(yawAngle)

        # adjust_numbers(yawAngles)

        Vx = abs(Vx)
        Vy = abs(Vy)

        velocity_X.append(Vx)
        velocity_Y.append(Vy)

        ###########################################################################################################################################################
        # Steering Control
        steering_inputs.append(steering_input)
        steering_input, e_phi, saturated_e_l, levercoordsplot, idealcoordsplot, error_values, refLine_orientation, refDistance = calculate_steering_control(Vx, Vy, yawAngle, x, y)
        if abs(e_phi) > 5:
            print(f"Breaking loop at iteration {_+1} due to Angle Error > 5")
            break
        # if refDistance > 2290:
        #     print(f"Breaking loop at iteration {_+1} due to Complete Lap")
        #     break
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
        # if previous_targetVelocity is not None and abs(targetVelocity - previous_targetVelocity) > 20:
        #     print(f"Breaking loop at iteration {_+1} due to Target Velocity difference > 20")
        #     break
        
        previous_targetVelocity = targetVelocity
        acc_array.append(longAcceleration)
        slipThrust_array.append(slipThrust)
        oldThrust_array.append(oldLongThrust)

       

        ###########################################################################################################################################################

            
    plotTraj(x_positions, y_positions, levercoordsplot, idealcoordsplot, error_values)


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