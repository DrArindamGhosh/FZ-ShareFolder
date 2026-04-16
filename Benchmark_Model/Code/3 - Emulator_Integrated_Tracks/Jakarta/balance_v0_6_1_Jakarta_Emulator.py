from steer_control_v3_4_Jakarta import calculate_steering_control
from speed_control_v2_3_Jakarta import calculateTorqueControl
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
import hashlib

def send_data_with_retries(client_socket, data_to_send, retries=3, delay=1):
    for attempt in range(retries):
        try:
            client_socket.sendall(data_to_send)
            return True
        except socket.error as e:
            print(f"Error sending data: {e}. Retrying in {delay} seconds...")
            time.sleep(delay)
    print("Failed to send data after multiple attempts.")
    return False

def receive_data_with_retries(client_socket, expected_length, retries=3, delay=1):
    for attempt in range(retries):
        data = b''  # Initialize an empty byte string to store the received data
        try:
            while len(data) < expected_length:
                chunk = client_socket.recv(expected_length - len(data))
                if not chunk:
                    print("Connection closed by the server.")
                    return None
                data += chunk  # Append the chunk to the data buffer
            
            if len(data) == expected_length:
                return data
            else:
                print(f"Incomplete data received. Expected {expected_length} bytes, got {len(data)} bytes.")
        except socket.error as e:
            print(f"Error receiving data: {e}. Retrying in {delay} seconds...")
            time.sleep(delay)
    
    print("Failed to receive data after multiple attempts.")
    return None

def calculate_hash(data):
    hasher = hashlib.sha256()
    hasher.update(data)
    return hasher.digest()  # Return the hash as a binary digest

def adjust_numbers(last_number, new_number):
    
    yawAngle = new_number
    print(f"Last number: {last_number}, New number: {new_number}")
    
    if new_number - last_number > 1:
        yawAngle = (new_number - np.pi)
        print(f"{yawAngle} = {new_number} - {np.pi}") 
    
    if last_number - new_number < -5:
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
        # Prepare data to send
        data_to_send = struct.pack('fffffff', steering_input, 0, 0, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR)
        data_hash = calculate_hash(data_to_send)
        data_to_send_with_hash = data_to_send + data_hash

        # Create a socket object
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.settimeout(5)  # Set a timeout for the connection

        try:
            # Connect to the server
            client_socket.connect((SERVER_IP, SERVER_PORT))
            print(f"Connected to server at {SERVER_IP}:{SERVER_PORT}")

            # Send the data with retries
            if not send_data_with_retries(client_socket, data_to_send_with_hash):
                print(f"Failed to send data on iteration {_+1}")
                break  # Exit if data fails to send after retries

            # Receive data with retries
            expected_length = 16 * 4  # 16 floats, each 4 bytes
            received_data = receive_data_with_retries(client_socket, expected_length)

            if received_data:
                received_bytes = received_data[:-len(data_hash)]  # Separate data from hash
                received_hash = received_data[-len(data_hash):]
                print(f"Received {len(received_bytes)} bytes of data and {len(received_hash)} bytes of hash")

                if calculate_hash(received_bytes) == received_hash:
                    print(f"Data integrity verified on iteration {_+1}")
                    # Process received data
                    float_data = struct.unpack('f' * 16, received_bytes)
                    
                    (x, y, z, Vx, Vy, Vz, yawAngle, frontLHSWheelSlipRatio, frontRHSWheelSlipRatio, 
                    rearLHSWheelSlipRatio, rearRHSWheelSlipRatio, frontLHSWheelVelocity, frontRHSWheelVelocity, 
                    rearLHSWheelVelocity, rearRHSWheelVelocity, simTime) = float_data
                    print(f"Received and unpacked data: x={x}, y={y}, Vx={Vx}, Vy={Vy}, yawAngle={yawAngle}")

                else:
                    print(f"Data integrity check failed on iteration {_+1}")
                    break  # Exit if data integrity fails
            else:
                print(f"Failed to receive data on iteration {_+1}")
                break  # Exit if data fails to receive after retries

        except socket.timeout:
            print(f"Timeout occurred on iteration {_+1}")
        except Exception as e:
            print(f"An unexpected error occurred on iteration {_+1}: {str(e)}")

        client_socket.close()
        print("Connection closed")
        
        print(f"Iteration {_+1}:") 

        x_positions.append(x)
        y_positions.append(y)

        yawAngle = yawAngle + np.pi

        if yawAngles:
            last_yawAngle = yawAngles[-1]
        else:
            last_yawAngle = yawAngle
        
        yawAngle = adjust_numbers(last_yawAngle, yawAngle)
        yawAngles.append(yawAngle)

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
        if refDistance > 2290:
            print(f"Breaking loop at iteration {_+1} due to Complete Lap")
            break
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
        
        previous_targetVelocity = targetVelocity
        acc_array.append(longAcceleration)
        slipThrust_array.append(slipThrust)
        oldThrust_array.append(oldLongThrust)

    plotTraj(x_positions, y_positions, levercoordsplot, idealcoordsplot, error_values)

    plot_e_phi_Values(e_phi_values)

    plot_e_l_Values(saturated_e_l_values)

    plot_balance_Values(steering_inputs, refLine_orientations)

    plot_Angles(yawAngles, refLine_orientations)

    plot_Torque(MDriveRL_array, MDriveRR_array, MBrakeFL_array, MBrakeFR_array, MBrakeRL_array, MBrakeRR_array)

    plot_TargetValues(tVelocity_array, acc_array, velocity_X)

if __name__ == "__main__":
    main()
