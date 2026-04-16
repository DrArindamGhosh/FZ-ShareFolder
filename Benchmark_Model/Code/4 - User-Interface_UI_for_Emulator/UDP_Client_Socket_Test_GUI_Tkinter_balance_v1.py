import socket
import struct
import win32com.client
import time

def udp_client():
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

    SERVER_IP = "127.0.0.1"  # The server's hostname or IP address
    SERVER_PORT = 6020  # The port used by the server

    # Create a UDP socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # Send a "ready" signal (single byte) to the server
        print("Sending 'ready' signal to server")
        ready_signal = struct.pack('B', 1)  # 1 byte indicating ready
        client_socket.sendto(ready_signal, (SERVER_IP, SERVER_PORT))
        print("Sent 'ready' signal to server")




        # Receive response
        # print("Waiting for response from server...")
        while True:
            
            # Wait for server to acknowledge and start the simulation
            print("Waiting for server to start the simulation...")
            response_data, server_address = client_socket.recvfrom(1024)
            print(f"Received response from server: {response_data}")

            # Print the received data for debugging
            # print(f"Received raw data: {response_data}")
            # print(f"Length of received data: {len(response_data)} bytes")

            # Calculate the number of floats in the response
            received_length = len(response_data)
            num_floats = received_length // 4

            # Unpack the received data
            float_data = struct.unpack('f' * num_floats, response_data)
            # (steering_input, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR) = float_data
            (steering_input, MDriveFL, MDriveFR, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR) = float_data
            print("Received data from server:")
            for i, value in enumerate(float_data):
                print(f"Value {i+1}: {value}")

            # Send the data to the balance
            # balance.DynamicsSimExtDrive2(carHandle, steering_input, 0, 0, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR)
            balance.DynamicsSimExtDrive2(carHandle, steering_input, MDriveFL, MDriveFR, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, 0, 0)

            # Chassis Position
            x, y, z = balance.ReadChannelVector(carHandle, "Chassis", "", "Position")
            # if x == 0.0 and y == 0.0 and z == 0.0:
            #     print("Warning: Received all zero data, retrying...")
            #     continue

            # Yaw Rates
            yawAngle = balance.ReadChannelScalar(carHandle, "Chassis", "", "Vehicle Yaw")

            # Velocities
            Vx, Vy, Vz = balance.ReadChannelVector(carHandle, "Chassis", "", "Vehicle Velocity")

            # Slip Ratios
            frontLHSWheelSlipRatio = balance.ReadChannelScalar(carHandle, "Tyre", "Front LHS", "Slip Ratio")
            frontRHSWheelSlipRatio = balance.ReadChannelScalar(carHandle, "Tyre", "Front RHS", "Slip Ratio")
            rearLHSWheelSlipRatio = balance.ReadChannelScalar(carHandle, "Tyre", "Rear LHS", "Slip Ratio")
            rearRHSWheelSlipRatio = balance.ReadChannelScalar(carHandle, "Tyre", "Rear RHS", "Slip Ratio")

            # Wheel Velocities
            frontLHSWheelVelocity = balance.ReadChannelScalar(carHandle, "Tyre", "Front LHS", "Omega")
            frontRHSWheelVelocity = balance.ReadChannelScalar(carHandle, "Tyre", "Front RHS", "Omega")
            rearLHSWheelVelocity = balance.ReadChannelScalar(carHandle, "Tyre", "Rear LHS", "Omega")
            rearRHSWheelVelocity = balance.ReadChannelScalar(carHandle, "Tyre", "Rear RHS", "Omega")

            # Sim Time
            simTime = balance.ReadChannelScalar(carHandle, "Chassis", "", "Sim Time")

            # Send the data to the server
            print(f"Data sent:\nX: {x}\nY: {y}\nZ: {z}\nVx: {Vx}\nVy: {Vy}\nVz: {Vz}\nYaw Angle: {yawAngle}\n")
            # Front LHS Wheel Slip Ratio: {frontLHSWheelSlipRatio}\nFront RHS Wheel Slip Ratio: {frontRHSWheelSlipRatio}\nRear LHS Wheel Slip Ratio: {rearLHSWheelSlipRatio}\nRear RHS Wheel Slip Ratio: {rearRHSWheelSlipRatio}\nFront LHS Wheel Velocity: {frontLHSWheelVelocity}\nFront RHS Wheel Velocity: {frontRHSWheelVelocity}\nRear LHS Wheel Velocity: {rearLHSWheelVelocity}\nRear RHS Wheel Velocity: {rearRHSWheelVelocity}\nSim Time: {simTime}")
            print("Sending Data")
            # time.sleep(0.1)
            data_to_send = struct.pack('ffffffffffffffff', x, y, z, Vx, Vy, Vz, yawAngle, frontLHSWheelSlipRatio, frontRHSWheelSlipRatio, rearLHSWheelSlipRatio, 
                                       rearRHSWheelSlipRatio, frontLHSWheelVelocity, frontRHSWheelVelocity, rearLHSWheelVelocity, rearRHSWheelVelocity, simTime)
            client_socket.sendto(data_to_send, (SERVER_IP, SERVER_PORT))
            print("Data Sent")

    except KeyboardInterrupt:
        print("Client stopped.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        client_socket.close()

if __name__ == "__main__":
    udp_client()  # Run the UDP client