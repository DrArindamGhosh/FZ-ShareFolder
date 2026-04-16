import socket
import struct
import win32com.client
import gym
import numpy as np
from stable_baselines3 import PPO

class BalanceEnv(gym.Env):
    def __init__(self, balance, carHandle, server_ip, server_port):
        super(BalanceEnv, self).__init__()
        self.balance = balance
        self.carHandle = carHandle
        self.server_ip = server_ip
        self.server_port = server_port
        
        # Define action and observation space
        self.action_space = gym.spaces.Box(low=-28, high=28, shape=(1,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(2,), dtype=np.float32)

        # UDP socket for communication with Model Application (server)
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def step(self, action):
        # Send steering action to Balance
        self.balance.DynamicsSimExtDrive2(self.carHandle, action[0], 0, 0, 0, 0, 0, 0, 0, 0)

        # Get the new state from Balance
        x, y, z = self.balance.ReadChannelVector(self.carHandle, "Chassis", "", "Position")
        yawAngle = self.balance.ReadChannelScalar(self.carHandle, "Chassis", "", "Vehicle Yaw")
        Vx, Vy, Vz = self.balance.ReadChannelVector(self.carHandle, "Chassis", "", "Vehicle Velocity")

        # Send state to Model Application (server)
        state_data = struct.pack('fffffff', x, y, z, Vx, Vy, Vz, yawAngle)
        self.client_socket.sendto(state_data, (self.server_ip, self.server_port))

        # Wait for the server (Model Application) to send angular and lateral offsets
        response_data, _ = self.client_socket.recvfrom(1024)
        angular_offset, lateral_offset = struct.unpack('ff', response_data)

        # Calculate reward using angular and lateral offsets
        reward = self.calculate_reward(angular_offset, lateral_offset)

        # Check terminal condition (optional, you can define this based on offsets)
        done = abs(angular_offset) > np.pi or abs(lateral_offset) > 10

        # Return the observation (angular and lateral offsets), reward, and done flag
        state = np.array([angular_offset, lateral_offset], dtype=np.float32)
        return state, reward, done, {}

    def reset(self):
        # Reset the simulation
        self.balance.DynamicsConvertCar_Vinit(self.carHandle, 0, 0, 0)
        return np.array([0.0, 0.0], dtype=np.float32)

    def calculate_reward(self, angular_offset, lateral_offset):
        # Reward function (penalizing large offsets)
        return -(angular_offset**2 + lateral_offset**2)

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

    SERVER_IP = "127.0.0.1"
    SERVER_PORT = 6020

    # Create the environment
    env = BalanceEnv(balance, carHandle, SERVER_IP, SERVER_PORT)

    # Load or Train the PPO model
    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=100000)  # Train the model

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        while True:
            # Reset environment and get initial observation
            obs = env.reset()

            # Get the action from the model
            action, _ = model.predict(obs)

            # Take a step with the PPO agent
            obs, reward, done, info = env.step(action)

            if done:
                break  # End episode if done is True
    
    except KeyboardInterrupt:
        print("Client stopped.")
    
    finally:
        client_socket.close()

if __name__ == "__main__":
    udp_client()
