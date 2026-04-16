# Base correct validated Console App

import tkinter as tk
from tkinter import filedialog
import pandas as pd
import threading
import socket
import struct
import time
import os
import logging
import csv
from steer_control_Dev import calculate_steering_control
from speed_control_Dev import calculateTorqueControl
#from model_traj_test import plotTraj, plot_e_phi_Values, plot_e_l_Values, plot_balance_Values, plot_Angles, plot_Torque, plot_TargetValues
from model_traj_test_v1 import plotTraj, plot_e_phi_Values, plot_e_l_Values, plot_balance_Values, plot_Angles, plot_Torque, plot_TargetValues
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

logging.basicConfig(level=logging.DEBUG, filename='simulation.log', filemode='w',
                    format='%(asctime)s - %(levelname)s - %(message)s')


def adjust_numbers(last_number, new_number, lapCompleted):
    yawAngle = new_number
    if lapCompleted == True:
        yawAngle = new_number + np.pi
        print(f"Yaw Angle after Lap Completion: {yawAngle} = {new_number} + {np.pi} #########################################")
        lapCompleted = False
    else:
        if new_number - last_number > 1:
            yawAngle = new_number - np.pi
        if last_number - new_number < -5:
            yawAngle = new_number - 2 * np.pi
        if last_number - new_number > 5:
            yawAngle = new_number + 2 * np.pi
    return yawAngle, lapCompleted



def generate_driving_line(tracks_df, x, y, yawAngle):
    
    tracks_df['Orientation'] = tracks_df['Orientation'] - tracks_df.at[0, 'Orientation'] + yawAngle
    # tracks_df['Orientation2'] = tracks_df['Orientation2'] - tracks_df.at[0, 'Orientation2'] + tracks_df['Orientation'].dropna().iloc[-1]
    # tracks_df['Orientation2'] = tracks_df['Orientation2'] - tracks_df.at[0, 'Orientation2'] + yawAngle 

    # Set the initial coordinates
    tracks_df.at[0, 'x'] = x
    tracks_df.at[0, 'y'] = y

    # Calculate the x and y coordinates
    for i in range(1, len(tracks_df)):
        delta_distance = tracks_df.at[i, 'sDistance'] - tracks_df.at[i-1, 'sDistance']
        tracks_df.at[i, 'x'] = tracks_df.at[i-1, 'x'] + delta_distance * np.cos(tracks_df.at[i, 'Orientation'])
        tracks_df.at[i, 'y'] = tracks_df.at[i-1, 'y'] + delta_distance * np.sin(tracks_df.at[i, 'Orientation'])

    # # tracks_df.at[0, 'x2'] = tracks_df['x'].dropna().iloc[-1]
    # # tracks_df.at[0, 'y2'] = tracks_df['y'].dropna().iloc[-1]

    # # for i in range(1, len(tracks_df)):
    # #     delta_distance = tracks_df.at[i, 'sDistance2'] - tracks_df.at[i-1, 'sDistance2']
    # #     tracks_df.at[i, 'x2'] = tracks_df.at[i-1, 'x2'] + delta_distance * np.cos(tracks_df.at[i, 'Orientation2'])
    # #     tracks_df.at[i, 'y2'] = tracks_df.at[i-1, 'y2'] + delta_distance * np.sin(tracks_df.at[i, 'Orientation2'])

    tracks_df = tracks_df.loc[:, ~tracks_df.columns.str.contains('^Unnamed')]
    tracks_df.head()

    return tracks_df


def save_results_to_csv(data, filename='Model Applicaiton Emulator Output.csv'):
    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        if file.tell() == 0:
            writer.writerow([
                'x', 'y', 'z', 'Vx', 'Vy', 'Vz', 'yawAngle',
                'frontLHSWheelSlipRatio', 'frontRHSWheelSlipRatio',
                'rearLHSWheelSlipRatio', 'rearRHSWheelSlipRatio',
                'frontLHSWheelVelocity', 'frontRHSWheelVelocity',
                'rearLHSWheelVelocity', 'rearRHSWheelVelocity', 'simTime'
            ])
        writer.writerow(data)


class ConsoleApp:
    def __init__(self, root):
        self.root = root
        self.root.title("User Interface (UI) for Emulator")

        self.tracks_file = ""
        self.velocity_profile_file = ""
        self.abort_flag = threading.Event()
        self.reset_flag = threading.Event()
        self.simulation_thread = None
        self.server_thread = None
        self.reset_listener_thread = None
        self.figures = []
        self.server_socket = None
        self.client_address = None

        self.create_widgets()
        self.update_connection_status("Disconnected")

        self.start_reset_listener()

    def create_widgets(self):
        self.create_ip_port_input()
        self.create_control_buttons()
        self.create_connection_status()
        self.create_socket_output()
        self.create_parameter_buttons()
        self.create_console_output()
        self.create_refresh_button()
        self.create_abort_button()
        self.create_save_graphs_button()  # New Save All Graphs button
        self.create_show_graphs_button()  # New Show Graphs button
        self.create_reset_button()  # New reset button

    def create_ip_port_input(self):
        tk.Label(self.root, text="IP-Address:").grid(row=0, column=0)
        self.ip_entry = tk.Entry(self.root)
        self.ip_entry.grid(row=0, column=1)
        self.ip_entry.insert(0, "127.0.0.1")

        tk.Label(self.root, text="Port:").grid(row=1, column=0)
        self.port_entry = tk.Entry(self.root)
        self.port_entry.grid(row=1, column=1)
        self.port_entry.insert(0, "6020")

    def create_connection_status(self):
        self.connection_status = tk.Label(self.root, text="Status: Disconnected", fg="red")
        self.connection_status.grid(row=2, column=0, columnspan=2, sticky="w")

    def create_control_buttons(self):
        tk.Button(self.root, text="Start Server & Listen for Incoming Connections", bg="light yellow", command=self.start_server).grid(row=3, column=1, columnspan=1, pady=10)
        tk.Button(self.root, text="START SIM", bg="green", command=self.start_simulation).grid(row=4, column=0, pady=10)
        tk.Button(self.root, text="STOP SIM", bg="red", command=self.stop_simulation).grid(row=4, column=2, pady=10)
         # Train and Play Mode Buttons
        tk.Button(self.root, text="Train Model", bg="blue", command=self.train_model).grid(row=5, column=0, pady=10)
        tk.Button(self.root, text="Play Model", bg="purple", command=self.play_model).grid(row=5, column=2, pady=10)

    def create_socket_output(self):
        num_variables = 10
        height = num_variables + 2
        width = 50
        self.socket_output = tk.Text(self.root, height=height, width=width, bg="black", fg="green",
                                     highlightbackground="gray", highlightcolor="black", highlightthickness=1)
        self.socket_output.grid(row=5, column=0, columnspan=3, pady=10)

    def create_parameter_buttons(self):
        self.param_frame = tk.Frame(self.root, bd=2, relief=tk.SUNKEN)
        self.param_frame.grid(row=6, column=0, columnspan=2, pady=10, sticky="ew")

        tk.Label(self.param_frame, text="Parameter Selection Option:", anchor="center").grid(row=0, column=0, columnspan=4, sticky="ew")

        self.tracks_var = tk.StringVar(self.param_frame)
        self.tracks_var.set("Select Tracks")
        tk.Label(self.param_frame, text="Select Track and Velocity Profile:").grid(row=1, column=0, sticky="w")
        self.tracks_dropdown = tk.OptionMenu(self.param_frame, self.tracks_var, *self.get_combined_files(),
                                             command=self.set_files)
        self.tracks_dropdown.grid(row=1, column=1, columnspan=3, sticky="ew")

        tk.Label(self.param_frame, text="Max Engine Brake Torque:").grid(row=2, column=0, sticky="w")
        self.max_engine_brake_torque_entry = tk.Entry(self.param_frame)
        self.max_engine_brake_torque_entry.grid(row=2, column=1, sticky="ew")
        self.max_engine_brake_torque_entry.insert(0, "1.0")

        tk.Label(self.param_frame, text="Differential Gain 1:").grid(row=3, column=0, sticky="w")
        self.differential_gain_1_entry = tk.Entry(self.param_frame)
        self.differential_gain_1_entry.grid(row=3, column=1, sticky="ew")
        self.differential_gain_1_entry.insert(0, "0.1")

        tk.Label(self.param_frame, text="Driving Differential Gain:").grid(row=4, column=0, sticky="w")
        self.driving_differential_gain_entry = tk.Entry(self.param_frame)
        self.driving_differential_gain_entry.grid(row=4, column=1, sticky="ew")
        self.driving_differential_gain_entry.insert(0, "0.1")

        tk.Label(self.param_frame, text="Overrun Differential Gain:").grid(row=5, column=0, sticky="w")
        self.overrun_differential_gain_entry = tk.Entry(self.param_frame)
        self.overrun_differential_gain_entry.grid(row=5, column=1, sticky="ew")
        self.overrun_differential_gain_entry.insert(0, "0.1")

        tk.Label(self.param_frame, text="Front Wheel Coefficient:").grid(row=2, column=2, sticky="w")
        self.front_wheel_coefficient_entry = tk.Entry(self.param_frame)
        self.front_wheel_coefficient_entry.grid(row=2, column=3, sticky="ew")
        self.front_wheel_coefficient_entry.insert(0, "0.5")

        tk.Label(self.param_frame, text="Rear Wheel Coefficient:").grid(row=3, column=2, sticky="w")
        self.rear_wheel_coefficient_entry = tk.Entry(self.param_frame)
        self.rear_wheel_coefficient_entry.grid(row=3, column=3, sticky="ew")
        self.rear_wheel_coefficient_entry.insert(0, "0.5")

        tk.Label(self.param_frame, text="Max Drive Torque:").grid(row=4, column=2, sticky="w")
        self.max_drive_torque_entry = tk.Entry(self.param_frame)
        self.max_drive_torque_entry.grid(row=4, column=3, sticky="ew")
        self.max_drive_torque_entry.insert(0, "4000")

        tk.Label(self.param_frame, text="Max Brake Torque:").grid(row=5, column=2, sticky="w")
        self.max_brake_torque_entry = tk.Entry(self.param_frame)
        self.max_brake_torque_entry.grid(row=5, column=3, sticky="ew")
        self.max_brake_torque_entry.insert(0, "4000")

        tk.Label(self.param_frame, text="Velocity Percentage:").grid(row=6, column=0, sticky="w")
        self.num_Velocity_var = tk.StringVar(self.param_frame)
        self.num_Velocity_var.set("70")
        self.num_Velocity_dropdown = tk.OptionMenu(self.param_frame, self.num_Velocity_var, *[str(i) for i in range(10, 101, 10)])
        self.num_Velocity_dropdown.grid(row=6, column=1, sticky="ew")

        tk.Label(self.param_frame, text="Angular Offset Gain:").grid(row=7, column=0, sticky="w")
        self.e_phi_gain_entry = tk.Entry(self.param_frame)
        self.e_phi_gain_entry.grid(row=7, column=1, sticky="ew")
        self.e_phi_gain_entry.insert(0, "2.0")

        self.unlimited_laps_var = tk.IntVar()
        self.unlimited_laps_checkbox = tk.Checkbutton(self.param_frame, text="Unlimited Laps", variable=self.unlimited_laps_var)
        self.unlimited_laps_checkbox.grid(row=8, column=0, sticky="w")

        self.num_laps_label = tk.Label(self.param_frame, text="Number of Laps:")
        self.num_laps_label.grid(row=8, column=2, sticky="w")

        self.num_laps_entry = tk.Entry(self.param_frame, state="normal")
        self.num_laps_entry.grid(row=8, column=3, sticky="ew")

        self.unlimited_laps_var.trace_add("write", self.toggle_num_laps_entry)

    def toggle_num_laps_entry(self, *args):
        if self.unlimited_laps_var.get() == 1:
            self.num_laps_entry.config(state="disabled")
        else:
            self.num_laps_entry.config(state="normal")

    def create_console_output(self):
        num_variables = 16
        height = num_variables + 2
        width = 50
        self.console_output = tk.Text(self.root, height=height, width=width, bg="black", fg="white",
                                      highlightbackground="black", highlightcolor="black", highlightthickness=1)
        self.console_output.grid(row=7, column=0, columnspan=3, pady=10)

    def create_refresh_button(self):
        tk.Label(self.root, text="Refresh Rate (ms):").grid(row=8, column=0)
        self.refresh_rate_entry = tk.Entry(self.root)
        self.refresh_rate_entry.grid(row=8, column=1, padx=(0, 10))
        self.refresh_rate_entry.insert(0, "2500")

        self.refresh_button = tk.Button(self.root, text="Refresh", command=self.refresh_console_output)
        self.refresh_button.grid(row=8, column=2, pady=10)

    def create_abort_button(self):
        self.abort_button = tk.Button(self.root, text="ABORT", bg="orange", command=self.abort_simulation)
        self.abort_button.grid(row=8, column=3, pady=10)

    def create_save_graphs_button(self):
        self.save_graphs_button = tk.Button(self.root, text="Save All Graphs", command=self.save_all_graphs)
        self.save_graphs_button.grid(row=9, column=0, pady=10)

    def create_show_graphs_button(self):
        self.show_graphs_button = tk.Button(self.root, text="Show Graphs", command=self.show_graphs_from_folder)
        self.show_graphs_button.grid(row=9, column=1, pady=10)

    def create_reset_button(self):
        self.reset_button = tk.Button(self.root, text="RESET SIM", bg="light blue", command=self.reset_simulation_button)
        self.reset_button.grid(row=4, column=1, pady=10)

    def train_model(self):
        self.console_output.insert(tk.END, "Starting training mode...\n")
        self.train_mode = True
        self.run_backend_script()

    def play_model(self):
        self.console_output.insert(tk.END, "Starting play mode...\n")
        self.train_mode = False
        self.run_backend_script()

    def send_data(self):
        ip = self.ip_entry.get()
        port = int(self.port_entry.get())
        self.console_output.insert(tk.END, f"Sending data to {ip}:{port}\n")

    def start_server(self):
        if self.server_thread is None or not self.server_thread.is_alive():
            self.server_thread = threading.Thread(target=self.run_server)
            self.server_thread.start()

    def run_server(self):
        try:
            SERVER_IP = self.ip_entry.get()
            SERVER_PORT = int(self.port_entry.get())

            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.server_socket.bind((SERVER_IP, SERVER_PORT))
            self.update_connection_status("Listening")
            self.socket_output.insert(tk.END, "Server started and listening for incoming connections...\n")
            self.socket_output.see(tk.END)

            # For UDP, we do not have a dedicated accept function, so we wait for a message from the client
            while not self.abort_flag.is_set():
                try:
                    data, self.client_address = self.server_socket.recvfrom(1024)
                    if data:
                        self.socket_output.insert(tk.END, f"Client connected from {self.client_address}\n")
                        self.socket_output.see(tk.END)
                        self.update_connection_status("Connected")
                        break
                except socket.timeout:
                    continue
        except Exception as e:
            self.socket_output.insert(tk.END, f"Error starting server: {e}\n")
            self.socket_output.see(tk.END)

    # def run_server(self):
    #     try:
    #         SERVER_IP = self.ip_entry.get()
    #         SERVER_PORT = int(self.port_entry.get())

    #         self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #         self.server_socket.bind((SERVER_IP, SERVER_PORT))
    #         self.update_connection_status("Listening")
    #         self.socket_output.insert(tk.END, "Server started and listening for incoming connections...\n")
    #         self.socket_output.see(tk.END)

    #         # For UDP, we do not have a dedicated accept function, so we wait for a message from the client
    #         while not self.abort_flag.is_set():
    #             try:
    #                 ready_signal, self.client_address = self.server_socket.recvfrom(1024)
    #                 if ready_signal == struct.pack('B', 1):
    #                     self.socket_output.insert(tk.END, f"Client connected from {self.client_address}\n")
    #                     self.socket_output.see(tk.END)
    #                     self.update_connection_status("Connected")
    #                     break
    #             except socket.timeout:
    #                 continue
    #     except Exception as e:
    #         self.socket_output.insert(tk.END, f"Error starting server: {e}\n")
    #         self.socket_output.see(tk.END)

    def start_simulation(self):
        if not self.client_address:
            self.console_output.insert(tk.END, "No client connected. Start the server first.\n")
            return

        self.console_output.insert(tk.END, "Start SIM pressed\n")
        if self.simulation_thread is None or not self.simulation_thread.is_alive():
            self.simulation_thread = threading.Thread(target=self.run_backend_script)
            self.simulation_thread.start()

    def stop_simulation(self):
        self.console_output.insert(tk.END, "Stop SIM pressed\n")
        self.abort_flag.set()
        if self.simulation_thread is not None:
            self.simulation_thread.join()
            self.simulation_thread = None

    def display_position(self):
        self.console_output.insert(tk.END, "Displaying current position\n")

    def get_combined_files(self):
        combined_files = []
        try:
            traj_files = [file for file in os.listdir('Old_Trajectories') if file.endswith('.xlsx') or file.endswith('.xls')]
            vel_files = [file for file in os.listdir('New_Velocities') if file.endswith('.xlsx') or file.endswith('.xls')]
            combined_files = [f"{traj_file} | {vel_file}" for traj_file, vel_file in zip(traj_files, vel_files)]
        except FileNotFoundError as e:
            self.console_output.insert(tk.END, f"Error: {e}\n")
        return combined_files

    def set_files(self, selected_files):
        traj_file, vel_file = selected_files.split(' | ')
        self.tracks_file = os.path.join('Old_Trajectories', traj_file)
        self.velocity_profile_file = os.path.join('New_Velocities', vel_file)
        self.console_output.insert(tk.END, f"Selected Tracks File: {self.tracks_file}\n")
        self.console_output.insert(tk.END, f"Selected Velocity Profile File: {self.velocity_profile_file}\n")

    def refresh_console_output(self):
        self.console_output.delete(1.0, tk.END)
        refresh_rate = int(self.refresh_rate_entry.get())
        self.root.after(refresh_rate, self.refresh_console_output)

    def read_excel(self, file_path):
        try:
            df = pd.read_excel(file_path)
            self.console_output.insert(tk.END, f"Excel Content:\n{df.head()}\n")
        except Exception as e:
            self.console_output.insert(tk.END, f"Error reading Excel file: {e}\n")

    def validate_excel_files(self):
        self.console_output.insert(tk.END, "Validating selected files...\n")
        if not self.tracks_file or not self.velocity_profile_file:
            self.console_output.insert(tk.END, "Error: Please select both a tracks file and a velocity profile file.\n")
            return False
        try:
            pd.read_excel(self.tracks_file)
            pd.read_excel(self.velocity_profile_file)
        except Exception as e:
            self.console_output.insert(tk.END, f"Error reading selected files: {e}\n")
            return False
        self.console_output.insert(tk.END, "Files validated successfully.\n")
        return True

    def update_connection_status(self, status):
        if status == "Connected":
            self.connection_status.config(text="Status: Connected", fg="green")
        elif status == "Listening":
            self.connection_status.config(text="Status: Listening", fg="blue")
        else:
            self.connection_status.config(text="Status: Disconnected", fg="red")

    def abort_simulation(self):
        self.abort_flag.set()
        self.update_connection_status("Disconnected")

    def reset_simulation(self):
        self.reset_flag.set()
        self.console_output.insert(tk.END, "Simulation reset initiated.\n")

    def reset_simulation_button(self):
        self.console_output.insert(tk.END, "Reset SIM pressed\n")
        self.reset_flag.set()

    def start_reset_listener(self):
        def listen_for_reset():
            reset_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            reset_socket.bind(("0.0.0.0", 6015))  # Listening on port 6015 for reset messages
            while True:
                data, addr = reset_socket.recvfrom(1)
                if data == b'\x00':  # Single byte message with value zero
                    self.reset_simulation()

        self.reset_listener_thread = threading.Thread(target=listen_for_reset, daemon=True)
        self.reset_listener_thread.start()

    def run_backend_script(self):
        if self.train_mode:
            self.train_model_logic()
        else:
            self.play_model_logic()

    def train_model_logic(self):
        self.console_output.insert(tk.END, "Training the model...\n")

        # Import the necessary modules
        from stable_baselines3 import PPO
        from gym import Env
        from gym.spaces import Box
        import numpy as np
        import win32com.client
        from steer_control_Dev import calculate_steering_control
        from speed_control_Dev import calculateTorqueControl

        # Define a custom Gym-compatible environment using the provided calculateTorqueControl logic
        class BalanceEnv(Env):
            def __init__(self, tracks_df, velocity_profile_df, torque_control_params):
                super(BalanceEnv, self).__init__()

                tracks_df = pd.read_excel(self.tracks_file)
                velocity_profile_df = pd.read_excel(self.velocity_profile_file)
                max_engine_brake_torque = float(self.max_engine_brake_torque_entry.get())
                differential_gain_1 = float(self.differential_gain_1_entry.get())
                driving_differential_gain = float(self.driving_differential_gain_entry.get())
                overrun_differential_gain = float(self.overrun_differential_gain_entry.get())
                front_wheel_coefficient = float(self.front_wheel_coefficient_entry.get())
                rear_wheel_coefficient = float(self.rear_wheel_coefficient_entry.get())
                max_drive_torque = float(self.max_drive_torque_entry.get())
                max_brake_torque = float(self.max_brake_torque_entry.get())
                velocity_percentage = int(self.num_Velocity_var.get())
                e_phi_gain = float(self.e_phi_gain_entry.get())
                velocity_profile_df['vCar'] = velocity_profile_df['vCar'] * velocity_percentage / 100
                velocity_profile_df['vCar2'] = velocity_profile_df['vCar2'] * velocity_percentage / 100
                configured_laps = 0
                if self.unlimited_laps_var.get() == 0:
                    configured_laps = int(self.num_laps_entry.get())
                    self.console_output.insert(tk.END, f"Laps Initiated: {configured_laps}\n")
                else:
                    configured_laps = float('inf')
                    self.console_output.insert(tk.END, f"Laps Initiated: {configured_laps}\n")

                iterations = 180000
                nLap = 0
                lastNLap = 0
                LapCompleted = False
                nLaps = []
                lastRefDistance = 0

                # Initialize the variables here
                steering_input = 0
                MDriveRL = 0
                MDriveRR = 0
                MBrakeFL = 0
                MBrakeFR = 0
                MBrakeRL = 0
                MBrakeRR = 0
                simTime = 0


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
                vCarArray = []

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

                # Load track and velocity profiles
                self.tracks_df = tracks_df
                self.velocity_profile_df = velocity_profile_df

                # Set torque control parameters
                self.params = torque_control_params

                # Observation space: State variables
                self.observation_space = Box(low=-np.inf, high=np.inf, shape=(16,), dtype=np.float32)

                # Action space: Steering input [-1, 1]
                self.action_space = Box(low=-28.0, high=28.0, shape=(1,), dtype=np.float32)

                # Initialize state
                self.state = None
                self.time_step = 0
                self.done = False

            def reset(self):
                # Reset the environment and return the initial state
                self.state = [0.0] * 16  # Replace with appropriate initialization logic
                self.time_step = 0
                self.done = False
                return np.array(self.state, dtype=np.float32)

            def step(self, action):
                # Extract action (steering)
                steering_input = action[0]

                steering_inputs.append(steering_input)
                steering_input, e_phi, saturated_e_l, levercoordsplot, idealcoordsplot, error_values, refLine_orientation, refDistance = calculate_steering_control(
                    tracks_df, Vx, Vy, yawAngle, x, y, e_phi_gain, timeStep)

                # Speed Control
                MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, targetVelocity, longAcceleration, slipThrust, oldLongThrust, nLap = calculateTorqueControl(
                    tracks_df, velocity_profile_df, Vx, Vy, yawAngle, x, y, rearLHSWheelVelocity,
                    rearRHSWheelVelocity, frontLHSWheelSlipRatio, frontRHSWheelSlipRatio, rearLHSWheelSlipRatio, rearRHSWheelSlipRatio,
                    max_engine_brake_torque, differential_gain_1, driving_differential_gain, overrun_differential_gain, front_wheel_coefficient, 
                    rear_wheel_coefficient, max_drive_torque, max_brake_torque, timeStep, velocity_percentage, nLap
                )

                # Unpack parameters for calculateTorqueControl
                params = self.params
                torque_control_result = calculateTorqueControl(
                    self.tracks_df,
                    self.velocity_profile_df,
                    self.state[3],  # Vx
                    self.state[4],  # Vy
                    self.state[6],  # yawAngle
                    self.state[0],  # x
                    self.state[1],  # y
                    self.state[12],  # rearLHSWheelVelocity
                    self.state[13],  # rearRHSWheelVelocity
                    self.state[7],  # frontLHSWheelSlipRatio
                    self.state[8],  # frontRHSWheelSlipRatio
                    self.state[9],  # rearLHSWheelSlipRatio
                    self.state[10],  # rearRHSWheelSlipRatio
                    params['max_engine_brake_torque'],
                    params['differential_gain_1'],
                    params['driving_differential_gain'],
                    params['overrun_differential_gain'],
                    params['front_wheel_coefficient'],
                    params['rear_wheel_coefficient'],
                    params['max_drive_torque'],
                    params['max_brake_torque'],
                    params['time_step'],
                    params['velocity_percentage'],
                    params['nLap']
                )

                

                # Extract the next state, reward, and done flag
                MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, targetVelocity, longAcceleration, slipThrust, oldLongThrust, nLap = torque_control_result

                # Update state and compute reward
                self.state = self.update_state(steering_input, torque_control_result)

                reward = self.calculate_reward(self.state, targetVelocity)
                self.time_step += 1
                self.done = self.time_step >= 1000  # Example episode limit

                return np.array(self.state, dtype=np.float32), reward, self.done, {}

            def update_state(self, steering_input, torque_result):
                # Unpack the torque control results
                MDriveFL, MDriveFR, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR = (
                    0, 0, torque_result[0], torque_result[1], torque_result[2], torque_result[3]
                )

                try:
                    # Simulate vehicle dynamics using the Balance API
                    self.balance.DynamicsSimExtDrive2(
                        self.carHandle, steering_input, MDriveFL, MDriveFR, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, 0, 0
                    )

                    # Read state variables from Balance
                    x, y, z = self.balance.ReadChannelVector(self.carHandle, "Chassis", "", "Position")
                    yawAngle = self.balance.ReadChannelScalar(self.carHandle, "Chassis", "", "Vehicle Yaw")
                    Vx, Vy, Vz = self.balance.ReadChannelVector(self.carHandle, "Chassis", "", "Vehicle Velocity")
                    frontLHSWheelSlipRatio = self.balance.ReadChannelScalar(self.carHandle, "Tyre", "Front LHS", "Slip Ratio")
                    frontRHSWheelSlipRatio = self.balance.ReadChannelScalar(self.carHandle, "Tyre", "Front RHS", "Slip Ratio")
                    rearLHSWheelSlipRatio = self.balance.ReadChannelScalar(self.carHandle, "Tyre", "Rear LHS", "Slip Ratio")
                    rearRHSWheelSlipRatio = self.balance.ReadChannelScalar(self.carHandle, "Tyre", "Rear RHS", "Slip Ratio")
                    frontLHSWheelVelocity = self.balance.ReadChannelScalar(self.carHandle, "Tyre", "Front LHS", "Omega")
                    frontRHSWheelVelocity = self.balance.ReadChannelScalar(self.carHandle, "Tyre", "Front RHS", "Omega")
                    rearLHSWheelVelocity = self.balance.ReadChannelScalar(self.carHandle, "Tyre", "Rear LHS", "Omega")
                    rearRHSWheelVelocity = self.balance.ReadChannelScalar(self.carHandle, "Tyre", "Rear RHS", "Omega")
                    simTime = self.balance.ReadChannelScalar(self.carHandle, "Chassis", "", "Sim Time")

                    # Construct the next state
                    next_state = [
                        x, y, z,  # Position
                        Vx, Vy, Vz,  # Velocities
                        yawAngle,  # Vehicle yaw
                        frontLHSWheelSlipRatio, frontRHSWheelSlipRatio, rearLHSWheelSlipRatio, rearRHSWheelSlipRatio,  # Slip Ratios
                        frontLHSWheelVelocity, frontRHSWheelVelocity, rearLHSWheelVelocity, rearRHSWheelVelocity,  # Wheel Velocities
                        simTime,  # Simulation time
                    ]
                except Exception as e:
                    # Handle potential Balance API errors
                    self.console_output.insert(tk.END, f"Error updating state: {e}\n")
                    next_state = [0.0] * 16  # Reset to a default state in case of errors

                return next_state

            def calculate_reward(self, state, target_velocity):
                # Reward based on trajectory adherence and speed optimization
                reward = -np.abs(state[3] - target_velocity)  # Example: Minimize velocity error
                return reward

            def render(self, mode='human'):
                pass

            def close(self):
                pass

        # Load track and velocity profiles
        tracks_df = pd.read_excel(self.tracks_file)
        velocity_profile_df = pd.read_excel(self.velocity_profile_file)

        # Torque control parameters from UI inputs
        torque_control_params = {
            'max_engine_brake_torque': float(self.max_engine_brake_torque_entry.get()),
            'differential_gain_1': float(self.differential_gain_1_entry.get()),
            'driving_differential_gain': float(self.driving_differential_gain_entry.get()),
            'overrun_differential_gain': float(self.overrun_differential_gain_entry.get()),
            'front_wheel_coefficient': float(self.front_wheel_coefficient_entry.get()),
            'rear_wheel_coefficient': float(self.rear_wheel_coefficient_entry.get()),
            'max_drive_torque': float(self.max_drive_torque_entry.get()),
            'max_brake_torque': float(self.max_brake_torque_entry.get()),
            'velocity_percentage': int(self.num_Velocity_var.get()),
            'time_step': 0.002,  # Example time step
            'nLap': 0  # Lap counter
        }

        # Initialize the environment
        env = BalanceEnv(tracks_df, velocity_profile_df, torque_control_params)

        # Initialize RL model
        model = PPO("MlpPolicy", env, verbose=1)

        # Train the model
        model.learn(total_timesteps=10000)

        # Save the trained model
        model.save("trained_model.zip")

        self.console_output.insert(tk.END, "Model training complete and saved.\n")


    def play_model_logic(self):
        self.console_output.insert(tk.END, "Playing the model...\n")
        # Example play logic (replace with actual play implementation)
        model = PPO.load("trained_model.zip")
        env = None  # Initialize environment here
        obs = env.reset()
        done = False
        while not done:
            action, _ = model.predict(obs)
            obs, reward, done, info = env.step(action)
        self.console_output.insert(tk.END, "Model play complete.\n")

    def run_backend_script(self):
        def run_script():
            try:
                if not self.validate_excel_files():
                    return

                self.update_connection_status("Connected")
                tracks_df = pd.read_excel(self.tracks_file)
                velocity_profile_df = pd.read_excel(self.velocity_profile_file)
                max_engine_brake_torque = float(self.max_engine_brake_torque_entry.get())
                differential_gain_1 = float(self.differential_gain_1_entry.get())
                driving_differential_gain = float(self.driving_differential_gain_entry.get())
                overrun_differential_gain = float(self.overrun_differential_gain_entry.get())
                front_wheel_coefficient = float(self.front_wheel_coefficient_entry.get())
                rear_wheel_coefficient = float(self.rear_wheel_coefficient_entry.get())
                max_drive_torque = float(self.max_drive_torque_entry.get())
                max_brake_torque = float(self.max_brake_torque_entry.get())
                velocity_percentage = int(self.num_Velocity_var.get())
                e_phi_gain = float(self.e_phi_gain_entry.get())
                velocity_profile_df['vCar'] = velocity_profile_df['vCar'] * velocity_percentage / 100
                velocity_profile_df['vCar2'] = velocity_profile_df['vCar2'] * velocity_percentage / 100
                configured_laps = 0
                if self.unlimited_laps_var.get() == 0:
                    configured_laps = int(self.num_laps_entry.get())
                    self.console_output.insert(tk.END, f"Laps Initiated: {configured_laps}\n")
                else:
                    configured_laps = float('inf')
                    self.console_output.insert(tk.END, f"Laps Initiated: {configured_laps}\n")

                iterations = 180000
                nLap = 0
                lastNLap = 0
                LapCompleted = False
                nLaps = []
                lastRefDistance = 0

                # Initialize the variables here
                steering_input = 0
                MDriveRL = 0
                MDriveRR = 0
                MBrakeFL = 0
                MBrakeFR = 0
                MBrakeRL = 0
                MBrakeRR = 0
                simTime = 0


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
                vCarArray = []

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

                def reset_variables():
                    nonlocal steering_input, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR
                    nonlocal MDriveRL_array, MDriveRR_array, MBrakeFL_array, MBrakeFR_array, MBrakeRL_array, MBrakeRR_array
                    nonlocal tVelocity_array, acc_array, slipThrust_array, oldThrust_array
                    nonlocal x_positions, y_positions, yawAngles, velocity_X, velocity_Y
                    nonlocal e_phi_values, refLine_orientations, saturated_e_l_values
                    nonlocal lever_coordinates, ideal_coordinates, steering_inputs, refDistances, tVCs
                    nonlocal errors, previous_targetVelocity

                    steering_input = 0
                    MDriveRL = 0
                    MDriveRR = 0
                    MBrakeFL = 0
                    MBrakeFR = 0
                    MBrakeRL = 0
                    MBrakeRR = 0

                    MDriveRL_array.clear()
                    MDriveRR_array.clear()
                    MBrakeFL_array.clear()
                    MBrakeFR_array.clear()
                    MBrakeRL_array.clear()
                    MBrakeRR_array.clear()
                    tVelocity_array.clear()
                    acc_array.clear()
                    slipThrust_array.clear()
                    oldThrust_array.clear()

                    x_positions.clear()
                    y_positions.clear()
                    yawAngles.clear()
                    velocity_X.clear()
                    velocity_Y.clear()

                    e_phi_values.clear()
                    refLine_orientations.clear()
                    saturated_e_l_values = [[] for _ in range(8)]

                    lever_coordinates.clear()
                    ideal_coordinates.clear()

                    steering_inputs.clear()
                    refDistances.clear()
                    tVCs.clear()

                    errors = [[] for _ in range(8)]
                    previous_targetVelocity = None

                reset_variables()

                while not self.abort_flag.is_set():
                    self.console_output.insert(tk.END, "Waiting for a client to connect...\n")
                    self.console_output.see(tk.END)

                    try:
                        for i in range(iterations):
                            if self.abort_flag.is_set():
                                self.console_output.insert(tk.END, "Simulation aborted.\n")
                                break
                            if self.reset_flag.is_set():
                                self.console_output.insert(tk.END, "Resetting simulation...\n")
                                reset_variables()
                                self.reset_flag.clear()
                                self.console_output.insert(tk.END, "Simulation reset completed. Restarting iterations.\n")
                                break

                            self.console_output.insert(tk.END, f"Iteration {i + 1}\n")
                            self.console_output.see(tk.END)  # Ensure the console always scrolls to the end
                            # data_to_send = struct.pack('fffffff', steering_input, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR)
                            data_to_send = struct.pack('fffffff', steering_input, 0, 0, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR)

                            try:
                                self.server_socket.sendto(data_to_send, self.client_address)
                                data, _ = self.server_socket.recvfrom(1024)  # Buffer size is 1024 bytes
                                received_length = len(data)
                                expected_length = 16 * 4  # 16 floats, each 4 bytes

                                if received_length == expected_length:
                                    num_floats = 16  # The number of floats in the byte stream (same as in the server)
                                    float_data = struct.unpack('f' * num_floats, data[:num_floats * 4])  # Unpack as 4-byte floats

                                    (x, y, z, Vx, Vy, Vz, yawAngle, frontLHSWheelSlipRatio, frontRHSWheelSlipRatio,
                                    rearLHSWheelSlipRatio, rearRHSWheelSlipRatio, frontLHSWheelVelocity, frontRHSWheelVelocity,
                                    rearLHSWheelVelocity, rearRHSWheelVelocity, newSimTime) = float_data

                                    # save_results_to_csv(float_data)

                                    output_text = (
                                        f"Received data:\n"
                                        f"x = {x}\n"
                                        f"y = {y}\n"
                                        f"z = {z}\n"
                                        f"Vx = {Vx}\n"
                                        f"Vy = {Vy}\n"
                                        f"Vz = {Vz}\n"
                                        f"yawAngle = {yawAngle}\n"
                                    #     f"frontLHSWheelSlipRatio = {frontLHSWheelSlipRatio}\n"
                                    #     f"frontRHSWheelSlipRatio = {frontRHSWheelSlipRatio}\n"
                                    #     f"rearLHSWheelSlipRatio = {rearLHSWheelSlipRatio}\n"
                                    #     f"rearRHSWheelSlipRatio = {rearRHSWheelSlipRatio}\n"
                                    #     f"frontLHSWheelVelocity = {frontLHSWheelVelocity}\n"
                                    #     f"frontRHSWheelVelocity = {frontRHSWheelVelocity}\n"
                                    #     f"rearLHSWheelVelocity = {rearLHSWheelVelocity}\n"
                                    #     f"rearRHSWheelVelocity = {rearRHSWheelVelocity}\n"
                                        f"simTime = {simTime}\n\n"
                                    )
                                    self.console_output.insert(tk.END, output_text)
                                else:
                                    self.console_output.insert(tk.END, f"Received data length = {received_length} bytes\n")
                                    self.console_output.insert(tk.END, f"Expected data length = {expected_length} bytes\n")
                                    self.console_output.insert(tk.END, "Error: Received data length does not match the expected length. Breaking the loop due to potential data loss.\n")
                                    break  # Break the loop if data is lost or corrupted
                            except socket.timeout:
                                self.socket_output.insert(tk.END, "Socket timeout: No response from client\n")
                                self.socket_output.see(tk.END)
                            except Exception as e:
                                self.socket_output.insert(tk.END, f"Socket error: {e}\n")
                                self.socket_output.see(tk.END)

                            if i == 1:
                                tracks_df = generate_driving_line(tracks_df, x, y, yawAngle)
                                print(tracks_df.head())
                                print(tracks_df.tail(36))
                                velocity_profile_df = velocity_profile_df.loc[:, ~velocity_profile_df.columns.str.contains('^Unnamed')]
                                newSim = True

                            x = -x
                            y = -y

                            x_positions.append(x)
                            y_positions.append(y)

                            # yawAngle = yawAngle + np.pi

                            if yawAngles:
                                last_yawAngle = yawAngles[-1]
                            else:
                                last_yawAngle = yawAngle

                            self.console_output.insert(tk.END, f"Current Lap: {nLap}\n")
                            # if nLaps:
                            if lastNLap < nLap:
                                LapCompleted = True
                            lastNLap = nLap
                            # else:
                            #     lastNLap = nLap
                            
                            yawAngle, LapCompleted = adjust_numbers(last_yawAngle, yawAngle, LapCompleted)
                            yawAngles.append(yawAngle)

                            Vx = abs(Vx)
                            Vy = abs(Vy)

                            vCar = np.sqrt(Vx**2 + Vy**2)
                            vCarArray.append(vCar)

                            velocity_X.append(Vx)
                            velocity_Y.append(Vy)

                            timeStep = newSimTime - simTime
                            simTime = newSimTime

                            # Steering Control
                            steering_inputs.append(steering_input)
                            steering_input, e_phi, saturated_e_l, levercoordsplot, idealcoordsplot, error_values, refLine_orientation, refDistance = calculate_steering_control(
                                tracks_df, Vx, Vy, yawAngle, x, y, e_phi_gain, timeStep)
                            # if abs(e_phi) > 5:
                            #     print(f"Breaking loop at iteration {i+1} due to Angle Error > 5")
                            #     break
                            max_sdistance = np.max(tracks_df['sDistance'])
                            if refDistance > max_sdistance:
                                print(f"Breaking loop at iteration {i+1} due to Complete Lap")
                                break

                            print(f"Ref Distance: {refDistance}")
                            if lastRefDistance >= refDistance + 1000:
                                print("Lap Completed")
                                nLap += 1
                            lastRefDistance = refDistance
                            

                            e_phi_values.append(e_phi)
                            for j in range(8):
                                saturated_e_l_values[j].append(saturated_e_l[j])
                            lever_coordinates.append(levercoordsplot)
                            ideal_coordinates.append(idealcoordsplot)
                            refLine_orientations.append(refLine_orientation)

                            # Speed Control
                            MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, targetVelocity, longAcceleration, slipThrust, oldLongThrust, nLap = calculateTorqueControl(
                                tracks_df, velocity_profile_df, Vx, Vy, yawAngle, x, y, rearLHSWheelVelocity,
                                rearRHSWheelVelocity, frontLHSWheelSlipRatio, frontRHSWheelSlipRatio, rearLHSWheelSlipRatio, rearRHSWheelSlipRatio,
                                max_engine_brake_torque, differential_gain_1, driving_differential_gain, overrun_differential_gain, front_wheel_coefficient, 
                                rear_wheel_coefficient, max_drive_torque, max_brake_torque, timeStep, velocity_percentage, nLap
                            )

                            # Store the control results
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

                            output_text2 = (
                                f"Steering Input: {steering_input}\n"
                                f"MDriveRL: {MDriveRL}\n"
                                f"MDriveRR: {MDriveRR}\n"
                                f"MBrakeFL: {MBrakeFL}\n"
                                f"MBrakeFR: {MBrakeFR}\n\n"
                                # f"MBrakeRL: {MBrakeRL}\n"
                                # f"MBrakeRR: {MBrakeRR}\n"    
                            )

                            self.console_output.insert(tk.END, output_text2)

                        # Reset the iteration counter to restart the loop after resetting
                        if self.reset_flag.is_set():
                            continue
                        else:
                            break
                    finally:
                        self.socket_output.insert(tk.END, "Client connection closed.\n")
                        self.socket_output.see(tk.END)

                self.server_socket.close()

                # Plotting the results after the iterations
                def plot_and_store(figures, plot_func, *args):
                    try:
                        fig = plot_func(*args)
                        if fig is not None:
                            figures.append(fig)
                    except Exception as e:
                        self.console_output.insert(tk.END, f"Error plotting {plot_func.__name__}: {e}\n")

                if lever_coordinates and ideal_coordinates:
                    plot_and_store(self.figures, plotTraj, tracks_df, x_positions, y_positions, levercoordsplot, idealcoordsplot, error_values)
                plot_and_store(self.figures, plot_e_phi_Values, e_phi_values)
                plot_and_store(self.figures, plot_e_l_Values, saturated_e_l_values)
                plot_and_store(self.figures, plot_balance_Values, steering_inputs, refLine_orientations)
                plot_and_store(self.figures, plot_Angles, yawAngles, refLine_orientations)
                plot_and_store(self.figures, plot_Torque, MDriveRL_array, MDriveRR_array, MBrakeFL_array, MBrakeFR_array, MBrakeRL_array, MBrakeRR_array)
                plot_and_store(self.figures, plot_TargetValues, tVelocity_array, acc_array, velocity_X)

                self.console_output.insert(tk.END, "Figures saved locally.\n")

                self.display_graphs(self.figures)

                self.console_output.insert(tk.END, "Script finished execution.\n")
                self.update_connection_status("Disconnected")
            except Exception as e:
                self.console_output.insert(tk.END, f"Error running script: {e}\n")

        self.abort_flag.clear()
        self.simulation_thread = threading.Thread(target=run_script)
        self.simulation_thread.start()


    def save_all_graphs(self):
        if not self.figures:
            self.console_output.insert(tk.END, "No graphs available to save. Run the simulation first.\n")
            return

        save_dir = 'SimulationGraphs'
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        for i, fig in enumerate(self.figures, start=1):
            fig_path = os.path.join(save_dir, f'graph_{i}.png')
            fig.savefig(fig_path)
            self.console_output.insert(tk.END, f"Saved {fig_path}\n")

        self.console_output.insert(tk.END, "All graphs have been saved.\n")

    def show_graphs_from_folder(self):
        folder_path = 'SimulationGraphs'
        if not os.path.exists(folder_path):
            self.console_output.insert(tk.END, "No graphs available to display. Run the simulation and save graphs first.\n")
            return

        graphs_window = tk.Toplevel(self.root)
        graphs_window.title("Select Graph to Display")

        graph_files = [file for file in os.listdir(folder_path) if file.endswith('.png')]
        if not graph_files:
            self.console_output.insert(tk.END, "No graphs found in the folder.\n")
            return

        tk.Label(graphs_window, text="Select a graph to display:").pack()

        selected_graph = tk.StringVar(graphs_window)
        selected_graph.set(graph_files[0])
        graph_dropdown = tk.OptionMenu(graphs_window, selected_graph, *graph_files)
        graph_dropdown.pack()

        def display_selected_graph():
            graph_path = os.path.join(folder_path, selected_graph.get())
            graph_fig = plt.figure()
            img = plt.imread(graph_path)
            plt.imshow(img)
            plt.axis('off')
            graph_canvas = FigureCanvasTkAgg(graph_fig, master=graphs_window)
            graph_canvas.draw()
            graph_canvas.get_tk_widget().pack()

        display_button = tk.Button(graphs_window, text="Display Graph", command=display_selected_graph)
        display_button.pack()

    def display_graphs(self):
        if not self.figures:
            self.console_output.insert(tk.END, "No graphs available to display. Run the simulation first.\n")
            return

        graphs_window = tk.Toplevel(self.root)
        graphs_window.title("Simulation Graphs")

        for fig in self.figures:
            canvas = FigureCanvasTkAgg(fig, master=graphs_window)
            canvas.draw()
            canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

def run_app():
    root = tk.Tk()
    app = ConsoleApp(root)
    root.mainloop()


if __name__ == "__main__":
    run_app()  # Run the application