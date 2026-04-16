# Adding more UI elements to the console app

import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
import pandas as pd
import threading
import socket
import struct
import time
import os
import logging
import csv
from steer_control_v14_4_Dev import calculate_steering_control
from speed_control_v14_4_1_1_Dev import calculateTorqueControl
from model_traj_test_v3 import plotTraj, plot_e_phi_Values, plot_e_l_Values, plot_balance_Values, plot_Angles, plot_Torque, plot_TargetValues
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import csv
from datetime import datetime
import tensorflow as tf
from keras import layers
from keras.optimizers import Adam
from keras.models import load_model

########################################### PPO Model ###########################################

class PPOAgent:
    def __init__(self, state_shape, action_space, learning_rate=0.0003):
        self.state_shape = state_shape
        self.action_space = action_space

        # Build the actor-critic model
        self.model = self.build_model()
        self.model.compile(optimizer=Adam(learning_rate=learning_rate))

    def build_model(self):
        # Actor-Critic Network
        state_input = layers.Input(shape=self.state_shape)
        common = layers.Dense(128, activation='relu')(state_input)
        common = layers.Dense(128, activation='relu')(common)

        # Actor output (for steering input)
        action_output = layers.Dense(self.action_space, activation='tanh')(common)  # steering angle between -1 and 1

        # Critic output (for value estimation)
        value_output = layers.Dense(1)(common)

        return tf.keras.Model(inputs=state_input, outputs=[action_output, value_output])

    def get_action(self, state):
        state = np.reshape(state, [1, *self.state_shape])
        action, value = self.model(state)
        return action[0], value[0]

    def train(self, states, actions, advantages, rewards, values, gamma=0.99, lambda_=0.95, epsilon=0.2, entropy_coef=0.01):
        # Training logic for PPO
        with tf.GradientTape() as tape:
            predicted_actions, predicted_values = self.model(states)

            # Compute the loss based on the PPO objective
            advantage = rewards + (gamma * values) - values
            actor_loss = self.ppo_loss(predicted_actions, actions, advantages, epsilon)
            critic_loss = tf.reduce_mean(tf.square(predicted_values - rewards))
            entropy_loss = -tf.reduce_mean(predicted_actions * tf.math.log(predicted_actions + 1e-10))

            loss = actor_loss + critic_loss + entropy_coef * entropy_loss

        gradients = tape.gradient(loss, self.model.trainable_variables)
        self.model.optimizer.apply_gradients(zip(gradients, self.model.trainable_variables))

    def ppo_loss(self, predicted_actions, actions, advantages, epsilon):
        ratio = tf.exp(tf.math.log(predicted_actions + 1e-10) - tf.math.log(actions + 1e-10))
        clipped_ratio = tf.clip_by_value(ratio, 1 - epsilon, 1 + epsilon)
        loss = -tf.reduce_mean(tf.minimum(ratio * advantages, clipped_ratio * advantages))
        return loss

    def load_model(self, model_path):
        self.model = load_model(model_path)

########################################### Training the PPO Model ###########################################

def train_model(client_socket, total_episodes=1000, max_steps_per_episode=1000):
    # Define the agent
    state_shape = (7,)  # Assuming the state is [x, y, z, yawAngle, Vx, Vy, Vz]
    action_space = 1     # Single continuous action for steering input (-28 to 28)
    agent = PPOAgent(state_shape, action_space)

    gamma = 0.99
    lambda_ = 0.95
    epsilon = 0.2
    entropy_coef = 0.01

    for episode in range(total_episodes):
        # Reset environment (start from initial position)
        done = False
        total_reward = 0
        step = 0

        # Get the initial state from the client
        state = reset_simulation(client_socket)  # Function to reset the simulation and get the initial state

        while not done and step < max_steps_per_episode:
            # Get action from agent
            action, value = agent.get_action(state)

            # Send the action (steering) to the client and receive the next state, reward, and done flag
            next_state, reward, done = self.step_simulation(client_socket, action)

            # Calculate advantage
            advantage = reward + gamma * value - value

            # Train the PPO agent
            agent.train(state, action, advantage, reward, value, gamma, lambda_, epsilon, entropy_coef)

            state = next_state
            total_reward += reward
            step += 1

        print(f"Episode {episode}/{total_episodes} - Total Reward: {total_reward}")

        # Save the model periodically
        if episode % 100 == 0:
            agent.model.save(f"ppo_steering_model_{episode}.h5")

    # Save the final model
    agent.model.save("ppo_steering_model_final.h5")

########################################### Playing the PPO Model ###########################################

def play_model(client_socket, model_path="ppo_steering_model_final.h5"):
    # Load the pre-trained model
    state_shape = (7,)
    action_space = 1
    agent = PPOAgent(state_shape, action_space)
    agent.load_model(model_path)

    # Reset simulation to initial state
    done = False
    state = reset_simulation(client_socket)  # Function to reset the simulation and get the initial state

    while not done:
        # Get action from the trained agent
        action, _ = agent.get_action(state)

        # Send the action to the client and receive the next state
        next_state, reward, done = step_simulation(client_socket, action)

        state = next_state

        time.sleep(0.002)

########################################### Helper Functions ###########################################

def reset_simulation(client_socket):
    # Send reset signal to client and receive initial state
    reset_signal = struct.pack('B', 1)  # Assuming reset is signaled by a 1-byte message
    client_socket.sendto(reset_signal, ("127.0.0.1", 6020))

    # Receive initial state (x, y, z, yawAngle, Vx, Vy, Vz)
    data, _ = client_socket.recvfrom(1024)
    state = struct.unpack('fffffff', data)  # Assuming state is sent as 7 floats
    return np.array(state)




########################################### Application ###########################################

logging.basicConfig(level=logging.DEBUG, filename='simulation.log', filemode='w',
                    format='%(asctime)s - %(levelname)s - %(message)s')

def adjust_numbers(last_number, new_number, lapCompleted, nLap):
    yawAngle = new_number
    if lapCompleted == True:
        if nLap == 1:
            yawAngle = new_number - np.pi
            lapCompleted = False
        if nLap > 1:
            yawAngle = new_number + np.pi
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
    tracks_df['Orientation2'] = tracks_df['Orientation2'] - tracks_df.at[0, 'Orientation2'] + tracks_df['Orientation'].dropna().iloc[-1]
    # tracks_df['Orientation2'] = tracks_df['Orientation2'] - tracks_df.at[0, 'Orientation2'] + yawAngle 

    # Set the initial coordinates
    tracks_df.at[0, 'x'] = x
    tracks_df.at[0, 'y'] = y

    # Calculate the x and y coordinates
    for i in range(1, len(tracks_df)):
        delta_distance = tracks_df.at[i, 'sDistance'] - tracks_df.at[i-1, 'sDistance']
        tracks_df.at[i, 'x'] = tracks_df.at[i-1, 'x'] + delta_distance * np.cos(tracks_df.at[i, 'Orientation'])
        tracks_df.at[i, 'y'] = tracks_df.at[i-1, 'y'] + delta_distance * np.sin(tracks_df.at[i, 'Orientation'])

    tracks_df.at[0, 'x2'] = tracks_df['x'].dropna().iloc[-1]
    tracks_df.at[0, 'y2'] = tracks_df['y'].dropna().iloc[-1]

    for i in range(1, len(tracks_df)):
        delta_distance = tracks_df.at[i, 'sDistance2'] - tracks_df.at[i-1, 'sDistance2']
        tracks_df.at[i, 'x2'] = tracks_df.at[i-1, 'x2'] + delta_distance * np.cos(tracks_df.at[i, 'Orientation2'])
        tracks_df.at[i, 'y2'] = tracks_df.at[i-1, 'y2'] + delta_distance * np.sin(tracks_df.at[i, 'Orientation2'])

    tracks_df = tracks_df.loc[:, ~tracks_df.columns.str.contains('^Unnamed')]
    tracks_df.head()

    return tracks_df

# Background Task Class
class BackgroundTask():
    def __init__(self, task_func):
        self._task_func = task_func
        self._worker_thread = None
        self._is_running = False

    def is_running(self):
        return self._is_running and self._worker_thread.is_alive()

    def start(self):
        if not self._is_running:
            self._is_running = True
            self._worker_thread = self.WorkerThread(self._task_func, self)
            self._worker_thread.start()

    def stop(self):
        self._is_running = False

    class WorkerThread(threading.Thread):
        def __init__(self, task_func, bg_task):
            threading.Thread.__init__(self)
            self._task_func = task_func
            self._bg_task = bg_task

        def run(self):
            try:
                self._task_func()
            except Exception as e:
                print(e)
            self._bg_task.stop()


# Repeating Task Class for UI updates
class TkRepeatingTask():
    def __init__(self, tk_root, task_func, frequency_ms):
        self.__tk = tk_root
        self.__func = task_func
        self.__freq = frequency_ms
        self.__is_running = False

    def is_running(self):
        return self.__is_running

    def start(self):
        self.__is_running = True
        self.__on_timer()

    def stop(self):
        self.__is_running = False

    def __on_timer(self):
        if self.__is_running:
            self.__func()
            self.__tk.after(self.__freq, self.__on_timer)


class ConsoleApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ExtDriver Console App")

        # Make the root window resizable
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        # self.root.resizable(False, False)

        # Create a canvas and attach scrollbars
        self.canvas = tk.Canvas(self.root, width=900, height=1000)
        self.canvas.grid(row=0, column=0, sticky="nsew")

        self.scrollbar_y = tk.Scrollbar(self.root, orient="vertical", command=self.canvas.yview)
        self.scrollbar_y.grid(row=0, column=1, sticky="ns")
        self.scrollbar_x = tk.Scrollbar(self.root, orient="horizontal", command=self.canvas.xview)
        self.scrollbar_x.grid(row=1, column=0, sticky="ew")

        self.canvas.configure(yscrollcommand=self.scrollbar_y.set, xscrollcommand=self.scrollbar_x.set)

        # Create a frame inside the canvas that will hold the widgets
        self.scrollable_frame = ttk.Frame(self.canvas)
        self.scrollable_frame.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))

        # Create a window in the canvas to embed the scrollable frame
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")

        self.tracks_file = ""
        self.velocity_profile_file = ""
        self.abort_flag = threading.Event()
        self.reset_flag = threading.Event()
        self.skip_flag = threading.Event()
        self.figures = []
        self.server_socket = None
        self.client_address = None
        

        # Initialize BackgroundTask and TkRepeatingTask for various tasks
        self.simulation_task = BackgroundTask(self.run_backend_script)
        self.reset_task = BackgroundTask(self.reset_simulation_task)
        self.server_task = BackgroundTask(self.run_server)
        self.refresh_task = TkRepeatingTask(self.scrollable_frame, self._refresh_console, 2500)

        self.create_widgets()
        self.update_connection_status("Disconnected")

        self.start_reset_listener()

    def create_widgets(self):
        self.create_ip_port_input()
        self.create_connection_status()
        self.create_control_buttons()
        self.create_parameter_buttons()
        self.create_socket_output()
        self.create_force_input_buttons()
        self.create_steer_control_parameters()
        self.create_torque_control_parameters()
        self.create_console_output()
        self.create_bottom_buttons()

    def create_ip_port_input(self):
        ip_port_frame = tk.Frame(self.scrollable_frame)
        ip_port_frame.grid(row=0, column=0, columnspan=4, sticky="ew")

        tk.Label(ip_port_frame, text="IP:").grid(row=0, column=0, padx=5)
        self.ip_entry = tk.Entry(ip_port_frame, width=15)
        self.ip_entry.grid(row=0, column=1, padx=5)
        self.ip_entry.insert(0, "127.0.0.1")

        tk.Label(ip_port_frame, text="Port:").grid(row=0, column=2, padx=5)
        self.port_entry = tk.Entry(ip_port_frame, width=10)
        self.port_entry.grid(row=0, column=3, padx=5)
        self.port_entry.insert(0, "6020")

    def create_connection_status(self):
        self.connection_status = tk.Label(self.scrollable_frame, text="Status: Disconnected", fg="red")
        self.connection_status.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(5, 5))

    def create_control_buttons(self):
        control_button_frame = tk.Frame(self.scrollable_frame)
        control_button_frame.grid(row=2, column=0, columnspan=4, sticky="ew", pady=5)

        tk.Button(control_button_frame, text="Start Server & Listen for Incoming Connections", bg="light yellow", command=self.start_server).grid(row=0, column=0, padx=5, pady=5)
        tk.Button(control_button_frame, text="START SIM", bg="green", command=self.start_simulation).grid(row=0, column=1, padx=5, pady=5)
        tk.Button(control_button_frame, text="RESET SIM", bg="light blue", command=self.reset_simulation_button).grid(row=0, column=2, padx=5, pady=5)
        tk.Button(control_button_frame, text="STOP SIM", bg="red", command=self.stop_simulation).grid(row=0, column=3, padx=5, pady=5)

    def create_socket_output(self):
        self.socket_output = tk.Text(self.scrollable_frame, height=12, width=50, bg="black", fg="green")
        self.socket_output.grid(row=3, column=0, columnspan=4, pady=5)

    def create_parameter_buttons(self):
        self.param_frame = tk.Frame(self.scrollable_frame, bd=2, relief=tk.SUNKEN)
        self.param_frame.grid(row=4, column=0, columnspan=4, sticky="ew", pady=5)

        tk.Label(self.param_frame, text="Parameter Selection:").grid(row=0, column=0, columnspan=4, sticky="ew", pady=5)

        tk.Label(self.param_frame, text="Select Track and Velocity Profile:").grid(row=1, column=0, sticky="w", padx=5)
        self.tracks_var = tk.StringVar(self.param_frame)
        self.tracks_var.set("Select Tracks")
        self.tracks_dropdown = tk.OptionMenu(self.param_frame, self.tracks_var, *self.get_combined_files(), command=self.set_files)
        self.tracks_dropdown.grid(row=1, column=1, columnspan=3, sticky="ew")

        tk.Label(self.param_frame, text="Velocity Percentage:").grid(row=2, column=0, sticky="w", padx=5)
        self.num_Velocity_var = tk.StringVar(self.param_frame)
        self.num_Velocity_var.set("70")
        self.num_Velocity_dropdown = tk.OptionMenu(self.param_frame, self.num_Velocity_var, *[str(i) for i in range(10, 101, 10)])
        self.num_Velocity_dropdown.grid(row=2, column=1, padx=5, pady=5)

        tk.Label(self.param_frame, text="Logging Frequency:").grid(row=3, column=0, sticky="w", padx=5)
        self.log_frequency_var = tk.StringVar(self.param_frame)
        self.log_frequency_var.set("1")
        self.log_frequency_dropdown = tk.OptionMenu(self.param_frame, self.log_frequency_var, *[str(i) for i in range(0, 51, 5)])
        self.log_frequency_dropdown.grid(row=3, column=1, padx=5, pady=5)

        tk.Button(self.param_frame, text="Train", command=self.set_train_mode).grid(row=5, column=0, padx=5, pady=5)
        tk.Button(self.param_frame, text="Play", command=self.set_play_mode).grid(row=5, column=1, padx=5, pady=5)

        self.unlimited_laps_var = tk.IntVar()
        self.unlimited_laps_checkbox = tk.Checkbutton(self.param_frame, text="Unlimited Laps", variable=self.unlimited_laps_var)
        self.unlimited_laps_checkbox.grid(row=4, column=0, sticky="w", padx=5)

        tk.Label(self.param_frame, text="Number of Laps:").grid(row=4, column=1, sticky="w", padx=5)
        self.num_laps_entry = tk.Entry(self.param_frame, width=10, state="normal")
        self.num_laps_entry.grid(row=4, column=2, padx=5)
        self.num_laps_entry.insert(0, "1")
        self.unlimited_laps_var.trace_add("write", self.toggle_num_laps_entry)

    def set_train_mode(self):
        self.train_mode = True
        self._append_to_console_output_safe("Train mode activated.\n")

    def set_play_mode(self):
        self.train_mode = False
        self._append_to_console_output_safe("Play mode activated.\n")

    def toggle_num_laps_entry(self, *args):
        if self.unlimited_laps_var.get() == 1:
            self.num_laps_entry.config(state="disabled")
        else:
            self.num_laps_entry.config(state="normal")

    def create_force_input_buttons(self):
        self.force_input_frame = tk.Frame(self.scrollable_frame, bd=2, relief=tk.SUNKEN)
        self.force_input_frame.grid(row=4, column=2, columnspan=4, sticky="ew", pady=5)

        tk.Label(self.force_input_frame, text="Force Driver Inputs:").grid(row=0, column=0, columnspan=4, sticky="ew")

        tk.Label(self.force_input_frame, text="Steering Angle:").grid(row=1, column=0, sticky="w")
        self.steering_angle_entry = tk.Entry(self.force_input_frame, width=10)
        self.steering_angle_entry.grid(row=1, column=1, padx=5)
        self.steering_angle_entry.insert(0, "0")

        tk.Label(self.force_input_frame, text="MDriveFL:").grid(row=2, column=0, sticky="w")
        self.MDriveFL_entry = tk.Entry(self.force_input_frame, width=10)
        self.MDriveFL_entry.grid(row=2, column=1, padx=5)
        self.MDriveFL_entry.insert(0, "0")

        tk.Label(self.force_input_frame, text="MDriveFR:").grid(row=2, column=2, sticky="w")
        self.MDriveFR_entry = tk.Entry(self.force_input_frame, width=10)
        self.MDriveFR_entry.grid(row=2, column=3, padx=5)
        self.MDriveFR_entry.insert(0, "0")

        tk.Label(self.force_input_frame, text="MDriveRL:").grid(row=3, column=0, sticky="w")
        self.MDriveRL_entry = tk.Entry(self.force_input_frame, width=10)
        self.MDriveRL_entry.grid(row=3, column=1, padx=5)
        self.MDriveRL_entry.insert(0, "0")

        tk.Label(self.force_input_frame, text="MDriveRR:").grid(row=3, column=2, sticky="w")
        self.MDriveRR_entry = tk.Entry(self.force_input_frame, width=10)
        self.MDriveRR_entry.grid(row=3, column=3, padx=5)
        self.MDriveRR_entry.insert(0, "0")

        tk.Label(self.force_input_frame, text="MBrakeFL:").grid(row=4, column=0, sticky="w")
        self.MBrakeFL_entry = tk.Entry(self.force_input_frame, width=10)
        self.MBrakeFL_entry.grid(row=4, column=1, padx=5)
        self.MBrakeFL_entry.insert(0, "0")

        tk.Label(self.force_input_frame, text="MBrakeFR:").grid(row=4, column=2, sticky="w")
        self.MBrakeFR_entry = tk.Entry(self.force_input_frame, width=10)
        self.MBrakeFR_entry.grid(row=4, column=3, padx=5)
        self.MBrakeFR_entry.insert(0, "0")

        tk.Button(self.force_input_frame, text="FORCE", command=self.on_force_clicked).grid(row=5, column=0, columnspan=2, pady=5)
        tk.Button(self.force_input_frame, text="DISABLE", command=self.on_disable_clicked).grid(row=5, column=2, columnspan=2, pady=5)

        self.disable_var = tk.IntVar()
        self.disable_checkbox = tk.Checkbutton(self.force_input_frame, text="Disable Slip Controls", variable=self.disable_var)
        self.disable_checkbox.grid(row=6, column=0, sticky="w")

    def create_steer_control_parameters(self):
        self.steer_control_frame = tk.Frame(self.scrollable_frame, bd=2, relief=tk.SUNKEN)
        self.steer_control_frame.grid(row=5, column=0, columnspan=2, sticky="ew", pady=5)

        tk.Label(self.steer_control_frame, text="Steer Control Parameters:").grid(row=0, column=0, columnspan=4, sticky="ew")
        
        tk.Label(self.steer_control_frame, text="Angular Offset Gain:").grid(row=1, column=0, sticky="w")
        self.e_phi_gain_entry = tk.Entry(self.steer_control_frame, width=5)
        self.e_phi_gain_entry.grid(row=1, column=1, padx=5)
        self.e_phi_gain_entry.insert(0, "2.0")

        tk.Label(self.steer_control_frame, text="Lateral Offset Gains:").grid(row=2, column=0, sticky="w")
        self.lateral_offset_gains_entries = [tk.Entry(self.steer_control_frame, width=5) for _ in range(8)]
        default_values = [3, 2, 1.3, 0.8, 0.55, 0.32, 0.12, 0.04]
        for i, (entry, value) in enumerate(zip(self.lateral_offset_gains_entries, default_values)):
            entry.grid(row=2, column=i + 1, padx=2)
            entry.insert(0, str(value))

        tk.Label(self.steer_control_frame, text="Individual Lateral Offset Saturations:").grid(row=3, column=0, sticky="w")
        self.individual_lateral_offset_saturations_entries = [tk.Entry(self.steer_control_frame, width=5) for _ in range(8)]
        default_values = [10, 10, 2, 2.75, 3, 2.75, 2.25, 1.7]
        for i, (entry, value) in enumerate(zip(self.individual_lateral_offset_saturations_entries, default_values)):
            entry.grid(row=3, column=i + 1, padx=2)
            entry.insert(0, str(value))

        tk.Label(self.steer_control_frame, text="Total Lateral Offset Saturations:").grid(row=4, column=0, sticky="w")
        self.total_lateral_lever_saturations_entry = tk.Entry(self.steer_control_frame, width=5)
        self.total_lateral_lever_saturations_entry.grid(row=4, column=1, padx=5)
        self.total_lateral_lever_saturations_entry.insert(0, "25")

        tk.Label(self.steer_control_frame, text="Steering Control Saturations:").grid(row=5, column=0, sticky="w")
        self.steering_control_saturations_entry = tk.Entry(self.steer_control_frame, width=5)
        self.steering_control_saturations_entry.grid(row=5, column=1, padx=5)
        self.steering_control_saturations_entry.insert(0, "40")

        self.invert_steering_var = tk.IntVar()
        self.invert_steering_checkbox = tk.Checkbutton(self.steer_control_frame, text="Invert Steering", variable=self.invert_steering_var)
        self.invert_steering_checkbox.grid(row=6, column=0, sticky="w", padx=5)

    def create_torque_control_parameters(self):
        self.torque_control_frame = tk.Frame(self.scrollable_frame, bd=2, relief=tk.SUNKEN)
        self.torque_control_frame.grid(row=5, column=2, columnspan=2, sticky="ew", pady=5)

        tk.Label(self.torque_control_frame, text="Torque Control Parameters:").grid(row=0, column=0, columnspan=4, sticky="ew")

        tk.Label(self.torque_control_frame, text="Max Engine Brake Torque:").grid(row=1, column=0, sticky="w")
        self.max_engine_brake_torque_entry = tk.Entry(self.torque_control_frame, width=5)
        self.max_engine_brake_torque_entry.grid(row=1, column=1, padx=5)
        self.max_engine_brake_torque_entry.insert(0, "1.0")

        tk.Label(self.torque_control_frame, text="Differential Gain 1:").grid(row=2, column=0, sticky="w")
        self.differential_gain_1_entry = tk.Entry(self.torque_control_frame, width=5)
        self.differential_gain_1_entry.grid(row=2, column=1, padx=5)
        self.differential_gain_1_entry.insert(0, "0.002")

        tk.Label(self.torque_control_frame, text="Driving Differential Gain:").grid(row=3, column=0, sticky="w")
        self.driving_differential_gain_entry = tk.Entry(self.torque_control_frame, width=5)
        self.driving_differential_gain_entry.grid(row=3, column=1, padx=5)
        self.driving_differential_gain_entry.insert(0, "0.002")

        tk.Label(self.torque_control_frame, text="Overrun Differential Gain:").grid(row=4, column=0, sticky="w")
        self.overrun_differential_gain_entry = tk.Entry(self.torque_control_frame, width=5)
        self.overrun_differential_gain_entry.grid(row=4, column=1, padx=5)
        self.overrun_differential_gain_entry.insert(0, "0.002")

        tk.Label(self.torque_control_frame, text="Front Wheel Coefficient:").grid(row=1, column=2, sticky="w")
        self.front_wheel_coefficient_entry = tk.Entry(self.torque_control_frame, width=5)
        self.front_wheel_coefficient_entry.grid(row=1, column=3, padx=5)
        self.front_wheel_coefficient_entry.insert(0, "0.5")

        tk.Label(self.torque_control_frame, text="Rear Wheel Coefficient:").grid(row=2, column=2, sticky="w")
        self.rear_wheel_coefficient_entry = tk.Entry(self.torque_control_frame, width=5)
        self.rear_wheel_coefficient_entry.grid(row=2, column=3, padx=5)
        self.rear_wheel_coefficient_entry.insert(0, "0.5")

        tk.Label(self.torque_control_frame, text="Max Drive Torque:").grid(row=3, column=2, sticky="w")
        self.max_drive_torque_entry = tk.Entry(self.torque_control_frame, width=5)
        self.max_drive_torque_entry.grid(row=3, column=3, padx=5)
        self.max_drive_torque_entry.insert(0, "2000")

        tk.Label(self.torque_control_frame, text="Max Brake Torque:").grid(row=4, column=2, sticky="w")
        self.max_brake_torque_entry = tk.Entry(self.torque_control_frame, width=5)
        self.max_brake_torque_entry.grid(row=4, column=3, padx=5)
        self.max_brake_torque_entry.insert(0, "2000")

        tk.Label(self.torque_control_frame, text="Max Long Slip:").grid(row=5, column=0, sticky="w")
        self.max_long_slip_entry = tk.Entry(self.torque_control_frame, width=5)
        self.max_long_slip_entry.grid(row=5, column=1, padx=5)
        self.max_long_slip_entry.insert(0, "0.1")

        tk.Label(self.torque_control_frame, text="Slip Constant:").grid(row=5, column=2, sticky="w")
        self.slip_constant_entry = tk.Entry(self.torque_control_frame, width=5)
        self.slip_constant_entry.grid(row=5, column=3, padx=5)
        self.slip_constant_entry.insert(0, "10")

    def create_console_output(self):
        self.console_output = tk.Text(self.scrollable_frame, height=16, width=50, bg="black", fg="white")
        self.console_output.grid(row=7, column=0, columnspan=4, pady=5)

    def create_bottom_buttons(self):
        bottom_button_frame = tk.Frame(self.scrollable_frame)
        bottom_button_frame.grid(row=8, column=0, columnspan=4, sticky="ew", pady=5)

        tk.Button(bottom_button_frame, text="Refresh", command=self.refresh_console_output).grid(row=0, column=0, padx=5, pady=5)
        tk.Button(bottom_button_frame, text="ABORT", bg="orange", command=self.abort_simulation).grid(row=0, column=1, padx=5, pady=5)
        tk.Button(bottom_button_frame, text="Save Graphs", command=self.save_all_graphs).grid(row=0, column=2, padx=5, pady=5)
        tk.Button(bottom_button_frame, text="Show Graphs", command=self.show_graphs_from_folder).grid(row=0, column=3, padx=5, pady=5)


    def on_force_clicked(self):
        try:
            self.skip_flag.set()
            self._append_to_console_output_safe(f"Forced inputs applied:\n")


        except ValueError as e:
            # Handle invalid input (e.g., non-numeric values in input fields)
            self._append_to_console_output_safe(f"Error: Invalid input values. Please enter valid numbers.\n")
    
    def on_disable_clicked(self):
        self._append_to_console_output_safe(f"Slip Controls Disabled\n")
        self.skip_flag.clear()

    def start_server(self):
        if not self.server_task.is_running():
            self.server_task.start()

    def run_server(self):
        try:
            SERVER_IP = self.ip_entry.get()
            SERVER_PORT = int(self.port_entry.get())

            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.server_socket.bind((SERVER_IP, SERVER_PORT))
            self._update_connection_status_safe("Listening")
            self._append_to_socket_output_safe("Server started and listening for incoming connections...\n")

            while not self.abort_flag.is_set():
                try:
                    data, self.client_address = self.server_socket.recvfrom(1024)
                    if data:
                        self._append_to_socket_output_safe(f"Client connected from {self.client_address}\n")
                        self._update_connection_status_safe("Connected")
                        break
                except socket.timeout:
                    continue
        except Exception as e:
            self._append_to_socket_output_safe(f"Error starting server: {e}\n")

    def start_simulation(self):
        if not self.client_address:
            self._append_to_console_output_safe("No client connected. Start the server first.\n")
            return
        self._append_to_console_output_safe("Start SIM pressed\n")
        if not self.simulation_task.is_running():
            self.simulation_task = BackgroundTask(
                lambda: self.run_backend_script(
                    train_mode=self.train_mode, 
                    model_path="ppo_steering_model_final.h5" if not self.train_mode else None, 
                    total_episodes=1000 if self.train_mode else 1  # 1 episode for play mode
                )
            )
            self.simulation_task.start()

    def stop_simulation(self):
        self._append_to_console_output_safe("Stop SIM pressed\n")
        self.abort_flag.set()
        if self.simulation_task.is_running():
            self.simulation_task.stop()

    def reset_simulation_button(self):
        self._append_to_console_output_safe("Reset SIM pressed\n")
        if not self.reset_task.is_running():
            self.reset_task.start()

    def reset_simulation_task(self):
        self.reset_flag.set()
        self._append_to_console_output_safe("Simulation reset initiated.\n")
        self.reset_GUI_variables()
        self._append_to_console_output_safe("Simulation reset completed.\n")
        self.reset_flag.clear()

    def reset_GUI_variables(self):
        # Reset all the variables used in the simulation
        self.tracks_var.set("Select Tracks")
        self.max_engine_brake_torque_entry.delete(0, tk.END)
        self.max_engine_brake_torque_entry.insert(0, "0.1")
        self.differential_gain_1_entry.delete(0, tk.END)
        self.differential_gain_1_entry.insert(0, "0.1")
        self.driving_differential_gain_entry.delete(0, tk.END)
        self.driving_differential_gain_entry.insert(0, "0.1")
        self.overrun_differential_gain_entry.delete(0, tk.END)
        self.overrun_differential_gain_entry.insert(0, "0.1")
        self.front_wheel_coefficient_entry.delete(0, tk.END)
        self.front_wheel_coefficient_entry.insert(0, "0.5")
        self.rear_wheel_coefficient_entry.delete(0, tk.END)
        self.rear_wheel_coefficient_entry.insert(0, "0.5")
        self.max_drive_torque_entry.delete(0, tk.END)
        self.max_drive_torque_entry.insert(0, "4000")
        self.max_brake_torque_entry.delete(0, tk.END)
        self.max_brake_torque_entry.insert(0, "4000")
        self.num_Velocity_var.set("70")
        self.e_phi_gain_entry.delete(0, tk.END)
        self.e_phi_gain_entry.insert(0, "2.0")
        self.unlimited_laps_var.set(0)
        self.disable_var.set(0)
        self.num_laps_entry.delete(0, tk.END)
        self.num_laps_entry.insert(0, "1")
        self.num_laps_entry.config(state="normal")
        self.steering_angle_entry.delete(0, tk.END)
        self.steering_angle_entry.insert(0, "0")
        self.MDriveFL_entry.delete(0, tk.END)
        self.MDriveFL_entry.insert(0, "0")
        self.MDriveFR_entry.delete(0, tk.END)
        self.MDriveFR_entry.insert(0, "0")
        self.MDriveRL_entry.delete(0, tk.END)
        self.MDriveRL_entry.insert(0, "0")
        self.MDriveRR_entry.delete(0, tk.END)
        self.MDriveRR_entry.insert(0, "0")
        self.MBrakeFL_entry.delete(0, tk.END)
        self.MBrakeFL_entry.insert(0, "0")
        self.MBrakeFR_entry.delete(0, tk.END)
        self.MBrakeFR_entry.insert(0, "0")
        for entry, value in zip(self.lateral_offset_gains_entries, [3, 2, 1.3, 0.8, 0.55, 0.32, 0.12, 0.04]):
            entry.delete(0, tk.END)
            entry.insert(0, str(value))
        for entry, value in zip(self.individual_lateral_offset_saturations_entries, [1.5, 1.5, 2, 2.75, 3, 2.75, 2.25, 1.7]):
            entry.delete(0, tk.END)
            entry.insert(0, str(value))
        self.total_lateral_lever_saturations_entry.delete(0, tk.END)
        self.total_lateral_lever_saturations_entry.insert(0, "18")
        self.steering_control_saturations_entry.delete(0, tk.END)
        self.steering_control_saturations_entry.insert(0, "28")
        self.invert_steering_var.set(0)
        self.log_frequency_var.set("20")

    def refresh_console_output(self):
        refresh_rate = int(self.refresh_rate_entry.get())
        self.refresh_task.stop()  # Stop the current task if running
        self.refresh_task = TkRepeatingTask(self.scrollable_frame, self._refresh_console, refresh_rate)
        self.refresh_task.start()

    def _refresh_console(self):
        self._clear_console_output_safe()

    def abort_simulation(self):
        self.abort_flag.set()
        self._update_connection_status_safe("Disconnected")
        if self.simulation_task.is_running():
            self.simulation_task.stop()

    def validate_excel_files(self):
        self._append_to_console_output_safe(f"Validating selected files...\n")
        if not self.tracks_file or not self.velocity_profile_file:
            self._append_to_console_output_safe("Error: Please select both a tracks file and a velocity profile file.\n")
            return False
        try:
            self._append_to_console_output_safe("Reading Files...\n")
            self.scrollable_frame.after(0, pd.read_excel, self.tracks_file)
            self.scrollable_frame.after(0, pd.read_excel, self.velocity_profile_file)
            self._append_to_console_output_safe("Files Read...\n")
        except Exception as e:
            self._append_to_console_output_safe(f"Error reading selected files: {e}\n")
            return False
        self._append_to_console_output_safe("Files validated successfully.\n")
        return True

    def update_connection_status(self, status):
        if status == "Connected":
            self.connection_status.config(text="Status: Connected", fg="green")
        elif status == "Listening":
            self.connection_status.config(text="Status: Listening", fg="blue")
        else:
            self.connection_status.config(text="Status: Disconnected", fg="red")

    def _update_connection_status_safe(self, status):
        self.scrollable_frame.after(0, self.update_connection_status, status)

    def _append_to_console_output_safe(self, text):
        self.scrollable_frame.after(0, self.console_output.insert, tk.END, text)

    def _clear_console_output_safe(self):
        self.scrollable_frame.after(0, self.console_output.delete, 1.0, tk.END)

    def _append_to_socket_output_safe(self, text):
        self.scrollable_frame.after(0, self.socket_output.insert, tk.END, text)

    def get_combined_files(self):
        combined_files = []
        try:
            traj_files = [file for file in os.listdir('Trajectories') if file.endswith('.xlsx') or file.endswith('.xls')]
            vel_files = [file for file in os.listdir('Velocities') if file.endswith('.xlsx') or file.endswith('.xls')]
            combined_files = [f"{traj_file} | {vel_file}" for traj_file, vel_file in zip(traj_files, vel_files)]
        except FileNotFoundError as e:
            self._append_to_console_output_safe(f"Error: {e}\n")
        return combined_files

    def set_files(self, selected_files):
        traj_file, vel_file = selected_files.split(' | ')
        self.tracks_file = os.path.join('Trajectories', traj_file)
        self.velocity_profile_file = os.path.join('Velocities', vel_file)
        self._append_to_console_output_safe(f"Selected Tracks File: {self.tracks_file}\n")
        self._append_to_console_output_safe(f"Selected Velocity Profile File: {self.velocity_profile_file}\n")

    def save_all_graphs(self):
        if not self.figures:
            self._append_to_console_output_safe("No graphs available to save. Run the simulation first.\n")
            return

        save_dir = 'SimulationGraphs'
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        for i, fig in enumerate(self.figures, start=1):
            fig_path = os.path.join(save_dir, f'graph_{i}.png')
            fig.savefig(fig_path)
            self._append_to_console_output_safe(f"Saved {fig_path}\n")

        self._append_to_console_output_safe("All graphs have been saved.\n")

    def show_graphs_from_folder(self):
        folder_path = 'SimulationGraphs'
        if not os.path.exists(folder_path):
            self._append_to_console_output_safe("No graphs available to display. Run the simulation and save graphs first.\n")
            return

        graphs_window = tk.Toplevel(self.scrollable_frame)
        graphs_window.title("Select Graph to Display")

        graph_files = [file for file in os.listdir(folder_path) if file.endswith('.png')]
        if not graph_files:
            self._append_to_console_output_safe("No graphs found in the folder.\n")
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

    def start_reset_listener(self):
        def listen_for_reset():
            reset_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            reset_socket.bind(("0.0.0.0", 6015))  # Listening on port 6015 for reset messages
            while True:
                data, addr = reset_socket.recvfrom(1)
                if data == b'\x00':  # Single byte message with value zero
                    self.reset_simulation_task()

        self.reset_listener_thread = threading.Thread(target=listen_for_reset, daemon=True)
        self.reset_listener_thread.start()

    
    def run_backend_script(self, train_mode, model_path, total_episodes):
        try:
            i = 0
            nLap = 0
            lastNLap = 0
            LapCompleted = False
            force_controls = False
            nLaps = []
            last_e_phi = 0
            lastRefDistance = 0
            disable_flag = False
            yawAngle = 0
            x = 0
            y = 0
            z = 0
            Vx = 0
            Vy = 0

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

            # Initialize PPO Agent (used for both training and playing)
            state_shape = (7,)  # Assuming the state contains [x, y, z, yawAngle, Vx, Vy, Vz]
            action_space = 1     # Single continuous action for steering input (-28 to 28)
            ppo_agent = PPOAgent(state_shape, action_space)

            def reset_variables():
                nonlocal steering_input, MDriveFL, MDriveFR, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, e_phi, total_e_l
                nonlocal MDriveRL_array, MDriveRR_array, MBrakeFL_array, MBrakeFR_array, MBrakeRL_array, MBrakeRR_array
                nonlocal tVelocity_array, acc_array, oldThrust_array
                nonlocal x_positions, y_positions, yawAngles, velocity_X, velocity_Y
                nonlocal e_phi_values, refLine_orientations, saturated_e_l_values
                nonlocal lever_coordinates, ideal_coordinates, steering_inputs, refDistances, tVCs
                nonlocal errors, previous_targetVelocity

                steering_input = 0
                MDriveFL = 0
                MDriveFR = 0
                MDriveRL = 0
                MDriveRR = 0
                MBrakeFL = 0
                MBrakeFR = 0
                MBrakeRL = 0
                MBrakeRR = 0
                e_phi = 0
                total_e_l = 0

                MDriveRL_array.clear()
                MDriveRR_array.clear()
                MBrakeFL_array.clear()
                MBrakeFR_array.clear()
                MBrakeRL_array.clear()
                MBrakeRR_array.clear()
                tVelocity_array.clear()
                acc_array.clear()
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

            def step_simulation(self, server_socket, client_address, action, tracks_df, nLap, e_phi_gain, e_l_gains, e_l_limits, total_e_l_limits, steer_limits):
                """
                Send the steering action to the client, receive the state, and calculate the reward and done flag.
                """
                try:
                    self.server_socket.sendto(data_to_send, self.client_address)
                    data, _ = self.server_socket.recvfrom(1024)

                    received_length = len(data)
                    expected_length = 16 * 4  # 16 floats, each 4 bytes

                    if received_length == expected_length:
                        num_floats = 16
                        float_data = struct.unpack('f' * num_floats, data[:num_floats * 4])

                        (x, y, z, Vx, Vy, Vz, yawAngle, frontLHSWheelSlipRatio, frontRHSWheelSlipRatio,
                            rearLHSWheelSlipRatio, rearRHSWheelSlipRatio, frontLHSWheelVelocity, frontRHSWheelVelocity,
                            rearLHSWheelVelocity, rearRHSWheelVelocity, newSimTime) = float_data
                        
                        if log_frequency != 0 and i % log_frequency == 0:
                            with open(vehicle_input_log_file, mode='a', newline='') as input_log:
                                input_writer = csv.writer(input_log)
                                input_writer.writerow([simTime, steering_input, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR])
                            with open(vehicle_state_log_file, mode='a', newline='') as state_log:
                                state_writer = csv.writer(state_log)
                                state_writer.writerow([simTime, x, y, z, Vx, Vy, Vz, yawAngle])

                        output_text = (
                            f"Received data:\n"
                            f"x = {x}\n"
                            f"y = {y}\n"
                            f"z = {z}\n"
                            f"Vx = {Vx}\n"
                            f"Vy = {Vy}\n"
                            f"Vz = {Vz}\n"
                            f"yawAngle = {yawAngle}\n"
                            f"simTime = {simTime}\n\n"
                        )
                        self._append_to_console_output_safe(output_text)
                    else:
                        self._append_to_console_output_safe(f"Received data length = {received_length} bytes\n")
                        self._append_to_console_output_safe(f"Expected data length = {expected_length} bytes\n")
                        self._append_to_console_output_safe("Error: Received data length does not match the expected length. Breaking the loop due to potential data loss.\n")
                        
                # except socket.timeout:
                #     self._append_to_socket_output_safe("Socket timeout: No response from client\n")
                # except Exception as e:
                #     self._append_to_socket_output_safe(f"Socket error: {e}\n")
                        
                    # Extract values from state
                    state = x, y, z, yawAngle, Vx, Vy, Vz

                    x = -x
                    y = -y

                    x_positions.append(x)
                    y_positions.append(y)

                    if yawAngles:
                        last_yawAngle = yawAngles[-1]
                    else:
                        last_yawAngle = yawAngle

                    self._append_to_console_output_safe(f"Lap: {nLap}\n")
                    if lastNLap < nLap:
                        LapCompleted = True
                        self._append_to_console_output_safe(f"Yaw Angle after Lap Completion: {yawAngle} = {last_yawAngle} + {2 * np.pi} #########################################\n\n")
                    lastNLap = nLap

                    yawAngle, LapCompleted = adjust_numbers(last_yawAngle, yawAngle, LapCompleted, nLap)
                    yawAngles.append(yawAngle)

                    Vx = abs(Vx)
                    Vy = abs(Vy)

                    vCar = np.sqrt(Vx ** 2 + Vy ** 2)
                    vCarArray.append(vCar)

                    plot_Vx = Vx * 3.6
                    velocity_X.append(plot_Vx)

                    # Calculate the angular and lateral offsets using the existing steer control functions
                    timestep = newSimTime - simTime
                    simTime = newSimTime
                    newSim = False

                    # Call the existing steering control function to get e_phi and e_l
                    steering_input, e_phi, saturated_e_l, _, _, _, _, ref_distance, _ = calculate_steering_control(
                        tracks_df, Vx, Vy, yawAngle, x, y, e_phi_gain, timestep, nLap, newSim, 
                        e_l_gains, e_l_limits, total_e_l_limits, steer_limits
                    )

                    if invert_steering:
                        steering_input = -steering_input
                    
                    if force_controls == True:
                        steering_input = float(self.steering_angle_entry.get())

                    e_phi_values.append(e_phi)
                    for j in range(8):
                        saturated_e_l_values[j].append(saturated_e_l[j])
                    lever_coordinates.append(levercoordsplot)
                    ideal_coordinates.append(idealcoordsplot)
                    refLine_orientations.append(refLine_orientation)

                    MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, targetVelocity, longAcceleration, oldLongThrust = calculateTorqueControl(
                        tracks_df, velocity_profile_df, Vx, Vy, yawAngle, x, y, rearLHSWheelVelocity,
                        rearRHSWheelVelocity, frontLHSWheelSlipRatio, frontRHSWheelSlipRatio, rearLHSWheelSlipRatio, rearRHSWheelSlipRatio,
                        max_engine_brake_torque, differential_gain_1, driving_differential_gain, overrun_differential_gain, front_wheel_coefficient,
                        rear_wheel_coefficient, max_drive_torque, max_brake_torque, max_long_slip, slip_constant, timeStep, velocity_percentage, refDistance, nLap, disable_flag
                    )


                    # Calculate the reward based on the angular and lateral offsets
                    reward = -(abs(e_phi) + abs(total_e_l))

                    # Check termination condition (done flag)
                    if 0.25*(1/Vx) <= scaling_factor <= 4*(1/Vx)
                    done = check_done_condition(x, y, ref_distance, nLap)

                    # Return the next state, reward, and done flag
                    return np.array(state), reward, done, timestep
                except Exception as e:
                    print(f"Error in step_simulation: {e}")
                    return None, 0, True  # Return done=True in case of error

            reset_variables()

            if not self.validate_excel_files():
                return

            self._update_connection_status_safe("Connected")
            self._append_to_console_output_safe("Starting simulation...\n")
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
            max_long_slip = float(self.max_long_slip_entry.get())
            slip_constant = float(self.slip_constant_entry.get())
            velocity_percentage = int(self.num_Velocity_var.get())
            log_frequency = int(self.log_frequency_var.get())
            e_phi_gain = float(self.e_phi_gain_entry.get())
            e_l_gains = [float(entry.get()) for entry in self.lateral_offset_gains_entries]
            e_l_limits = [float(entry.get()) for entry in self.individual_lateral_offset_saturations_entries]
            total_e_l_limits = float(self.total_lateral_lever_saturations_entry.get())
            steer_limits = float(self.steering_control_saturations_entry.get())
            invert_steering = self.invert_steering_var.get()
            velocity_profile_df['vCar'] = velocity_profile_df['vCar'] * velocity_percentage / 100
            velocity_profile_df['vCar2'] = velocity_profile_df['vCar2'] * velocity_percentage / 100
            configured_laps = 0
            if self.unlimited_laps_var.get() == 0:
                configured_laps = int(self.num_laps_entry.get())
                self._append_to_console_output_safe(f"Laps Initiated: {configured_laps}\n")
            else:
                configured_laps = float('inf')
                self._append_to_console_output_safe(f"Laps Initiated: {configured_laps}\n")

            if self.disable_var.get() == 1:
                disable_flag = True
                self._append_to_console_output_safe("Slip Controls Disabled\n")
            else:
                disable_flag = False
                self._append_to_console_output_safe("Slip Controls Enabled\n")

            log_folder = "SimulationLogs"
            if not os.path.exists(log_folder):
                os.makedirs(log_folder)
            current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            base_name = os.path.splitext(os.path.basename(self.tracks_file))[0]

            if log_frequency != 0:
                vehicle_input_log_file = os.path.join(log_folder, f"{current_time}_{base_name}_Vehicle_Input_Log.csv")
                vehicle_state_log_file = os.path.join(log_folder, f"{current_time}_{base_name}_Vehicle_State_Log.csv")
                with open(vehicle_input_log_file, mode='w', newline='') as input_log:
                    input_writer = csv.writer(input_log)
                    input_writer.writerow(["simTime", "steering_input", "MDriveRL", "MDriveRR", "MBrakeFL", "MBrakeFR"])
                with open(vehicle_state_log_file, mode='w', newline='') as state_log:
                    state_writer = csv.writer(state_log)
                    state_writer.writerow(["simTime", "x", "y", "z", "Vx", "Vy", "Vz", "yawAngle"])

            while not self.abort_flag.is_set():
                self._append_to_console_output_safe("Waiting for a client to connect...\n")

                if train_mode:
                    # Training Loop
                    for episode in range(total_episodes):
                        print(f"Starting Episode {episode + 1}/{total_episodes}")
                        state = reset_simulation(self.server_socket, self.client_address)
                        done = False
                        episode_reward = 0
                        step = 0

                        while not done and step < 1000:
                            # Get action from PPO agent
                            action, value = ppo_agent.get_action(state)

                            # Send the action to the client and get the next state, reward, and done
                            next_state, reward, done, _ = step_simulation(
                                self.server_socket, self.client_address, action, 
                                tracks_df, nLap, e_phi_gain, e_l_gains, e_l_limits, total_e_l_limits, steer_limits
                            )

                            # Update the agent policy
                            advantage = reward + 0.99 * value - value
                            ppo_agent.train(state, action, advantage, reward, value)

                            # Move to the next state
                            state = next_state
                            episode_reward += reward
                            step += 1

                    else:
                        # Playing mode
                        state = reset_simulation(self.server_socket, self.client_address)
                        done = False

                        while not done:
                            action, _ = ppo_agent.get_action(state)
                            next_state, _, done, timestep = step_simulation(
                                self.server_socket, self.client_address, action, 
                                tracks_df, nLap, e_phi_gain, e_l_gains, e_l_limits, total_e_l_limits, steer_limits
                            )

                            state = next_state
                            time.sleep(timestep)

                    print(f"Episode {episode} finished with total reward: {episode_reward}")

                    if episode % 100 == 0:
                        ppo_agent.model.save(f"ppo_steering_model_{episode}.h5")

                ppo_agent.model.save("ppo_steering_model_final.h5")

                try:
                    while nLap < configured_laps:
                        i += 1
                        # print(f"Iteration {i}")
                        if self.abort_flag.is_set():
                            self._append_to_console_output_safe("Simulation aborted.\n")
                            self.abort_flag.clear()
                            break
                        if self.reset_flag.is_set():
                            self._append_to_console_output_safe("Resetting simulation...\n")
                            reset_variables()
                            self.reset_flag.clear()
                            self._append_to_console_output_safe("Simulation reset completed. Restarting iterations.\n")
                            break
                        if self.skip_flag.is_set():
                            force_controls = True
                        else:
                            force_controls = False

                        self._append_to_console_output_safe(f"Iteration {i}\n")
                        self.console_output.see(tk.END)  # Ensure the console always scrolls to the end

                        data_to_send = struct.pack('fffffff', steering_input, MDriveFL, MDriveFR, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR)

                        try:
                            self.server_socket.sendto(data_to_send, self.client_address)
                            data, _ = self.server_socket.recvfrom(1024)

                            received_length = len(data)
                            expected_length = 16 * 4  # 16 floats, each 4 bytes

                            if received_length == expected_length:
                                num_floats = 16
                                float_data = struct.unpack('f' * num_floats, data[:num_floats * 4])

                                (x, y, z, Vx, Vy, Vz, yawAngle, frontLHSWheelSlipRatio, frontRHSWheelSlipRatio,
                                 rearLHSWheelSlipRatio, rearRHSWheelSlipRatio, frontLHSWheelVelocity, frontRHSWheelVelocity,
                                 rearLHSWheelVelocity, rearRHSWheelVelocity, newSimTime) = float_data
                                
                                if log_frequency != 0 and i % log_frequency == 0:
                                    with open(vehicle_input_log_file, mode='a', newline='') as input_log:
                                        input_writer = csv.writer(input_log)
                                        input_writer.writerow([simTime, steering_input, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR])
                                    with open(vehicle_state_log_file, mode='a', newline='') as state_log:
                                        state_writer = csv.writer(state_log)
                                        state_writer.writerow([simTime, x, y, z, Vx, Vy, Vz, yawAngle])

                                output_text = (
                                    f"Received data:\n"
                                    f"x = {x}\n"
                                    f"y = {y}\n"
                                    f"z = {z}\n"
                                    f"Vx = {Vx}\n"
                                    f"Vy = {Vy}\n"
                                    f"Vz = {Vz}\n"
                                    f"yawAngle = {yawAngle}\n"
                                    f"simTime = {simTime}\n\n"
                                )
                                self._append_to_console_output_safe(output_text)
                            else:
                                self._append_to_console_output_safe(f"Received data length = {received_length} bytes\n")
                                self._append_to_console_output_safe(f"Expected data length = {expected_length} bytes\n")
                                self._append_to_console_output_safe("Error: Received data length does not match the expected length. Breaking the loop due to potential data loss.\n")
                                break
                        except socket.timeout:
                            self._append_to_socket_output_safe("Socket timeout: No response from client\n")
                        except Exception as e:
                            self._append_to_socket_output_safe(f"Socket error: {e}\n")

                        if i == 1:
                            tracks_df = generate_driving_line(tracks_df, x, y, yawAngle)
                            print(tracks_df.head())
                            print(tracks_df.tail(36))
                            velocity_profile_df = velocity_profile_df.loc[:, ~velocity_profile_df.columns.str.contains('^Unnamed')]
                            newSim = True

                        

                        timeStep = newSimTime - simTime
                        simTime = newSimTime

                        steering_inputs.append(steering_input)
                        steering_input, e_phi, saturated_e_l, levercoordsplot, idealcoordsplot, error_values, refLine_orientation, refDistance, newSim, total_e_l = calculate_steering_control(
                            tracks_df, Vx, Vy, yawAngle, x, y, e_phi_gain, timeStep, nLap, newSim, e_l_gains, e_l_limits, total_e_l_limits, steer_limits)
                        
                        if force_controls == True:
                            MDriveFL = float(self.MDriveFL_entry.get())  
                            MDriveFR = float(self.MDriveFR_entry.get())  
                            MDriveRL = float(self.MDriveRL_entry.get())  
                            MDriveRR = float(self.MDriveRR_entry.get())
                            MBrakeFL = float(self.MBrakeFL_entry.get())
                            MBrakeFR = float(self.MBrakeFR_entry.get())
                        if force_controls == False:
                            MDriveFL = 0
                            MDriveFR = 0
                        
                        # print(f"Last Ref Distance: {lastRefDistance}")
                        print(f"Ref Distance: {refDistance}")
                        if lastRefDistance >= refDistance + 1000:
                            print("Lap Completed")
                            nLap += 1
                        lastRefDistance = refDistance

                        MDriveRL_array.append(MDriveRL)
                        MDriveRR_array.append(MDriveRR)
                        MBrakeFL_array.append(MBrakeFL)
                        MBrakeFR_array.append(MBrakeFR)
                        MBrakeRL_array.append(MBrakeRL)
                        MBrakeRR_array.append(MBrakeRR)
                        tVelocity_array.append(targetVelocity)
                        previous_targetVelocity = targetVelocity
                        acc_array.append(longAcceleration)
                        oldThrust_array.append(oldLongThrust)

                        output_text2 = (
                            f"Steering Input: {steering_input}\n"
                            f"MDriveRL: {MDriveRL}\n"
                            f"MDriveRR: {MDriveRR}\n"
                            f"MBrakeFL: {MBrakeFL}\n"
                            f"MBrakeFR: {MBrakeFR}\n\n"
                        )
                        self._append_to_console_output_safe(output_text2)

                    if self.reset_flag.is_set():
                        continue
                    else:
                        break
                finally:
                    self._append_to_socket_output_safe("Client connection closed.\n")

            self.server_socket.close()

            def plot_and_store(figures, plot_func, *args):
                try:
                    fig = plot_func(*args)
                    if fig is not None:
                        figures.append(fig)
                except Exception as e:
                    self._append_to_console_output_safe(f"Error plotting {plot_func.__name__}: {e}\n")

            if lever_coordinates and ideal_coordinates:
                plot_and_store(self.figures, plotTraj, base_name, tracks_df, x_positions, y_positions, levercoordsplot, idealcoordsplot, error_values, current_time)
            plot_and_store(self.figures, plot_e_phi_Values, base_name, e_phi_values, current_time)
            plot_and_store(self.figures, plot_e_l_Values, base_name, saturated_e_l_values, current_time)
            plot_and_store(self.figures, plot_balance_Values, base_name, steering_inputs, refLine_orientations, current_time)
            plot_and_store(self.figures, plot_Angles, base_name, yawAngles, refLine_orientations, current_time)
            plot_and_store(self.figures, plot_Torque, base_name, MDriveRL_array, MDriveRR_array, MBrakeFL_array, MBrakeFR_array, MBrakeRL_array, MBrakeRR_array, current_time)
            plot_and_store(self.figures, plot_TargetValues, base_name, tVelocity_array, acc_array, velocity_X, current_time)

            self._append_to_console_output_safe("Figures saved locally.\n")
            self.display_graphs(self.figures)
            self.tracks_file = None
            self.velocity_profile_file = None
            self._append_to_console_output_safe("Script finished execution.\n")
            self._update_connection_status_safe("Disconnected")
        except Exception as e:
            self._append_to_console_output_safe(f"Error running script: {e}\n")

    def display_graphs(self):
        if not self.figures:
            self._append_to_console_output_safe("No graphs available to display. Run the simulation first.\n")
            return

        graphs_window = tk.Toplevel(self.scrollable_frame)
        graphs_window.title("Simulation Graphs")

        for fig in self.figures:
            canvas = FigureCanvasTkAgg(fig, master=graphs_window)
            canvas.draw()
            canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    def close(self):
        if self.simulation_task.is_running():
            self.simulation_task.stop()
        if self.reset_task.is_running():
            self.reset_task.stop()
        if self.refresh_task.is_running():
            self.refresh_task.stop()
        self.scrollable_frame.quit()


def run_app():
    root = tk.Tk()
    app = ConsoleApp(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()


if __name__ == "__main__":
    run_app()