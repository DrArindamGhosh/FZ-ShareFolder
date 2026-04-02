# UI DATA DISPLAY
# Speedometer and Stopwatch Dials added
"""
This script implements a Tkinter-based GUI application for running and monitoring the Virtual Vehicle Validation Model (V3M).
It provides a user interface for configuring simulation parameters, starting/stopping/resetting the simulation,
and visualizing simulation results. The application communicates with a backend simulation client via UDP sockets,
sending control commands and receiving vehicle state data in real time.
Major Components:
- Imports: Loads required libraries for GUI, networking, data processing, plotting, and logging.
- Utility Functions:
    - adjust_numbers: Handles yaw angle wrapping and lap completion logic.
    - calculate_orientation: Computes orientation angles from x/y coordinates, ensuring continuity.
    - resample_uniformly: Interpolates trajectory data to a uniform distance grid.
    - generate_driving_line_DiL / generate_driving_line_Dev: Prepares and processes track data for DiL (Driver-in-the-Loop) and Dev (Development) modes.
- BackgroundTask: 
    - Class for running long-running or blocking tasks in a separate thread to keep the UI responsive.
- TkRepeatingTask:
    - Class for scheduling periodic UI updates (e.g., refreshing console output).
- ConsoleApp (Main GUI Class):
    - Initializes the main window, layout, and all UI widgets.
    - Handles user input for simulation parameters, file selection, and control commands.
    - Manages socket server for communication with the simulation backend.
    - Provides methods for starting, stopping, aborting, and resetting the simulation.
    - Handles forced driver inputs and slip control disabling.
    - Validates input files and parameters.
    - Collects and processes simulation data, including control signals and vehicle states.
    - Plots and saves simulation results using matplotlib.
    - Provides UI for displaying and saving generated graphs.
    - Implements a listener for external reset commands via UDP.
- run_app:
    - Entry point for launching the Tkinter application.
- __main__ block:
    - Starts the application if the script is run directly.
Key Features:
- Modular UI with sections for parameter input, force controls, steer/torque control, and output.
- Asynchronous background execution for simulation and server tasks.
- Real-time socket communication with simulation backend.
- Data logging, plotting, and visualization of simulation results.
- Support for both DiL and Dev simulation modes.
- Error handling and user feedback via the console output.
Note:
- The script expects trajectory and velocity profile files in specific folders.
- External modules (e.g., steer_control_v16_2, pedal_control_v1_3, model_traj_test_v6) are required for full functionality.
"""
import tkinter as tk
from tkinter import ttk
import pandas as pd
import threading
import socket
import struct
import os
import logging
from steer_control_v16_2 import calculate_steering_control
from pedal_control_v1_6 import calculatePedalPercent
from model_traj_test_v6 import plotTraj, plot_e_phi_Values, plot_e_l_Values, plot_balance_Values, plot_Angles, plot_Torque, plot_TargetValues, plot_PedalValues, plot_nMGU_Values, plot_Throttle_PID_Values, plot_Brake_PID_Values, plot_vCar_error_Values, plot_refDistances
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from datetime import datetime
import math

logging.basicConfig(level=logging.DEBUG, filename='simulation.log', filemode='w',
                    format='%(asctime)s - %(levelname)s - %(message)s')

def adjust_numbers(last_number, new_number, lapCompleted, nLap):
    yawAngle = new_number
    if lapCompleted == True:
        lapCompleted = False
        if last_number < -3*np.pi:
            yawAngle = last_number + 2 * np.pi
    else:
        if new_number - last_number > 1:
            yawAngle = new_number - np.pi
        if last_number - new_number < -6:
            yawAngle = new_number - 2 * np.pi
            if last_number < -9 and new_number > 0:
                yawAngle = new_number - 4 * np.pi
        if last_number - new_number > 6:
            yawAngle = new_number + 2 * np.pi
        last_number = new_number
        
    return yawAngle, lapCompleted

def calculate_orientation(x, y):
        orientations = np.arctan2(np.diff(y), np.diff(x))
        orientations = np.concatenate(([orientations[0]], orientations))
        # Convert orientations to range [0, 2π]
        orientations = (orientations + 2 * np.pi) % (2 * np.pi)
        # Ensure orientation continuity
        for i in range(1, len(orientations)):
            while orientations[i] - orientations[i - 1] > np.pi:
                orientations[i] -= 2 * np.pi
            while orientations[i] - orientations[i - 1] < -np.pi:
                orientations[i] += 2 * np.pi
        
        if orientations[0] > 5:
            orientations -= 2 * np.pi
        
        return orientations


def resample_uniformly(data, distance_col, x_col, y_col, orientation_col, num_points):

    data = data.sort_values(by=distance_col).reset_index(drop=True)

    uniform_distances = np.linspace(data[distance_col].min(), data[distance_col].max(), num_points)

    interpolated_x = np.interp(uniform_distances, data[distance_col], data[x_col])
    interpolated_y = np.interp(uniform_distances, data[distance_col], data[y_col])
    interpolated_orientation = np.interp(uniform_distances, data[distance_col], data[orientation_col])

    return uniform_distances, interpolated_x, interpolated_y, interpolated_orientation


def generate_driving_line_DiL(tracks_df, yawAngle):

    x = tracks_df['x']
    y = tracks_df['y']

    x2 = tracks_df['x2']
    y2 = tracks_df['y2']

    orientation = calculate_orientation(x, y)
    print("Orientation Calculated")

    orientation2 = calculate_orientation(x2, y2)
    print("Orientation2 Calculated")

    tracks_df['Orientation'] = orientation
    tracks_df['Orientation2'] = orientation2

    sDistance, x, y, orientation = resample_uniformly(tracks_df, 'sDistance', 'x', 'y', 'Orientation', num_points=1500)
    sDistance2, x2, y2, orientation2 = resample_uniformly(tracks_df, 'sDistance2', 'x2', 'y2', 'Orientation2', num_points=1500)

    tracks_df = pd.DataFrame({
        'sDistance': sDistance,
        'x': x,
        'y': y,
        'Orientation': orientation,
        'sDistance2': sDistance2,
        'x2': x2,
        'y2': y2,
        'Orientation2': orientation2
    })

    x = tracks_df['x']
    y = tracks_df['y']

    x2 = tracks_df['x2']
    y2 = tracks_df['y2']

    orientation = calculate_orientation(x, y)

    orientation2 = calculate_orientation(x2, y2)

    tracks_df['Orientation'] = orientation
    tracks_df['Orientation2'] = orientation2

    tracks_df['sDistance'] = tracks_df['sDistance'] - tracks_df['sDistance'].iloc[0]
    tracks_df['sDistance2'] = tracks_df['sDistance2'] - tracks_df['sDistance2'].iloc[0]
    
    tracks_df['Orientation'] = tracks_df['Orientation'] - tracks_df.at[0, 'Orientation'] + yawAngle
    clockwise_value = tracks_df['Orientation'].dropna().iloc[-1]
    if clockwise_value < 0:
        tracks_df['Orientation2'] = tracks_df['Orientation2'] - tracks_df.at[0, 'Orientation2'] + tracks_df['Orientation'].dropna().iloc[-1] + 2*np.pi
        clockwise = True
    else:
        tracks_df['Orientation2'] = tracks_df['Orientation2'] - tracks_df.at[0, 'Orientation2'] + tracks_df['Orientation'].dropna().iloc[-1] - 2*np.pi
        clockwise = False

    max_sDistance_lap1 = tracks_df['sDistance'].max()
    max_sDistance_lap2 = tracks_df['sDistance2'].max()

    n_wrap = 30
    s_offset_1 = tracks_df['sDistance'].iloc[-1]
    wrap_df = pd.DataFrame({
        'sDistance': tracks_df['sDistance2'].iloc[:n_wrap].values + s_offset_1,
        'x': tracks_df['x2'].iloc[:n_wrap].values,
        'y': tracks_df['y2'].iloc[:n_wrap].values,
        'Orientation': tracks_df['Orientation2'].iloc[:n_wrap].values - 3 * np.pi
    })

    s_offset_lap2 = tracks_df['sDistance2'].iloc[-1]
    wrap_lap2 = pd.DataFrame({
        'sDistance2': tracks_df['sDistance2'].iloc[:n_wrap].values + s_offset_lap2,
        'x2': tracks_df['x2'].iloc[:n_wrap].values,
        'y2': tracks_df['y2'].iloc[:n_wrap].values,
        'Orientation2': tracks_df['Orientation2'].iloc[:n_wrap].values - 2 * np.pi
    })

    combined_wrap = pd.concat([wrap_df.reset_index(drop=True), wrap_lap2.reset_index(drop=True)], axis=1)
    for col in tracks_df.columns:
        if col not in combined_wrap.columns:
            combined_wrap[col] = np.nan
    combined_wrap = combined_wrap[tracks_df.columns]
    tracks_df = pd.concat([tracks_df, combined_wrap], ignore_index=True)
    tracks_df.head()

    return tracks_df, max_sDistance_lap1, max_sDistance_lap2, clockwise

def generate_driving_line_Dev(tracks_df, initial_x, initial_y, yawAngle):

    x = tracks_df['x']
    y = tracks_df['y']

    x2 = tracks_df['x2']
    y2 = tracks_df['y2']

    orientation = calculate_orientation(x, y)
    print("Orientation Calculated")

    orientation2 = calculate_orientation(x2, y2)
    print("Orientation2 Calculated")

    tracks_df['Orientation'] = orientation
    tracks_df['Orientation2'] = orientation2

    tracks_df['sDistance'] = tracks_df['sDistance'] - tracks_df['sDistance'].iloc[0]
    tracks_df['sDistance2'] = tracks_df['sDistance2'] - tracks_df['sDistance2'].iloc[0]

    sDistance, x, y, orientation = resample_uniformly(tracks_df, 'sDistance', 'x', 'y', 'Orientation', num_points=1500)
    sDistance2, x2, y2, orientation2 = resample_uniformly(tracks_df, 'sDistance2', 'x2', 'y2', 'Orientation2', num_points=1500)

    tracks_df = pd.DataFrame({
        'sDistance': sDistance,
        'x': x,
        'y': y,
        'Orientation': orientation,
        'sDistance2': sDistance2,
        'x2': x2,
        'y2': y2,
        'Orientation2': orientation2
    })

    tracks_df['Orientation'] = tracks_df['Orientation'] - tracks_df.at[0, 'Orientation'] + yawAngle
    clockwise_value = tracks_df['Orientation'].dropna().iloc[-1]
    if clockwise_value < 0:
        tracks_df['Orientation2'] = tracks_df['Orientation2'] - tracks_df.at[0, 'Orientation2'] + tracks_df['Orientation'].dropna().iloc[-1] + 2*np.pi
        clockwise = True
    else:
        tracks_df['Orientation2'] = tracks_df['Orientation2'] - tracks_df.at[0, 'Orientation2'] + tracks_df['Orientation'].dropna().iloc[-1] - 2*np.pi
        clockwise = False

    # Set the initial coordinates
    tracks_df.at[0, 'x'] = initial_x
    tracks_df.at[0, 'y'] = initial_y

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

    max_sDistance_lap1 = tracks_df['sDistance'].max()
    max_sDistance_lap2 = tracks_df['sDistance2'].max()

    n_wrap = 30
    s_offset_1 = tracks_df['sDistance'].iloc[-1]
    wrap_df = pd.DataFrame({
        'sDistance': tracks_df['sDistance2'].iloc[:n_wrap].values + s_offset_1,
        'x': tracks_df['x2'].iloc[:n_wrap].values,
        'y': tracks_df['y2'].iloc[:n_wrap].values,
        'Orientation': tracks_df['Orientation2'].iloc[:n_wrap].values - 3 * np.pi
    })

    s_offset_lap2 = tracks_df['sDistance2'].iloc[-1]
    wrap_lap2 = pd.DataFrame({
        'sDistance2': tracks_df['sDistance2'].iloc[:n_wrap].values + s_offset_lap2,
        'x2': tracks_df['x2'].iloc[:n_wrap].values,
        'y2': tracks_df['y2'].iloc[:n_wrap].values,
        'Orientation2': tracks_df['Orientation2'].iloc[:n_wrap].values - 2 * np.pi
    })

    combined_wrap = pd.concat([wrap_df.reset_index(drop=True), wrap_lap2.reset_index(drop=True)], axis=1)
    for col in tracks_df.columns:
        if col not in combined_wrap.columns:
            combined_wrap[col] = np.nan
    combined_wrap = combined_wrap[tracks_df.columns]
    tracks_df = pd.concat([tracks_df, combined_wrap], ignore_index=True)

    tracks_df = tracks_df.loc[:, ~tracks_df.columns.str.contains('^Unnamed')]
    tracks_df.head()

    return tracks_df, max_sDistance_lap1, max_sDistance_lap2, clockwise

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

class ToolTip:
    def __init__(self, widget, text):
        self.widget = widget
        self.text = text
        self.tip_window = None
        widget.bind("<Enter>", self.show_tip)
        widget.bind("<Leave>", self.hide_tip)

    def show_tip(self, event=None):
        if self.tip_window or not self.text:
            return
        x, y, cx, cy = self.widget.bbox("insert")
        x += self.widget.winfo_rootx() + 20
        y += self.widget.winfo_rooty() + 20
        self.tip_window = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.geometry(f"+{x}+{y}")
        label = tk.Label(tw, text=self.text, justify="left",
                         background="#ffffe0", relief="solid", borderwidth=1,
                         font=("tahoma", "8"))
        label.pack(ipadx=1)

    def hide_tip(self, event=None):
        if self.tip_window:
            self.tip_window.destroy()
            self.tip_window = None

class ConsoleApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ExtDriver Console App")

        # Variables for stopwatch dials
        self.sim_current_time = 0.0  # in seconds
        self.speed_current = 0.0     # in km/h

        # Make the root window resizable
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        # self.root.resizable(False, False)

        # Create a canvas and attach scrollbars
        self.canvas = tk.Canvas(self.root, width=900, height=1050)
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
        self.stop_flag = threading.Event()
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

        # Start the dial update loop
        self.root.after(50, self.update_dials)

    def create_widgets(self):
        self.create_ip_port_input()
        self.create_connection_status()
        self.create_control_buttons()
        self.create_parameter_buttons()
        self.create_socket_output()
        self.create_force_input_buttons()
        self.create_steer_control_parameters()
        self.create_pedal_control_parameters()
        self.create_torque_control_parameters()
        self.create_live_data_frame()
        self.create_console_output()
        self.create_bottom_buttons()

    def disable_widgets(self, frame):
        for child in frame.winfo_children():
            if isinstance(child, (tk.Entry, tk.OptionMenu, tk.Checkbutton, tk.Button)):
                child.config(state="disabled")

    def enable_widgets(self, frame):
        for child in frame.winfo_children():
            if isinstance(child, (tk.Entry, tk.OptionMenu, tk.Checkbutton, tk.Button)):
                child.config(state="normal")

    def create_ip_port_input(self):
        ip_port_frame = tk.Frame(self.scrollable_frame)
        ip_port_frame.grid(row=0, column=0, columnspan=5, sticky="ew")

        tk.Label(ip_port_frame, text="IP:").grid(row=0, column=0, padx=5)
        self.ip_entry = tk.Entry(ip_port_frame, width=15)
        self.ip_entry.grid(row=0, column=1, padx=5)
        # Dropdown for IP selection
        self.ip_entry.insert(0, "127.0.0.1")

        tk.Label(ip_port_frame, text="Port:").grid(row=0, column=2, padx=5)
        self.port_entry = tk.Entry(ip_port_frame, width=10)
        self.port_entry.grid(row=0, column=3, padx=5)
        self.port_entry.insert(0, "6020")

        self.start_server_button = tk.Button(ip_port_frame,text="Start Server & Listen for Incoming Connections", bg="light yellow", command=self.start_server)
        self.start_server_button.grid(row=0, column=4, padx=5)
        self.abort_button = tk.Button(ip_port_frame, text="ABORT", bg="orange", command=self.abort_simulation, state="disabled")
        self.abort_button.grid(row=0, column=5, padx=5)

    def create_connection_status(self):
        self.connection_status = tk.Label(self.scrollable_frame, text="Status: Disconnected", fg="red")
        self.connection_status.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(5, 5))

    def create_control_buttons(self):
        control_button_frame = tk.Frame(self.scrollable_frame)
        control_button_frame.grid(row=7, column=0, columnspan=4, sticky="ew", pady=5)

        button_row = tk.Frame(control_button_frame)
        button_row.pack(anchor="center")

        
        self.start_sim_button = tk.Button(button_row, text="START SIM", bg="green", command=self.start_simulation, state="disabled")
        self.stop_button = tk.Button(button_row, text="STOP SIM", bg="red", command=self.stop_simulation, state="disabled")
        self.reset_button = tk.Button(button_row, text="RESET SIM", bg="light blue", command=self.reset_simulation_button, state="disabled")

        self.start_sim_button.pack(side="left", padx=5, pady=5)
        self.stop_button.pack(side="left", padx=5, pady=5)
        self.reset_button.pack(side="left", padx=5, pady=5)
        

    def create_socket_output(self):
        self.socket_output = tk.Text(self.scrollable_frame, height=6, width=50, bg="black", fg="green")
        self.socket_output.grid(row=3, column=0, columnspan=4, pady=5)

    def create_parameter_buttons(self):
        self.param_frame = tk.Frame(self.scrollable_frame, bd=2, relief=tk.SUNKEN)
        self.param_frame.grid(row=4, column=0, columnspan=2, sticky="ew", pady=5)

        self.param_frame.grid_columnconfigure(0, weight=1)
        self.param_frame.grid_columnconfigure(1, weight=0)
        self.param_frame.grid_columnconfigure(2, weight=0)
        self.param_frame.grid_columnconfigure(3, weight=0)
        self.param_frame.grid_columnconfigure(4, weight=1)

        tk.Label(self.param_frame, text="Parameter Selection:").grid(row=0, column=1, columnspan=3, sticky="ew", pady=5)

        tk.Label(self.param_frame, text="Select Track and Velocity Profile:").grid(row=1, column=1, sticky="w", padx=5)
        self.tracks_var = tk.StringVar(self.param_frame)
        self.tracks_var.set("Select Tracks")
        self.tracks_dropdown = tk.OptionMenu(self.param_frame, self.tracks_var, *self.get_combined_files(), command=self.set_files)
        self.tracks_dropdown.grid(row=1, column=2, columnspan=2, sticky="ew")
        ToolTip(self.tracks_dropdown, "Select a track and velocity profile CSV file")

        tk.Label(self.param_frame, text="Velocity Percentage (%):").grid(row=2, column=1, sticky="w", padx=5)
        self.num_Velocity_var = tk.StringVar(self.param_frame)
        self.num_Velocity_var.set("70")
        self.num_Velocity_dropdown = tk.OptionMenu(self.param_frame, self.num_Velocity_var, *["70", "80", "85", "90", "92", "94", "96", "98", "100"])
        self.num_Velocity_dropdown.grid(row=2, column=2, padx=5, pady=5)
        ToolTip(self.num_Velocity_dropdown, "Set the velocity scaling percentage")

        self.unlimited_laps_var = tk.IntVar()
        self.unlimited_laps_checkbox = tk.Checkbutton(self.param_frame, text="Unlimited Laps", variable=self.unlimited_laps_var)
        self.unlimited_laps_checkbox.grid(row=3, column=1, sticky="w", padx=5)
        ToolTip(self.unlimited_laps_checkbox, "Run the simulation continuously without lap limit")

        tk.Label(self.param_frame, text="Number of Laps:").grid(row=3, column=2, sticky="w", padx=5)
        self.num_laps_entry = tk.Entry(self.param_frame, width=10, state="normal")
        self.num_laps_entry.grid(row=3, column=3, padx=5)
        self.num_laps_entry.insert(0, "1")
        self.unlimited_laps_var.trace_add("write", self.toggle_num_laps_entry)
        ToolTip(self.num_laps_entry, "Set a fixed number of laps for the simulation")

        self.DiL_Mode_var = tk.IntVar(value=1)
        self.DiL_Mode_checkbox = tk.Checkbutton(self.param_frame, text="DiL Mode", variable=self.DiL_Mode_var)
        self.DiL_Mode_checkbox.grid(row=4, column=1, sticky="w", padx=5)
        ToolTip(self.DiL_Mode_checkbox, "Enable Driver-in-the-Loop mode for real-time input")

        self.invert_coords_var = tk.IntVar()
        self.invert_coords_checkbox = tk.Checkbutton(self.param_frame, text="Invert X and Y", variable=self.invert_coords_var)
        self.invert_coords_checkbox.grid(row=4, column=3, sticky="w", padx=5)
        ToolTip(self.invert_coords_checkbox, "Invert reference line X and Y coordinates")

    def toggle_num_laps_entry(self, *args):
        if self.unlimited_laps_var.get() == 1:
            self.num_laps_entry.config(state="disabled")
        else:
            self.num_laps_entry.config(state="normal")


    def create_steer_control_parameters(self):
        self.steer_control_frame = tk.Frame(self.scrollable_frame, bd=2, relief=tk.SUNKEN)
        self.steer_control_frame.grid(row=5, column=0, sticky="nsew", pady=5)

        tk.Label(self.steer_control_frame, text="Steer Control Parameters:").grid(row=0, column=0, columnspan=4, sticky="ew")
        
        tk.Label(self.steer_control_frame, text="Angular Offset Gain:").grid(row=1, column=0, sticky="w")
        self.e_phi_gain_entry = tk.Entry(self.steer_control_frame, width=5)
        self.e_phi_gain_entry.grid(row=1, column=1, padx=5)
        self.e_phi_gain_entry.insert(0, "2.0")
        ToolTip(self.e_phi_gain_entry, "Angular offset gain for heading correction")

        tk.Label(self.steer_control_frame, text="Lateral Offset Gains:").grid(row=2, column=0, sticky="w")
        self.lateral_offset_gains_entries = [tk.Entry(self.steer_control_frame, width=5) for _ in range(8)]
        default_values = [3, 2, 1.3, 0.8, 0.55, 0.32, 0.12, 0.04]
        for i, (entry, value) in enumerate(zip(self.lateral_offset_gains_entries, default_values)):
            entry.grid(row=2, column=i + 1, padx=2)
            entry.insert(0, str(value))
        for entry in self.lateral_offset_gains_entries:
            ToolTip(entry, "Gain for lateral offset correction by lookahead distance")

        tk.Label(self.steer_control_frame, text="Individual Lateral Offset Saturations:").grid(row=3, column=0, sticky="w")
        self.individual_lateral_offset_saturations_entries = [tk.Entry(self.steer_control_frame, width=5) for _ in range(8)]
        default_values = [10, 10, 2, 2.75, 3, 2.75, 2.25, 1.7]
        for i, (entry, value) in enumerate(zip(self.individual_lateral_offset_saturations_entries, default_values)):
            entry.grid(row=3, column=i + 1, padx=2)
            entry.insert(0, str(value))
        for entry in self.individual_lateral_offset_saturations_entries:
            ToolTip(entry, "Limits for individual lateral offset contribution")

        tk.Label(self.steer_control_frame, text="Total Lateral Offset Saturations:").grid(row=4, column=0, sticky="w")
        self.total_lateral_lever_saturations_entry = tk.Entry(self.steer_control_frame, width=5)
        self.total_lateral_lever_saturations_entry.grid(row=4, column=1, padx=5)
        self.total_lateral_lever_saturations_entry.insert(0, "25")

        tk.Label(self.steer_control_frame, text="Steering Control Saturations:").grid(row=5, column=0, sticky="w")
        self.steering_control_saturations_entry = tk.Entry(self.steer_control_frame, width=5)
        self.steering_control_saturations_entry.grid(row=5, column=1, padx=5)
        self.steering_control_saturations_entry.insert(0, "28")

        ToolTip(self.total_lateral_lever_saturations_entry, "Total saturation limit for lateral offset control")
        ToolTip(self.steering_control_saturations_entry, "Maximum output value from the steering control algorithm")
        

    def create_force_input_buttons(self):
        self.force_input_frame = tk.Frame(self.scrollable_frame, bd=2, relief=tk.SUNKEN)
        self.force_input_frame.grid(row=5, column=1, sticky="nsew", pady=5)

        tk.Label(self.force_input_frame, text="Force Driver Inputs:").grid(row=0, column=0, columnspan=4, sticky="ew")

        tk.Label(self.force_input_frame, text="Steering Angle:").grid(row=1, column=0, sticky="w")
        self.steering_angle_entry = tk.Entry(self.force_input_frame, width=10)
        self.steering_angle_entry.grid(row=1, column=1, padx=5)
        self.steering_angle_entry.insert(0, "0")
        ToolTip(self.steering_angle_entry, "Force Steering input in degrees (- for left, + for right)")

        self.force_dil_labels = []
        self.force_dil_fields = []

        label = tk.Label(self.force_input_frame, text="Accelerator Pedal (%):")
        label.grid(row=2, column=0, sticky="w")
        self.force_dil_labels.append(label)
        self.accelerator_pedal_entry = tk.Entry(self.force_input_frame, width=10)
        self.accelerator_pedal_entry.grid(row=2, column=1, padx=5)
        self.accelerator_pedal_entry.insert(0, "0")
        ToolTip(self.accelerator_pedal_entry, "Force accelerator pedal position (0–100%)")

        label = tk.Label(self.force_input_frame, text="Brake Pressure (bar):")
        label.grid(row=2, column=2, sticky="w")
        self.force_dil_labels.append(label)
        self.brake_pressure_entry = tk.Entry(self.force_input_frame, width=10)
        self.brake_pressure_entry.grid(row=2, column=3, padx=5)
        self.brake_pressure_entry.insert(0, "0")
        ToolTip(self.brake_pressure_entry, "Force brake pressure in bars")

        # --- Force Input Fields for Dev Mode ---
        self.force_dev_fields = []
        self.force_dev_labels = []

        label = tk.Label(self.force_input_frame, text="MDriveFL:")
        label.grid(row=2, column=0, sticky="w")
        self.force_dev_labels.append(label)
        self.MDriveFL_entry = tk.Entry(self.force_input_frame, width=10)
        self.MDriveFL_entry.grid(row=2, column=1, padx=5)
        self.MDriveFL_entry.insert(0, "0")
        self.force_dev_fields.extend([self.MDriveFL_entry])

        label = tk.Label(self.force_input_frame, text="MDriveFR:")
        label.grid(row=2, column=2, sticky="w")
        self.force_dev_labels.append(label)
        self.MDriveFR_entry = tk.Entry(self.force_input_frame, width=10)
        self.MDriveFR_entry.grid(row=2, column=3, padx=5)
        self.MDriveFR_entry.insert(0, "0")
        self.force_dev_fields.extend([self.MDriveFR_entry])

        label = tk.Label(self.force_input_frame, text="MDriveRL:")
        label.grid(row=3, column=0, sticky="w")
        self.force_dev_labels.append(label)
        self.MDriveRL_entry = tk.Entry(self.force_input_frame, width=10)
        self.MDriveRL_entry.grid(row=3, column=1, padx=5)
        self.MDriveRL_entry.insert(0, "0")
        self.force_dev_fields.extend([self.MDriveRL_entry])

        label = tk.Label(self.force_input_frame, text="MDriveRR:")
        label.grid(row=3, column=2, sticky="w")
        self.force_dev_labels.append(label)
        self.MDriveRR_entry = tk.Entry(self.force_input_frame, width=10)
        self.MDriveRR_entry.grid(row=3, column=3, padx=5)
        self.MDriveRR_entry.insert(0, "0")
        self.force_dev_fields.extend([self.MDriveRR_entry])

        label = tk.Label(self.force_input_frame, text="MBrakeFL:")
        label.grid(row=4, column=0, sticky="w")
        self.force_dev_labels.append(label)
        self.MBrakeFL_entry = tk.Entry(self.force_input_frame, width=10)
        self.MBrakeFL_entry.grid(row=4, column=1, padx=5)
        self.MBrakeFL_entry.insert(0, "0")
        self.force_dev_fields.extend([self.MBrakeFL_entry])

        label = tk.Label(self.force_input_frame, text="MBrakeFR:")
        label.grid(row=4, column=2, sticky="w")
        self.force_dev_labels.append(label)
        self.MBrakeFR_entry = tk.Entry(self.force_input_frame, width=10)
        self.MBrakeFR_entry.grid(row=4, column=3, padx=5)
        self.MBrakeFR_entry.insert(0, "0")
        self.force_dev_fields.extend([self.MBrakeFR_entry])

        # Hide Dev fields if DiL_Mode is checked at startup
        self.update_force_dev_fields_visibility()

        # Attach callback to DiL_Mode_var to update visibility
        self.DiL_Mode_var.trace_add("write", lambda *args: self.update_force_dev_fields_visibility())

        self.force_button = tk.Button(self.force_input_frame, text="FORCE", command=self.on_force_clicked)
        self.disble_button = tk.Button(self.force_input_frame, text="DISABLE", command=self.on_disable_clicked)
        self.force_button.grid(row=5, column=0, columnspan=2, pady=5)
        self.disble_button.grid(row=5, column=2, columnspan=2, pady=5)
        ToolTip(self.force_button, "Send forced input values to the simulation")
        ToolTip(self.disble_button, "Stop forcing inputs and return to V3M control")


    def create_pedal_control_parameters(self):
        self.pedal_control_frame = tk.Frame(self.scrollable_frame, bd=2, relief=tk.SUNKEN)
        self.pedal_control_frame.grid(row=6, column=0, sticky="nsew", pady=5)

        tk.Label(self.pedal_control_frame, text="Pedal Control Parameters:").grid(row=0, column=0, columnspan=4, sticky="ew")

        tk.Label(self.pedal_control_frame, text="Throttle Kp:").grid(row=1, column=0, sticky="w")
        self.throttle_Kp_entry = tk.Entry(self.pedal_control_frame, width=5)
        self.throttle_Kp_entry.grid(row=1, column=1, padx=5)
        self.throttle_Kp_entry.insert(0, "5.0")

        tk.Label(self.pedal_control_frame, text="Throttle Ki:").grid(row=2, column=0, sticky="w")
        self.throttle_Ki_entry = tk.Entry(self.pedal_control_frame, width=5)
        self.throttle_Ki_entry.grid(row=2, column=1, padx=5)
        self.throttle_Ki_entry.insert(0, "2.0")

        tk.Label(self.pedal_control_frame, text="Throttle Kd:").grid(row=3, column=0, sticky="w")
        self.throttle_Kd_entry = tk.Entry(self.pedal_control_frame, width=5)
        self.throttle_Kd_entry.grid(row=3, column=1, padx=5)
        self.throttle_Kd_entry.insert(0, "0.0")

        ToolTip(self.throttle_Kp_entry, "Proportional gain for throttle control")
        ToolTip(self.throttle_Ki_entry, "Integral gain for throttle control")
        ToolTip(self.throttle_Kd_entry, "Derivative gain for throttle control")

        tk.Label(self.pedal_control_frame, text="Brake Kp:").grid(row=1, column=2, sticky="w")
        self.brake_Kp_entry = tk.Entry(self.pedal_control_frame, width=5)
        self.brake_Kp_entry.grid(row=1, column=3, padx=5)
        self.brake_Kp_entry.insert(0, "20.0")

        tk.Label(self.pedal_control_frame, text="Brake Ki:").grid(row=2, column=2, sticky="w")
        self.brake_Ki_entry = tk.Entry(self.pedal_control_frame, width=5)
        self.brake_Ki_entry.grid(row=2, column=3, padx=5)
        self.brake_Ki_entry.insert(0, "5.0")

        tk.Label(self.pedal_control_frame, text="Brake Kd:").grid(row=3, column=2, sticky="w")
        self.brake_Kd_entry = tk.Entry(self.pedal_control_frame, width=5)
        self.brake_Kd_entry.grid(row=3, column=3, padx=5)
        self.brake_Kd_entry.insert(0, "0.0")

        ToolTip(self.brake_Kp_entry, "Proportional gain for brake control")
        ToolTip(self.brake_Ki_entry, "Integral gain for brake control")
        ToolTip(self.brake_Kd_entry, "Derivative gain for brake control")

        tk.Label(self.pedal_control_frame, text="Future TD:").grid(row=7, column=0, sticky="w")
        self.futureTD = tk.Entry(self.pedal_control_frame, width=5)
        self.futureTD.grid(row=7, column=1, padx=5)
        self.futureTD.insert(0, "5")
        ToolTip(self.futureTD, "Target distance used for feedforward throttle prediction in metres")

    def create_torque_control_parameters(self):

        self.torque_control_frame = tk.Frame(self.scrollable_frame, bd=2, relief=tk.SUNKEN)
        self.torque_control_frame.grid(row=6, column=1, sticky="nsew", pady=5)

        tk.Label(self.torque_control_frame, text="Torque Control Parameters:").grid(row=0, column=0, columnspan=4, sticky="ew")

        tk.Label(self.torque_control_frame, text="Max Engine Brake Torque:").grid(row=1, column=0, sticky="w")
        self.max_engine_brake_torque_entry = tk.Entry(self.torque_control_frame, width=5)
        self.max_engine_brake_torque_entry.grid(row=1, column=1, padx=5)
        self.max_engine_brake_torque_entry.insert(0, "0.1")

        tk.Label(self.torque_control_frame, text="Differential Gain 1:").grid(row=1, column=2, sticky="w")
        self.differential_gain_1_entry = tk.Entry(self.torque_control_frame, width=5)
        self.differential_gain_1_entry.grid(row=1, column=3, padx=5)
        self.differential_gain_1_entry.insert(0, "0.1")

        tk.Label(self.torque_control_frame, text="Driving Differential Gain:").grid(row=2, column=0, sticky="w")
        self.driving_differential_gain_entry = tk.Entry(self.torque_control_frame, width=5)
        self.driving_differential_gain_entry.grid(row=2, column=1, padx=5)
        self.driving_differential_gain_entry.insert(0, "0.1")

        tk.Label(self.torque_control_frame, text="Overrun Differential Gain:").grid(row=2, column=2, sticky="w")
        self.overrun_differential_gain_entry = tk.Entry(self.torque_control_frame, width=5)
        self.overrun_differential_gain_entry.grid(row=2, column=3, padx=5)
        self.overrun_differential_gain_entry.insert(0, "0.1")

        tk.Label(self.torque_control_frame, text="Max Drive Torque:").grid(row=3, column=0, sticky="w")
        self.max_drive_torque_entry = tk.Entry(self.torque_control_frame, width=5)
        self.max_drive_torque_entry.grid(row=3, column=1, padx=5)
        self.max_drive_torque_entry.insert(0, "3500")

        tk.Label(self.torque_control_frame, text="Max Brake Torque:").grid(row=3, column=2, sticky="w")
        self.max_brake_torque_entry = tk.Entry(self.torque_control_frame, width=5)
        self.max_brake_torque_entry.grid(row=3, column=3, padx=5)
        self.max_brake_torque_entry.insert(0, "3500")

        ToolTip(self.max_engine_brake_torque_entry, "*OFFLINE TESTING ONLY* Maximum torque applied by engine braking")
        ToolTip(self.differential_gain_1_entry, "*OFFLINE TESTING ONLY* Gain for balancing torque between rear wheels")
        ToolTip(self.driving_differential_gain_entry, "*OFFLINE TESTING ONLY* Gain for distributing torque while accelerating")
        ToolTip(self.overrun_differential_gain_entry, "*OFFLINE TESTING ONLY* Gain for differential behavior during deceleration")
        ToolTip(self.max_drive_torque_entry, "Maximum driveshaft torque the V3M is allowed to deliver")
        ToolTip(self.max_brake_torque_entry, "Maximum brake torque the V3M is allowed to apply")

    def update_force_dev_fields_visibility(self):
        if self.DiL_Mode_var.get() == 1:
            # Hide labels and entries
            for label in self.force_dev_labels:
                label.grid_remove()
            for entry in self.force_dev_fields:
                entry.grid_remove()
            for label in self.force_dil_labels:
                label.grid()
            for entry in self.force_dil_fields:
                entry.grid()
        else:
            for label in self.force_dev_labels:
                label.grid()
            for entry in self.force_dev_fields:
                entry.grid()

            for label in self.force_dil_labels:
                label.grid_remove()
            for entry in self.force_dil_fields:
                entry.grid_remove()

    def create_live_data_frame(self):
        self.live_data_frame = tk.Frame(self.scrollable_frame, bd=2, relief=tk.SUNKEN)
        self.live_data_frame.grid(row=5, column=0, sticky="nsew", pady=5)
        self.live_data_frame.grid_remove()

        tk.Label(self.live_data_frame, text="Live Data:").grid(row=0, column=0, columnspan=4, sticky="ew")

        # Add live data labels and entries here as needed
        # Example:
        self.speed_label_var = tk.StringVar(value="Speed: 0 km/h")
        tk.Label(self.live_data_frame, textvariable=self.speed_label_var, font=("Arial", 12)).grid(row=1, column=0, sticky="w")

        self.lap_label_var = tk.StringVar(value="Lap: 0")
        tk.Label(self.live_data_frame, textvariable=self.lap_label_var, font=("Arial", 12)).grid(row=2, column=0, sticky="w")
        self._current_nLap = 0  # Internal variable to store current lap

        self.lap_completion_label_var = tk.StringVar(value="Lap Completion: 0%")
        tk.Label(self.live_data_frame, textvariable=self.lap_completion_label_var, font=("Arial", 12)).grid(row=3, column=0, sticky="w")
        self.nLap_Completion = 0  # Internal variable to store current lap

        # Progress bar for lap completion
        self.lap_completion_progress = ttk.Progressbar(self.live_data_frame, orient="horizontal", length=180, mode="determinate")
        self.lap_completion_progress.grid(row=4, column=0, columnspan=3, sticky="ew", padx=5, pady=(0, 5))
        self.lap_completion_progress["maximum"] = 100
        self.lap_completion_progress["value"] = 0

        # --- Integrated Stopwatch Dials ---

        # Sim Time Dial Canvas
        self.sim_canvas = tk.Canvas(
            self.live_data_frame, width=200, height=200, bg="white"
        )
        self.sim_canvas.grid(row=1, column=1, rowspan=2, padx=10, pady=5)
        self.sim_center_x, self.sim_center_y = 100, 100
        self.sim_radius = 80
        self.draw_watch_face(self.sim_canvas, self.sim_center_x, self.sim_center_y, self.sim_radius)

        # Digital Sim Time Label
        self.sim_time_label = ttk.Label(
            self.live_data_frame, text="00:00.000", font=("Helvetica", 14)
        )
        self.sim_time_label.grid(row=3, column=1)

        # Speed Dial Canvas
        self.speed_canvas = tk.Canvas(
            self.live_data_frame, width=200, height=200, bg="white"
        )
        self.speed_canvas.grid(row=1, column=2, rowspan=2, padx=10, pady=5)
        self.speed_center_x, self.speed_center_y = 100, 100
        self.speed_radius = 80
        self.draw_speed_dial_face(self.speed_canvas, self.speed_center_x, self.speed_center_y, self.speed_radius)

        # Digital Speed Label
        self.speed_digital_label = ttk.Label(
            self.live_data_frame, text="0.00 km/h", font=("Helvetica", 14)
        )
        self.speed_digital_label.grid(row=3, column=2)

    def update_lap_label(self, nLap):
        nLap = nLap + 1
        self._current_nLap = nLap
        self.lap_label_var.set(f"Lap: {nLap}")

    def update_speed_label(self, speed_kph):
        self.speed_label_var.set(f"Speed: {speed_kph:.2f} km/h")

    def update_nLap_completion_label(self, nLap_Completion):
        self.lap_completion_label_var.set(f"Lap Completion: {nLap_Completion:.2f}%")
        self.lap_completion_progress["value"] = nLap_Completion
        self.nLap_Completion = nLap_Completion
    
    def create_console_output(self):
        self.console_output = tk.Text(self.scrollable_frame, height=23, width=50, bg="black", fg="white")
        self.console_output.grid(row=8, column=0, columnspan=4, pady=5)

    def create_bottom_buttons(self):
        bottom_button_frame = tk.Frame(self.scrollable_frame)
        bottom_button_frame.grid(row=9, column=0, columnspan=4, sticky="ew", pady=5)

        # tk.Button(bottom_button_frame, text="Save Graphs", command=self.save_all_graphs).grid(row=0, column=2, padx=5, pady=5)
        tk.Button(bottom_button_frame, text="Show Graphs", command=self.show_graphs_from_folder).grid(row=0, column=0, padx=5, pady=5)


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
        self.start_server_button.config(state="disabled")
        self.abort_button.config(state="normal")
        if not self.server_task.is_running():
            self.server_task.start()

    def run_server(self):
        if self.abort_flag.is_set():
            self._append_to_socket_output_safe("Server aborted.\n")
            return
        try:
            print("Starting server...")
            SERVER_IP = self.ip_entry.get()
            SERVER_PORT = int(self.port_entry.get())

            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.server_socket.bind((SERVER_IP, SERVER_PORT))
            self._update_connection_status_safe("Listening")
            self._append_to_socket_output_safe("Server started and listening for incoming connections...\n")

            # Check abort_flag before entering the loop
            while not self.abort_flag.is_set():
                try:
                    self.server_socket.settimeout(1.0)  # Short timeout to check abort_flag regularly
                    data, self.client_address = self.server_socket.recvfrom(1024)
                    if data:
                        self._append_to_socket_output_safe(f"Client connected from {self.client_address}\n")
                        self._update_connection_status_safe("Connected")
                        self.start_sim_button.config(state="normal")
                        break
                    
                except socket.timeout:
                    continue
                    
        except Exception as e:
            self._append_to_socket_output_safe(f"Error starting server: {e}\n") # Server WinError found here
        print("Server stopped.")

    def start_simulation(self):
        if not self.client_address:
            self._append_to_console_output_safe("No client connected. Start the server first.\n")
            return
        # if not self.tracks_file or not self.velocity_profile_file:
        #     self._append_to_console_output_safe("Error: Please select both a track and a velocity profile file.\n")
        #     return
        self._append_to_console_output_safe("Start SIM pressed\n")
        self.disable_widgets(self.param_frame)
        self.disable_widgets(self.steer_control_frame)
        self.disable_widgets(self.pedal_control_frame)
        self.steer_control_frame.grid_remove()
        self.pedal_control_frame.grid_remove()
        # self.force_input_frame.grid_remove()
        self.torque_control_frame.grid_remove()
        self.live_data_frame.grid()
        self.start_server_button.config(state="disabled")
        self.start_sim_button.config(state="disabled")
        self.reset_button.config(state="disabled")
        self.stop_button.config(state="normal")
        if not self.simulation_task.is_running():
            self.simulation_task.start()
        if self.simulation_task.is_running():
            self.stop_flag.clear()

    def stop_simulation(self):
        self._append_to_console_output_safe("Stop SIM pressed\n")
        self.enable_widgets(self.param_frame)
        self.enable_widgets(self.steer_control_frame)
        self.enable_widgets(self.pedal_control_frame)
        self.enable_widgets(self.force_input_frame)
        self.enable_widgets(self.torque_control_frame)
        self.live_data_frame.grid_remove()
        self.steer_control_frame.grid()
        self.pedal_control_frame.grid()
        # self.force_input_frame.grid()
        self.torque_control_frame.grid()
        self.start_sim_button.config(state="normal")
        self.reset_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.abort_button.config(state="disabled")
        self.stop_flag.set()
        # if not self.simulation_task.is_running():
        #     self.abort_flag.clear()
        # if self.simulation_task.is_running():
        #     self.simulation_task.stop()
        
    def abort_simulation(self):
        self._append_to_console_output_safe("Abort SIM pressed\n")
        self.abort_flag.set()
        self._update_connection_status_safe("Disconnected")
        self.live_data_frame.grid_remove()
        self.enable_widgets(self.param_frame)
        self.enable_widgets(self.steer_control_frame)
        self.enable_widgets(self.pedal_control_frame)
        self.enable_widgets(self.force_input_frame)
        self.enable_widgets(self.torque_control_frame)
        self.steer_control_frame.grid()
        self.pedal_control_frame.grid()
        self.force_input_frame.grid()
        self.torque_control_frame.grid()
        self.start_server_button.config(state="normal")
        self.start_sim_button.config(state="normal")
        self.reset_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.abort_button.config(state="disabled")
        if not self.simulation_task.is_running():
            self.server_socket.close()
            self.abort_flag.clear()
        if self.simulation_task.is_running():
            self.simulation_task.stop()   
        if self.server_task.is_running():
            self.server_task.stop()


    def reset_simulation_button(self):
        self._append_to_console_output_safe("Reset SIM pressed\n")
        self.start_server_button.config(state="normal")
        self.start_sim_button.config(state="disabled")
        self.abort_button.config(state="disabled")
        self.abort_button.config(state="disabled")
        if not self.reset_task.is_running():
            self.reset_task.start()
            self.reset_task.stop()

    def reset_simulation_task(self):
        self.reset_flag.set()
        self._append_to_console_output_safe("Simulation reset initiated.\n")
        # self.reset_GUI_variables()
        self.abort_flag.set()
        self._append_to_console_output_safe("Simulation reset completed.\n")
        self.reset_flag.clear()

    def reset_GUI_variables(self):
        # Reset all the variables used in the simulation to their default values

        # Track and velocity profile selection
        self.tracks_var.set("Select Tracks")
        self.num_Velocity_var.set("70")

        # Unlimited laps and number of laps
        self.unlimited_laps_var.set(0)
        self.num_laps_entry.config(state="normal")
        self.num_laps_entry.delete(0, tk.END)
        self.num_laps_entry.insert(0, "1")

        # DiL Mode and invert coordinates
        self.DiL_Mode_var.set(1)
        self.invert_coords_var.set(0)

        # Steer control parameters
        self.e_phi_gain_entry.delete(0, tk.END)
        self.e_phi_gain_entry.insert(0, "2.0")
        for entry, value in zip(self.lateral_offset_gains_entries, [3, 2, 1.3, 0.8, 0.55, 0.32, 0.12, 0.04]):
            entry.delete(0, tk.END)
            entry.insert(0, str(value))
        for entry, value in zip(self.individual_lateral_offset_saturations_entries, [10, 10, 2, 2.75, 3, 2.75, 2.25, 1.7]):
            entry.delete(0, tk.END)
            entry.insert(0, str(value))
        self.total_lateral_lever_saturations_entry.delete(0, tk.END)
        self.total_lateral_lever_saturations_entry.insert(0, "25")
        self.steering_control_saturations_entry.delete(0, tk.END)
        self.steering_control_saturations_entry.insert(0, "28")

        # Force driver inputs
        self.steering_angle_entry.delete(0, tk.END)
        self.steering_angle_entry.insert(0, "0")
        self.accelerator_pedal_entry.delete(0, tk.END)
        self.accelerator_pedal_entry.insert(0, "0")
        self.brake_pressure_entry.delete(0, tk.END)
        self.brake_pressure_entry.insert(0, "0")

        # Pedal control parameters
        self.throttle_Kp_entry.delete(0, tk.END)
        self.throttle_Kp_entry.insert(0, "5.0")
        self.throttle_Ki_entry.delete(0, tk.END)
        self.throttle_Ki_entry.insert(0, "2.0")
        self.throttle_Kd_entry.delete(0, tk.END)
        self.throttle_Kd_entry.insert(0, "0.0")
        self.brake_Kp_entry.delete(0, tk.END)
        self.brake_Kp_entry.insert(0, "20.0")
        self.brake_Ki_entry.delete(0, tk.END)
        self.brake_Ki_entry.insert(0, "5.0")
        self.brake_Kd_entry.delete(0, tk.END)
        self.brake_Kd_entry.insert(0, "0.0")
        self.futureTD.delete(0, tk.END)
        self.futureTD.insert(0, "5")

        # Torque control parameters
        self.max_engine_brake_torque_entry.delete(0, tk.END)
        self.max_engine_brake_torque_entry.insert(0, "0.1")
        self.differential_gain_1_entry.delete(0, tk.END)
        self.differential_gain_1_entry.insert(0, "0.1")
        self.driving_differential_gain_entry.delete(0, tk.END)
        self.driving_differential_gain_entry.insert(0, "0.1")
        self.overrun_differential_gain_entry.delete(0, tk.END)
        self.overrun_differential_gain_entry.insert(0, "0.1")
        self.max_drive_torque_entry.delete(0, tk.END)
        self.max_drive_torque_entry.insert(0, "3500")
        self.max_brake_torque_entry.delete(0, tk.END)
        self.max_brake_torque_entry.insert(0, "3500")


    def _refresh_console(self):
        self._clear_console_output_safe()

    def validate_csv_files(self):
        self._append_to_console_output_safe(f"Validating selected files...\n")
        if not self.tracks_file or not self.velocity_profile_file:
            self._append_to_console_output_safe("Error: Please select both a tracks file and a velocity profile file.\n")
            return False
        try:
            self._append_to_console_output_safe("Reading Files...\n")
            self.scrollable_frame.after(0, pd.read_csv, self.tracks_file)
            self.scrollable_frame.after(0, pd.read_csv, self.velocity_profile_file)
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
            # traj_files = [file for file in os.listdir('Trajectories') if file.endswith('.xlsx') or file.endswith('.xls')]
            # vel_files = [file for file in os.listdir('Velocities') if file.endswith('.xlsx') or file.endswith('.xls')]
            traj_files = [file for file in os.listdir('Trajectories') if file.endswith('.csv') or file.endswith('.xls')]
            vel_files = [file for file in os.listdir('Velocities') if file.endswith('.csv') or file.endswith('.xls')]
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

    
    def run_backend_script(self):
        try:
            i = 0
            nLap = 0
            lastNLap = 0
            LapCompleted = False
            nLap_Completion = 0
            force_controls = False
            StopFlag = False
            rAccel = 0
            nLaps = []
            last_e_phi = 0
            refDistance = 0
            lastRefDistance = 0
            yawAngle = 0
            x = 0
            y = 0
            z = 0
            Vx = 0
            Vy = 0
            lastAvx = 0
            throttle_integral = 0
            brake_integral = 0
            throttle_prev_error = 0
            brake_prev_error = 0

            # Initialize the variables here
            steering_input = 0
            MDriveRL = 0
            MDriveRR = 0
            MBrakeFL = 0
            MBrakeFR = 0
            MBrakeRL = 0
            MBrakeRR = 0
            rAccel = 0
            brakePressure = 0
            simTime = 0

            MDriveRL_array = []
            MDriveRR_array = []
            MBrakeFL_array = []
            MBrakeFR_array = []
            MBrakeRL_array = []
            MBrakeRR_array = []
            tVelocity_array = []
            acc_array = []
            bPressure_array = []
            nMGU_array = []
            t_P_array = []
            t_I_array = []
            t_D_array = []
            b_P_array = []
            b_I_array = []
            b_D_array = []
            vCar_error_array = []
            slipThrust_array = []
            oldThrust_array = []
            vCarArray = []
            scaling_factor_array = []

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
                nonlocal steering_input, MDriveFL, MDriveFR, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR
                nonlocal MDriveRL_array, MDriveRR_array, MBrakeFL_array, MBrakeFR_array, MBrakeRL_array, MBrakeRR_array
                nonlocal t_P_array, t_I_array, t_D_array, b_P_array, b_I_array, b_D_array, vCar_error_array
                nonlocal tVelocity_array, acc_array, bPressure_array, nMGU_array, oldThrust_array
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

                MDriveRL_array.clear()
                MDriveRR_array.clear()
                MBrakeFL_array.clear()
                MBrakeFR_array.clear()
                MBrakeRL_array.clear()
                MBrakeRR_array.clear()
                tVelocity_array.clear()
                acc_array.clear()
                bPressure_array.clear()
                nMGU_array.clear()
                t_P_array.clear()
                t_I_array.clear()
                t_D_array.clear()
                b_P_array.clear()
                b_I_array.clear()
                b_D_array.clear()
                vCar_error_array.clear()
                oldThrust_array.clear()
                scaling_factor_array.clear()

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

            if not self.validate_csv_files():
                return

            self._update_connection_status_safe("Connected")
            self._append_to_console_output_safe("Starting simulation...\n")
            tracks_df = pd.read_csv(self.tracks_file)
            velocity_profile_df = pd.read_csv(self.velocity_profile_file)
            throttle_Kp = float(self.throttle_Kp_entry.get())
            throttle_Ki = float(self.throttle_Ki_entry.get())
            throttle_Kd = float(self.throttle_Kd_entry.get())
            brake_Kp = float(self.brake_Kp_entry.get())
            brake_Ki = float(self.brake_Ki_entry.get())
            brake_Kd = float(self.brake_Kd_entry.get())
            max_engine_brake_torque = float(self.max_engine_brake_torque_entry.get())
            differential_gain_1 = float(self.differential_gain_1_entry.get())
            driving_differential_gain = float(self.driving_differential_gain_entry.get())
            overrun_differential_gain = float(self.overrun_differential_gain_entry.get())
            max_drive_torque = float(self.max_drive_torque_entry.get())
            max_brake_torque = float(self.max_brake_torque_entry.get())
            futureTD = float(self.futureTD.get())
            velocity_percentage = int(self.num_Velocity_var.get())
            velocity_profile_df['vCar'] = velocity_profile_df['vCar'] * velocity_percentage / 100
            velocity_profile_df['vCar2'] = velocity_profile_df['vCar2'] * velocity_percentage / 100
            invert_x_y = float(self.invert_coords_var.get())
            e_phi_gain = float(self.e_phi_gain_entry.get())
            e_l_gains = [float(entry.get()) for entry in self.lateral_offset_gains_entries]
            e_l_limits = [float(entry.get()) for entry in self.individual_lateral_offset_saturations_entries]
            total_e_l_limits = float(self.total_lateral_lever_saturations_entry.get())
            steer_limits = float(self.steering_control_saturations_entry.get())
            configured_laps = 0
            if self.unlimited_laps_var.get() == 0:
                configured_laps = int(self.num_laps_entry.get())
                self._append_to_console_output_safe(f"Laps Initiated: {configured_laps}\n")
            else:
                configured_laps = float('inf')
                self._append_to_console_output_safe(f"Laps Initiated: {configured_laps}\n")

            if self.DiL_Mode_var.get() == 0:
                Dil_Mode_enabled = False
                self._append_to_console_output_safe("DiL Mode Disabled\n")
            else:
                Dil_Mode_enabled = True
                self._append_to_console_output_safe("DiL Mode Enabled\n")
                

            log_folder = "SimulationLogs"
            if not os.path.exists(log_folder):
                os.makedirs(log_folder)
            current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            base_name = os.path.splitext(os.path.basename(self.tracks_file))[0]

            while not self.abort_flag.is_set():
                self._append_to_console_output_safe("Waiting for a client to connect...\n")

                try:
                    while nLap < configured_laps:
                        i += 1
                        if i:
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

                            if Dil_Mode_enabled:
                                data_to_send = struct.pack('fff', steering_input, rAccel, brakePressure)
                            else:
                                data_to_send = struct.pack('fffffff', steering_input, MDriveFL, MDriveFR, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR)

                            try:
                                self.server_socket.sendto(data_to_send, self.client_address)
                                data, _ = self.server_socket.recvfrom(1024)
                                received_length = len(data)
                                expected_length = 17 * 4  # 16 floats, each 4 bytes

                                if received_length == expected_length:
                                    num_floats = 17
                                    float_data = struct.unpack('f' * num_floats, data[:num_floats * 4])

                                    (x, y, z, Vx, Vy, Vz, yawAngle, SlipFL, SlipFR, SlipRL, SlipRR, WheelVelocityFL, WheelVelocityFR, WheelVelocityRL, WheelVelocityRR, nMGU, newSimTime) = float_data

                                    output_text = (
                                        f"Received data:\n"
                                        f"x = {x}\n"
                                        f"y = {y}\n"
                                        f"z = {z}\n"
                                        f"Vx = {Vx}\n"
                                        f"Vy = {Vy}\n"
                                        f"Vz = {Vz}\n"
                                        f"yawAngle = {yawAngle}\n"
                                        f"nMGU = {nMGU}\n"
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
                                if Dil_Mode_enabled == True:
                                    tracks_df, max_sDistance_Lap1, max_sDistance_Lap2, clockwise = generate_driving_line_DiL(tracks_df, yawAngle)
                                if Dil_Mode_enabled == False:
                                    tracks_df, max_sDistance_Lap1, max_sDistance_Lap2, clockwise = generate_driving_line_Dev(tracks_df, x, y, yawAngle)
                                velocity_profile_df['sDistance'] = velocity_profile_df['sDistance'] - velocity_profile_df['sDistance'].iloc[0]
                                velocity_profile_df['sDistance2'] = velocity_profile_df['sDistance2'] - velocity_profile_df['sDistance2'].iloc[0]

                                print(tracks_df.head())
                                print(tracks_df.tail(36))
                                newSim = True

                        if invert_x_y:
                            x = -x
                            y = -y

                        x_positions.append(x)
                        y_positions.append(y)

                        if yawAngles:
                            last_yawAngle = yawAngles[-1]
                        else:
                            last_yawAngle = yawAngle

                        self._append_to_console_output_safe(f"Laps Completed: {nLap}\n")
                        if lastNLap < nLap:
                            LapCompleted = True
                        lastNLap = nLap

                        if nLap == 0:
                            nLap_Completion = (refDistance/max_sDistance_Lap1) * 100
                        else:
                            nLap_Completion = (refDistance/max_sDistance_Lap2) * 100


                        yawAngle, LapCompleted = adjust_numbers(last_yawAngle, yawAngle, LapCompleted, nLap)
                        yawAngles.append(yawAngle)

                        Vx = abs(Vx)
                        Vy = abs(Vy)

                        vCar = np.sqrt(Vx ** 2 + Vy ** 2)
                        vCarArray.append(vCar)

                        plot_Vx = Vx * 3.6
                        velocity_X.append(plot_Vx)

                    
                        speed_kph = vCar * 3.6
                        self.speed_current = speed_kph

                        timeStep = newSimTime - simTime
                        simTime = newSimTime

                        self.update_lap_label(nLap)
                        self.update_speed_label(speed_kph)
                        self.update_nLap_completion_label(nLap_Completion)

                        # Update simulation time and speed for dials
                        self.sim_current_time = simTime

                        steering_inputs.append(steering_input)
                        steering_input, e_phi, saturated_e_l, levercoordsplot, idealcoordsplot, error_values, refLine_orientation, refDistance, newSim, nLap = calculate_steering_control(
                            tracks_df, Vx, Vy, yawAngle, x, y, e_phi_gain, timeStep, nLap, newSim, e_l_gains, e_l_limits, total_e_l_limits, steer_limits, Dil_Mode_enabled, max_sDistance_Lap1, max_sDistance_Lap2, clockwise)
                        
                        print(f"Ref Distance: {refDistance}")

                        if Dil_Mode_enabled == False:
                            steering_input = -steering_input

                        e_phi_values.append(e_phi)
                        # if e_phi > 4 or e_phi < -4:
                        #     self._append_to_console_output_safe(f"Vehicle has left the track\n")
                        #     break

                        for j in range(8):
                            saturated_e_l_values[j].append(saturated_e_l[j])
                        lever_coordinates.append(levercoordsplot)
                        ideal_coordinates.append(idealcoordsplot)
                        refLine_orientations.append(refLine_orientation)

                        MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, rAccel, brakePressure, throttle_integral, throttle_prev_error, brake_integral, brake_prev_error, targetVelocity, t_P, t_I, t_D, b_P, b_I, b_D, velocity_error = calculatePedalPercent(tracks_df, velocity_profile_df, 
                        Vx, Vy, timeStep, refDistance, nLap, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error, brake_Kp, brake_Ki, brake_Kd, brake_integral, brake_prev_error, nMGU, WheelVelocityRL, WheelVelocityRR,
                        max_engine_brake_torque, differential_gain_1, driving_differential_gain, overrun_differential_gain, max_drive_torque, max_brake_torque, futureTD, StopFlag, rAccel, Dil_Mode_enabled)

                        # rAccel and brakePressure to be sent to DiL
                        # Need Dev and DiL mode

                        if force_controls == True:
                            if Dil_Mode_enabled == True:
                                steering_input = float(self.steering_angle_entry.get())
                                rAccel = float(self.accelerator_pedal_entry.get())
                                brakePressure = float(self.brake_pressure_entry.get())
                            if Dil_Mode_enabled == False:
                                steering_input = float(self.steering_angle_entry.get())
                                MDriveFL = float(self.MDriveFL_entry.get())
                                MDriveFR = float(self.MDriveFR_entry.get())
                                MDriveRL = float(self.MDriveRL_entry.get())
                                MDriveRR = float(self.MDriveRR_entry.get())
                                MBrakeFL = float(self.MBrakeFL_entry.get())
                                MBrakeFR = float(self.MBrakeFR_entry.get())
                                MBrakeRL = 0
                                MBrakeRR = 0
                        else:
                            MDriveFL = 0
                            MDriveFR = 0
                        
                        # print(f"Last Ref Distance: {lastRefDistance}")
                        if self.stop_flag.is_set():
                            self._append_to_console_output_safe("Simulation stopped by user.\n")
                            StopFlag = True
                            if Dil_Mode_enabled:
                                rAccel = 0
                                brakePressure = 120
                            else:
                                rAccel = 0
                                brakePressure = 120
                                MDriveFL = 0
                                MDriveFR = 0
                                MDriveRL = 0
                                MDriveRR = 0
                                MBrakeFL = 2000
                                MBrakeFR = 2000
                        else:
                            StopFlag = False

                        MDriveRL_array.append(MDriveRL)
                        MDriveRR_array.append(MDriveRR)
                        MBrakeFL_array.append(MBrakeFL)
                        MBrakeFR_array.append(MBrakeFR)
                        MBrakeRL_array.append(MBrakeRL)
                        MBrakeRR_array.append(MBrakeRR)
                        tVelocity_array.append(targetVelocity)
                        acc_array.append(rAccel)
                        bPressure_array.append(brakePressure)
                        nMGU = np.clip(nMGU, 0, 2000)
                        nMGU_array.append(nMGU)
                        t_P_array.append(t_P)
                        t_I_array.append(t_I)
                        t_D_array.append(t_D)
                        b_P_array.append(b_P)
                        b_I_array.append(b_I)
                        b_D_array.append(b_D)
                        vCar_error_array.append(velocity_error)
                        previous_targetVelocity = targetVelocity
                        refDistances.append(refDistance)


                        output_text2 = (
                            f"Steering Input: {steering_input}\n"
                            f"Throttle: {rAccel}\n"
                            f"Brake Pressure: {brakePressure}\n"
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
                    steering_input = 0
                    MDriveFL = 0
                    MDriveFR = 0
                    MDriveRL = -4000
                    MDriveRR = -4000
                    MBrakeFL = 4000
                    MBrakeFR = 4000
                    MBrakeRL = 0
                    MBrakeRR = 0
                    rAccel = 0
                    brakePressure = 100
                    self._append_to_socket_output_safe("Client connection closed.\n")

            if Dil_Mode_enabled == True:
                data_to_send = struct.pack('fff', steering_input, rAccel, brakePressure)
            if Dil_Mode_enabled == False:
                data_to_send = struct.pack('fffffff', steering_input, MDriveFL, MDriveFR, MDriveRL, MDriveRR, MBrakeFL, MBrakeFR)

            self.server_socket.sendto(data_to_send, self.client_address)
            data, _ = self.server_socket.recvfrom(1024)
            received_length = len(data)
            expected_length = 16 * 4
            output_text = (
                f":::::Hard Braking:::::\n\n"
            )
            self._append_to_console_output_safe(output_text)

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
            plot_and_store(self.figures, plot_TargetValues, base_name, tVelocity_array, acc_array, bPressure_array, velocity_X, current_time)
            plot_and_store(self.figures, plot_PedalValues, base_name, acc_array, bPressure_array, current_time)
            plot_and_store(self.figures, plot_nMGU_Values, base_name, nMGU_array, current_time)
            plot_and_store(self.figures, plot_Throttle_PID_Values, base_name, t_P_array, t_I_array, t_D_array, current_time)
            plot_and_store(self.figures, plot_Brake_PID_Values, base_name, b_P_array, b_I_array, b_D_array, current_time)
            plot_and_store(self.figures, plot_vCar_error_Values, base_name, vCar_error_array, current_time)
            plot_and_store(self.figures, plot_refDistances, base_name, refDistances, current_time)

            self._append_to_console_output_safe("Figures saved locally.\n")
            # self.display_graphs(self.figures)
            # self.tracks_file = None
            # self.velocity_profile_file = None
            self._append_to_console_output_safe("Script finished execution.\n")
            self._update_connection_status_safe("Disconnected")
            self.start_server_button.config(state="normal")
            self.start_sim_button.config(state="normal")
            self.reset_button.config(state="normal")
            self.abort_button.config(state="disabled")
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
    
    # --- Stopwatch Dial Methods ---

    def draw_watch_face(self, canvas, cx, cy, radius):
        # Draw outer circle
        canvas.create_oval(
            cx - radius, cy - radius,
            cx + radius, cy + radius,
            outline="black", width=2
        )
        # Draw tick marks and hour numbers
        for i in range(60):
            angle = math.radians(i * 6)
            x_outer = cx + radius * math.sin(angle)
            y_outer = cy - radius * math.cos(angle)
            x_inner = cx + (radius - 10) * math.sin(angle)
            y_inner = cy - (radius - 10) * math.cos(angle)
            canvas.create_line(x_inner, y_inner, x_outer, y_outer)
            if i % 5 == 0:
                num = i // 5 or 12
                x_text = cx + (radius - 25) * math.sin(angle)
                y_text = cy - (radius - 25) * math.cos(angle)
                canvas.create_text(x_text, y_text, text=str(num), font=("Helvetica", 8, "bold"))

    def draw_speed_dial_face(self, canvas, cx, cy, radius):
        # Draw outer circle
        canvas.create_oval(
            cx - radius, cy - radius,
            cx + radius, cy + radius,
            outline="black", width=2
        )
        # Draw tick marks and speed labels every 20 km/h for 12 increments
        for i in range(60):
            angle = math.radians(i * 6)
            x_outer = cx + radius * math.sin(angle)
            y_outer = cy - radius * math.cos(angle)
            x_inner = cx + (radius - 10) * math.sin(angle)
            y_inner = cy - (radius - 10) * math.cos(angle)
            canvas.create_line(x_inner, y_inner, x_outer, y_outer)
            if i % 5 == 0:
                speed_label = (i // 5) * 20  # 0,20,40,...,220
                x_text = cx + (radius - 25) * math.sin(angle)
                y_text = cy - (radius - 25) * math.cos(angle)
                canvas.create_text(x_text, y_text, text=str(speed_label), font=("Helvetica", 8, "bold"))

    def update_dials(self):
        # Update Sim Time Dial
        self.sim_canvas.delete("hand")
        total_seconds = self.sim_current_time
        # Digital Sim Time
        minutes = int(total_seconds) // 60
        seconds = int(total_seconds) % 60
        millis = int((total_seconds - int(total_seconds)) * 1000)
        time_str = f"{minutes:02}:{seconds:02}.{millis:03}"
        self.sim_time_label.config(text=time_str)

        # Second hand
        sec_angle = math.radians((total_seconds % 60) * 6)
        xs = self.sim_center_x + (self.sim_radius - 20) * math.sin(sec_angle)
        ys = self.sim_center_y - (self.sim_radius - 20) * math.cos(sec_angle)
        self.sim_canvas.create_line(
            self.sim_center_x, self.sim_center_y, xs, ys,
            fill="red", width=2, tags="hand"
        )
        # Minute hand
        min_angle = math.radians((total_seconds / 60) * 6)
        xm = self.sim_center_x + (self.sim_radius - 35) * math.sin(min_angle)
        ym = self.sim_center_y - (self.sim_radius - 35) * math.cos(min_angle)
        self.sim_canvas.create_line(
            self.sim_center_x, self.sim_center_y, xm, ym,
            fill="blue", width=3, tags="hand"
        )
        # Hour hand
        hr_angle = math.radians((total_seconds / 3600) * 360)
        xh = self.sim_center_x + (self.sim_radius - 50) * math.sin(hr_angle)
        yh = self.sim_center_y - (self.sim_radius - 50) * math.cos(hr_angle)
        self.sim_canvas.create_line(
            self.sim_center_x, self.sim_center_y, xh, yh,
            fill="green", width=4, tags="hand"
        )

        # Update Speed Dial
        self.speed_canvas.delete("hand")
        speed = self.speed_current
        self.speed_digital_label.config(text=f"{speed:.2f} km/h")
        # Scale: map 0-240 km/h onto 0-360°
        angle = math.radians((speed % 240) / 240 * 360)
        xs = self.speed_center_x + (self.speed_radius - 20) * math.sin(angle)
        ys = self.speed_center_y - (self.speed_radius - 20) * math.cos(angle)
        self.speed_canvas.create_line(
            self.speed_center_x, self.speed_center_y, xs, ys,
            fill="red", width=3, tags="hand"
        )

        # Schedule next update
        self.root.after(50, self.update_dials)


def run_app():
    root = tk.Tk()
    app = ConsoleApp(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()


if __name__ == "__main__":
    run_app()
