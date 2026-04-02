# Initial Acceleration Control

import numpy as np
import math
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy.io import loadmat
import pandas as pd
from scipy.interpolate import RegularGridInterpolator

sdistance = 0
nLap = 0

mass = 840
frontRadii = .3274                          # Based on Current Gen 3 # Received
rearRadii = .3485                     
gearRatioRear = 12.67                       # Approved by Thomas
gearRatioFront = 8.47
 
prev_tP = 0
prev_tI = 0
prev_tD = 0
prev_bP = 0
prev_bI = 0
prev_bD = 0


def calculateTargetVelocityDistance(refDistance):

    shortDistance = 0.01
    targetDistance = refDistance + shortDistance

    return targetDistance                    # Target and short distance in Eq. 4.28


def calculateTargetVelocityBrake(vCarData, trajectory_matrix, targetDistance, nLap):

    # max_speed = 235
    max_speed = 65.2777778

    max_sdistance = np.max(trajectory_matrix['sDistance'])
    velocity_interpolator = interp1d(vCarData['sDistance'], vCarData['vCar'], kind='linear', fill_value="extrapolate")

    if nLap > 0:
        max_sdistance = np.max(trajectory_matrix['sDistance2'])
        velocity_interpolator = interp1d(vCarData['sDistance2'], vCarData['vCar2'], kind='linear', fill_value="extrapolate")
    if targetDistance > max_sdistance:
        targetDistance -= max_sdistance

    
    targetVelocityBrake = velocity_interpolator(targetDistance)
    

    targetVelocityBrake = targetVelocityBrake / 3.6
    targetVelocityBrake = min(targetVelocityBrake, max_speed)
    # targetVelocity = targetVelocity * (velocity_percentage/100)

    # print("Target Velocity: ", targetVelocity)                                                                         

    return targetVelocityBrake



def calculateTargetAcceleration(targetVelocity, Vx, Vy):

    global targetAvx
    maxA = 19
    minA = -19
    shortDistance = 0.01

    # if abs(targetVelocity - Vx) <= velocity_error:
    #     targetAvx = last_Avx

    targetAvx = ((targetVelocity - Vx)/shortDistance)

    targetAvx = min(targetAvx, maxA)                                                                                   
    targetAvx = max(targetAvx, minA)
                             

    return targetAvx


def calculateLongitudinalThrust(longAcceleration):
    global mass
    longThrust = longAcceleration * mass         # Eq. 4.29                                                 # CONVERT TARGET ACCELERATION STRAIGHT TO NEWTONS
    return longThrust

#########################################################################################################################################################

def calculateVelocityError(Vx, Vy, vCarData, trajectory_matrix, refDistance, nLap):

    velocity_interpolator = interp1d(vCarData['sDistance'], vCarData['vCar'], kind='linear', fill_value="extrapolate")
    if nLap > 0:
        velocity_interpolator = interp1d(vCarData['sDistance2'], vCarData['vCar2'], kind='linear', fill_value="extrapolate")
    targetVelocity = velocity_interpolator(refDistance)
    targetVelocity = targetVelocity / 3.6
    vCar_error = targetVelocity - Vx
    

    return targetVelocity, vCar_error

def compute_throttle_pid(error, dt, Kp, Ki, Kd, integral, prev_error):
    """ Compute the PID correction """
    P = Kp * error
    
    integral += error * dt
    I = Ki * integral
    
    derivative = (error - prev_error) / dt if dt > 0 else 0
    D = Kd * derivative
    
    rAccel = P + I + D
    rAccel = np.clip(rAccel, 0, 100)
    
    prev_error = error
    
    return rAccel, integral, prev_error, P, I, D

def compute_brake_pid(error, dt, Kp, Ki, Kd, integral, prev_error):
    """ Compute the PID correction """
    P = Kp * abs(error)
    
    integral += abs(error) * dt
    I = Ki * integral
    
    derivative = (abs(error) - prev_error) / dt if dt > 0 else 0
    D = Kd * derivative
    
    bPressure = P + I + D
    bPressure = np.clip(bPressure, 0, 120)
    
    prev_error = error
    
    return bPressure, integral, prev_error, P, I, D

#########################################################################################################################################################

def saturation(value, min_value, max_value):

    saturated_values = np.clip(value, min_value, max_value)
    return saturated_values

#########################################################################################################################################################

def calculatePedalPercent(tracks_df, vProfiledf, Vx, Vy, timestep, ref_distance, nLap, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error,
                          brake_Kp, brake_Ki, brake_Kd, brake_integral, brake_prev_error, max_engine_brake_torque, differential_gain_1, driving_differential_gain, overrun_differential_gain, front_wheel_coefficient, rear_wheel_coefficient, 
                          max_drive_torque, torque_injection, Dil_Mode_enabled):
    
    trajectory_matrix = tracks_df
    vCarData = vProfiledf

    global frontBrakeCoeff
    global rearBrakeCoeff
    global maxEngineBrakeTorque
    global diffTorqueTransferGain1
    global diffTorqueTransferGainD
    global diffTorqueTransferGainO

    global prev_tP
    global prev_tI
    global prev_tD
    global prev_bP
    global prev_bI
    global prev_bD

    frontBrakeCoeff = front_wheel_coefficient                      # 0.5 is fine for both
    rearBrakeCoeff = rear_wheel_coefficient
    maxEngineBrakeTorque = max_engine_brake_torque
    diffTorqueTransferGain1 = differential_gain_1                # To be tweaked
    diffTorqueTransferGainD = driving_differential_gain
    diffTorqueTransferGainO = overrun_differential_gain

    targetVelocity, velocity_error = calculateVelocityError(Vx, Vy, vCarData, trajectory_matrix, ref_distance, nLap)

    if velocity_error >= 0:
        if Dil_Mode_enabled == True:
            rAccel, throttle_integral, throttle_prev_error, throttle_P, throttle_I, throttle_D = compute_throttle_pid(velocity_error, timestep, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error)
            prev_tP = throttle_P
            prev_tI = throttle_I
            prev_tD = throttle_D
            # torque = get_torque(rAccel, nMGU)
            # torque = antiSlip(torque, SlipRL, SlipRR, max_long_slip, slip_constant)
            # MDriveRL, MDriveRR = calculateDriveTorque(torque, WheelVelocityRL, WheelVelocityRR)
            MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
            MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
            MDriveRL = MDriveRL + torque_injection
            MDriveRR = MDriveRR + torque_injection
            MBrakeFL = 0
            MBrakeFR = 0
            MBrakeRL = 0 
            MBrakeRR = 0
            brakePressure = 0
            brake_P = 0
            brake_I = 0
            brake_D = 0
            brake_integral = 0
            
    if velocity_error < 0:
        if Dil_Mode_enabled == True:
            brakePressure, brake_integral, brake_prev_error, brake_P, brake_I, brake_D = compute_brake_pid(velocity_error, timestep, brake_Kp, brake_Ki, brake_Kd, brake_integral, brake_prev_error)
            prev_bP = brake_P
            prev_bI = brake_I
            prev_bD = brake_D
            rAccel = 0
            throttle_P = 0
            throttle_I = 0
            throttle_D = 0
            throttle_integral = 0
            MDriveRL = 0
            MDriveRR = 0
            MBrakeFL = 0
            MBrakeFR = 0
            MBrakeRL = 0 
            MBrakeRR = 0
    
    targetVelocity = targetVelocity * 3.6

    return MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, rAccel, brakePressure, throttle_integral, throttle_prev_error, brake_integral, brake_prev_error, targetVelocity, throttle_P, throttle_I, throttle_D, brake_P, brake_I, brake_D, velocity_error