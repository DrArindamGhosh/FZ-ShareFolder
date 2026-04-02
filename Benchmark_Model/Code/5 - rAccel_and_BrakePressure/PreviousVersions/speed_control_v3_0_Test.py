# Added np.sign to Differential control as per the research paper

import numpy as np
import math
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy.io import loadmat
import pandas as pd

sdistance = 0
nLap = 0
targetAvx = 0

mass = 840
frontRadii = .3274                          # Based on Current Gen 3 # Received
rearRadii = .3485                     
gearRatioRear = 12.67                       # Approved by Thomas
gearRatioFront = 8.47 

# def calculateTargetVelocityDistance(refDistance, longitudinal_velocity, variable_TD_enabled, shortDistance):

#     if variable_TD_enabled == True:
#         shortDistance = longitudinal_velocity * shortDistance
#         targetDistance = refDistance + shortDistance
#     else:    
#         targetDistance = refDistance + shortDistance


#     return targetDistance, shortDistance                    # Target and short distance in Eq. 4.28


def calculateVelocityError(Vx, Vy, vCarData, trajectory_matrix, refDistance, nLap):

    # max_speed = 235
    max_speed = 65.2777778

    vCar = np.sqrt(Vx ** 2 + Vy ** 2)

    max_sdistance = np.max(trajectory_matrix['sDistance'])
    velocity_interpolator = interp1d(vCarData['sDistance'], vCarData['vCar'], kind='linear', fill_value="extrapolate")
    if nLap > 0:
        max_sdistance = np.max(trajectory_matrix['sDistance2'])
        velocity_interpolator = interp1d(vCarData['sDistance2'], vCarData['vCar2'], kind='linear', fill_value="extrapolate")
    # if targetDistance > max_sdistance:
    #     targetDistance -= max_sdistance
    targetVelocity = velocity_interpolator(refDistance)
    targetVelocity = targetVelocity / 3.6
    vCar_error = targetVelocity - Vx
    

    # targetVelocity = targetVelocity / 3.6
    # targetVelocity = min(targetVelocity, max_speed)
    # targetVelocity = targetVelocity * (velocity_percentage/100)

    # print("Target Velocity: ", targetVelocity)                                                                         # TAKE TARGET VELOCITIES FROM LEGACY DATA BUT TAKE INTO ACCOUNT PREVIOUS STEERING ANGLE

    # return targetVelocity
    return targetVelocity, vCar_error



def calculateScalingFactor(trajectory_matrix, refDistance, x, y, yawAngle, Vx, Vy, nLap):

    k_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['k'], kind='linear', fill_value="extrapolate")
    phi_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['Orientation'], kind='linear', fill_value="extrapolate")
    x_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['x'], kind='linear', fill_value="extrapolate")
    y_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['y'], kind='linear', fill_value="extrapolate")
    if nLap > 0:
        k_interpolator = interp1d(trajectory_matrix['sDistance2'], trajectory_matrix['k2'], kind='linear', fill_value="extrapolate")
        phi_interpolator = interp1d(trajectory_matrix['sDistance2'], trajectory_matrix['Orientation2'], kind='linear', fill_value="extrapolate")
        x_interpolator = interp1d(trajectory_matrix['sDistance2'], trajectory_matrix['x2'], kind='linear', fill_value="extrapolate")
        y_interpolator = interp1d(trajectory_matrix['sDistance2'], trajectory_matrix['y2'], kind='linear', fill_value="extrapolate")
    k = k_interpolator(refDistance)
    phi = phi_interpolator(refDistance)
    traj_x = x_interpolator(refDistance)
    traj_y = y_interpolator(refDistance)

    radius = 1 / np.abs(k)
    distance = abs(((y - traj_y)*np.cos(phi)) - ((x - traj_x)*np.sin(phi)))
    denominator = (Vx * np.cos(yawAngle - phi)) - (Vy * np.sin(yawAngle - phi))

    scalingFactor = (1 - distance / radius) / denominator
    # print("Distance: ", distance)
    # print("Radius: ", k)
    # print("Denominator: ", denominator)
    # print("Scaling Factor: ", scalingFactor)

    return scalingFactor



def calculateTargetAcceleration(targetVelocity, Vx, Vy, dt, velocity_error, timestep, Kp, Ki, Kd, torque_integral, torque_prev_error):
# def calculateTargetAcceleration(targetVelocity, Vx, Vy, dt):

    global targetAvx
    maxA = 19
    minA = -19

    vCar = np.sqrt(Vx ** 2 + Vy ** 2)

    # if abs(targetVelocity - vCar) <= velocity_error:
    #     targetAvx = last_Avx
    #       # Eq. 4.28/5.4

    targetAvx = (targetVelocity - Vx)/dt

    targetAvx, torque_integral, torque_prev_error = compute_pid(velocity_error, timestep, Kp, Ki, Kd, torque_integral, torque_prev_error)

    targetAvx = min(targetAvx, maxA)                                                                                   
    targetAvx = max(targetAvx, minA)

    last_Avx = targetAvx
                               

    return targetAvx, last_Avx


# def calculateLongitudinalThrust(longAcceleration, velocity_error, timestep, Kp, Ki, Kd, torque_integral, torque_prev_error):
def calculateLongitudinalThrust(longAcceleration):
    global mass
    longThrust = longAcceleration * mass         # Eq. 4.29                                                 # CONVERT TARGET ACCELERATION STRAIGHT TO NEWTONS
    # print(f"Longitudunal Thrust: {longThrust}\n")
    oldLongThrust = longThrust
    # longThrust, torque_integral, torque_prev_error = compute_pid(velocity_error, timestep, Kp, Ki, Kd, torque_integral, torque_prev_error)
    return longThrust, oldLongThrust





#######################################################################
# Chapter 5

def antiSlip(longThrust, LRslip, RRslip, max_long_slip, slip_constant):

    antiSpinLR = (-0.5 * np.sin(np.arctan(slip_constant * (LRslip - max_long_slip))) + 0.5)
    antiLockLR = (0.5 * np.sin(np.arctan(slip_constant * (LRslip + max_long_slip))) + 0.5)
    
    antiSpinRR = (-0.5 * np.sin(np.arctan(slip_constant * (RRslip - max_long_slip))) + 0.5)
    antiLockRR = (0.5 * np.sin(np.arctan(slip_constant * (RRslip + max_long_slip))) + 0.5)
    
    totalAntiSlipLR = antiSpinLR * antiLockLR
    totalAntiSlipRR = antiSpinRR * antiLockRR

    longThrust = longThrust * totalAntiSlipLR * totalAntiSlipRR

    return longThrust


def antiLockBrakes(longThrust, LFslip, RFslip, LRslip, RRslip, max_long_slip, slip_constant):

    antiLockLF = 0.5 * np.sin(np.arctan(slip_constant * (LFslip + max_long_slip))) + 0.5
    antiLockRF = 0.5 * np.sin(np.arctan(slip_constant * (RFslip + max_long_slip))) + 0.5
    
    antiSpinLR = -0.5 * np.sin(np.arctan(slip_constant * (LRslip - max_long_slip))) + 0.5
    antiLockLR = 0.5 * np.sin(np.arctan(slip_constant * (LRslip + max_long_slip))) + 0.5
    
    antiSpinRR = -0.5 * np.sin(np.arctan(slip_constant * (RRslip - max_long_slip))) + 0.5
    antiLockRR = 0.5 * np.sin(np.arctan(slip_constant * (RRslip + max_long_slip))) + 0.5

    totalAntiLockLR = antiSpinLR * antiLockLR
    totalAntiLockRR = antiSpinRR * antiLockRR

    longThrust = longThrust * antiLockLF * antiLockRF * totalAntiLockLR * totalAntiLockRR
    
    return longThrust



def calculateDrivingTorqueTransfer(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity):
    global rearRadii

    inputDifferentialtorque = -longThrust * rearRadii                                                                   # Eq. 5.10
    torqueTransfer = -diffTorqueTransferGain1 + diffTorqueTransferGainD * inputDifferentialtorque                       # Eq. 5.1
    deltaTorque = torqueTransfer * np.sign(rearRHSWheelVelocity - rearLHSWheelVelocity)                                        # Eq. 5.2            ROTATIONAL VELOCITIES FROM BALANCE -> TYRE OMEGA 
    return deltaTorque


def calculateOverrunTorqueTransfer(rearLHSWheelVelocity, rearRHSWheelVelocity):
    global maxEngineBrakeTorque
    global gearRatioRear

    inputDifferentialtorque = maxEngineBrakeTorque * gearRatioRear                                  # Eq. 5.17
    torqueTransfer = -diffTorqueTransferGain1 - diffTorqueTransferGainO * inputDifferentialtorque   # Eq. 5.1
    deltaTorque = torqueTransfer * np.sign(rearRHSWheelVelocity - rearLHSWheelVelocity)                                      # Eq. 5.2
    return deltaTorque



def calculateDriveTorque(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity, disable_flag):
# def calculateDriveTorque(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity, disable_flag, velocity_error, timestep, Kp, Ki, Kd, torque_integral, torque_prev_error, max_drive_torque):

    global rearRadii
    
    MDriveRL = 0.5 * longThrust * rearRadii
    MDriveRR = 0.5 * longThrust * rearRadii          # Eq. 5.9
    
    if disable_flag == False:
        deltaTorque = calculateDrivingTorqueTransfer(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity)
        MDriveRL = MDriveRL - deltaTorque
        MDriveRR = MDriveRR + deltaTorque                           # Eq. 5.11
    # MDriveRL, torque_integral, torque_prev_error = compute_drive_pid(velocity_error, timestep, Kp, Ki, Kd, torque_integral, torque_prev_error, max_drive_torque)
    # MDriveRR, torque_integral, torque_prev_error = compute_drive_pid(velocity_error, timestep, Kp, Ki, Kd, torque_integral, torque_prev_error, max_drive_torque)
    
    return MDriveRL, MDriveRR
    
     

def calculateBrakeTorque(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity, disable_flag):
# def calculateBrakeTorque(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity, disable_flag, velocity_error, timestep, Kp, Ki, Kd, torque_integral, torque_prev_error, max_brake_torque):
    global frontRadii                    
    global rearRadii                      
    global frontBrakeCoeff
    global rearBrakeCoeff
    global maxEngineBrakeTorque
    global gearRatioRear

    if (longThrust*-1) <= ((maxEngineBrakeTorque * gearRatioRear)/rearRadii):
        print("Rear Braking")
        MBrakeRL = maxEngineBrakeTorque * gearRatioRear
        MBrakeRR = maxEngineBrakeTorque * gearRatioRear
        
        MBrakeFL = 0
        MBrakeFR = 0
        if disable_flag == False:
            deltaTorque = calculateOverrunTorqueTransfer(rearLHSWheelVelocity, rearRHSWheelVelocity)
            MBrakeRL = MBrakeRL - deltaTorque
            MBrakeRR = MBrakeRR + deltaTorque # Eq. 5.11
        # MBrakeRL, torque_integral, torque_prev_error = compute_brake_pid(velocity_error, timestep, Kp, Ki, Kd, torque_integral, torque_prev_error, max_brake_torque)
        # MBrakeRR, torque_integral, torque_prev_error = compute_brake_pid(velocity_error, timestep, Kp, Ki, Kd, torque_integral, torque_prev_error, max_brake_torque)
    else: 
        residualForce = longThrust + ((maxEngineBrakeTorque * gearRatioRear)/rearRadii) # Eq. 5.15

        MBrakeFL = -residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff))
        MBrakeFR = -residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff))
        MBrakeRL = maxEngineBrakeTorque * gearRatioRear - residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff))
        MBrakeRR = maxEngineBrakeTorque * gearRatioRear - residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff)) # Eq. 5.16
        
        if disable_flag == False:
            deltaTorque = calculateOverrunTorqueTransfer(rearLHSWheelVelocity, rearRHSWheelVelocity)
            MBrakeRL = MBrakeRL - deltaTorque
            MBrakeRR = MBrakeRR + deltaTorque # Eq. 5.11

        # MBrakeRL, torque_integral, torque_prev_error = compute_brake_pid(velocity_error, timestep, Kp, Ki, Kd, torque_integral, torque_prev_error, max_brake_torque)
        # MBrakeRR, torque_integral, torque_prev_error = compute_brake_pid(velocity_error, timestep, Kp, Ki, Kd, torque_integral, torque_prev_error, max_brake_torque)

    return MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR 

def saturation(value, min_value, max_value):

    saturated_values = np.clip(value, min_value, max_value)
    return saturated_values

# def compute_drive_pid(error, dt, Kp, Ki, Kd, integral, prev_error, max_drive_torque):
#     """ Compute the PID correction """
#     P = Kp * error
    
#     integral += error * dt
#     I = Ki * integral
    
#     derivative = (error - prev_error) / dt if dt > 0 else 0
#     D = Kd * derivative
    
#     output = P + I + D
#     output = np.clip(output, -max_drive_torque, max_drive_torque)
    
#     prev_error = error
    
#     return output, integral, prev_error

# def compute_brake_pid(error, dt, Kp, Ki, Kd, integral, prev_error, max_brake_torque):
#     """ Compute the PID correction """
#     P = Kp * error
    
#     integral += error * dt
#     I = Ki * integral
    
#     derivative = (error - prev_error) / dt if dt > 0 else 0
#     D = Kd * derivative
    
#     output = P + I + D
#     output = np.clip(output, -max_brake_torque, max_brake_torque)
    
#     prev_error = error
    
#     return output, integral, prev_error

def compute_pid(error, dt, Kp, Ki, Kd, integral, prev_error):
    """ Compute the PID correction """
    P = Kp * error
    
    integral += error * dt
    I = Ki * integral
    
    derivative = (error - prev_error) / dt if dt > 0 else 0
    D = Kd * derivative
    
    output = P + I + D
    # output = np.clip(output, -max_brake_torque, max_brake_torque)
    
    prev_error = error
    
    return output, integral, prev_error

def saturation(value, min_value, max_value):
    saturated_values = np.clip(value, min_value, max_value)
    return saturated_values


##############################################################################################################################

# MAIN FUNCTION

def calculateTorqueControl(tracks_df, vProfiledf, Vx, Vy, yawAngle, x, y, WheelVelocityRL, WheelVelocityRR, SlipFL, SlipFR, SlipRL, SlipRR, 
                           mEBT, diffGain1, diffGainD, diffGainO, frontCoeff, rearCoeff, max_drive_torque, max_brake_torque, max_long_slip, slip_constant, 
                           timestep, ref_distance, nLap, disable_flag, initial_thrust, torque_Kp, torque_Ki, torque_Kd, 
                           torque_integral, torque_prev_error):

    trajectory_matrix = tracks_df
    trajectory_matrix.head(), trajectory_matrix.columns

    vCarData = vProfiledf
    vCarData.head(), vCarData.columns

    global frontBrakeCoeff
    global rearBrakeCoeff
    global maxEngineBrakeTorque
    global diffTorqueTransferGain1
    global diffTorqueTransferGainD
    global diffTorqueTransferGainO

    frontBrakeCoeff = frontCoeff                       # 0.5 is fine for both
    rearBrakeCoeff = rearCoeff
    maxEngineBrakeTorque = mEBT
    diffTorqueTransferGain1 = diffGain1                # To be tweaked
    diffTorqueTransferGainD = diffGainD
    diffTorqueTransferGainO = diffGainO

    # Constants
    # Vx = Vx * (1/3.6)

    targetVelocity, velocity_error = calculateVelocityError(Vx, Vy, vCarData, trajectory_matrix, ref_distance, nLap)

    # scalingFactor = calculateScalingFactor(trajectory_matrix, ref_distance, x, y, yawAngle, Vx, Vy, nLap)

    # longAcceleration, lastAvx = calculateTargetAcceleration(targetVelocity, Vx, Vy, timestep)
    longAcceleration, lastAvx = calculateTargetAcceleration(targetVelocity, Vx, Vy, timestep, velocity_error, timestep, torque_Kp, torque_Ki, torque_Kd, torque_integral, torque_prev_error)

    # longThrust, oldLongThrust = calculateLongitudinalThrust(longAcceleration, velocity_error, timestep, torque_Kp, torque_Ki, torque_Kd, torque_integral, torque_prev_error)
    longThrust, oldLongThrust = calculateLongitudinalThrust(longAcceleration)

    targetVelocity = targetVelocity * 3.6


    if longThrust > 0:
        if disable_flag == False:
            longThrust = antiSlip(longThrust, SlipRL, SlipRR, max_long_slip, slip_constant)
        # MDriveRL, MDriveRR = calculateDriveTorque(longThrust, WheelVelocityRL, WheelVelocityRR, disable_flag, velocity_error, timestep, torque_Kp, torque_Ki, torque_Kd, torque_integral, torque_prev_error, max_drive_torque)
        MDriveRL, MDriveRR = calculateDriveTorque(longThrust, WheelVelocityRL, WheelVelocityRR, disable_flag)
        MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
        MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
        if Vx < 3:
            MDriveRL = MDriveRL + initial_thrust
            MDriveRR = MDriveRR + initial_thrust
        MBrakeFL = 0
        MBrakeFR = 0
        MBrakeRL = 0 
        MBrakeRR = 0

    # if longThrust < 0:
    else:
        if disable_flag == False:
            longThrust = antiLockBrakes(longThrust, SlipFL, SlipFR, SlipRL, SlipRR, max_long_slip, slip_constant)
        # MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR = calculateBrakeTorque(longThrust, WheelVelocityRL, WheelVelocityRR, disable_flag, velocity_error, timestep, torque_Kp, torque_Ki, torque_Kd, torque_integral, torque_prev_error, max_brake_torque)
        MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR = calculateBrakeTorque(longThrust, WheelVelocityRL, WheelVelocityRR, disable_flag)
        MBrakeFL = saturation(MBrakeFL, 0, max_brake_torque)
        MBrakeFR = saturation(MBrakeFR, 0, max_brake_torque)
        MBrakeRL = saturation(MBrakeRL, 0, max_brake_torque)
        MBrakeRR = saturation(MBrakeRR, 0, max_brake_torque)
        MDriveRL = 0
        MDriveRR = 0
        MDriveRL = - MBrakeRL
        MDriveRR = - MBrakeRR
        MBrakeRL = 0
        MBrakeRR = 0

    return MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, targetVelocity, longAcceleration, oldLongThrust, torque_integral, torque_prev_error