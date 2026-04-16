# Variable Target Distances for Velocity Profile + Velocity Tolerance

import numpy as np
import math
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy.io import loadmat
import pandas as pd

sdistance = 0
nLap = 0

mass = 840
frontRadii = .3274                          # Based on Current Gen 3 # Received
rearRadii = .3485                     
gearRatioRear = 12.67                       # Approved by Thomas
gearRatioFront = 8.47 



###############################
# Chapter 4

# def refStateNow(trajectory_matrix, Vx, Vy, orientation, timestep, newSim):
#     refDistance = 0

#     if newSim is True:
#         sdistance = sdistance - sdistance
#         refDistance = refDistance - refDistance
#         newSim = False

#     max_sdistance = np.max(trajectory_matrix['sDistance'])

#     V_magnitude = np.sqrt(Vx**2 + Vy**2)
    
#     phi_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['Orientation'], kind='linear', fill_value="extrapolate")
#     phi = phi_interpolator(sdistance)

#     sdistance += V_magnitude * timestep  * max(0, np.cos(orientation - phi))
    
#     if sdistance >= max_sdistance:
#         sdistance -= max_sdistance
#         phi = phi_interpolator(sdistance)

#     refDistance = sdistance

#     return refDistance, newSim


def calculateTargetVelocityDistance(refDistance, longitudinal_velocity, variable_TD_enabled, shortDistance):

    if variable_TD_enabled == True:
        shortDistance = longitudinal_velocity * shortDistance
        targetDistance = refDistance + shortDistance
    else:    
        targetDistance = refDistance + shortDistance


    return targetDistance, shortDistance                    # Target and short distance in Eq. 4.28


def calculateTargetVelocity(vCarData, trajectory_matrix, targetDistance, nLap):

    # max_speed = 235
    max_speed = 65.2777778

    max_sdistance = np.max(trajectory_matrix['sDistance'])
    velocity_interpolator = interp1d(vCarData['sDistance'], vCarData['vCar'], kind='linear', fill_value="extrapolate")
    if nLap > 0:
        max_sdistance = np.max(trajectory_matrix['sDistance2'])
        velocity_interpolator = interp1d(vCarData['sDistance2'], vCarData['vCar2'], kind='linear', fill_value="extrapolate")
    if targetDistance > max_sdistance:
        targetDistance -= max_sdistance
    targetVelocity = velocity_interpolator(targetDistance)
    

    targetVelocity = targetVelocity / 3.6
    targetVelocity = min(targetVelocity, max_speed)
    # targetVelocity = targetVelocity * (velocity_percentage/100)

    # print("Target Velocity: ", targetVelocity)                                                                         # TAKE TARGET VELOCITIES FROM LEGACY DATA BUT TAKE INTO ACCOUNT PREVIOUS STEERING ANGLE

    return targetVelocity



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



def calculateTargetAcceleration(targetVelocity, Vx, Vy, shortDistance, scalingFactor, velocity_error, last_Avx, scaling_factor_enabled, Kp, maxChangeAx, steering_input):

    maxA = 19
    minA = -19

    if abs(targetVelocity - Vx) <= velocity_error:
        targetAvx = last_Avx

    if scaling_factor_enabled == True:
        targetAvx = ((Kp*(targetVelocity - Vx))/shortDistance) * 1/scalingFactor            # Eq. 4.28/5.4
    else:
        targetAvx = ((targetVelocity - Vx)/shortDistance)

    targetAvx = min(targetAvx, maxA)                                                                                   
    targetAvx = max(targetAvx, minA)

    targetAvx = np.clip(targetAvx, last_Avx - maxChangeAx, last_Avx + maxChangeAx)

    # if abs(steering_input) > 5 and targetAvx > 0:
    #     targetAvx *= 0.7
    # elif abs(steering_input) > 15 and targetAvx > 0:
    #     targetAvx *= 0.5

    last_Avx = targetAvx
    # Avx = Avx / 12960
    # print("Target Acceleration: ", targetAvx)
                                                                                 

    return targetAvx, last_Avx


def calculateLongitudinalThrust(longAcceleration):
    global mass
    longThrust = longAcceleration * mass         # Eq. 4.29                                                 # CONVERT TARGET ACCELERATION STRAIGHT TO NEWTONS
    # print(f"Longitudunal Thrust: {longThrust}\n")
    oldLongThrust = longThrust
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
    deltaTorque = torqueTransfer * (rearRHSWheelVelocity - rearLHSWheelVelocity)                                        # Eq. 5.2            ROTATIONAL VELOCITIES FROM BALANCE -> TYRE OMEGA 
    return deltaTorque


def calculateOverrunTorqueTransfer(rearLHSWheelVelocity, rearRHSWheelVelocity):
    global maxEngineBrakeTorque
    global gearRatioRear

    inputDifferentialtorque = maxEngineBrakeTorque * gearRatioRear                                  # Eq. 5.17
    torqueTransfer = -diffTorqueTransferGain1 - diffTorqueTransferGainO * inputDifferentialtorque   # Eq. 5.1
    deltaTorque = torqueTransfer * (rearRHSWheelVelocity - rearLHSWheelVelocity)                                      # Eq. 5.2
    return deltaTorque



def calculateDriveTorque(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity, disable_flag):
    global rearRadii
    
    MDriveRL = 0.5 * longThrust * rearRadii
    MDriveRR = 0.5 * longThrust * rearRadii          # Eq. 5.9
    if disable_flag == False:
        deltaTorque = calculateDrivingTorqueTransfer(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity)
        MDriveRL = MDriveRL - deltaTorque
        MDriveRR = MDriveRR + deltaTorque                           # Eq. 5.11
    
    return MDriveRL, MDriveRR
    
     

def calculateBrakeTorque(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity, disable_flag):
    global frontRadii                    
    global rearRadii                      
    global frontBrakeCoeff
    global rearBrakeCoeff
    global maxEngineBrakeTorque
    global gearRatioRear

    residualForce = longThrust + ((maxEngineBrakeTorque * gearRatioRear)/rearRadii) # Eq. 5.15

    MBrakeFL = -residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff))
    MBrakeFR = -residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff))
    MBrakeRL = maxEngineBrakeTorque * gearRatioRear - residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff))
    MBrakeRR = maxEngineBrakeTorque * gearRatioRear - residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff)) # Eq. 5.16
    
    if disable_flag == False:
        deltaTorque = calculateOverrunTorqueTransfer(rearLHSWheelVelocity, rearRHSWheelVelocity)
        MBrakeRL = MBrakeRL - deltaTorque
        MBrakeRR = MBrakeRR + deltaTorque # Eq. 5.11

    return MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR 

def saturation(value, min_value, max_value):

    saturated_values = np.clip(value, min_value, max_value)
    return saturated_values


##############################################################################################################################

# MAIN FUNCTION

def calculateTorqueControl(tracks_df, vProfiledf, Vx, Vy, yawAngle, x, y, WheelVelocityRL, WheelVelocityRR, SlipFL, SlipFR, SlipRL, SlipRR, 
                           mEBT, diffGain1, diffGainD, diffGainO, frontCoeff, rearCoeff, max_drive_torque, max_brake_torque, max_long_slip, slip_constant, 
                           timestep, velocity_percentage, ref_distance, nLap, disable_flag, velocity_error, scaling_factor_enabled, lastAvx, Kp, maxChangeAx, steering_input, variable_TD_enabled, shortDistance):

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

    # ref_distance, newSim = refStateNow(trajectory_matrix, Vx, Vy, yawAngle, timestep, newSim)
    #ref_distance, ref_orientation = refStateNow(trajectory_matrix, Vx, Vy, yawAngle, timestep, sdistance, nLap)

    targetDistance, shortDistance = calculateTargetVelocityDistance(ref_distance, Vx, variable_TD_enabled, shortDistance)

    targetVelocity = calculateTargetVelocity(vCarData, trajectory_matrix, targetDistance, nLap)

    scalingFactor = calculateScalingFactor(trajectory_matrix, ref_distance, x, y, yawAngle, Vx, Vy, nLap)

    longAcceleration, lastAvx = calculateTargetAcceleration(targetVelocity, Vx, Vy, scalingFactor, shortDistance, velocity_error, lastAvx, scaling_factor_enabled, Kp, maxChangeAx, steering_input)

    longThrust, oldLongThrust = calculateLongitudinalThrust(longAcceleration)

    targetVelocity = targetVelocity * 3.6


    if longThrust > 0:
        if disable_flag == False:
            longThrust = antiSlip(longThrust, SlipRL, SlipRR, max_long_slip, slip_constant)
        MDriveRL, MDriveRR = calculateDriveTorque(longThrust, WheelVelocityRL, WheelVelocityRR, disable_flag)
        MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
        MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
        if Vx < 3:
            MDriveRL = MDriveRL + 200
            MDriveRR = MDriveRR + 200
        MBrakeFL = 0
        MBrakeFR = 0
        MBrakeRL = 0 
        MBrakeRR = 0

    # if longThrust < 0:
    else:
        if disable_flag == False:
            longThrust = antiLockBrakes(longThrust, SlipFL, SlipFR, SlipRL, SlipRR, max_long_slip, slip_constant)
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


    # print("MDriveRL: ", MDriveRL)
    # print("MDriveRR: ", MDriveRR)
    # print("MBrakeFL: ", MBrakeFL)
    # print("MBrakeFR: ", MBrakeFR)
    # print("MBrakeRL: ", MBrakeRL)
    # print("MBrakeRR: ", MBrakeRR)

    

    return MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, targetVelocity, longAcceleration, oldLongThrust, scalingFactor, lastAvx