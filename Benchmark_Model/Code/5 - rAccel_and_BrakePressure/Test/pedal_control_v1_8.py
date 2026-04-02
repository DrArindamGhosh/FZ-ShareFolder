# Stop Flag Variant

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
stop_error = False



def calculateTargetVelocityDistance(refDistance):

    shortDistance = 0.01
    targetDistance = refDistance + shortDistance

    return targetDistance                    # Target and short distance in Eq. 4.28


def calculateTargetVelocityBrake(vCarData, lap_data, targetDistance, nLap):

    # max_speed = 235
    max_speed = 65.2777778

    max_sdistance = np.max(lap_data['sDistance'])
    velocity_interpolator = interp1d(vCarData['sDistance'], vCarData['vCar'], kind='linear', fill_value="extrapolate")

    # if nLap > 0:
    #     max_sdistance = np.max(trajectory_matrix['sDistance2'])
    #     velocity_interpolator = interp1d(vCarData['sDistance2'], vCarData['vCar2'], kind='linear', fill_value="extrapolate")
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

def calculateVelocityError(Vx, Vy, lap_data, vCar_Data, refDistance, nLap):
    max_speed = 65.2777778

    vCar = np.sqrt(Vx ** 2 + Vy ** 2)

    max_sdistance = np.max(lap_data['sDistance'])
    velocity_interpolator = interp1d(vCar_Data['sDistance'], vCar_Data['vCar'], kind='linear', fill_value="extrapolate")
    targetVelocity = velocity_interpolator(refDistance)
    targetVelocity = targetVelocity / 3.6
    vCar_error = targetVelocity - Vx
    

    return targetVelocity, vCar_error

def calculateFutureVelocity(Vx, Vy, lap_data, vCarData, refDistance, nLap, futureTD):
    targetDistance = refDistance + futureTD
    velocity_interpolator = interp1d(vCarData['sDistance'], vCarData['vCar'], kind='linear', fill_value="extrapolate")
    futureVelocity = velocity_interpolator(targetDistance)
    futureVelocity = futureVelocity / 3.6
    
    return futureVelocity




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

def get_torque(rAccel, nMGU):

    nMGU = np.clip(nMGU, 0, 2000)

    MpedalMap_nMGU_Y = np.array([0, 133.3, 266.7, 400, 533.3, 666.7, 823, 960, 1066.7, 1200, 
                              1333.3, 1466.7, 1600, 1733.3, 1866.7, 2000])

    MpedalMap_rAccel_X = np.array([0, 7, 14, 21, 28, 35, 42, 49, 56, 63, 70, 77, 84, 91, 98, 100])

    MPedalMap_Torque_Z = np.array([
        [0, 102.7, 279.3, 510.4, 831.8, 1224.1, 1635.5, 1932.6, 2198, 2436.5, 2652.9, 2852.4, 3040.3, 3237.7, 3589.2, 3589.2],
        [0, 102.7, 279.3, 510.4, 831.8, 1224.1, 1635.5, 1932.6, 2198, 2436.5, 2652.9, 2852.4, 3040.3, 3237.7, 3589.2, 3589.2], 
        [0, 102.7, 279.3, 510.4, 831.8, 1224.1, 1635.5, 1932.6, 2198, 2436.5, 2652.9, 2852.4, 3040.3, 3237.7, 3589.2, 3589.2],
        [0, 102.7, 279.3, 464.1, 732.5, 1090.5, 1525.7, 1932.6, 2198, 2436.5, 2652.9, 2852.4, 3040.3, 3237.7, 3589.2, 3589.2],
        [0, 94.8, 209.5, 348.1, 549.4, 817.8, 1144.2, 1452.9, 1799.1, 2184.6, 2611.4, 2852.4, 3040.3, 3237.7, 3589.2, 3589.2], 
        [0, 75.8, 167.6, 278.5, 439.5, 654.3, 915.4, 1162.3, 1439.3, 1747.7, 2089.1, 2470, 2900.4, 3237.7, 3589.2, 3589.2],
        [0, 61.4, 135.8, 225.6, 356, 530, 741.5, 941.5, 1165.9, 1415.7, 1692.3, 2000.8, 2349.5, 2760.1, 3376.5, 3376.5],
        [0, 52.6, 116.4, 193.4, 305.2, 454.4, 635.7, 807.1, 999.5, 1213.7, 1450.8, 1715.3, 2014.2, 2366.2, 2894.6, 2894.6], 
        [0, 47.4, 104.8, 174, 274.7, 408.9, 572.1, 726.4, 899.5, 1092.3, 1305.7, 1543.7, 1812.8, 2129.6, 2605.2, 2605.2], 
        [0, 42.1, 93.1, 154.7, 244.2, 363.5, 508.6, 645.7, 799.6, 970.9, 1160.6, 1372.2, 1611.3, 1893, 2315.7, 2315.7], 
        [0, 37.9, 83.8, 139.2, 219.8, 327.1, 457.7, 581.1, 719.6, 873.8, 1044.6, 1235, 1450.2, 1703.7, 2084.1, 2084.1], 
        [0, 34.5, 76.2, 126.6, 199.8, 297.4, 416.1, 528.3, 654.2, 794.4, 949.6, 1122.7, 1318.4, 1548.8, 1894.7, 1894.7], 
        [0, 31.6, 69.8, 116, 183.1, 272.6, 381.4, 484.3, 599.7, 728.2, 870.5, 1029.2, 1208.5, 1419.7, 1736.8, 1736.8], 
        [0, 29.2, 64.5, 107.1, 169, 251.6, 352.1, 447, 553.6, 672.2, 803.5, 950, 1115.5, 1310.5, 1603.2, 1603.2], 
        [0, 27.1, 59.9, 99.5, 157, 233.7, 326.9, 415.1, 514, 624.2, 746.1, 882.1, 1035.9, 1216.9, 1488.7, 1488.7], 
        [0, 25.3, 55.9, 92.8, 146.5, 218.1, 305.1, 387.4, 479.8, 582.6, 696.4, 823.3, 966.8, 1135.8, 1389.4, 1389.4]
    ])

    torque_interpolator = RegularGridInterpolator((MpedalMap_nMGU_Y, MpedalMap_rAccel_X), MPedalMap_Torque_Z)

    torque = torque_interpolator((nMGU, rAccel))

    return torque

#########################################################################################################################################################

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



def calculateDriveTorque(torque, rearLHSWheelVelocity, rearRHSWheelVelocity):
    global rearRadii
    
    # MDriveRL = 0.5 * longThrust * rearRadii
    # MDriveRR = 0.5 * longThrust * rearRadii          # Eq. 5.9

    # MDriveRL = torque/2
    # MDriveRR = torque/2

    MDriveRL = torque
    MDriveRR = torque

    deltaTorque = calculateDrivingTorqueTransfer(torque, rearLHSWheelVelocity, rearRHSWheelVelocity)
    MDriveRL = MDriveRL - deltaTorque
    MDriveRR = MDriveRR + deltaTorque                           # Eq. 5.11
    
    return MDriveRL, MDriveRR
    
     

def calculateBrakeTorque(torque, rearLHSWheelVelocity, rearRHSWheelVelocity):
    frontBrakeCoeff = 0.5
    rearBrakeCoeff = 0.5
    global frontRadii                    
    global rearRadii                      
    global maxEngineBrakeTorque
    global gearRatioRear

    if (torque*-1) <= ((maxEngineBrakeTorque * gearRatioRear)/rearRadii):
        print("Rear Braking")
        MBrakeRL = maxEngineBrakeTorque * gearRatioRear
        MBrakeRR = maxEngineBrakeTorque * gearRatioRear
        MBrakeFL = 0
        MBrakeFR = 0
        deltaTorque = calculateOverrunTorqueTransfer(rearLHSWheelVelocity, rearRHSWheelVelocity)
        MBrakeRL = MBrakeRL - deltaTorque
        MBrakeRR = MBrakeRR + deltaTorque # Eq. 5.11

    else: 
        residualForce = torque + ((maxEngineBrakeTorque * gearRatioRear)/rearRadii) # Eq. 5.15

        MBrakeFL = -residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff))
        MBrakeFR = -residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff))
        MBrakeRL = maxEngineBrakeTorque * gearRatioRear - residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff))
        MBrakeRR = maxEngineBrakeTorque * gearRatioRear - residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff)) # Eq. 5.16
        

        deltaTorque = calculateOverrunTorqueTransfer(rearLHSWheelVelocity, rearRHSWheelVelocity)
        MBrakeRL = MBrakeRL - deltaTorque
        MBrakeRR = MBrakeRR + deltaTorque # Eq. 5.11

    return MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR 

def saturation(value, min_value, max_value):

    saturated_values = np.clip(value, min_value, max_value)
    return saturated_values

#########################################################################################################################################################

def calculatePedalPercent(lap_df, vCar_Profile_df, Vx, Vy, timestep, ref_distance, nLap, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error,
                          brake_Kp, brake_Ki, brake_Kd, brake_integral, brake_prev_error, nMGU, WheelVelocityRL, WheelVelocityRR,
                          max_engine_brake_torque, differential_gain_1, driving_differential_gain, overrun_differential_gain, 
                          max_drive_torque, max_brake_torque, futureTD, StopFlag, rAccel, Dil_Mode_enabled):
    
    lap_data = lap_df
    vCar_Data = vCar_Profile_df

    global maxEngineBrakeTorque
    global diffTorqueTransferGain1
    global diffTorqueTransferGainD
    global diffTorqueTransferGainO
    global stop_error

    maxEngineBrakeTorque = max_engine_brake_torque
    diffTorqueTransferGain1 = differential_gain_1                # To be tweaked
    diffTorqueTransferGainD = driving_differential_gain
    diffTorqueTransferGainO = overrun_differential_gain

    targetVelocity, velocity_error = calculateVelocityError(Vx, Vy, lap_data, vCar_Data, ref_distance, nLap)
    futureVelocity = calculateFutureVelocity(Vx, Vy, lap_data, vCar_Data, ref_distance, nLap, futureTD)

    if StopFlag == False:
        if stop_error == True:
            rAccel = rAccel + 0.01
            torque = get_torque(rAccel, nMGU)
            MDriveRL, MDriveRR = calculateDriveTorque(torque, WheelVelocityRL, WheelVelocityRR)
            MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
            MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
            MBrakeFL = 0
            MBrakeFR = 0
            MBrakeRL = 0 
            MBrakeRR = 0
            brakePressure = 0
            brake_P = 0
            brake_I = 0
            brake_D = 0
            brake_integral = 0
            throttle_P = 0
            throttle_I = 0
            throttle_D = 0
            throttle_integral = 0
            # if velocity_error >= 0:
            #     _, throttle_integral, throttle_prev_error, throttle_P, throttle_I, throttle_D = compute_throttle_pid(
            #         velocity_error, timestep,
            #         throttle_Kp, throttle_Ki, throttle_Kd,
            #         throttle_integral, throttle_prev_error
            #     )
            if velocity_error <= 0.1:
                stop_error = False
        else:    
            if velocity_error >= 0:
                if Dil_Mode_enabled == True:
                    rAccel, throttle_integral, throttle_prev_error, throttle_P, throttle_I, throttle_D = compute_throttle_pid(velocity_error, timestep, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error)
                    torque = get_torque(rAccel, nMGU)
                    MDriveRL, MDriveRR = calculateDriveTorque(torque, WheelVelocityRL, WheelVelocityRR)
                    MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
                    MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
                    MBrakeFL = 0
                    MBrakeFR = 0
                    MBrakeRL = 0 
                    MBrakeRR = 0
                    brakePressure = 0
                    brake_P = 0
                    brake_I = 0
                    brake_D = 0
                    brake_integral = 0

                if Dil_Mode_enabled == False:
                    rAccel, throttle_integral, throttle_prev_error, throttle_P, throttle_I, throttle_D = compute_throttle_pid(velocity_error, timestep, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error)
                    torque = get_torque(rAccel, nMGU)
                    MDriveRL, MDriveRR = calculateDriveTorque(torque, WheelVelocityRL, WheelVelocityRR)
                    MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
                    MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
                    MBrakeFL = 0
                    MBrakeFR = 0
                    MBrakeRL = 0 
                    MBrakeRR = 0
                    brake_integral = 0
                    brakePressure = 0
                    brake_P = 0
                    brake_I = 0
                    brake_D = 0
                    
            if velocity_error < 0:
                if Dil_Mode_enabled == True:
                    if futureVelocity >= targetVelocity:
                        rAccel, throttle_integral, throttle_prev_error, throttle_P, throttle_I, throttle_D = compute_throttle_pid(velocity_error, timestep, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error)
                        torque = get_torque(rAccel, nMGU)
                        MDriveRL, MDriveRR = calculateDriveTorque(torque, WheelVelocityRL, WheelVelocityRR)
                        MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
                        MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
                        MBrakeFL = 0
                        MBrakeFR = 0
                        MBrakeRL = 0 
                        MBrakeRR = 0
                        brake_integral = 0
                        brakePressure = 0
                        brake_P = 0
                        brake_I = 0
                        brake_D = 0

                    else:
                        brakePressure, brake_integral, brake_prev_error, brake_P, brake_I, brake_D = compute_brake_pid(velocity_error, timestep, brake_Kp, brake_Ki, brake_Kd, brake_integral, brake_prev_error)
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

                if Dil_Mode_enabled == False:
                    if futureVelocity >= targetVelocity:
                        rAccel, throttle_integral, throttle_prev_error, throttle_P, throttle_I, throttle_D = compute_throttle_pid(velocity_error, timestep, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error)
                        torque = get_torque(rAccel, nMGU)
                        # torque = antiSlip(torque, SlipRL, SlipRR, max_long_slip, slip_constant)
                        MDriveRL, MDriveRR = calculateDriveTorque(torque, WheelVelocityRL, WheelVelocityRR)
                        MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
                        MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
                        MBrakeFL = 0
                        MBrakeFR = 0
                        MBrakeRL = 0 
                        MBrakeRR = 0
                        brake_integral = 0
                        brakePressure = 0
                        brake_P = 0
                        brake_I = 0
                        brake_D = 0

                    else:
                        brakePressure, brake_integral, brake_prev_error, brake_P, brake_I, brake_D = compute_brake_pid(velocity_error, timestep, brake_Kp, brake_Ki, brake_Kd, brake_integral, brake_prev_error)
                        targetDistance = calculateTargetVelocityDistance(ref_distance)
                        targetVelocityBrake = calculateTargetVelocityBrake(vCar_Profile_df, lap_data, targetDistance, nLap)
                        targetAvx = calculateTargetAcceleration(targetVelocityBrake, Vx, Vy)
                        torque = calculateLongitudinalThrust(targetAvx)
                        # torque = antiLockBrakes(torque, SlipFL, SlipFR, SlipRL, SlipRR, max_long_slip, slip_constant)
                        MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR = calculateBrakeTorque(torque, WheelVelocityRL, WheelVelocityRR)
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
                        rAccel = 0
                        throttle_P = 0
                        throttle_I = 0
                        throttle_D = 0
                        throttle_integral = 0
    else:
        MDriveRL = 0
        MDriveRR = 0
        MBrakeFL = 0
        MBrakeFR = 0
        MBrakeRL = 0
        MBrakeRR = 0
        rAccel = 0
        brakePressure = 0
        throttle_integral = 0
        throttle_prev_error = 0
        brake_integral = 0
        brake_prev_error = 0
        throttle_P = 0
        throttle_I = 0
        throttle_D = 0
        brake_P = 0
        brake_I = 0
        brake_D = 0
        stop_error = True
    
    targetVelocity = targetVelocity * 3.6

    return MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, rAccel, brakePressure, throttle_integral, throttle_prev_error, brake_integral, brake_prev_error, targetVelocity, throttle_P, throttle_I, throttle_D, brake_P, brake_I, brake_D, velocity_error