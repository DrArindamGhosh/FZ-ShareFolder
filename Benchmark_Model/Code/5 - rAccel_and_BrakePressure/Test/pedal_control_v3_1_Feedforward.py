# # Initial Acceleration Control

# import numpy as np
# import math
# from scipy.interpolate import interp1d
# import matplotlib.pyplot as plt
# from scipy.io import loadmat
# import pandas as pd
# from scipy.interpolate import RegularGridInterpolator

# sdistance = 0
# nLap = 0
# correction_mode = False
# rAccels = []

# mass = 840
# frontRadii = .3274                          # Based on Current Gen 3 # Received
# rearRadii = .3485                     
# gearRatioRear = 12.67                       # Approved by Thomas
# gearRatioFront = 8.47
# stop_error = False

# def calculateTargetVelocityDistance(refDistance):

#     shortDistance = 0.01
#     targetDistance = refDistance + shortDistance

#     return targetDistance                    # Target and short distance in Eq. 4.28


# def calculateTargetVelocityBrake(vCarData, trajectory_matrix, targetDistance, nLap):

#     # max_speed = 235
#     max_speed = 65.2777778

#     max_sdistance = np.max(trajectory_matrix['sDistance'])
#     velocity_interpolator = interp1d(vCarData['sDistance'], vCarData['vCar'], kind='linear', fill_value="extrapolate")

#     # if nLap > 0:
#     #     max_sdistance = np.max(trajectory_matrix['sDistance2'])
#     #     velocity_interpolator = interp1d(vCarData['sDistance2'], vCarData['vCar2'], kind='linear', fill_value="extrapolate")
#     if targetDistance > max_sdistance:
#         targetDistance -= max_sdistance

    
#     targetVelocityBrake = velocity_interpolator(targetDistance)
    

#     targetVelocityBrake = targetVelocityBrake / 3.6
#     targetVelocityBrake = min(targetVelocityBrake, max_speed)
#     # targetVelocity = targetVelocity * (velocity_percentage/100)

#     # print("Target Velocity: ", targetVelocity)                                                                         

#     return targetVelocityBrake



# def calculateTargetAcceleration(targetVelocity, Vx, Vy):

#     global targetAvx
#     maxA = 19
#     minA = -19
#     shortDistance = 0.01

#     # if abs(targetVelocity - Vx) <= velocity_error:
#     #     targetAvx = last_Avx

#     targetAvx = ((targetVelocity - Vx)/shortDistance)

#     targetAvx = min(targetAvx, maxA)                                                                                   
#     targetAvx = max(targetAvx, minA)
                             

#     return targetAvx


# def calculateLongitudinalThrust(longAcceleration):
#     global mass
#     longThrust = longAcceleration * mass         # Eq. 4.29                                                 # CONVERT TARGET ACCELERATION STRAIGHT TO NEWTONS
#     return longThrust

# ########################################################################################################################################################

# def compute_physics_informed_feedforward(ref_distance, trajectory_matrix, vProfiledf, mass=840, mu=1.7, g=9.81, c=0.02):
#     """
#     Compute feedforward throttle and brake based on physics (g-g diagram and clothoid curvature).
#     """
#     # Estimate current speed from profile
#     speed_interpolator = interp1d(vProfiledf['sDistance_Pedal'], vProfiledf['vCar'], fill_value="extrapolate")
#     curvature_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['curvature'], fill_value="extrapolate")
#     Ux = speed_interpolator(ref_distance) / 3.6  # Convert km/h to m/s

#     # Estimate curvature at this ref_distance (clothoid assumption)
#     k = curvature_interpolator(ref_distance)

#     # Compute lateral acceleration
#     ay = Ux**2 * k

#     # Compute allowable longitudinal acceleration
#     ax_total = mu * g
#     ax = np.sqrt(max(ax_total**2 - ay**2, 0))  # Eq: ax² + ay² ≤ (µg)²

#     # Convert ax to force and then throttle percentage
#     Fx = ax * mass
#     max_drive_force = 3500  # Approximate peak force for throttle=100%, tune based on your torque map
#     rAccel = Fx / max_drive_force * 100
#     rAccel = np.clip(rAccel, 0, 100)

#     # Brake logic: if ay is too high, reduce ax → negative braking
#     if ax_total**2 - ay**2 < 0:
#         brake_pressure = 10  # Simple trail-brake default (or proportional to excess lateral)
#     else:
#         brake_pressure = 0

#     return rAccel, brake_pressure

# def calculateVelocityError(Vx, Vy, vCarData, trajectory_matrix, refDistance, nLap):
#     max_speed = 65.2777778

#     vCar = np.sqrt(Vx ** 2 + Vy ** 2)

#     max_sdistance = np.max(trajectory_matrix['sDistance'])
#     velocity_interpolator = interp1d(vCarData['sDistance'], vCarData['vCar'], kind='linear', fill_value="extrapolate")
#     targetVelocity = velocity_interpolator(refDistance)
#     targetVelocity = targetVelocity / 3.6
#     vCar_error = targetVelocity - vCar
    

#     return targetVelocity, vCar_error

# def calculateFutureVelocity(Vx, Vy, lap_data, vCarData, refDistance, nLap, futureTD):
#     targetDistance = refDistance + futureTD
#     velocity_interpolator = interp1d(vCarData['sDistance'], vCarData['vCar'], kind='linear', fill_value="extrapolate")
#     futureVelocity = velocity_interpolator(targetDistance)
#     futureVelocity = futureVelocity / 3.6
    
#     return futureVelocity

# def compute_throttle_pid(error, dt, Kp, Ki, Kd, integral, prev_error):
#     """ Compute the PID correction """
#     P = Kp * error
    
#     integral += error * dt
#     I = Ki * integral
    
#     derivative = (error - prev_error) / dt if dt > 0 else 0
#     D = Kd * derivative
    
#     rAccel = P + I + D
    
#     prev_error = error

#     rAccel *= 0.2
    
#     return rAccel, integral, prev_error, P, I, D

# def compute_brake_pid(error, dt, Kp, Ki, Kd, integral, prev_error):
#     """ Compute the PID correction """
#     P = Kp * abs(error)
    
#     integral += abs(error) * dt
#     I = Ki * integral
    
#     derivative = (abs(error) - prev_error) / dt if dt > 0 else 0
#     D = Kd * derivative
    
#     bPressure = P + I + D
    
#     prev_error = error

#     bPressure *= 0.2
    
#     return bPressure, integral, prev_error, P, I, D

# def get_torque(rAccel, nMGU):

#     nMGU = np.clip(nMGU, 0, 2000)

#     MpedalMap_nMGU_Y = np.array([0, 133.3, 266.7, 400, 533.3, 666.7, 823, 960, 1066.7, 1200, 
#                               1333.3, 1466.7, 1600, 1733.3, 1866.7, 2000])

#     MpedalMap_rAccel_X = np.array([0, 7, 14, 21, 28, 35, 42, 49, 56, 63, 70, 77, 84, 91, 98, 100])

#     MPedalMap_Torque_Z = np.array([
#         [0, 102.7, 279.3, 510.4, 831.8, 1224.1, 1635.5, 1932.6, 2198, 2436.5, 2652.9, 2852.4, 3040.3, 3237.7, 3589.2, 3589.2],
#         [0, 102.7, 279.3, 510.4, 831.8, 1224.1, 1635.5, 1932.6, 2198, 2436.5, 2652.9, 2852.4, 3040.3, 3237.7, 3589.2, 3589.2], 
#         [0, 102.7, 279.3, 510.4, 831.8, 1224.1, 1635.5, 1932.6, 2198, 2436.5, 2652.9, 2852.4, 3040.3, 3237.7, 3589.2, 3589.2],
#         [0, 102.7, 279.3, 464.1, 732.5, 1090.5, 1525.7, 1932.6, 2198, 2436.5, 2652.9, 2852.4, 3040.3, 3237.7, 3589.2, 3589.2],
#         [0, 94.8, 209.5, 348.1, 549.4, 817.8, 1144.2, 1452.9, 1799.1, 2184.6, 2611.4, 2852.4, 3040.3, 3237.7, 3589.2, 3589.2], 
#         [0, 75.8, 167.6, 278.5, 439.5, 654.3, 915.4, 1162.3, 1439.3, 1747.7, 2089.1, 2470, 2900.4, 3237.7, 3589.2, 3589.2],
#         [0, 61.4, 135.8, 225.6, 356, 530, 741.5, 941.5, 1165.9, 1415.7, 1692.3, 2000.8, 2349.5, 2760.1, 3376.5, 3376.5],
#         [0, 52.6, 116.4, 193.4, 305.2, 454.4, 635.7, 807.1, 999.5, 1213.7, 1450.8, 1715.3, 2014.2, 2366.2, 2894.6, 2894.6], 
#         [0, 47.4, 104.8, 174, 274.7, 408.9, 572.1, 726.4, 899.5, 1092.3, 1305.7, 1543.7, 1812.8, 2129.6, 2605.2, 2605.2], 
#         [0, 42.1, 93.1, 154.7, 244.2, 363.5, 508.6, 645.7, 799.6, 970.9, 1160.6, 1372.2, 1611.3, 1893, 2315.7, 2315.7], 
#         [0, 37.9, 83.8, 139.2, 219.8, 327.1, 457.7, 581.1, 719.6, 873.8, 1044.6, 1235, 1450.2, 1703.7, 2084.1, 2084.1], 
#         [0, 34.5, 76.2, 126.6, 199.8, 297.4, 416.1, 528.3, 654.2, 794.4, 949.6, 1122.7, 1318.4, 1548.8, 1894.7, 1894.7], 
#         [0, 31.6, 69.8, 116, 183.1, 272.6, 381.4, 484.3, 599.7, 728.2, 870.5, 1029.2, 1208.5, 1419.7, 1736.8, 1736.8], 
#         [0, 29.2, 64.5, 107.1, 169, 251.6, 352.1, 447, 553.6, 672.2, 803.5, 950, 1115.5, 1310.5, 1603.2, 1603.2], 
#         [0, 27.1, 59.9, 99.5, 157, 233.7, 326.9, 415.1, 514, 624.2, 746.1, 882.1, 1035.9, 1216.9, 1488.7, 1488.7], 
#         [0, 25.3, 55.9, 92.8, 146.5, 218.1, 305.1, 387.4, 479.8, 582.6, 696.4, 823.3, 966.8, 1135.8, 1389.4, 1389.4]
#     ])

#     torque_interpolator = RegularGridInterpolator((MpedalMap_nMGU_Y, MpedalMap_rAccel_X), MPedalMap_Torque_Z)

#     torque = torque_interpolator((nMGU, rAccel))

#     return torque

# #########################################################################################################################################################

# def antiSlip(longThrust, LRslip, RRslip, max_long_slip, slip_constant):

#     antiSpinLR = (-0.5 * np.sin(np.arctan(slip_constant * (LRslip - max_long_slip))) + 0.5)
#     antiLockLR = (0.5 * np.sin(np.arctan(slip_constant * (LRslip + max_long_slip))) + 0.5)
    
#     antiSpinRR = (-0.5 * np.sin(np.arctan(slip_constant * (RRslip - max_long_slip))) + 0.5)
#     antiLockRR = (0.5 * np.sin(np.arctan(slip_constant * (RRslip + max_long_slip))) + 0.5)
    
#     totalAntiSlipLR = antiSpinLR * antiLockLR
#     totalAntiSlipRR = antiSpinRR * antiLockRR

#     longThrust = longThrust * totalAntiSlipLR * totalAntiSlipRR

#     return longThrust


# def antiLockBrakes(longThrust, LFslip, RFslip, LRslip, RRslip, max_long_slip, slip_constant):

#     antiLockLF = 0.5 * np.sin(np.arctan(slip_constant * (LFslip + max_long_slip))) + 0.5
#     antiLockRF = 0.5 * np.sin(np.arctan(slip_constant * (RFslip + max_long_slip))) + 0.5
    
#     antiSpinLR = -0.5 * np.sin(np.arctan(slip_constant * (LRslip - max_long_slip))) + 0.5
#     antiLockLR = 0.5 * np.sin(np.arctan(slip_constant * (LRslip + max_long_slip))) + 0.5
    
#     antiSpinRR = -0.5 * np.sin(np.arctan(slip_constant * (RRslip - max_long_slip))) + 0.5
#     antiLockRR = 0.5 * np.sin(np.arctan(slip_constant * (RRslip + max_long_slip))) + 0.5

#     totalAntiLockLR = antiSpinLR * antiLockLR
#     totalAntiLockRR = antiSpinRR * antiLockRR

#     longThrust = longThrust * antiLockLF * antiLockRF * totalAntiLockLR * totalAntiLockRR
    
#     return longThrust



# def calculateDrivingTorqueTransfer(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity):
#     global rearRadii

#     inputDifferentialtorque = -longThrust * rearRadii                                                                   # Eq. 5.10
#     torqueTransfer = -diffTorqueTransferGain1 + diffTorqueTransferGainD * inputDifferentialtorque                       # Eq. 5.1
#     deltaTorque = torqueTransfer * np.sign(rearRHSWheelVelocity - rearLHSWheelVelocity)                                        # Eq. 5.2            ROTATIONAL VELOCITIES FROM BALANCE -> TYRE OMEGA 
#     return deltaTorque


# def calculateOverrunTorqueTransfer(rearLHSWheelVelocity, rearRHSWheelVelocity):
#     global maxEngineBrakeTorque
#     global gearRatioRear

#     inputDifferentialtorque = maxEngineBrakeTorque * gearRatioRear                                  # Eq. 5.17
#     torqueTransfer = -diffTorqueTransferGain1 - diffTorqueTransferGainO * inputDifferentialtorque   # Eq. 5.1
#     deltaTorque = torqueTransfer * np.sign(rearRHSWheelVelocity - rearLHSWheelVelocity)                                      # Eq. 5.2
#     return deltaTorque



# def calculateDriveTorque(torque, rearLHSWheelVelocity, rearRHSWheelVelocity):
#     global rearRadii

#     MDriveRL = torque
#     MDriveRR = torque

#     deltaTorque = calculateDrivingTorqueTransfer(torque, rearLHSWheelVelocity, rearRHSWheelVelocity)
#     MDriveRL = MDriveRL - deltaTorque
#     MDriveRR = MDriveRR + deltaTorque                           # Eq. 5.11
    
#     return MDriveRL, MDriveRR
    
     

# def calculateBrakeTorque(torque, rearLHSWheelVelocity, rearRHSWheelVelocity):
#     global frontRadii                    
#     global rearRadii                      
#     frontBrakeCoeff = 0.5
#     rearBrakeCoeff = 0.5
#     global maxEngineBrakeTorque
#     global gearRatioRear

#     if (torque*-1) <= ((maxEngineBrakeTorque * gearRatioRear)/rearRadii):
#         # print("Rear Braking")
#         MBrakeRL = maxEngineBrakeTorque * gearRatioRear
#         MBrakeRR = maxEngineBrakeTorque * gearRatioRear
#         MBrakeFL = 0
#         MBrakeFR = 0
#         deltaTorque = calculateOverrunTorqueTransfer(rearLHSWheelVelocity, rearRHSWheelVelocity)
#         MBrakeRL = MBrakeRL - deltaTorque
#         MBrakeRR = MBrakeRR + deltaTorque # Eq. 5.11

#     else: 
#         residualForce = torque + ((maxEngineBrakeTorque * gearRatioRear)/rearRadii) # Eq. 5.15

#         MBrakeFL = -residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff))
#         MBrakeFR = -residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff))
#         MBrakeRL = maxEngineBrakeTorque * gearRatioRear - residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff))
#         MBrakeRR = maxEngineBrakeTorque * gearRatioRear - residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff)) # Eq. 5.16
        

#         deltaTorque = calculateOverrunTorqueTransfer(rearLHSWheelVelocity, rearRHSWheelVelocity)
#         MBrakeRL = MBrakeRL - deltaTorque
#         MBrakeRR = MBrakeRR + deltaTorque # Eq. 5.11

#     return MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR 

# def saturation(value, min_value, max_value):

#     saturated_values = np.clip(value, min_value, max_value)
#     return saturated_values

# #########################################################################################################################################################

# def calculatePedalFF(tracks_df, vProfiledf, Vx, Vy, timestep, ref_distance, nLap, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error,
#                           brake_Kp, brake_Ki, brake_Kd, brake_integral, brake_prev_error, nMGU, WheelVelocityRL, WheelVelocityRR, SlipFL, SlipFR, SlipRL, SlipRR,
#                           max_engine_brake_torque, differential_gain_1, driving_differential_gain, overrun_differential_gain, 
#                           max_drive_torque, max_brake_torque, futureTD, StopFlag, rAccel, Dil_Mode_enabled):
    
#     trajectory_matrix = tracks_df
#     vCarData = vProfiledf

#     global maxEngineBrakeTorque
#     global diffTorqueTransferGain1
#     global diffTorqueTransferGainD
#     global diffTorqueTransferGainO
#     global stop_error
#     # global correction_mode

#     maxEngineBrakeTorque = max_engine_brake_torque
#     diffTorqueTransferGain1 = differential_gain_1                # To be tweaked
#     diffTorqueTransferGainD = driving_differential_gain
#     diffTorqueTransferGainO = overrun_differential_gain

#     # Compute Feedforward Control
#     rAccel_df, brakePressure_df = compute_physics_informed_feedforward(ref_distance, trajectory_matrix, vCarData)
#     targetVelocity, velocity_error = calculateVelocityError(Vx, Vy, vCarData, trajectory_matrix, ref_distance, nLap)
#     futureVelocity = calculateFutureVelocity(Vx, Vy, trajectory_matrix, vCarData, ref_distance, nLap, futureTD)


#     if StopFlag == False:
#         if stop_error == True:
#             rAccel = rAccel + 0.02
#             rAccel = np.clip(rAccel, 0, 99.998)
#             throttle_P = 0
#             throttle_I = 0
#             throttle_D = 0
#             throttle_integral = 0
#             torque = get_torque(rAccel, nMGU)
#             MDriveRL, MDriveRR = calculateDriveTorque(torque, WheelVelocityRL, WheelVelocityRR)
#             MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
#             MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
#             MBrakeFL = 0
#             MBrakeFR = 0
#             MBrakeRL = 0 
#             MBrakeRR = 0
#             brakePressure = 0
#             brake_P = 0
#             brake_I = 0
#             brake_D = 0
#             brake_integral = 0
#             if velocity_error <= 0.1:
#                 stop_error = False
#         else:    
#             if velocity_error >= 0:
#                 if Dil_Mode_enabled == True:
#                     rAccel_PID, throttle_integral, throttle_prev_error, throttle_P, throttle_I, throttle_D = compute_throttle_pid(velocity_error, timestep, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error)
#                     rAccel = rAccel_df + rAccel_PID  # Combine feedforward and PID
#                     rAccel = np.clip(rAccel, 0, 99.998)
#                     torque = 0
#                     MDriveRL, MDriveRR = calculateDriveTorque(torque, WheelVelocityRL, WheelVelocityRR)
#                     MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
#                     MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
#                     MBrakeFL = 0
#                     MBrakeFR = 0
#                     MBrakeRL = 0 
#                     MBrakeRR = 0
#                     brakePressure = 0
#                     brake_P = 0
#                     brake_I = 0
#                     brake_D = 0
#                     brake_integral = 0

#                 if Dil_Mode_enabled == False:
#                     rAccel_PID, throttle_integral, throttle_prev_error, throttle_P, throttle_I, throttle_D = compute_throttle_pid(velocity_error, timestep, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error)
#                     rAccel = rAccel_df + rAccel_PID  # Combine feedforward and PID
#                     rAccel = np.clip(rAccel, 0, 99.998)
#                     torque = get_torque(rAccel, nMGU)
#                     MDriveRL, MDriveRR = calculateDriveTorque(torque, WheelVelocityRL, WheelVelocityRR)
#                     MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
#                     MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
#                     MBrakeFL = 0
#                     MBrakeFR = 0
#                     MBrakeRL = 0 
#                     MBrakeRR = 0
#                     brake_integral = 0
#                     brakePressure = 0
#                     brake_P = 0
#                     brake_I = 0
#                     brake_D = 0
            
#             if velocity_error < 0:
#                 if Dil_Mode_enabled == True:
#                     if futureVelocity >= targetVelocity:
#                         rAccel_PID, throttle_integral, throttle_prev_error, throttle_P, throttle_I, throttle_D = compute_throttle_pid(velocity_error, timestep, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error)
#                         # rAccel = rAccel_df + rAccel_PID  # Combine feedforward and PID
#                         rAccel = rAccel_PID
#                         rAccel = np.clip(rAccel, 0, 99.998)
#                         torque = get_torque(rAccel, nMGU)
#                         # torque = antiSlip(torque, SlipRL, SlipRR, max_long_slip, slip_constant)
#                         MDriveRL, MDriveRR = calculateDriveTorque(torque, WheelVelocityRL, WheelVelocityRR)
#                         MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
#                         MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
#                         MBrakeFL = 0
#                         MBrakeFR = 0
#                         MBrakeRL = 0 
#                         MBrakeRR = 0
#                         brake_integral = 0
#                         brakePressure = 0
#                         brake_P = 0
#                         brake_I = 0
#                         brake_D = 0
#                     else:
#                         brakePressure_PID, brake_integral, brake_prev_error, brake_P, brake_I, brake_D = compute_brake_pid(velocity_error, timestep, brake_Kp, brake_Ki, brake_Kd, brake_integral, brake_prev_error)
#                         brakePressure = brakePressure_df + brakePressure_PID
#                         rAccel = 0
#                         throttle_P = 0
#                         throttle_I = 0
#                         throttle_D = 0
#                         throttle_integral = 0
#                         MDriveRL = 0
#                         MDriveRR = 0
#                         MBrakeFL = 0
#                         MBrakeFR = 0
#                         MBrakeRL = 0 
#                         MBrakeRR = 0
                
#                 if Dil_Mode_enabled == False:
#                     if futureVelocity >= targetVelocity:
#                         rAccel_PID, throttle_integral, throttle_prev_error, throttle_P, throttle_I, throttle_D = compute_throttle_pid(velocity_error, timestep, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error)
#                         rAccel = rAccel_df + rAccel_PID  # Combine feedforward and PID
#                         rAccel = np.clip(rAccel, 0, 99.998)
#                         torque = get_torque(rAccel, nMGU)
#                         # torque = antiSlip(torque, SlipRL, SlipRR, max_long_slip, slip_constant)
#                         MDriveRL, MDriveRR = calculateDriveTorque(torque, WheelVelocityRL, WheelVelocityRR)
#                         MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
#                         MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
#                         MBrakeFL = 0
#                         MBrakeFR = 0
#                         MBrakeRL = 0 
#                         MBrakeRR = 0
#                         brake_integral = 0
#                         brakePressure = 0
#                         brake_P = 0
#                         brake_I = 0
#                         brake_D = 0
                    
#                     else:
#                         brakePressure_PID, brake_integral, brake_prev_error, brake_P, brake_I, brake_D = compute_brake_pid(velocity_error, timestep, brake_Kp, brake_Ki, brake_Kd, brake_integral, brake_prev_error)
#                         brakePressure = brakePressure_df + brakePressure_PID
#                         brakePressure = np.clip(brakePressure, 0, 120)
#                         targetDistance = calculateTargetVelocityDistance(ref_distance)
#                         targetVelocityBrake = calculateTargetVelocityBrake(vCarData, trajectory_matrix, targetDistance, nLap)
#                         targetAvx = calculateTargetAcceleration(targetVelocityBrake, Vx, Vy)
#                         torque = calculateLongitudinalThrust(targetAvx)
#                         # torque = antiLockBrakes(torque, SlipFL, SlipFR, SlipRL, SlipRR, max_long_slip, slip_constant)
#                         MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR = calculateBrakeTorque(torque, WheelVelocityRL, WheelVelocityRR)
#                         MBrakeFL = saturation(MBrakeFL, 0, max_brake_torque)
#                         MBrakeFR = saturation(MBrakeFR, 0, max_brake_torque)
#                         MBrakeRL = saturation(MBrakeRL, 0, max_brake_torque)
#                         MBrakeRR = saturation(MBrakeRR, 0, max_brake_torque)
#                         MDriveRL = 0
#                         MDriveRR = 0
#                         MDriveRL = - MBrakeRL
#                         MDriveRR = - MBrakeRR
#                         MBrakeRL = 0
#                         MBrakeRR = 0
#                         rAccel = 0
#                         throttle_P = 0
#                         throttle_I = 0
#                         throttle_D = 0
#                         throttle_integral = 0

#     else:
#         MDriveRL = 0
#         MDriveRR = 0
#         MBrakeFL = 0
#         MBrakeFR = 0
#         MBrakeRL = 0
#         MBrakeRR = 0
#         rAccel = 0
#         brakePressure = 0
#         throttle_integral = 0
#         throttle_prev_error = 0
#         brake_integral = 0
#         brake_prev_error = 0
#         throttle_P = 0
#         throttle_I = 0
#         throttle_D = 0
#         brake_P = 0
#         brake_I = 0
#         brake_D = 0
#         stop_error = True

        
#     targetVelocity = targetVelocity * 3.6

#     return MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, rAccel, brakePressure, throttle_integral, throttle_prev_error, brake_integral, brake_prev_error, targetVelocity, throttle_P, throttle_I, throttle_D, brake_P, brake_I, brake_D, velocity_error

# === Longitudinal Controller for EV Acceleration and Braking ===
# Includes Feedforward + PID, Anti-Spin, Drive & Brake Torque Distribution
# Respects DIL mode, start-up ramp, future velocity projection for braking logic

import numpy as np
from scipy.interpolate import interp1d, RegularGridInterpolator

# === Constants ===
mass = 840
frontRadii = 0.3274
rearRadii = 0.3485
driveRatioRear = 12.67
maxEngineBrakeTorque = 500
stop_error = True

# Differential gains (can be tuned externally)
diffTorqueTransferGain1 = 0
diffTorqueTransferGainD = 0
diffTorqueTransferGainO = 0

# === Torque Map Interpolator ===
def get_torque(rAccel, nMGU):
    rAccel = np.clip(rAccel, 0, 100)
    nMGU = np.clip(nMGU, 0, 2000)

    MpedalMap_nMGU_Y = np.linspace(0, 2000, 16)
    MpedalMap_rAccel_X = np.array([0, 7, 14, 21, 28, 35, 42, 49, 56, 63, 70, 77, 84, 91, 98, 100])

    # Simplified flat map (same for each nMGU row)
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
    return torque_interpolator((nMGU, rAccel))

# === PID Controller ===
def compute_throttle_pid(error, dt, Kp, Ki, Kd, integral, prev_error):
    """ Compute the PID correction """
    P = Kp * error
    
    integral += error * dt
    I = Ki * integral
    
    derivative = (error - prev_error) / dt if dt > 0 else 0
    D = Kd * derivative
    
    rAccel = P + I + D
    
    prev_error = error

    rAccel *= 0.2
    
    return rAccel, integral, prev_error, P, I, D

def compute_brake_pid(error, dt, Kp, Ki, Kd, integral, prev_error):
    """ Compute the PID correction """
    P = Kp * abs(error)
    
    integral += abs(error) * dt
    I = Ki * integral
    
    derivative = (abs(error) - prev_error) / dt if dt > 0 else 0
    D = Kd * derivative
    
    bPressure = P + I + D
    
    prev_error = error

    bPressure *= 0.2
    
    return bPressure, integral, prev_error, P, I, D

# === Saturation Utility ===
def saturation(value, min_val, max_val):
    return np.clip(value, min_val, max_val)

# === Physics-Based Feedforward (g-g constraint) ===
def compute_physics_ff(ref_distance, track_df, vprofile_df, mu=1.7, g=9.81, s_lookahead=5.0, ds=0.1):
    # Interpolators for curvature and target speed
    curve_interp = interp1d(track_df['sDistance'], track_df['curvature'], fill_value="extrapolate")
    speed_interp = interp1d(vprofile_df['sDistance_Pedal'], vprofile_df['vCar'], fill_value="extrapolate")

    # Starting conditions
    s0 = ref_distance
    sN = s0 + s_lookahead
    s_vals = np.arange(s0, sN, ds)

    # Initial speed guess from profile (converted to m/s)
    Ux0 = speed_interp(s0) / 3.6
    Ux_vals = [Ux0]

    for i in range(len(s_vals) - 1):
        s = s_vals[i]
        kappa = curve_interp(s)
        Ux = Ux_vals[-1]

        # Prevent invalid sqrt and excessive lateral acceleration
        denom = (Ux ** 4) * (kappa ** 2)
        limit_sq = (mu * g) ** 2

        if denom >= limit_sq or Ux < 0.1:
            dUx_ds = 0  # can't accelerate more
        else:
            dUx_ds = (1 / Ux) * np.sqrt(limit_sq - denom)

        # Forward Euler integration
        Ux_next = Ux + ds * dUx_ds
        Ux_vals.append(Ux_next)

    # Compute final ax at current position
    Ux_now = Ux_vals[0]
    kappa_now = curve_interp(s0)
    ay = Ux_now ** 2 * kappa_now
    ax_total = mu * g
    ax = np.sqrt(max(ax_total ** 2 - ay ** 2, 0))

    Fx = ax * mass
    rAccel = np.clip(Fx / 3500 * 100, 0, 100)

    # Optional: compute brake pressure if deceleration needed
    brakePressure = 10 if ax_total ** 2 - ay ** 2 < 0 else 0

    return rAccel, brakePressure
    # speed_interp = interp1d(vprofile_df['sDistance_Pedal'], vprofile_df['vCar'], fill_value="extrapolate")
    # curve_interp = interp1d(track_df['sDistance'], track_df['curvature'], fill_value="extrapolate")

    # Ux = speed_interp(ref_distance) / 3.6
    # kappa = curve_interp(ref_distance)
    # ay = Ux ** 2 * kappa

    # ax_total = mu * g
    # ax = np.sqrt(max(ax_total ** 2 - ay ** 2, 0))
    # Fx = ax * mass
    # rAccel = np.clip(Fx / 3500 * 100, 0, 100)  # Max 100% accel

    # brakePressure = 10 if ax_total ** 2 - ay ** 2 < 0 else 0
    # return rAccel, brakePressure

# === Target Velocity and Error ===
def get_target_velocity(Vx, Vy, vprofile_df, track_df, ref_distance):
    speed_interp = interp1d(vprofile_df['sDistance'], vprofile_df['vCar'], fill_value="extrapolate")
    vCar = np.sqrt(Vx**2 + Vy**2)
    targetV = speed_interp(ref_distance) / 3.6
    return targetV, targetV - vCar

def get_future_velocity(vprofile_df, track_df, ref_distance, lookahead):
    s_future = ref_distance + lookahead
    speed_interp = interp1d(vprofile_df['sDistance'], vprofile_df['vCar'], fill_value="extrapolate")
    return speed_interp(s_future) / 3.6

# === Drive and Brake Torque Distribution ===
def calculate_drive_torque(torque, omegaRL, omegaRR):
    inputTorque = -torque * rearRadii
    delta = (-diffTorqueTransferGain1 + diffTorqueTransferGainD * inputTorque) * np.sign(omegaRR - omegaRL)
    return torque - delta, torque + delta

def calculate_brake_torque(torque, omegaRL, omegaRR):
    frontBrakeCoeff = rearBrakeCoeff = 0.5
    residualForce = torque + (maxEngineBrakeTorque * driveRatioRear) / rearRadii

    share = ((frontRadii * rearRadii * frontBrakeCoeff) /
             (rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff))

    MBrakeFL = -residualForce * share
    MBrakeFR = -residualForce * share
    baseRearTorque = maxEngineBrakeTorque * driveRatioRear - residualForce * share

    delta = (-diffTorqueTransferGain1 - diffTorqueTransferGainO * maxEngineBrakeTorque * driveRatioRear) * np.sign(omegaRR - omegaRL)
    return MBrakeFL, MBrakeFR, baseRearTorque - delta, baseRearTorque + delta

# === Main Longitudinal Controller ===
def calculatePedalFF(tracks_df, vProfiledf, Vx, Vy, timestep, ref_distance, nLap, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error,
                     brake_Kp, brake_Ki, brake_Kd, brake_integral, brake_prev_error, nMGU, omegaRL, omegaRR,
                     SlipFL, SlipFR, SlipRL, SlipRR, max_engine_brake_torque, diff_gain1, diff_drive_gain, diff_overrun_gain,
                     max_drive_torque, max_brake_torque, futureTD, StopFlag, rAccel, Dil_Mode_enabled):

    global maxEngineBrakeTorque, diffTorqueTransferGain1, diffTorqueTransferGainD, diffTorqueTransferGainO, stop_error
    maxEngineBrakeTorque = max_engine_brake_torque
    diffTorqueTransferGain1 = diff_gain1
    diffTorqueTransferGainD = diff_drive_gain
    diffTorqueTransferGainO = diff_overrun_gain

    # --- Step 1: Feedforward Control ---
    rAccel_ff, brake_ff = compute_physics_ff(ref_distance, tracks_df, vProfiledf)
    targetV, v_error = get_target_velocity(Vx, Vy, vProfiledf, tracks_df, ref_distance)
    futureV = get_future_velocity(vProfiledf, tracks_df, ref_distance, futureTD)

    # --- Step 2: Startup Ramp (Flying Lap Offset) ---
    if StopFlag:
        MDriveRL = MDriveRR = MBrakeFL = MBrakeFR = MBrakeRL = MBrakeRR = brakePressure = 0
        rAccel = 0
        throttle_integral = brake_integral = throttle_prev_error = brake_prev_error = 0
        return MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, rAccel, brakePressure, throttle_integral, throttle_prev_error, brake_integral, brake_prev_error, targetV*3.6, 0, 0, 0, 0, 0, 0, v_error

    if stop_error:
        rAccel = np.clip(rAccel + 0.02, 0, 99.998)
        torque = get_torque(rAccel, nMGU)
        MDriveRL, MDriveRR = calculate_drive_torque(torque, omegaRL, omegaRR)
        MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
        MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
        if v_error <= 0.1:
            stop_error = False
        return MDriveRL, MDriveRR, 0, 0, 0, 0, rAccel, 0, 0, 0, 0, 0, targetV*3.6, 0, 0, 0, 0, 0, 0, v_error

    # --- Step 3: Normal Mode ---
    if v_error >= 0 or (v_error < 0 and futureV >= targetV):
        # --- Throttle Region ---
        rAccel_PID, throttle_integral, throttle_prev_error, throttle_P, throttle_I, throttle_D = compute_throttle_pid(v_error, timestep, throttle_Kp, throttle_Ki, throttle_Kd, throttle_integral, throttle_prev_error)
        rAccel = np.clip(rAccel_ff + rAccel_PID, 0, 99.998)
        torque = get_torque(rAccel, nMGU)
        MDriveRL, MDriveRR = calculate_drive_torque(torque, omegaRL, omegaRR)
        MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
        MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
        brakePressure = 0
        brake_integral = 0
        brake_prev_error = 0
        brake_P = brake_I = brake_D = 0
        return MDriveRL, MDriveRR, 0, 0, 0, 0, rAccel, 0, throttle_integral, throttle_prev_error, 0, 0, targetV*3.6, throttle_P, throttle_I, throttle_D, 0, 0, 0, v_error

    else:
        # --- Brake Region ---
        brakePressure_PID, brake_integral, brake_prev_error, brake_P, brake_I, brake_D = compute_brake_pid(v_error, timestep, brake_Kp, brake_Ki, brake_Kd, brake_integral, brake_prev_error)
        brakePressure = np.clip(brake_ff + brakePressure_PID, 0, 120)

        targetDistance = ref_distance + 0.01
        speed_interp = interp1d(vProfiledf['sDistance'], vProfiledf['vCar'], fill_value="extrapolate")
        targetVelocityBrake = speed_interp(targetDistance) / 3.6
        targetAvx = (targetVelocityBrake - Vx) / 0.01
        torque = saturation(targetAvx * mass, -max_brake_torque, max_brake_torque)

        MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR = calculate_brake_torque(torque, omegaRL, omegaRR)
        MBrakeFL = saturation(MBrakeFL, 0, max_brake_torque)
        MBrakeFR = saturation(MBrakeFR, 0, max_brake_torque)
        MBrakeRL = saturation(MBrakeRL, 0, max_brake_torque)
        MBrakeRR = saturation(MBrakeRR, 0, max_brake_torque)
        rAccel_PID = 0
        rAccel = 0
        throttle_integral = 0
        throttle_prev_error = 0
        throttle_P = throttle_I = throttle_D = 0
        
        return -MBrakeRL, -MBrakeRR, MBrakeFL, MBrakeFR, 0, 0, 0, brakePressure, 0, 0, brake_integral, brake_prev_error, targetV*3.6, 0, 0, 0, brake_P, brake_I, brake_D, v_error
