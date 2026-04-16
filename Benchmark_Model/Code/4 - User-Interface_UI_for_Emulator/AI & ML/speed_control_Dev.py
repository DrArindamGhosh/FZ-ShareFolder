import numpy as np
import math
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy.io import loadmat
import pandas as pd

sdistance = 0
# nLap = 0

mass = 840
frontRadii = .3274                          # Based on Current Gen 3 # Received
rearRadii = .3485                     
gearRatioRear = 12.67                       # Approved by Thomas
gearRatioFront = 8.47 



###############################
# Chapter 4

def refStateNow(trajectory_matrix, Vx, Vy, orientation, timestep):
    # Calculates the current reference state of the vehicle given the trajectory T, velocity V, orientation, and a timestep.
    # The code calculates the reference state, taking into account the vehicle’s orientation and updating the reference distance based on velocity and orientation
    # Corresponds with the paper’s approach of continuously updating the vehicle's reference position along the trajectory

    global sdistance
    # global nLap

    max_sdistance = np.max(trajectory_matrix['sDistance'])

    V_magnitude = np.sqrt(Vx**2 + Vy**2)
    
    # Create an interpolation function based on trajectory data
    phi_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['Orientation'], kind='linear', fill_value="extrapolate")
    phi = phi_interpolator(sdistance)


    sdistance += V_magnitude * timestep  * max(0, np.cos(orientation - phi))

    if sdistance >= max_sdistance:
        sdistance -= max_sdistance
        phi = phi_interpolator(sdistance)
        # nLap += 1

        # # Determine if we are going clockwise or anticlockwise
        # if trajectory_matrix[0, 2] > trajectory_matrix[-1, 2]:
        #     # Anticlockwise
        #     nLap -= 1
        # else:
        #     # Clockwise
        #     nLap += 1

    # print(f"sdistance: " , sdistance)

    # refDistance = sdistance + 30.98
    refDistance = sdistance
    # refOrientation = phi + nLap * 2 * np.pi

    return refDistance


def calculateTargetVelocityDistance(refDistance):
    shortDistance = 0.01                                   # Constant
    targetDistance = refDistance + shortDistance
    # print("Target Distance: ", targetDistance)

    return targetDistance, shortDistance                    # Target and short distance in Eq. 4.28


def calculateTargetVelocity(vCarData, trajectory_matrix, targetDistance, velocity_percentage, nLap):

    # max_speed = 235
    max_speed = 65.2777778

    max_sdistance = np.max(trajectory_matrix['sDistance'])
    velocity_interpolator = interp1d(vCarData['sDistance'], vCarData['vCar'], kind='linear', fill_value="extrapolate")
    # if nLap > 0:
    #     max_sdistance = np.max(trajectory_matrix['sDistance'])
    #     velocity_interpolator = interp1d(vCarData['sDistance2'], vCarData['vCar2'], kind='linear', fill_value="extrapolate")
    if targetDistance > max_sdistance:
        targetDistance -= max_sdistance
    targetVelocity = velocity_interpolator(targetDistance)
    

    targetVelocity = targetVelocity / 3.6
    targetVelocity = min(targetVelocity, max_speed)
    # targetVelocity = targetVelocity * (velocity_percentage/100)

    # print("Target Velocity: ", targetVelocity)                                                                         # TAKE TARGET VELOCITIES FROM LEGACY DATA BUT TAKE INTO ACCOUNT PREVIOUS STEERING ANGLE

    return targetVelocity



def calculateTargetAcceleration(targetVelocity, Vx, Vy, shortDistance):

    targetAvx = ((targetVelocity - Vx)/shortDistance) #* 1/scalingFactor            # Eq. 4.28/5.4
    # Avx = Avx / 12960
    # print("Target Acceleration: ", targetAvx)
                                                                                 

    return targetAvx


def calculateLongitudinalThrust(longAcceleration):
    global mass
    longThrust = longAcceleration * mass         # Eq. 4.29                                                 # CONVERT TARGET ACCELERATION STRAIGHT TO NEWTONS
    # print(f"Longitudunal Thrust: {longThrust}\n")
    oldLongThrust = longThrust
    return longThrust, oldLongThrust





#######################################################################
# Chapter 5

def antiSlip(longThrust, LRslip, RRslip):

    constantB = 10
    maxSlipL = 0.1         # Eq. 5.7
    antiSpinLR = -0.5 * np.sin(np.arctan(constantB * (LRslip + maxSlipL))) + 0.5
    antiSpinRR = -0.5 * np.sin(np.arctan(constantB * (RRslip + maxSlipL))) + 0.5
    # print("Antislip: ", antiSpinLR)
    longThrust = longThrust * antiSpinLR * antiSpinRR     
    slipThrust = longThrust                                                                                                       # Eq. 5.8
    
    # print("New Long Thrust: ", longThrust)

    return longThrust, slipThrust


def antiLockBrakes(longThrust, LFslip, RFslip, LRslip, RRslip):

    constantB = 10
    maxSlipL = 0.1
    # overallSlip = (LFslip + RFslip + LRslip + RRslip) / 4

    antiLockLF = 0.5 * np.sin(np.arctan(constantB * (LFslip + maxSlipL))) + 0.5
    antiLockRF = 0.5 * np.sin(np.arctan(constantB * (RFslip + maxSlipL))) + 0.5
    antiLockLR = 0.5 * np.sin(np.arctan(constantB * (LRslip + maxSlipL))) + 0.5
    antiLockRR = 0.5 * np.sin(np.arctan(constantB * (RRslip + maxSlipL))) + 0.5

    longThrust = longThrust * antiLockLF * antiLockRF * antiLockLR * antiLockRR                                                                                   # Eq. 5.8
    
    # print("New Long Thrust: ", longThrust)
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



def calculateDriveTorque(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity):
    global rearRadii
    
    MDriveRL = 0.5 * longThrust * rearRadii
    MDriveRR = 0.5 * longThrust * rearRadii          # Eq. 5.9
    MDriveRL = MDriveRL # + 7
    MDriveRR = MDriveRR # + 7
    deltaTorque = calculateDrivingTorqueTransfer(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity)
    MDriveRL = MDriveRL - deltaTorque
    MDriveRR = MDriveRR + deltaTorque                           # Eq. 5.11
    
    return MDriveRL, MDriveRR
    
     

def calculateBrakeTorque(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity):
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
                           mEBT, diffGain1, diffGainD, diffGainO, frontCoeff, rearCoeff, max_drive_torque, max_brake_torque, timestep, velocity_percentage, nLap):

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

    ref_distance = refStateNow(trajectory_matrix, Vx, Vy, yawAngle, timestep)

    targetDistance, shortDistance = calculateTargetVelocityDistance(ref_distance)

    targetVelocity = calculateTargetVelocity(vCarData, trajectory_matrix, targetDistance, velocity_percentage, nLap)

    # scalingFactor = calculateScalingFactor(trajectory_matrix, ref_distance, x, y, yawAngle, Vx, Vy)

    longAcceleration = calculateTargetAcceleration(targetVelocity, Vx, Vy, shortDistance)

    longThrust, oldLongThrust = calculateLongitudinalThrust(longAcceleration)

    


    if longThrust > 0:
        longThrust, slipThrust = antiSlip(longThrust, SlipRL, SlipRR)
        MDriveRL, MDriveRR = calculateDriveTorque(longThrust, WheelVelocityRL, WheelVelocityRR)
        MDriveRL = saturation(MDriveRL, 0, max_drive_torque)
        MDriveRR = saturation(MDriveRR, 0, max_drive_torque)
        if Vx < 3:
            MDriveRL = MDriveRL + 200
            MDriveRR = MDriveRR + 200
        MBrakeFL = 0
        MBrakeFR = 0
        MBrakeRL = 0 
        MBrakeRR = 0

    if longThrust < 0:
        longThrust = antiLockBrakes(longThrust, SlipFL, SlipFR, SlipRL, SlipRR)
        slipThrust = 0
        MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR = calculateBrakeTorque(longThrust, WheelVelocityRL, WheelVelocityRR)
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

    

    return MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, targetVelocity, longAcceleration, slipThrust, oldLongThrust, nLap