import numpy as np
import math
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy.io import loadmat
import pandas as pd

file_path = 'SaoPaulo_Traj_Mat.xlsx'
trajectory_matrix = pd.read_excel(file_path)
trajectory_matrix.head(), trajectory_matrix.columns

file_path = 'SaoPauloVelocity.xlsx'
vCarData = pd.read_excel(file_path)
vCarData.head(), vCarData.columns

sdistance = 0
nLap = 0

mass = 840
frontRadii = .3274                          # Based on Current Gen 3 # Received
rearRadii = .3485                     
frontBrakeCoeff = 0.5                       # 0.5 is fine for both
rearBrakeCoeff = 0.5
gearRatioRear = 12.67                       # Approved by Thomas
gearRatioFront = 8.47 
maxEngineBrakeTorque = 1.0
# diffTorqueTransferGain1 = 0.3                 # To be tweaked
# diffTorqueTransferGainD = 0.3
# diffTorqueTransferGainO = 0.3             # Balance Gains
diffTorqueTransferGain1 = 0.1            # To be tweaked
diffTorqueTransferGainD = 0.1
diffTorqueTransferGainO = 0.1                 # Emulator Gains


###############################
# Chapter 4

def refStateNow(trajectory_matrix, Vx, Vy, orientation, timestep):
    # Calculates the current reference state of the vehicle given the trajectory T, velocity V, orientation, and a timestep.
    # The code calculates the reference state, taking into account the vehicle’s orientation and updating the reference distance based on velocity and orientation
    # Corresponds with the paper’s approach of continuously updating the vehicle's reference position along the trajectory

    global sdistance
    global nLap

    max_sdistance = np.max(trajectory_matrix['sDistance'])

    V_magnitude = np.sqrt(Vx**2 + Vy**2)
    
    # Create an interpolation function based on trajectory data
    phi_interpolator = interp1d(trajectory_matrix['sDistance'], trajectory_matrix['Orientation2'], kind='linear', fill_value="extrapolate")
    phi = phi_interpolator(sdistance)


    sdistance += V_magnitude * timestep  * max(0, np.cos(orientation - phi))

    if sdistance >= max_sdistance:
        sdistance -= max_sdistance
        phi = phi_interpolator(sdistance)

        # # Determine if we are going clockwise or anticlockwise
        # if trajectory_matrix[0, 2] > trajectory_matrix[-1, 2]:
        #     # Anticlockwise
        #     nLap -= 1
        # else:
        #     # Clockwise
        #     nLap += 1

    # print(f"sdistance: " , sdistance)

    refDistance = sdistance
    refOrientation = phi + nLap * 2 * np.pi

    return refDistance, refOrientation


def calculateTargetVelocityDistance(refDistance):
    shortDistance = .01                                   # Constant
    targetDistance = refDistance + shortDistance
    print("Target Distance: ", targetDistance)

    return targetDistance, shortDistance                    # Target and short distance in Eq. 4.28


def calculateTargetVelocity(vCarData, trajectory_matrix, targetDistance, steering_input):

    # max_speed = 235
    max_speed = 65.2777778

    max_sdistance = np.max(trajectory_matrix['sDistance'])
    velocity_interpolator = interp1d(vCarData['sDistance'], vCarData['vCar2'], kind='linear', fill_value="extrapolate")
    if targetDistance > max_sdistance:
            targetDistance -= max_sdistance
    targetVelocity = velocity_interpolator(targetDistance)

    # steering_input = abs(steering_input)
    # targetVelocity = targetVelocity * (steering_input * 0.1)
    targetVelocity = targetVelocity / 3.6
    targetVelocity = min(targetVelocity, max_speed)
    

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
    maxSlipL = 0.1
    # overallSlip = (LRslip + RRslip) / 2
    # antiSpinLR = (-0.5 * np.sin(np.arctan(constantB * (LRslip - maxSlipL))) + 0.5) * (0.5 * np.sin(np.arctan(constantB * (LRslip + maxSlipL))))
    # antiSpinRR = (-0.5 * np.sin(np.arctan(constantB * (RRslip - maxSlipL))) + 0.5) * (0.5 * np.sin(np.arctan(constantB * (RRslip + maxSlipL))))         # Eq. 5.7
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

    # antiLockLF = 0.5 * np.sin(np.arctan(constantB * (LFslip + maxSlipL))) + 0.5
    # antiLockRF = 0.5 * np.sin(np.arctan(constantB * (RFslip + maxSlipL))) + 0.5                                                                              # Eq. 5.6
    # antiLockLR = (-0.5 * np.sin(np.arctan(constantB * (LRslip - maxSlipL))) + 0.5) * (0.5 * np.sin(np.arctan(constantB * (LRslip + maxSlipL))))
    # antiLockRR = (-0.5 * np.sin(np.arctan(constantB * (RRslip - maxSlipL))) + 0.5) * (0.5 * np.sin(np.arctan(constantB * (RRslip + maxSlipL))))         # Eq. 5.7
    # print("AntiLock F: ",antiLockLF)
    # print("AntiLock R: ",antiLockLR)
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

    MDriveRL = saturation(MDriveRL, 0, 4000)
    MDriveRR = saturation(MDriveRR, 0, 4000)
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

    MBrakeFL = saturation(MBrakeFL, 0, 4000)
    MBrakeFR = saturation(MBrakeFR, 0, 4000)
    MBrakeRL = saturation(MBrakeRL, 0, 4000)
    MBrakeRR = saturation(MBrakeRR, 0, 4000)

    return MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR 

def saturation(value, min_value, max_value):

    saturated_values = np.clip(value, min_value, max_value)
    return saturated_values


##############################################################################################################################

# MAIN FUNCTION

def calculateTorqueControl(Vx, Vy, yawAngle, x, y, WheelVelocityRL, WheelVelocityRR, SlipFL, SlipFR, SlipRL, SlipRR, steering_input):

    # Constants
    timestep = 0.002
    # Vx = Vx * (1/3.6)

    ref_distance, ref_orientation = refStateNow(trajectory_matrix, Vx, Vy, yawAngle, timestep)

    targetDistance, shortDistance = calculateTargetVelocityDistance(ref_distance)

    targetVelocity = calculateTargetVelocity(vCarData, trajectory_matrix, targetDistance, ref_distance)

    # scalingFactor = calculateScalingFactor(trajectory_matrix, ref_distance, x, y, yawAngle, Vx, Vy)

    longAcceleration = calculateTargetAcceleration(targetVelocity, Vx, Vy, shortDistance)

    longThrust, oldLongThrust = calculateLongitudinalThrust(longAcceleration)

    


    if longThrust > 0:
        longThrust, slipThrust = antiSlip(longThrust, SlipRL, SlipRR)
        MDriveRL, MDriveRR = calculateDriveTorque(longThrust, WheelVelocityRL, WheelVelocityRR)
        MBrakeFL = 0
        MBrakeFR = 0
        MBrakeRL = 0 
        MBrakeRR = 0

    if longThrust < 0:
        longThrust = antiLockBrakes(longThrust, SlipFL, SlipFR, SlipRL, SlipRR)
        slipThrust = 0
        MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR = calculateBrakeTorque(longThrust, WheelVelocityRL, WheelVelocityRR)
        # MDriveRL = 0
        # MDriveRR = 0
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

    

    return MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, targetVelocity, longAcceleration, slipThrust, oldLongThrust