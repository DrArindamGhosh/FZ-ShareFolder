import numpy as np
import math
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy.io import loadmat
import pandas as pd

mat_data = loadmat('T_MEX_CANOPY_V3_P200.mat')

file_path = 'TargetVelocityMexico.xlsx'
vCarData = pd.read_excel(file_path)
vCarData.head(), vCarData.columns

sdistance = 0
nLap = 0

mass = 840
frontRadii = .3274                    # Based on Current Gen 3 # Received
rearRadii = .3485                     
maxEngineBrakeTorque = 4000           # 250 front, 350 rear? # 4000 is fine as a range
frontBrakeCoeff = 0.5                 # 0.5 is fine for both
rearBrakeCoeff = 0.5
gearRatioRear = 12.67                       # Approved by Thomas
gearRatioFront = 8.47 
diffTorqueTransferGain1 = 1                 # To be tweaked
diffTorqueTransferGain2 = 1

###############################
# Chapter 4

def refStateNow(trajectory_matrix, Vx, Vy, orientation, timestep):
    # Calculates the current reference state of the vehicle given the trajectory T, velocity V, orientation, and a timestep.
    # The code calculates the reference state, taking into account the vehicle’s orientation and updating the reference distance based on velocity and orientation
    # Corresponds with the paper’s approach of continuously updating the vehicle's reference position along the trajectory

    global sdistance
    global nLap

    max_sdistance = np.max(trajectory_matrix[:, 0])

    V_magnitude = np.sqrt(Vx**2 + Vy**2)
    
    # Create an interpolation function based on trajectory data
    phi_interpolator = interp1d(trajectory_matrix[:, 0], trajectory_matrix[:, 2], kind='linear', fill_value="extrapolate")
    phi = phi_interpolator(sdistance)


    sdistance += V_magnitude * timestep  * max(0, np.cos(orientation - phi))

    if sdistance >= max_sdistance:
        sdistance -= max_sdistance
        phi = phi_interpolator(sdistance)

        # Determine if we are going clockwise or anticlockwise
        if trajectory_matrix[0, 2] > trajectory_matrix[-1, 2]:
            # Anticlockwise
            nLap -= 1
        else:
            # Clockwise
            nLap += 1

    # print(f"sdistance: " , sdistance)

    refDistance = sdistance
    refOrientation = phi + nLap * 2 * np.pi

    return refDistance, refOrientation


def calculateTargetVelocityDistance(refDistance):
    shortDistance = .5                                     # Constant
    targetDistance = refDistance + shortDistance
    print("Target Distance: ", targetDistance)

    return targetDistance, shortDistance                    # Target and short distance in Eq. 4.28


def calculateTargetVelocity(vCarData, trajectory_matrix, targetDistance, steering_input):

    targetVelocities = []

    max_speed = 235
    max_sdistance = np.max(trajectory_matrix[:, 0])
    velocity_interpolator = interp1d(vCarData['sDistance2'], vCarData['vCar'], kind='linear', fill_value="extrapolate")
    if targetDistance > max_sdistance:
            targetDistance -= max_sdistance
    targetVelocity = velocity_interpolator(targetDistance)

    # curv = curv_interpolator(targetDistance)
    # radius = 1/abs(curv)
    # # k = 20  # Tuning constant, adjust based on your needs and testing
    # # epsilon = 0.01  # To avoid division by zero
    # # targetVelocity = k / np.sqrt(abs(curv) + epsilon)        # Target Velocity in Eq. 4.28
    # targetVelocity = np.sqrt(radius * (9.81*1.5))*3.6 # radius of curv x max lat acceleration

    # targetVelocity = velocity_at_targetDistance
    # targetVelocity = 235

    # steering_input = abs(steering_input)
    # targetVelocity = targetVelocity * (steering_input * 0.1)



    targetVelocity = min(targetVelocity, max_speed)
    print("Target Velocity: ", targetVelocity)
    targetVelocities.append(targetVelocity)                                                                         # TAKE TARGET VELOCITIES FROM LEGACY DATA BUT TAKE INTO ACCOUNT PREVIOUS STEERING ANGLE

    return targetVelocity, targetVelocities


def calculateTargetAcceleration(idealVelocity, Vx, shortDistance):

    accelerations = []

    Avx = (idealVelocity - Vx)/shortDistance                # Eq. 4.28/5.4
    print("Target Acceleration: ", Avx)
    accelerations.append(Avx)                                                                                  

    return Avx, accelerations


def calculateLongitudinalThrust(longAcceleration, longVelocity):
    global mass
    longThrust = longAcceleration * mass         # Eq. 4.29                                                 # CONVERT TARGET ACCELERATION STRAIGHT TO NEWTONS
    print("Longitudunal Thrust: ", longThrust)
    return longThrust


#######################################################################
# Chapter 5


def antiLock(longThrust, LFslip, RFslip, LRslip, RRslip):

    constantB = 10
    maxSlipL = 0.1
    overallSlip = (LFslip * RFslip * LRslip * RRslip) / 4

    antiLockLF = 0.5 * np.sin(np.arctan(constantB * (overallSlip + maxSlipL))) + 0.5
    antiLockRF = 0.5 * np.sin(np.arctan(constantB * (overallSlip + maxSlipL))) + 0.5                                                                              # Eq. 5.6
    antiLockLR = (-0.5 * np.sin(np.arctan(constantB * (overallSlip + maxSlipL))) + 0.5) * (0.5 * np.sin(np.arctan(constantB * (overallSlip + maxSlipL))))
    antiLockRR = (-0.5 * np.sin(np.arctan(constantB * (overallSlip + maxSlipL))) + 0.5) * (0.5 * np.sin(np.arctan(constantB * (overallSlip + maxSlipL))))         # Eq. 5.7
    longThrust = longThrust * antiLockLF * antiLockRF * antiLockLR * antiLockRR                                                                                  # Eq. 5.8
    
    print("New Long Thrust: ", longThrust)
    return longThrust

def calculateDrivingTorqueTransfer(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity):
    global rearRadii

    inputDifferentialtorque = -longThrust * rearRadii                                             # Eq. 5.10
    torqueTransfer = -diffTorqueTransferGain1 + diffTorqueTransferGain2 * inputDifferentialtorque # Eq. 5.1
    deltaTorque = torqueTransfer * (rearRHSWheelVelocity - rearLHSWheelVelocity)                                      # Eq. 5.2            ROTATIONAL VELOCITIES FROM BALANCE -> TYRE OMEGA 
    return deltaTorque

def calculateOverrunTorqueTransfer(rearLHSWheelVelocity, rearRHSWheelVelocity):
    global maxEngineBrakeTorque
    global gearRatioRear

    inputDifferentialtorque = maxEngineBrakeTorque * gearRatioRear                                  # Eq. 5.17
    torqueTransfer = -diffTorqueTransferGain1 - diffTorqueTransferGain2 * inputDifferentialtorque # Eq. 5.1
    deltaTorque = torqueTransfer * (rearRHSWheelVelocity - rearLHSWheelVelocity)                                      # Eq. 5.2
    return deltaTorque


def calculateDriveTorque(longThrust, rearLHSWheelVelocity, rearRHSWheelVelocity):
    global rearRadii
    
    MDriveRL = 0.5 * longThrust * rearRadii
    MDriveRR = 0.5 * longThrust * rearRadii          # Eq. 5.9
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


##############################################################################################################################

# MAIN FUNCTION

def calculateTorqueControl(Vx, Vy, yawAngle, x, y, WheelVelocityRL, WheelVelocityRR, SlipFL, SlipFR, SlipRL, SlipRR, steering_input):

    # Constants
    trajectory_matrix = mat_data['T']
    timestep = 0.002

    ref_distance, ref_orientation = refStateNow(trajectory_matrix, Vx, Vy, yawAngle, timestep)

    targetDistance, shortDistance = calculateTargetVelocityDistance(ref_distance)

    targetVelocity, targetVelocityArray = calculateTargetVelocity(vCarData, trajectory_matrix, targetDistance, ref_distance)

    longAcceleration, longAccelerationArray = calculateTargetAcceleration(targetVelocity, Vx, shortDistance)

    longThrust = calculateLongitudinalThrust(longAcceleration, Vx)

    longThrust = antiLock(longThrust, SlipFL, SlipFR, SlipRL, SlipRR)


    if longThrust > 0:
        MDriveRL, MDriveRR = calculateDriveTorque(longThrust, WheelVelocityRL, WheelVelocityRR)
        MBrakeFL = 0
        MBrakeFR = 0
        MBrakeRL = 0 
        MBrakeRR = 0

    if longThrust < 0:
        MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR = calculateBrakeTorque(longThrust, WheelVelocityRL, WheelVelocityRR)
        MDriveRL = 0
        MDriveRR = 0

    # print("MDriveRL: ", MDriveRL)
    # print("MDriveRR: ", MDriveRR)
    # print("MBrakeFL: ", MBrakeFL)
    # print("MBrakeFR: ", MBrakeFR)
    # print("MBrakeRL: ", MBrakeRL)
    # print("MBrakeRR: ", MBrakeRR)

    

    return MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, targetVelocityArray, longAccelerationArray