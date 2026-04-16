import numpy as np
import math
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from scipy.io import loadmat

mat_data = loadmat('T_MEX_CANOPY_V3_P200.mat')

sdistance = 0
nLap = 0

frontRadii = .3274                    # Based on Current Gen 3
rearRadii = .3485                     
maxEngineBrakeTorque = 4000           # 250 front, 350 rear?
frontBrakeCoeff = 0.4
rearBrakeCoeff = 0.6
gearRatioRear = 12.67                       # To be acquired
gearRatioFront = 8.47 
diffTorqueTransferGain1 = 1
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
    shortDistance = 0.5
    targetDistance = refDistance + shortDistance
    print("Target Distance: ", targetDistance)

    return targetDistance, shortDistance                    # Target and short distance in Eq. 4.28


def calculateTargetVelocity(trajectory_matrix, targetDistance, refDistance):
    targetVelocities = []

    max_speed = 235
    max_sdistance = np.max(trajectory_matrix[:, 0])
    curv_interpolator = interp1d(trajectory_matrix[:, 0], trajectory_matrix[:, 1], kind='linear', fill_value="extrapolate")
    if targetDistance > max_sdistance:
            targetDistance -= max_sdistance

    curv = curv_interpolator(targetDistance)
    radius = 1/abs(curv)
    # k = 20  # Tuning constant, adjust based on your needs and testing
    # epsilon = 0.01  # To avoid division by zero
    # targetVelocity = k / np.sqrt(abs(curv) + epsilon)        # Target Velocity in Eq. 4.28
    targetVelocity = np.sqrt(radius * (9.81*1.5))*3.6 # radius of curv x max lat acceleration
    targetVelocity = min(targetVelocity, max_speed)
    print("Target Velocity: ", targetVelocity)
    targetVelocities.append(targetVelocity)

    return targetVelocity, targetVelocities


def calculateTargetAcceleration(idealVelocity, Vx, shortDistance):

    accelerations = []

    Avx = (idealVelocity - Vx)/shortDistance                # Eq. 4.28/5.4
    print("Target Acceleration: ", Avx)
    accelerations.append(Avx)

    return Avx, accelerations


def calculateLongitudinalThrust(longAcceleration, longVelocity):
    longThrust = longAcceleration # + longVelocity**2         # Eq. 4.29
    print("Longitudunal Thrust: ",longThrust)
    return longThrust


#######################################################################
# Chapter 5

def calculateDrivingTorqueTransfer(longThrust):
    global rearRadii

    inputDifferentialtorque = -longThrust * rearRadii                                             # Eq. 5.10
    torqueTransfer = -diffTorqueTransferGain1 + diffTorqueTransferGain2 * inputDifferentialtorque # Eq. 5.1
    deltaTorque = torqueTransfer #* (velocityRR - velocityLR)                                      # Eq. 5.2
    return deltaTorque

def calculateOverrunTorqueTransfer():
    global maxEngineBrakeTorque
    global gearRatio

    inputDifferentialtorque = maxEngineBrakeTorque * gearRatio                                    # Eq. 5.17
    torqueTransfer = -diffTorqueTransferGain1 - diffTorqueTransferGain2 * inputDifferentialtorque # Eq. 5.1
    deltaTorque = torqueTransfer #* (velocityRR - velocityLR)                                      # Eq. 5.2
    return deltaTorque


def calculateDriveTorque(longThrust):
    global rearRadii
    
    MDriveRL = 0.5 * longThrust * rearRadii
    MDriveRR = 0.5 * longThrust * rearRadii          # Eq. 5.9
    deltaTorque = calculateDrivingTorqueTransfer(longThrust)
    MDriveRL = MDriveRL - deltaTorque
    MDriveRR = MDriveRR + deltaTorque                           # Eq. 5.11
    return MDriveRL, MDriveRR
    
     

def calculateBrakeTorque(longThrust):
    global frontRadii                    # Based on Gen 2 Car
    global rearRadii                     # Based on Gen 2 Car 
    global frontBrakeCoeff
    global rearBrakeCoeff
    global maxEngineBrakeTorque
    global gearRatio

    if longThrust <= (maxEngineBrakeTorque*gearRatio)/rearRadii:    # Eq. 5.14
        MBrakeRL, MBrakeRR = maxEngineBrakeTorque * gearRatio       # Unsure
        deltaTorque = calculateOverrunTorqueTransfer(longThrust)
        MBrakeRL = MBrakeRL - deltaTorque
        MBrakeRR = MBrakeRR + deltaTorque                           # Eq. 5.11
        MBrakeFL, MBrakeFR = 0                                      # Eq. 12

    else:
        residualForce = longThrust + ((maxEngineBrakeTorque * gearRatio)/rearRadii) # Eq. 5.15
        MBrakeFL, MBrakeFR = -residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff))
        MBrakeRL, MBrakeRR = maxEngineBrakeTorque * gearRatio - residualForce * ((frontRadii * rearRadii * frontBrakeCoeff)/(rearRadii * frontBrakeCoeff + frontRadii * rearBrakeCoeff)) # Eq. 5.16
        deltaTorque = calculateOverrunTorqueTransfer(longThrust)
        MBrakeRL = MBrakeRL - deltaTorque
        MBrakeRR = MBrakeRR + deltaTorque # Eq. 5.11

    return MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR 


##############################################################################################################################

# MAIN FUNCTION

def calculateTorqueControl(Vx, Vy, yawAngle, x, y):

    # Constants
    trajectory_matrix = mat_data['T']
    timestep = 0.002

    ref_distance, ref_orientation = refStateNow(trajectory_matrix, Vx, Vy, yawAngle, timestep)

    targetDistance, shortDistance = calculateTargetVelocityDistance(ref_distance)

    targetVelocity, targetVelocityArray = calculateTargetVelocity(trajectory_matrix, targetDistance, ref_distance)

    longAcceleration, longAccelerationArray = calculateTargetAcceleration(targetVelocity, Vx, shortDistance)

    longThrust = calculateLongitudinalThrust(longAcceleration, Vx)


    if longThrust > 0:
        MDriveRL, MDriveRR = calculateDriveTorque(longThrust)
        MBrakeFL = 0
        MBrakeFR = 0
        MBrakeRL = 0 
        MBrakeRR = 0

    if longThrust < 0:
        MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR = calculateBrakeTorque(longThrust)
        MDriveRL = 0
        MDriveRR = 0

    print("MDriveRL: ", MDriveRL)
    print("MDriveRR: ", MDriveRR)
    print("MBrakeFL: ", MBrakeFL)
    print("MBrakeFR: ", MBrakeFR)
    print("MBrakeRL: ", MBrakeRL)
    print("MBrakeRR: ", MBrakeRR)

    

    return MDriveRL, MDriveRR, MBrakeFL, MBrakeFR, MBrakeRL, MBrakeRR, targetVelocityArray, longAccelerationArray