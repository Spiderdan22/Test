import sys
sys.path.append(r"C:\Users\dm8g22\Documents\Quanser QArm Student\libraries\python")
from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities
from pal.utilities.vision import Camera3D
from QArm_Workspace_Static_Startup import QArmWorkspace
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import warnings
warnings.filterwarnings('ignore')
import pandas as pd
import numpy as np
import threading
import time
import cv2
import csv
import os
from pathlib import Path
import threading

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Introduction

'''This code is designed to automate a pick-and-drop task using the QArm in both a virtual warehouse environment within the Quanser Interactive Lab Open World module 
and a physical operational environment. It spawns four cell widgets at predefined locations, randomly assigning one of three colours and two types to each. 
These cells are picked up and placed into designated bins based on their detected colour and type.

The detection is carried out using a RealSense D415 camera attached to the QArm, which analyses the pixel data in HSV images to determine each cell's colour and size 
within predefined regions of interest. Additionally, the system continuously logs real-time joint currents and PWM values in a CSV file, which are then used to 
calculate joint power and energy consumption throughout the operation.'''

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

# region: Setup
##### Importing QArm Workspace class #####

'''Import the QArm Workspace from class for running simulation in the open world environment in the Quanser Interactive Lab. 
Relaunch the warehouse environment if the workspace objects are not spawned.
This section can be commented out if run lab experiment. '''



##### Timing parameters and methods #####
startTime = time.time()
def elapsedTime():
    return time.time() - startTime

##### Load QArm in Position Mode and the RealSense camera #####

''' QArm hardware setup:
Set 'hardware' to 1 for physical experiments, and to 0 for interactive lab simulations. '''

# QArm
hardware = 0 # Set hardware = 1 for physical world, and = 0 for virtual world
save_dir = r"C:\Users\dm8g22\Documents" # Change this to your chosen directory!!!

if hardware == 0:
    filePathData = os.path.join(save_dir, 'Time_Current_PWM_Energy_Virtual.csv') 
    filePathImage = os.path.join(save_dir, 'Time_Current_PWM_Energy_Virtual.png')
    workSpace = QArmWorkspace()
    workSpace.connect_to_qlabs()
    workSpace.initialise_system()
    workSpace.create_workspace()
    workSpace.cell_spawn()
    workSpace.start_real_time_model()
    workSpace.control_conveyor(0.2)
else:
    filePathData = os.path.join(save_dir, 'Time_Current_PWM_Energy_Real.csv')
    filePathImage = os.path.join(save_dir, 'Time_Current_PWM_Energy_Real.png')
# BEGIN SECTION
## ------------------------------------------------------------------------------------------------------
                                                                                                        
myArm = QArm(hardware = hardware, hilPortNumber = 18900)
myArmUtilities = QArmUtilities()
atHomePosition = False # Initialise flag for home position

myArm2 = QArm(hardware = hardware, hilPortNumber = 18902)
myArmUtilities2 = QArmUtilities()

# Define sample rate for QArm
sampleRate = 200
sampleTime = 1 / sampleRate
print('Sample Rate is ', sampleRate, ' Hz. Simulation will run until you type Ctrl+C to exit.')

# Initialise the RealSense camera for RGB and Depth data

if hardware:
    id = '0'
    id2 = '1' # Physical camera 2
else:
    id = '0@tcpip://localhost:18901' 
    id2 = '0@tcpip://localhost:18903' # Virtual camera 2

# Define parameters for camera (Do not change)
imageWidth = 640
imageHeight = 480

myCam = Camera3D(mode='RGB&DEPTH', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight,
                  frameWidthDepth=imageWidth, frameHeightDepth=imageHeight, deviceId=id, readMode=0)
# ADD THIS: Arm 2 Camera
myCam2 = Camera3D(mode='RGB&DEPTH', frameWidthRGB=imageWidth, frameHeightRGB=imageHeight,
                  frameWidthDepth=imageWidth, frameHeightDepth=imageHeight, deviceId=id2, readMode=0)
##### Specify the directory to save real time data ####
 # Change this to your directory

fileAbsolutePath = os.path.abspath(filePathData)
if os.path.exists(fileAbsolutePath):
    os.remove(fileAbsolutePath)
    print(f"Deleted existing file: {fileAbsolutePath}")
fileExists = os.path.isfile(filePathData)

# endregion

# END SECTION
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# region: Parameters

''' This section includes all the position parameters defined for object positions, pick-and-drop handling set points, and bin positions.
Students should plan/optimise their trajectory to minimise travel distance and create the most direct path, in order to reduce energy consumption. '''

##### Create object positions in the physical domain ([X, Y, Z], gripper and base colour command). These are the locations the picker will travel to to grab the object#####
# Gripper command: 0 for fully open (~16 cm), 0.25 for ~10 cm, 0.5 for ~8 cm, 0.75 for ~3 cm, and 1 for fully closed #
objectPositions = [
    (np.array([0.605, 0.20, 0.15], dtype=np.float64), 0.6, np.array([0, 1, 1], dtype=np.float64)),  # Object 1
    (np.array([0.605, 0.10, 0.15], dtype=np.float64), 0.6, np.array([0, 1, 1], dtype=np.float64)),  # Object 2
    (np.array([0.605, 0.00, 0.15], dtype=np.float64), 0.6, np.array([0, 1, 1], dtype=np.float64)),  # Object 3
    (np.array([0.605, -0.10, 0.15], dtype=np.float64), 0.6, np.array([0, 1, 1], dtype=np.float64))   # Object 4
]

##### Define home, detect, and bin positions in the physical domain ([X, Y, Z], gripper and base colour command). Adjust if necessary, except for the home position. #####
homePosition = (np.array([0.45, 0.00, 0.49], dtype=np.float64), 0.0, np.array([1, 0, 0], dtype=np.float64))  # Home position
detectPosition = (np.array([0.40, 0.00, 0.25], dtype=np.float64), 0.5, np.array([1, 1, 1], dtype=np.float64))  # Detect position for viewing colour and size
pickUpPosition = (np.array([0.40, 0.00, 0.25], dtype=np.float64), 0.5, np.array([0, 1, 1], dtype=np.float64))  # Start Pickup position

red18650cellBinPosition = (np.array([0.30, -0.1, 0.15], dtype=np.float64), np.array([1, 0, 0], dtype=np.float64)) # Red D-cell bin position
green18650cellBinPosition = (np.array([0, 0.4, 0.25], dtype=np.float64), np.array([0, 1, 0], dtype=np.float64)) #Green bin position
blue18650cellBinPosition = (np.array([0.3, 0.15, 0.25], dtype=np.float64), np.array([0, 0, 1], dtype=np.float64)) #Blue bin position

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

''' This part defines the BGR colour ranges for the colours of interest and the regions of objects in images. BGR ranges should be 
adjusted if the coloured objects are not well detected in the single-channel masked image. Please check the pixel counts before making changes. 
Students need to refine the regions based on actual object positions. '''

##### BGR colour ranges for red cells (Adjust if necessary) #####

colourRanges = {
    'Red':((0, 0, 110),(80, 80, 255))   # Red simulation
    ,'Green':((0, 110, 0),(80, 255, 80))
    ,'Blue':((110, 0, 0),(255, 80, 80))
}


##### Outline individual rectangular region of objects (ROO) in the image [x, y, width, height] in pixel coordinates (Adjust if necessary) #####
# The origin point (0, 0) is located at the top-left corner of the image, and the image size is 640 x 480 pixels  
rooPositions= [
    [40, 200, 120, 130], # ROO 1 (Left side in image)
    [175, 200, 120, 130], # ROO 2
    [315, 200, 120, 130], # ROO 3
    [450, 200, 120, 130] # ROO 4 (Right side in image)
]

# endregion

## THE FOLLOWING REGION SELECTS THE DEFINED COLOUR REGIONS FROM THE IMAGE AND SHOULD NOT BE CHANGED
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# region: Cell detection
##### Function to detect the object colour and pixel counts at each specified region 1 - 4 #####
def detectColourAndSize(image, rooPosition):

    ''' This function calculates the pixel counts of coloured objects in the predefined regions.
    Students should adjust the pixel count threshold to differentiate object sizes. '''
    
    # Extract the region of object (ROO) from the image
    x, y, w, h = rooPosition # [x, y, width, height] in pixel coordinate
    rooImage = image[y:y + h, x:x + w]
        
    # Initialise pixel count, colour and size to None
    pixelCount = 0
    detectedColour = None
    detectedCellType = None

    # Dictionary to store pixel counts
    pixelCounts = {colourName: 0 for colourName in colourRanges.keys()}

    for colourName, (lower, upper) in colourRanges.items():
        mask = cv2.inRange(rooImage, np.array(lower), np.array(upper))
        
        # Calculate pixel count for the current mask
        pixelCount = cv2.countNonZero(mask)
        pixelCounts[colourName] = pixelCount

# ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
## THE FOLLOLOWING REGION ASIGNS CELL TYPES BASED ON THE FILTERED IMAGE AND SHOULD ONLY BE CHANGED TO INCLUDE DIFFERENT CELL SIZES
        '''Check pixel counts for object size and colour (Adjust this threshold if necessary)'''

        if pixelCount > 300:  # Remove noise
            detectedCellType = "18650 cell"
            detectedColour = colourName
            break # Exit loop if size and colour is detected

    # print pixel counts for each ROO
    pixelCountStr = ", ".join([f"{colourName} {pixelCounts[colourName]}" for colourName in colourRanges.keys()])
    print(f"Pixel counts for ROO {idx + 1}: {pixelCountStr}.")  
# ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **

    return detectedCellType, detectedColour

##### Function to display ROOs and plot combined masks (Red, green and blue mask) #####
def drawRoos(image, rooPositions):

    ''' This function configures the ROO drawings in the image, and should not be changed. '''

    for idx, (x, y, w, h) in enumerate(rooPositions):
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 255), 1)
        cv2.putText(image, f'ROO {idx + 1}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

def hsvRange(image, colourRanges):

    ''' This function configures the display of the combined mask, and should not be changed. '''

    # Initialise a dictionary to hold individual masks
    masks = {}

    # Create and combine masks for each colour
    combinedMask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
    for colourName, (lower, upper) in colourRanges.items():

        # Create a mask for the current colour
        mask = cv2.inRange(image, np.array(lower), np.array(upper))
        masks[colourName] = mask
            
        # Combine the current mask with the existing combined mask
        combinedMask = cv2.bitwise_or(combinedMask, mask)

    return combinedMask

# endregion

## THE FOLLOWING REGION CONTROLS MOVEMENT, AND SHOULD NOT BE CHANGED
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# region: Command QArm
##### Function to calculate the distance between real time end effector and cmd position #####
def positionChecker(arm, armUtilities, positionCmd, tolerance):

    ''' Check if the arm reaches the commanded position and wait until it does or report if it's not reachable within the timeout. 
    If position is unreachable, stop further operations. '''

    checkerTime = time.time()
    timeOut = 5 # Set a timeout duration

    while True:
        # Read the current arm state
        arm.read_std()

        # Get the current position using forward kinematics
        currentPosition, _ = armUtilities.qarm_forward_kinematics(arm.measJointPosition[0:4])

        # Calculate the distance between the current position and the commanded position
        distanceToCmd = np.linalg.norm(currentPosition - positionCmd)
        print(f'Distance to cmd position: {distanceToCmd:.4f} m')

        # Check if the distance is within the tolerance
        if distanceToCmd < tolerance:
            return True

        # Check if the timeout has been reached
        reachTime = time.time() - checkerTime
        if reachTime >= timeOut:
            print(f"Warning: Failed to move the position with the current tolerance. Stopping further operations. Time elapsed: {int(elapsedTime()):.1f} seconds.")
            exit()
            return False
            
        # Wait before checking again
        time.sleep(0.1)  # Check every 100ms

##### Function to command QArm to pick and drop #####

tolerance1 = 0.03 # Set the position tolerance 0.03, 0.045
tolerance2 = 0.045

def pickAndPlace(arm, armUtilities, position, colour, cellType, idx):

    ''' This function configures the pick-and-place task. '''

    positionCmd, gripCmd, ledCmd = position
    _, phiCmd = armUtilities.qarm_inverse_kinematics(positionCmd, 0, arm.measJointPosition[0:4])

    # Move to the object Pickup position
    print(f"Moving to Pickup Object {idx + 1}: {colour} {cellType}. Time elapsed: {int(elapsedTime()):.1f} seconds.")
    arm.read_write_std(phiCMD=phiCmd, grpCMD=gripCmd, baseLED=ledCmd)
    if not positionChecker(arm, armUtilities, positionCmd, tolerance1):
        return

    if cellType == 'D-cell':
        gripCmd = 0.80
    elif cellType == '18650 cell':
        gripCmd = 0.85
    else:
        gripCmd = 0.5 # Default open

    # Close the gripper based on cell type
    print(f"Closing gripper for {idx + 1} {colour} {cellType}: {gripCmd}. Time elapsed: {int(elapsedTime()):.1f} seconds.")
    arm.read_write_std(phiCMD=phiCmd, grpCMD=gripCmd, baseLED=ledCmd)  # Adjust if necessary
    time.sleep(1.5) # Gripper close time

    # Lift the object. Ensure only once per object without accumulating over cycles
    liftUpPosition = list(positionCmd)
    liftUpPosition[2] += 0.1 # Adjust if necessary
    print(f"Lifting Object {idx + 1}: {colour} {cellType}. Time elapsed: {int(elapsedTime()):.1f} seconds.")
    _, phiCmd = armUtilities.qarm_inverse_kinematics(liftUpPosition, 0, arm.measJointPosition[0:4])
    arm.read_write_std(phiCMD=phiCmd, grpCMD=gripCmd, baseLED=ledCmd)
    if not positionChecker(arm, armUtilities, np.array(liftUpPosition), tolerance2):
        return

    ''' Dropping objects according to their sizes and colours. '''
    if colour == 'Red':
        if cellType == '18650 cell':
            positionCmd, ledCmd = red18650cellBinPosition
        else:
            positionCmd, ledCmd = homePosition 
    elif colour == 'Green': 
        if cellType == '18650 cell':
            positionCmd, ledCmd = green18650cellBinPosition
        else:
            positionCmd, ledCmd = homePosition 
    elif colour == 'Blue': 
        if cellType == '18650 cell':
            positionCmd, ledCmd = blue18650cellBinPosition
        else:
            positionCmd, ledCmd = homePosition 
    else:
        positionCmd, ledCmd = homePosition 

    print(f"Moving Object {idx + 1}: {colour} {cellType} to bin position: {positionCmd}. Time elapsed: {int(elapsedTime()):.1f} seconds.")

    _, phiCmd = armUtilities.qarm_inverse_kinematics(positionCmd, 0, arm.measJointPosition[0:4])
    arm.read_write_std(phiCMD=phiCmd, grpCMD=gripCmd, baseLED=ledCmd)
    if not positionChecker(arm, armUtilities, positionCmd, tolerance1):
        return

    # Open gripper to drop object
    print(f"Dropping Object {idx + 1}: {colour} {cellType}. Time elapsed: {int(elapsedTime()):.1f} seconds.")
    arm.read_write_std(phiCMD=phiCmd, grpCMD=pickUpPosition[1], baseLED=ledCmd)  # Adjust if necessary
    time.sleep(1.5) # Gripper open time

    # Return to the Pickup position
    print(f"Returning to Pickup position. Time elapsed: {int(elapsedTime()):.1f} seconds.")
    _, phiCmd = armUtilities.qarm_inverse_kinematics(pickUpPosition[0], 0, arm.measJointPosition[0:4])
    arm.read_write_std(phiCMD=phiCmd, grpCMD=pickUpPosition[1], baseLED=pickUpPosition[2])
    if not positionChecker(arm, armUtilities, pickUpPosition[0], tolerance1):
        return

# endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# region: Current and energy data

''' This region configures the current, PWM and energy data log, energy consumption calculation and data plot.
Student do not need to modify this region. '''

##### Lists to store time, current, PWM, power and energy measurements, as well as total energy consumption #####
timeMeasurement = []
currentMeasurement = []
pwmMeasurement = []
powerMeasurement = []
EnergyMeasurement = []
totalEnergyConsumption = []

##### Update current measurement and calculate total energy consumption #####
# Column names
jointName = ['Base Joint', 'Shoulder Joint', 'Elbow Joint', 'Wrist Joint', 'Gripper', 'Total']

def updateAndWriteMeasurements():
    # Append latest measurements
    timeMeasurement.append(time.time() - startTime)
    currentMeasurement.append(myArm.measJointCurrent.copy())
    pwmMeasurement.append(myArm.measJointPWM.copy())

    if len(timeMeasurement) > 1:
        timeInterval = timeMeasurement[-1] - timeMeasurement[-2]
    else:
        timeInterval = 0 # Initial sample time is 0 for the first measurement

    # Calculate power for each joint
    jointPower = np.abs(pwmMeasurement[-1]) * 12 * np.abs(currentMeasurement[-1])
    jointPowerSum = np.sum(jointPower)
    powerMeasurement.append(jointPowerSum)

    # Calculate energy for this sample time (power * sample time)
    jointEnergy = jointPowerSum * timeInterval
    EnergyMeasurement.append(jointEnergy)

    # Calculate total energy consumption
    energyConsumption = np.sum(EnergyMeasurement)
    totalEnergyConsumption.append(energyConsumption)

    # Save data to CSV file using csv module
    with open(filePathData, 'w', newline='') as file:
        writer = csv.writer(file)
        header = ['Time (s)'] + [f'{jointName[i]} Current (A)' for i in range(5)] + ['Total Absolute Current (A)'] + \
                 [f'{jointName[i]} PWM' for i in range(5)] + ['Total Absolute PWM'] + ['Joint Power Sum (W)'] + \
                 ['Joint Energy Sum (J)'] + ['Total Energy Consumption (J)']
        writer.writerow(header)

        for i in range(len(timeMeasurement)):
            row = [timeMeasurement[i]] + [np.array(currentMeasurement[i][j]) for j in range(5)] + \
                  [np.sum(np.abs(currentMeasurement[i]))] + \
                  [np.array(pwmMeasurement[i][j]) for j in range(5)] + \
                  [np.sum(np.abs(pwmMeasurement[i]))] + \
                  [powerMeasurement[i]] + \
                  [EnergyMeasurement[i]] + \
                  [totalEnergyConsumption[i]]
            
            writer.writerow(row)

##### Define data login at sample time #####
stopLogging = False
def dataLogging():

    global stopLogging
    while myArm.status and not stopLogging:
        myArm.read_std()
        updateAndWriteMeasurements()
        time.sleep(sampleTime)

# endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# region: Data monitor and animate

''' This region handles the real-time monitoring and visualisation of joint currents. 
The system is designed to read data from a CSV file that is continually updated during the operation. 
It automatically detects changes in the file and refreshes the animation, providing a real time view. '''

##### Define column names to match in the csv file #####
columns = ['Time (s)', 'Base Joint Current (A)', 'Shoulder Joint Current (A)',
       'Elbow Joint Current (A)', 'Wrist Joint Current (A)',
       'Gripper Current (A)', 'Total Absolute Current (A)', 'Base Joint PWM',
       'Shoulder Joint PWM', 'Elbow Joint PWM', 'Wrist Joint PWM',
       'Gripper PWM', 'Total Absolute PWM', 'Joint Power Sum (W)',
       'Joint Energy Sum (J)', 'Total Energy Consumption (J)']

globalData = pd.DataFrame(columns=columns)

##### Define animate #####
def animate(i):
    if not globalData.empty and not stopEvent.is_set():

        ax.clear()
        for idx, joint in enumerate(jointName):
            columnName = columns[idx + 1]  # Skip the first column ('Time (s)'), and only plot current measurements
            if columnName in globalData.columns:
                ax.plot(globalData['Time (s)'], globalData[columnName].abs(), label=joint)

        # set up Labels and lagend on plot
        ax.set_xlabel('Elapsed Time (s)')
        ax.set_ylabel('| Current | (A)')
        ax.set_title('QArm Absolute Joints Current vs Elapsed Time')
        ax.legend(loc='upper left')
        ax.grid(True)

        if not globalData.empty:
            if len(timeMeasurement) > 0:
                ax.set_xlim(0, max(timeMeasurement) + 3)
                ax.set_ylim(0, max(np.sum(np.abs(currentMeasurement),axis =1 ))+ 1)
            
        plt.tight_layout()


##### Define read and update new data #####

stopEvent = threading.Event()

def readAndUpdateData():
    global globalData
    try:
        newData = pd.read_csv(filePathData)

        if newData.empty:
            print("The CSV file is empty. No data to update.")

        globalData = newData # Data loaded from {filePathData}.
        
    except Exception as e: 
        # print(f"An unexpected error occurred: {e}")
        pass
        

##### Class to handle file change #####
class FileChangeHandler(FileSystemEventHandler):
    def on_modified(self, event): # Check if the modified file is being watched

        if event.src_path == fileAbsolutePath:
            if not stopEvent.is_set():  # Check if stopEvent is set    
                time.sleep(0.05)  # A small delay to ensure the file has completed writing
                readAndUpdateData() # Read the file and update the globalData

##### Define read and plot data #####
fig, ax = None, None
def readTimePlotTask():
    global stopLogging
    global fig
    global ax

    while myArm.status and not stopLogging and not stopEvent.is_set():

        # Create a figure and axis for the plot
        fig, ax = plt.subplots(figsize=(16, 9))
        fig.canvas.manager.window.wm_geometry("+%d+%d" % (0, 0))
        directoryToWatch = os.path.dirname(fileAbsolutePath) # Directory to watch for changes in the file

        # Setup file change event handler and observer
        if myArm.status:
            eventHandler = FileChangeHandler()
            observer = Observer()
            observer.schedule(eventHandler, path=directoryToWatch, recursive=False)

        try:
            # Start the file observer
            observer.start()
            
            # Continuously animate the plot every 30 ms

            ani = animation.FuncAnimation(fig, animate, interval=30, cache_frame_data=False)

            # Show the plot and allow it to be updated without blocking the main thread
            plt.show(block=False)

            while not stopEvent.is_set():
                plt.show(block=True)

        # Handle Keyboard Interrupt and ensure the observer stops when the loop ends       
        except KeyboardInterrupt:
            pass

        finally:
            observer.stop()
            observer.join()
            print("Observer stopped")

def final_fig():
    data = pd.read_csv(filePathData)
    final = plt.figure()
    plt.clf()
    final.set_figheight(9)
    final.set_figwidth(16)
    column_names = ["Base Joint", "Shoulder Joint", "Elbow Joint", "Wrist Joint", "Gripper", "Total"]
    for idx, column in enumerate(data.columns):
        if idx != 0 and idx < 7:
            plt.plot(data['Time (s)'], data[column].abs())
    plt.title('QArm Absolute Joints Current vs Elapsed Time')
    plt.xlabel("Elapsed Time (s)")
    plt.ylabel("| Current | (A)")
    plt.legend(column_names, loc="upper left")
    plt.grid(True)
        
    final.savefig(filePathImage, bbox_inches = 'tight')
    plt.show()
    print("Figure Saved")
# endregion

## THE FOLLOWING LOOP CONTROLS THE OVERALL PROGRAM AND SHOULD NOT BE CHANGED

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# region: Main Loop
try:
    '''Main loop configures 5 steps:
    1. Move to home position and wait.
    2. Move to the detection position and capture an image.
    3. Detect size and colour in each region.
    4. Pick and place each object based on detected information.
    5. Return to home position and terminate the program.

    Adjust the wait times as needed to optimise for energy consumption.'''

    ### Start real time data logging thread and plot thread ###
    loggingThread = threading.Thread(target=dataLogging)
    loggingThread.daemon = True
    loggingThread.start()

    loggingThread2 = threading.Thread(target=readTimePlotTask)
    loggingThread2.daemon = True
    loggingThread2.start()
    count = 1
    
    while myArm.status:
        start = elapsedTime()

        ### Step 1: Move to home position ###
        if not atHomePosition:
            
            time.sleep(0.5) # Initialisation lag time
            print(f"QArm communication channels fully initialised")

            # Helper function to move an arm home so we can thread it
            def move_home(arm, armUtils, armName):
                _, phiCmd = armUtils.qarm_inverse_kinematics(homePosition[0], 0, arm.measJointPosition[0:4])
                arm.read_write_std(phiCMD=phiCmd, grpCMD=homePosition[1], baseLED=homePosition[2])
                if not positionChecker(arm, armUtils, homePosition[0], tolerance1):
                    print(f"Failed to move {armName} to home.")
                else:
                    print(f"{armName} at home position.")

            # Create threads for simultaneous movement
            t1 = threading.Thread(target=move_home, args=(myArm, myArmUtilities, "Arm 1"))
            t2 = threading.Thread(target=move_home, args=(myArm2, myArmUtilities2, "Arm 2"))

            # Start and wait for both threads to finish
            t1.start()
            t2.start()
            t1.join()
            t2.join()

            atHomePosition = True # Loop once

        ### Step 2: Move to the detection position ###
        if atHomePosition:
            
            def move_to_detect(arm, armUtils, armName):
                _, phiCmd = armUtils.qarm_inverse_kinematics(detectPosition[0], 0, arm.measJointPosition[0:4])
                arm.read_write_std(phiCMD=phiCmd, grpCMD=detectPosition[1], baseLED=detectPosition[2])
                
                if not positionChecker(arm, armUtils, detectPosition[0], tolerance1):
                    print(f"Failed to move {armName} to detection position.")
                else:
                    print(f"{armName} at detection position.")

            # Create threads for simultaneous movement
            t1 = threading.Thread(target=move_to_detect, args=(myArm, myArmUtilities, "Arm 1"))
            t2 = threading.Thread(target=move_to_detect, args=(myArm2, myArmUtilities2, "Arm 2"))
            
            t1.start()
            t2.start()
            t1.join()
            t2.join()

            print(f"Both arms at detection position. Start detecting... Time elapsed: {int(elapsedTime()):.1f} seconds.")
            time.sleep(1) # Dwell time for moving to detection position and settle

        ### Step 3: Capture image and detect size / colour in each region ###
        # Refresh the camera to be ready
        try:
            for _ in range(5):  # Refresh 2 times at least
                myCam.read_RGB()
                myCam2.read_RGB() # Refresh Arm 2 camera
            time.sleep(0.2)  # Refresh time

            # Capture the RGB image for Arm 1
            myCam.read_RGB()
            frame1 = myCam.imageBufferRGB
            drawRoos(frame1, rooPositions)
            combinedMask1 = hsvRange(frame1, colourRanges)  
            
            # Capture the RGB image for Arm 2
            myCam2.read_RGB()
            frame2 = myCam2.imageBufferRGB
            drawRoos(frame2, rooPositions)
            combinedMask2 = hsvRange(frame2, colourRanges)

            # Show feeds for both arms
            cv2.imshow('Arm 1: Original RGB with ROO', frame1)
            cv2.imshow('Arm 1: Combined Mask', combinedMask1)
            cv2.imshow('Arm 2: Original RGB with ROO', frame2)
            cv2.imshow('Arm 2: Combined Mask', combinedMask2)
            cv2.waitKey(1)

        except Exception as e:
            print(f"Camera error: {e}")
            break

        # Lists to store cell type and colour for BOTH arms
        detectedCellTypes1 = []
        detectedColours1 = []
        detectedCellTypes2 = []
        detectedColours2 = []

        for idx, rooPosition in enumerate(rooPositions):
            # Detect objects for Arm 1 using frame1
            cellType1, colour1 = detectColourAndSize(frame1, rooPosition)
            detectedCellTypes1.append(cellType1)
            detectedColours1.append(colour1)
            
            # Detect objects for Arm 2 using frame2
            cellType2, colour2 = detectColourAndSize(frame2, rooPosition)
            detectedCellTypes2.append(cellType2)
            detectedColours2.append(colour2)

            print(f"Arm 1 - Object {idx + 1}: {colour1} {cellType1}")
            print(f"Arm 2 - Object {idx + 1}: {colour2} {cellType2}")

        ### Step 4: Pick and place each object based on detected info ###

# ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
        ''' Process pick and drop task according to their sizes and colours. '''


        objectsDetected1 = any(colour is not None for colour in detectedColours1)
        objectsDetected2 = any(colour is not None for colour in detectedColours2)
        
        if objectsDetected1 or objectsDetected2:
            active_threads = []
            
            # Since there are 4 objects per arm, we loop 4 times
            for idx in range(4):
                
                # If Arm 1 sees an object at this index, send it to grab it
                if detectedColours1[idx] and detectedCellTypes1[idx]:
                    t1 = threading.Thread(target=pickAndPlace, args=(myArm, myArmUtilities, objectPositions[idx], detectedColours1[idx], detectedCellTypes1[idx], idx))
                    active_threads.append(t1)
                    t1.start()
                    
                # If Arm 2 sees an object at this index, send it to grab it
                if detectedColours2[idx] and detectedCellTypes2[idx]:
                    t2 = threading.Thread(target=pickAndPlace, args=(myArm2, myArmUtilities2, objectPositions[idx], detectedColours2[idx], detectedCellTypes2[idx], idx))
                    active_threads.append(t2)
                    t2.start()

                # Wait for both arms to finish their current object before moving to the next index
                for t in active_threads:
                    t.join()
                
                # Clear the thread list for the next object
                active_threads.clear()

# ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **

        ### Step 5: QArm return to home position and programme terminates ###
        else:
            if count < 5 and hardware == 0:
                count += 1
                time.sleep(5)
                workSpace.cell_spawn()
            else:
                print(f"No object is detected. Returning to home position. Time elapsed: {int(elapsedTime()):.1f} seconds.")
                
                # Helper function for returning home before shutting down
                def return_home(arm, armUtils):
                    _, phiCmd = armUtils.qarm_inverse_kinematics(homePosition[0], 0, arm.measJointPosition[0:4])
                    arm.read_write_std(phiCMD=phiCmd, grpCMD=homePosition[1], baseLED=homePosition[2])
                    positionChecker(arm, armUtils, homePosition[0], tolerance1)

                # Send both arms home simultaneously
                t1 = threading.Thread(target=return_home, args=(myArm, myArmUtilities))
                t2 = threading.Thread(target=return_home, args=(myArm2, myArmUtilities2))
                
                t1.start()
                t2.start()
                t1.join()
                t2.join()

                time.sleep(1) # Wait time to terminate
                
                # Terminate BOTH arms
                myArm.terminate()
                myArm2.terminate() 
                
                stopLogging = True
                stopEvent.set()
                final_fig()
                # plt.show()  # Show final plot          
                quit()

except KeyboardInterrupt:
    print("User interrupted!")


    
# endregion
