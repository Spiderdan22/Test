# Note use this method to get your qvl libraries to ensure you're using the
# latest version in GitHub. It is inserted first in the list to take precedence
# over all other libraries in your python path.
import sys
sys.path.insert(0, "../")

from qvl.qlabs import QuanserInteractiveLabs
from qvl.conveyor_curved import QLabsConveyorCurved
from qvl.conveyor_straight import QLabsConveyorStraight
from qvl.widget import QLabsWidget
from qvl.delivery_tube import QLabsDeliveryTube
from qvl.basic_shape import QLabsBasicShape
from qvl.generic_sensor import QLabsGenericSensor
from qvl.qarm import QLabsQArm
from qvl.real_time import QLabsRealTime
import pal.resources.rtmodels as rtmodels
import time
import math
import struct
import numpy as np
import cv2
import os
from random import randrange
from controller import *
########## Main setup script ##########

# Core variables
waitTime = 1.0

qlabs = QuanserInteractiveLabs()
print("Connecting to QLabs...")
try:
    qlabs.open("localhost")
except:
    print("Unable to connect to QLabs")

print("Connected")

qlabs.destroy_all_spawned_actors()
QLabsRealTime().terminate_all_real_time_models()

## CHANGE THE DIRECTORY IF WE ARE GOING TO MAKE THESE PUBLIC

__qalDirPath = os.environ['RTMODELS_DIR']

QARMS = os.path.normpath(
    os.path.join(__qalDirPath, 'QArms'))


#region : create conveyors and their supports and tube

########## Create two conveyors ##########
firstConvey = QLabsConveyorStraight(qlabs)
num = firstConvey.spawn_id_degrees(actorNumber = 0,
                                    location = [0.15, 0, 0.3],
                                    rotation = [0, 0, 0],
                                    scale = [1,1,1],
                                    configuration = 5)
firstConvey.set_speed(0.1)

secondConvey = QLabsConveyorStraight(qlabs)
num = secondConvey.spawn_id_degrees(actorNumber = 1,
                                    location = [-1.55, 0, 0.3],
                                    rotation = [0, 0, 0],
                                    scale = [1,1,1],
                                    configuration = 5)
secondConvey.set_speed(0.1)

# Create a simple supports for the conveyors
firstStand = QLabsBasicShape(qlabs)
firstStand.spawn_id_and_parent_with_relative_transform(actorNumber = 98,
                                                        location = [0.85, 0, -0.15],
                                                        rotation = [0, 0, 0],
                                                        scale = [1.65, 0.3, 0.3],
                                                        configuration = 0,
                                                        parentClassID = firstConvey.classID,
                                                        parentActorNumber = 0,
                                                        parentComponent = 0,
                                                        waitForConfirmation = True)
firstStand.set_material_properties(color = [0.3, 0.3, 0.3],
                                    roughness = 0.4,
                                    metallic = False)

secondStand = QLabsBasicShape(qlabs)
secondStand.spawn_id_and_parent_with_relative_transform(actorNumber = 99,
                                                        location = [0.85, 0, -0.15],
                                                        rotation = [0, 0, 0],
                                                        scale = [1.65, 0.3, 0.3],
                                                        configuration = 0,
                                                        parentClassID = secondConvey.classID,
                                                        parentActorNumber = 1,
                                                        parentComponent = 0,
                                                        waitForConfirmation = True)
secondStand.set_material_properties(color = [0.3, 0.3, 0.3],
                                    roughness = 0.4,
                                    metallic = False)

########## Create a widget tube ##########
deliveryTube = QLabsDeliveryTube(qlabs)
deliveryTube.spawn_id_degrees(actorNumber = 1,
                                location = [1.75, 0, 8],
                                rotation = [0, 180, 0],
                                scale = [1, 1, 1],
                                configuration = 1,
                                waitForConfirmation = True)
deliveryTube.set_height(height = 7)
#endregion


time.sleep(1)

#region : create bins

########## Create a disposal bin ##########
conveyorBin = QLabsBasicShape(qlabs)
conveyorBin.spawn_id_box_walls_from_center_degrees(actorNumbers = [2, 3, 4, 5, 6],
                                                    centerLocation = [-1.7, 0, 0],
                                                    yaw = 0,
                                                    xSize = 0.3, ySize = 0.3, zHeight = 0.1,
                                                    wallThickness = 0.01,
                                                    floorThickness = 0.1,
                                                    wallColor = [0.5, 0, 0],
                                                    floorColor = [0.5, 0, 0],
                                                    waitForConfirmation = True)

# ########## Create a checked bin ##########
conveyorBin = QLabsBasicShape(qlabs)
conveyorBin.spawn_id_box_walls_from_center_degrees(actorNumbers = [12, 13, 14, 15, 16],
                                                    centerLocation = [1.35, -0.5, 0],
                                                    yaw = 0,
                                                    xSize = 0.3, ySize = 0.3, zHeight = 0.1,
                                                    wallThickness = 0.01,
                                                    floorThickness = 0.1,
                                                    wallColor = [0, 0.5, 0],
                                                    floorColor = [0, 0.5, 0],
                                                    waitForConfirmation = True)

# ########## Create a checked bin ##########
conveyorBin2 = QLabsBasicShape(qlabs)
conveyorBin2.spawn_id_box_walls_from_center_degrees(actorNumbers = [17,18,19,20,21],
                                                    centerLocation = [0.35, -0.5, 0],
                                                    yaw = 0,
                                                    xSize = 0.3, ySize = 0.3, zHeight = 0.1,
                                                    wallThickness = 0.01,
                                                    floorThickness = 0.1,
                                                    wallColor = [0, 0, 0.5],
                                                    floorColor = [0, 0, 0.5],
                                                    waitForConfirmation = True)

#endregion

#region : create arms and supports
# ########## Create an arm ##########
firstArm = QLabsQArm(qlabs)
firstArm.spawn_id_degrees(actorNumber = 10,
                            location = [1, -0.5, 0.3],
                            rotation = [0, 0, 0],
                            scale = [1, 1, 1],
                            configuration = 0,
                            waitForConfirmation = True)

# Create a simple support for the arm
firstArmStand = QLabsBasicShape(qlabs)
firstArmStand.spawn_id_and_parent_with_relative_transform(actorNumber = 100,
                                                            location = [0, 0, -0.15],
                                                            rotation = [0, 0, 0],
                                                            scale = [0.3, 0.3, 0.3],
                                                            configuration = 0,
                                                            parentClassID = firstArm.classID,
                                                            parentActorNumber = 10,
                                                            parentComponent = 0,
                                                            waitForConfirmation = True)
firstArmStand.set_material_properties(color = [0.3, 0.3, 0.3],
                                        roughness = 0.4, metallic = False)

secondArm = QLabsQArm(qlabs)
secondArm.spawn_id_degrees(actorNumber = 11,
                            location = [0, -0.5, 0.3],
                            rotation = [0, 0, 0],
                            scale = [1, 1, 1],
                            configuration = 0,
                            waitForConfirmation = True)

# Create a simple support for the arm
secondArmStand = QLabsBasicShape(qlabs)
secondArmStand.spawn_id_and_parent_with_relative_transform(actorNumber = 101,
                                                            location = [0, 0, -0.15],
                                                            rotation = [0, 0, 0],
                                                            scale = [0.3, 0.3, 0.3],
                                                            configuration = 0,
                                                            parentClassID = secondArm.classID,
                                                            parentActorNumber = 11,
                                                            parentComponent = 0,
                                                            waitForConfirmation = True)
secondArmStand.set_material_properties(color = [0.3, 0.3, 0.3],
                                        roughness = 0.4, metallic = False)

#endregion

########## Create beam sensors ##########
beamSensorSpawn = QLabsGenericSensor(qlabs)
beamSensorSpawn.spawn_id_degrees(actorNumber = 101,
                            location=[1.45, .3, 0.45],
                            rotation=[0, 0, -90],
                            scale=[1, 1, 1],
                            configuration = 0,
                            waitForConfirmation = True)

beamSensorSpawn.show_sensor(showBeam=True,
                        showOriginIcon=True,
                        iconScale=0.1,
                        waitForConfirmation=True)
beamSensorSpawn.set_beam_size(startDistance=0,
                            endDistance=0.5,
                            heightOrRadius=0.01,
                            width=0.01,
                            waitForConfirmation=True)

beamSensorArm1 = QLabsGenericSensor(qlabs)
beamSensorArm1.spawn_id_degrees(actorNumber = 102,
                            location=[1, .3, 0.45],
                            rotation=[0, 0, -90],
                            scale=[1, 1, 1],
                            configuration = 0,
                            waitForConfirmation = True)

beamSensorArm1.show_sensor(showBeam=True,
                        showOriginIcon=True,
                        iconScale=0.1,
                        waitForConfirmation=True)
beamSensorArm1.set_beam_size(startDistance=0,
                            endDistance=0.5,
                            heightOrRadius=0.01,
                            width=0.01,
                            waitForConfirmation=True)


beamSensorArm2 = QLabsGenericSensor(qlabs)
beamSensorArm2.spawn_id_degrees(actorNumber = 103,
                            location=[0, .3, 0.45],
                            rotation=[0, 0, -90],
                            scale=[1, 1, 1],
                            configuration = 0,
                            waitForConfirmation = True)

beamSensorArm2.show_sensor(showBeam=True,
                        showOriginIcon=True,
                        iconScale=0.1,
                        waitForConfirmation=True)
beamSensorArm2.set_beam_size(startDistance=0,
                            endDistance=0.5,
                            heightOrRadius=0.01,
                            width=0.01,
                            waitForConfirmation=True)

# beamSensorBox1 = QLabsGenericSensor(qlabs)
# beamSensorBox1.spawn_id_degrees(actorNumber = 104,
#                             location=[2.9, 0, 0.2],
#                             rotation=[0, 0, 180],
#                             scale=[1, 8, .5],
#                             configuration = 0,
#                             waitForConfirmation = True)

# beamSensorBox1.show_sensor(showBeam=True,
#                         showOriginIcon=True,
#                         iconScale=0.1,
#                         waitForConfirmation=True)
# beamSensorBox1.set_beam_size(startDistance=0,
#                             endDistance=0.5,
#                             heightOrRadius=0.01,
#                             width=1,
#                             waitForConfirmation=True)


# Spawn a cylinder
cylinder = QLabsWidget(qlabs)


# # Start spawn model
QLabsRealTime().start_real_time_model(QARMS+'/QArm_Spawn0', actorNumber=10, additionalArguments='-uri_hil tcpip://localhost:18900 -uri_video tcpip://localhost:18901')
QLabsRealTime().start_real_time_model(QARMS+'/QArm_Spawn1', actorNumber=11, additionalArguments= '-uri_hil tcpip://localhost:18902 -uri_video tcpip://localhost:18903')

def createCylinder():
        value = ['green', 'blue', 'red']
        color = [[0,1,0], [0,0,1], [1,0,0]]
        position = randrange(3)

        cylinder.spawn(location = [1.75, 0, 1],
               rotation = [0, 0, 0],
               scale = [.05, .05, .05],
               configuration = cylinder.CYLINDER,
               color = color[position],
               measuredMass = 1,
               properties=value[position])

def moveConveyors(speed):
    firstConvey.set_speed(speed)
    secondConvey.set_speed(speed)

def findObj():
    ########## Test for object ##########
    sensedObj = QLabsWidget(qlabs)
    hit = False
    x = 0

    max_distance = 1 # meters (for depth camera)

    qarm1_State = 6
    qarm2_State = 6

    startTime = time.time()
    def elapsed_time(startTime):
        return time.time() - startTime

    # make sure port number matches ones from spawn model start
    qarm1 = createQarm(18900)
    pickAndPlace(qarm1, qarm1_State)

    qarm2 = createQarm(18902)
    pickAndPlace(qarm2, qarm2_State)

    qarm1Cam = createQarmCamera(18901)
    qarm2Cam = createQarmCamera(18903)

    moveConveyors(0.1)

    createCylinder()

    startTimeSpawn = startTimeQarm1 = startTimeQarm2 = time.time()

    while True:

        _, hitSpawn, _,_,_,_ = beamSensorSpawn.test_beam_hit_widget()
        _, hitArm1, _,_,_, propertiesArm1 = beamSensorArm1.test_beam_hit_widget()
        _, hitArm2, _,_,_, propertiesArm2 = beamSensorArm2.test_beam_hit_widget()

        if hitSpawn:
            if elapsed_time(startTimeSpawn) > 1:
                createCylinder()
                startTimeSpawn = time.time()

        if hitArm1 and propertiesArm1 == 'green':
            moveConveyors(0)
            if qarm1_State == 6:
                qarm1_State = 0
                pickAndPlace(qarm1, qarm1_State)
                startTimeQarm1 = time.time()

        if qarm1_State < 6:
            #print(qarm1_State)
            if elapsed_time(startTimeQarm1) > 1.5:
                qarm1_State =  qarm1_State + 1
                pickAndPlace(qarm1, qarm1_State)
                startTimeQarm1 = startTimeQarm1 + 1.5

                if qarm1_State == 4:
                    moveConveyors(0.1)

        if hitArm2 and propertiesArm2 == 'blue':
            moveConveyors(0)
            if qarm2_State == 6:
                qarm2_State = 1
                pickAndPlace(qarm2, qarm2_State)
                startTimeQarm2 = time.time()

        if qarm2_State < 6:
            if elapsed_time(startTimeQarm2) > 1:
                qarm2_State =  qarm2_State + 1
                pickAndPlace(qarm2, qarm2_State)
                startTimeQarm2 = startTimeQarm2 + 1

                if qarm2_State == 4:
                    moveConveyors(0.1)


        qarm1Cam.read_RGB()
        cv2.imshow('My RGB', qarm1Cam.imageBufferRGB)

        qarm1Cam.read_depth()
        cv2.imshow('My Depth', qarm1Cam.imageBufferDepthPX/max_distance)

        qarm2Cam.read_RGB()
        cv2.imshow('My RGB2', qarm2Cam.imageBufferRGB)

        qarm2Cam.read_depth()
        cv2.imshow('My Depth2', qarm2Cam.imageBufferDepthPX/max_distance)

        cv2.waitKey(1)


findObj()