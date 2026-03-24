import sys
sys.path.append(r'C:\Users\dm8g22\Documents\Quanser QArm Student\libraries\python')
from qvl.qlabs import QuanserInteractiveLabs
from qvl.system import QLabsSystem
from qvl.free_camera import QLabsFreeCamera
from qvl.basic_shape import QLabsBasicShape
from qvl.qarm import QLabsQArm
from qvl.widget import QLabsWidget
from qvl.real_time import QLabsRealTime
from pal.products.qarm import QArm, QArmRealSense
from pal.utilities.vision import Camera3D
from random import randrange
from qvl.conveyor_straight import QLabsConveyorStraight
import os
from pathlib import Path
from qvl.shredder import QLabsShredder
import numpy as np

class QArmWorkspace:

    ''' This QArm workspace class is used to create the work environment for QArm to proceed the pick and drop tasks. 
   There are 4 cells assigned with 3 random colours (red, green, blue) and 2 cell types (D-cell, 18650 cell).'''

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# region: Initialise the work space

    def __init__(self):
        self.qlabs = QuanserInteractiveLabs()
        self.hSystem = None
        self.camera = None
        self.baseTable = None
        self.baseMat = None
        self.QArm = None
        self.redBin = None
        self.greenBin = None
        self.blueBin = None
        self.dCellBin = None
        self.cell18650Bin = None
        self.cellRack = None
        self.cellWidget = None
        self.conveyor = None
        self.shredder = None

    def connect_to_qlabs(self):

        ''' Connect to the Qlab virtual environment. '''

        print("Connecting to QLabs...")
        try:
            self.qlabs.open("localhost")
            print("Connected")
        except Exception as e:
            print(f"Unable to connect to QLabs: {e}")

    def initialise_system(self):
        self.hSystem = QLabsSystem(self.qlabs)
        self.hSystem.set_title_string('dm8g22') ## CHANGE YOUR USERNAME
        self.qlabs.destroy_all_spawned_actors()
        QLabsRealTime().terminate_all_real_time_models()

# endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# region: Create components

    ##### Create QArm worksapce ######

    def create_workspace(self):

        ''' Create camera, based table, based mat, cell rack, bins and QArm in the workspace. '''
        # Create a conveyor
        # self.redconveyor = QLabsConveyorStraight(self.qlabs)
        # self.redconveyor.spawn_id(actorNumber=25,
        #                           location=[0.3,-0.5,0.15],
        #                           rotation=[0,0,np.deg2rad(90)],
        #                           scale=[1,1,1],
        #                           configuration=0,
        #                           waitForConfirmation=True)
        # self.greenconveyor = QLabsConveyorStraight(self.qlabs)
        # self.greenconveyor.spawn_id(actorNumber=27,
        #                           location=[0,0.8,0.25],
        #                           rotation=[0,0,np.deg2rad(270)],
        #                           scale=[1,1,1],
        #                           configuration=0,
        #                           waitForConfirmation=True)
        # self.greenconveyor2 = QLabsConveyorStraight(self.qlabs)
        # self.greenconveyor2.spawn_id(actorNumber=29,
        #                           location=[-0.35,0.9,0.15],
        #                           rotation=[0,0,np.deg2rad(0)],
        #                           scale=[1,1,1],
        #                           configuration=0,
        #                           waitForConfirmation=True)
        # self.blueconveyor = QLabsConveyorStraight(self.qlabs)
        # self.blueconveyor.spawn_id(actorNumber=28,
        #                           location=[0.3,0.8,0.25],
        #                           rotation=[0,0,np.deg2rad(270)],
        #                           scale=[1,1,1],
        #                           configuration=1,
        #                           waitForConfirmation=True)
        # self.blueconveyor2 = QLabsConveyorStraight(self.qlabs)
        # self.blueconveyor2.spawn_id(actorNumber=30,
        #                           location=[0.65,0.9,0.15],
        #                           rotation=[0,0,np.deg2rad(180)],
        #                           scale=[1,1,1],
        #                           configuration=0,
        #                           waitForConfirmation=True)                
        
        # self.redshredder = QLabsShredder(self.qlabs)
        # self.redshredder.spawn_id_degrees(actorNumber=26,
        #                      location=[0.3, -0.6, 0],
        #                      rotation=[0, 0, 0],
        #                      scale=[1, 1, 1],
        #                      configuration=0,
        #                      waitForConfirmation=True)
        # self.greenshredder = QLabsShredder(self.qlabs)
        # self.greenshredder.spawn_id_degrees(actorNumber=31,
        #                      location=[-0.4, 0.9, 0],
        #                      rotation=[0, 0, 0],
        #                      scale=[1, 1, 1],
        #                      configuration=1,
        #                      waitForConfirmation=True)
        # self.blueshredder = QLabsShredder(self.qlabs)
        # self.blueshredder.spawn_id_degrees(actorNumber=32,
        #                      location=[0.7, 0.9, 0],
        #                      rotation=[0, 0, 0],
        #                      scale=[1, 1, 1],
        #                      configuration=2,
        #                      waitForConfirmation=True)
        self.redconveyor = QLabsConveyorStraight(self.qlabs)
        self.redconveyor.spawn_id(actorNumber=25,
                                  location=[0.3, -1.4, 0.15],
                                  rotation=[0,0,np.deg2rad(90)],
                                  scale=[1,1,1],
                                  configuration=0,
                                  waitForConfirmation=True)
        
        # greenconveyor: 0.8 -> -0.1
        self.greenconveyor = QLabsConveyorStraight(self.qlabs)
        self.greenconveyor.spawn_id(actorNumber=27,
                                  location=[0, -0.1, 0.25],
                                  rotation=[0,0,np.deg2rad(270)],
                                  scale=[1,1,1],
                                  configuration=0,
                                  waitForConfirmation=True)

        # greenconveyor2: 0.9 -> 0.0
        self.greenconveyor2 = QLabsConveyorStraight(self.qlabs)
        self.greenconveyor2.spawn_id(actorNumber=29,
                                  location=[-0.35, 0.0, 0.15],
                                  rotation=[0,0,np.deg2rad(0)],
                                  scale=[1,1,1],
                                  configuration=0,
                                  waitForConfirmation=True)

        # blueconveyor: 0.8 -> -0.1
        self.blueconveyor = QLabsConveyorStraight(self.qlabs)
        self.blueconveyor.spawn_id(actorNumber=28,
                                  location=[0.3, -0.1, 0.25],
                                  rotation=[0,0,np.deg2rad(270)],
                                  scale=[1,1,1],
                                  configuration=1,
                                  waitForConfirmation=True)

        # blueconveyor2: 0.9 -> 0.0
        self.blueconveyor2 = QLabsConveyorStraight(self.qlabs)
        self.blueconveyor2.spawn_id(actorNumber=30,
                                  location=[0.65, 0.0, 0.15],
                                  rotation=[0,0,np.deg2rad(180)],
                                  scale=[1,1,1],
                                  configuration=0,
                                  waitForConfirmation=True)                
        
        # redshredder: -0.6 -> -1.5
        self.redshredder = QLabsShredder(self.qlabs)
        self.redshredder.spawn_id_degrees(actorNumber=26,
                                     location=[0.3, -1.5, 0],
                                     rotation=[0, 0, 0],
                                     scale=[1, 1, 1],
                                     configuration=0,
                                     waitForConfirmation=True)
        
        # greenshredder: 0.9 -> 0.0
        self.greenshredder = QLabsShredder(self.qlabs)
        self.greenshredder.spawn_id_degrees(actorNumber=31,
                                     location=[-0.4, 0.0, 0],
                                     rotation=[0, 0, 0],
                                     scale=[1, 1, 1],
                                     configuration=1,
                                     waitForConfirmation=True)
        
        # blueshredder: 0.9 -> 0.0
        self.blueshredder = QLabsShredder(self.qlabs)
        self.blueshredder.spawn_id_degrees(actorNumber=32,
                                     location=[0.7, 0.0, 0],
                                     rotation=[0, 0, 0],
                                     scale=[1, 1, 1],
                                     configuration=2,
                                     waitForConfirmation=True)        
# --- 2. MIRRORED EXTENSIONS FOR ROBOT 2 ---
        self.redconveyor2 = QLabsConveyorStraight(self.qlabs)
        self.redconveyor2.spawn_id(actorNumber=34,
                                  location=[0.3, 1.4, 0.15],
                                  rotation=[0,0,np.deg2rad(270)],
                                  scale=[1,1,1],
                                  configuration=0,
                                  waitForConfirmation=True)                
        self.greenconveyor3 = QLabsConveyorStraight(self.qlabs)
        self.greenconveyor3.spawn_id(actorNumber=35,
                                  location=[0, 0.1, 0.25],
                                  rotation=[0,0,np.deg2rad(90)],
                                  scale=[1,1,1],
                                  configuration=0,
                                  waitForConfirmation=True)
        self.blueconveyor3 = QLabsConveyorStraight(self.qlabs)
        self.blueconveyor3.spawn_id(actorNumber=36,
                                  location=[0.3, 0.1, 0.25],
                                  rotation=[0,0,np.deg2rad(90)],
                                  scale=[1,1,1],
                                  configuration=1,
                                  waitForConfirmation=True)
        self.redshredder2 = QLabsShredder(self.qlabs)
        self.redshredder2.spawn_id_degrees(actorNumber=37,
                                     location=[0.3, 1.5, 0],
                                     rotation=[0, 0, 0],
                                     scale=[1, 1, 1],
                                     configuration=0,
                                     waitForConfirmation=True)
        # Create camera and initialise it
        self.camera = QLabsFreeCamera(self.qlabs)
        self.camera.spawn_id(actorNumber=0,
                             location=[2, 2, 2],
                             rotation=[0, 0.5, -2.3],
                             scale=[1, 1, 1],
                             configuration=0,
                             waitForConfirmation=True)
        self.camera.possess()

        # Create base table
        self.baseTable = QLabsBasicShape(self.qlabs)
        self.baseTable.spawn_id(actorNumber=1,
                                location=[0, -0.9, 0.1],
                                rotation=[0, 0, 0],
                                scale=[0.25, 0.25, 0.2],
                                configuration=0,
                                waitForConfirmation=True)
        self.baseTable.set_material_properties(color=[0.1, 0.1, 0.1],
                                                roughness=0.4,
                                                metallic=False)
        self.baseTable2 = QLabsBasicShape(self.qlabs)
        self.baseTable2.spawn_id(actorNumber=33,
                                location=[0, 0.9, 0.1],
                                rotation=[0, 0, 0],
                                scale=[0.25, 0.25, 0.2],
                                configuration=0,
                                waitForConfirmation=True)
        self.baseTable2.set_material_properties(color=[0.1, 0.1, 0.1],
                                                roughness=0.4,
                                                metallic=False)
        # Create base mat
        # self.baseMat = QLabsBasicShape(self.qlabs)
        # self.baseMat.spawn_id(actorNumber=2,
        #                       location=[0, 0, 0.75],
        #                       rotation=[0, 0, 0],
        #                       scale=[2, 2, 0.1],
        #                       configuration=1,
        #                       waitForConfirmation=True)
        # self.baseMat.set_material_properties(color=[0.3, 0.3, 0.3],
        #                                       roughness=0.4,
        #                                       metallic=False)
        # self.baseMat.set_enable_collisions(enableCollisions=True,
        #                                     waitForConfirmation=True)
        
        # Create one QArm
        self.QArm = QLabsQArm(self.qlabs)
        self.QArm.spawn_id_degrees(actorNumber=3,
                                   location=[0, -0.9, 0.2],
                                   rotation=[0, 0, -90],
                                   scale=[1, 1, 1],
                                   configuration=0,
                                   waitForConfirmation=True)
        self.QArm2 = QLabsQArm(self.qlabs)
        self.QArm2.spawn_id_degrees(actorNumber=34,
                                   location=[0, 0.9, 0.2],
                                   rotation=[0, 0, -90],
                                   scale=[1, 1, 1],
                                   configuration=0,
                                   waitForConfirmation=True)
        # Create bins
        # Create red D-cell bin
        # self.redDcellBin = QLabsBasicShape(self.qlabs)
        # self.redDcellBin.spawn_id_box_walls_from_center_degrees(actorNumbers=[10, 11, 12, 13, 14],
        #                                                    centerLocation=[0.3, 0.45, 0],
        #                                                    yaw=0,
        #                                                    xSize=0.15, ySize=0.15, zHeight=0.1,
        #                                                    wallThickness=0.0015,
        #                                                    floorThickness=0.04,
        #                                                    wallColor=[0.8, 0.8, 0.8],
        #                                                    floorColor=[0.5, 0, 0],
        #                                                    waitForConfirmation=True)
        
        # # Create green bin
        # self.greenBatBin = QLabsBasicShape(self.qlabs)
        # self.greenBatBin.spawn_id_box_walls_from_center_degrees(actorNumbers=[15, 16, 17, 18, 19],
        #                                                    centerLocation=[0.147, 0.45, 0],
        #                                                    yaw=0,
        #                                                    xSize=0.15, ySize=0.15, zHeight=0.1,
        #                                                    wallThickness=0.0015,
        #                                                    floorThickness=0.04,
        #                                                    wallColor=[0.8, 0.8, 0.8],
        #                                                    floorColor=[0, 0.5, 0],
        #                                                    waitForConfirmation=True)

        # # Create blue bin
        # self.blueBatBin = QLabsBasicShape(self.qlabs)
        # self.blueBatBin.spawn_id_box_walls_from_center_degrees(actorNumbers=[20, 21, 22, 23, 24],
        #                                                    centerLocation=[-0.003, 0.45, 0],
        #                                                    yaw=0,
        #                                                    xSize=0.15, ySize=0.15, zHeight=0.1,
        #                                                    wallThickness=0.0015,
        #                                                    floorThickness=0.04,
        #                                                    wallColor=[0.8, 0.8, 0.8],
        #                                                    floorColor=[0, 0, 0.5],
        #                                                    waitForConfirmation=True)
        
        # Create cell rack
        self.cellRack = QLabsBasicShape(self.qlabs)
        self.cellRack.spawn_id(actorNumber=40,
                               location=[0.57, -0.85, 0.25],
                               rotation=[0, 0, 0],
                               scale=[0.1, 0.4, 0.1],
                               configuration=0,
                               waitForConfirmation=True)
        self.cellRack.set_material_properties(color=[0.4, 0.4, 0.4],
                                              roughness=0.1,
                                              metallic=False)
        self.cellRack2 = QLabsBasicShape(self.qlabs)
        self.cellRack2.spawn_id(actorNumber=38,
                               location=[0.57, 0.85, 0.25],
                               rotation=[0, 0, 0],
                               scale=[0.1, 0.4, 0.1],
                               configuration=0,
                               waitForConfirmation=True)
        self.cellRack2.set_material_properties(color=[0.4, 0.4, 0.4],
                                              roughness=0.1,
                                              metallic=False)
        
        #  ADJUST THIS SECTION TO INCLUDE RANDOM CELL SIZE AND COLOUR
        # Create 4 cell widgets in a row  
    def cell_spawn(self):
        self.cellWidget = QLabsWidget(self.qlabs)
        self.cellWidget.widget_spawn_shadow(enableShadow=True)
        scale = [[0.028, 0.028, 0.06]]
        color = [[0.6, 0, 0],[0,0.6,0],[0,0,0.6]] # 3 colors of random cell
        locations = [[0.58, -0.7, 0.4], [0.58, -0.8, 0.4], [0.58, -0.9, 0.4], [0.58, -1, 0.4]]

        for location in locations:
            scale1 = randrange(1)
            color1 = randrange(3)
            self.cellWidget.spawn(location=location,
                             rotation=[0, 0, 0],
                             scale=scale[scale1],
                             configuration= self.cellWidget.CYLINDER,
                             color=color[color1],
                             measuredMass=100,
                             IDTag=0,
                             properties='',
                             waitForConfirmation=True)
# endregion
    def control_conveyor(self, speed):
        ''' 
        Sets the conveyor belt speed in m/s.
        '''
        if self.redconveyor:
            # The library uses the instance itself to know which conveyor to move,
            # so you ONLY pass the speed value.
            self.redconveyor.set_speed(speed)
            self.blueconveyor.set_speed(speed)
            self.greenconveyor.set_speed(speed)
            self.greenconveyor2.set_speed(speed)
            self.blueconveyor2.set_speed(speed)
            self.redconveyor2.set_speed(speed)
            self.blueconveyor3.set_speed(speed)
            self.greenconveyor3.set_speed(speed)
            print(f"Conveyor speed set to {speed} m/s")
        else:
            print("Conveyor not initialized! Call create_workspace first.")
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# region: Real time model
        
    ##### Start real time model with the arm #####

    def start_real_time_model(self):

        ''' Start the real-time model for QArm. '''

        print("Starting real-time model...")

        qarm_model_path = os.path.join(os.environ['RTMODELS_DIR'], 'QArms/QArm_Spawn0')

        # current_dir = Path(__file__).resolve().parent
        # project_root = current_dir.parent
        # qarm_model_path = project_root / 'libraries' / 'resources' / 'rtmodels' / 'QArms' / 'QArm_Spawn0'

        QLabsRealTime().start_real_time_model(qarm_model_path, actorNumber=3,
                                               additionalArguments='-uri_hil tcpip://localhost:18900 -uri_video tcpip://localhost:18901')

        print("Real-time model started.")


# endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Example usage
# workSpace = QArmWorkspace()
# workSpace.connect_to_qlabs()
# workSpace.initialise_system()
# workSpace.create_workspace()
# workSpace.start_real_time_model()