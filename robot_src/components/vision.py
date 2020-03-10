# import wpilib
from networktables import NetworkTables

# from wpilib.command.subsystem import Subsystem
import math
from magicbot import feedback
from .common import Buffer

# import logging

# logging.basicConfig(level=logging.DEBUG)

TARGETING_TABLE = "targets"
POWERPORT_TABLE = TARGETING_TABLE + '/PowerPort'
POWERCELL_TABLE = TARGETING_TABLE + '/PowerCell'
CONTROL_TABLE = TARGETING_TABLE + '/control'

# camera resolution constants
CAM_RES_H = 320
CAM_RES_V = 240
FOV_RAD_H = 0.896376
FOV_RAD_V = 0.74858

VISION_NULLVALUE = -999


class FROGVision:

    TARGET_HEIGHT = 24
    CAMERA_HEIGHT = 8

    # calculates radians per pixel
    RAD_PER_PIX_H = FOV_RAD_H / CAM_RES_H
    RAD_PER_PIX_V = FOV_RAD_V / CAM_RES_V

    # calculates the center horizontal and vertical values
    FOV_CENTER_H = CAM_RES_H / 2
    FOV_CENTER_V = CAM_RES_V / 2

    def __init__(self):

        self.powerport_table = NetworkTables.getTable(POWERPORT_TABLE)
        self.powercell_table = NetworkTables.getTable(POWERCELL_TABLE)
        self.targeting_control = NetworkTables.getTable(CONTROL_TABLE)
        self.powerport_error_x = Buffer(12)
        self.powercell_error_x = Buffer(12)
        self.powercell_error_y = Buffer(12)

    @feedback(key="PowerPort_x")
    def getPowerPortX(self):
        targetx = self.powerport_table.getNumber("center_x", None)
        if targetx == VISION_NULLVALUE:
            return None
        else:
            return targetx

    @feedback(key='PowerPort_y')
    def getPowerPortY(self):
        targety = self.powerport_table.getNumber("center_y", None)
        if targety == VISION_NULLVALUE:
            return None
        else:
            return targety

    @feedback(key="PowerCell_x")
    def getPowerCellX(self):
        targetx = self.powercell_table.getNumber("center_x", None)
        if targetx == VISION_NULLVALUE:
            return None
        else:
            return targetx

    @feedback(key='PowerCell_y')
    def getPowerCellY(self):
        targety = self.powercell_table.getNumber("center_y", None)

        if targety == VISION_NULLVALUE:
            return None
        else:
            return targety

    def getPowerPortAngle(self):

        target_x = self.getPowerPortX()

        if target_x:

            return ((self.RAD_PER_PIX_H) * (target_x - self.FOV_CENTER_H)) * (
                180 / math.pi
            )

        else:

            return None

    def getTargetDistance(self):
        pass

    @feedback(key='PowerPortXError')
    def calcPowerPortXError(self):

        target_x = self.getPowerPortX()

        if target_x:
            return target_x - self.FOV_CENTER_H
        else:
            return None

    def calcPowerCellXError(self):
        target_x = self.getPowerCellX()
        if target_x:
            return target_x - self.FOV_CENTER_H
        else:
            return None

    def calcPowerCellYError(self):
        target_y = self.getPowerCellY()
        if target_y:
            return CAM_RES_V - target_y
        else:
            return None

    @feedback(key='PowerPortErrorX')
    def getPowerPortErrorX(self):
        return self.powerport_error_x.average()

    @feedback(key='PowerCellErrorX')
    def getPowerCellErrorX(self):
        return self.powercell_error_x.average()

    @feedback(key='PowerCellErrorY')
    def getPowerCellErrorY(self):
        return self.powercell_error_y.average()

    def execute(self):
        self.powerport_error_x.append(self.calcPowerPortXError())
        self.powercell_error_x.append(self.calcPowerCellXError())
        self.powercell_error_y.append(self.calcPowerCellYError())
