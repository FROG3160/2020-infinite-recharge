# import wpilib
from networktables import NetworkTables

# from wpilib.command.subsystem import Subsystem
import math
from magicbot import feedback
from .common import limit

# import logging

# logging.basicConfig(level=logging.DEBUG)

TARGETING_TABLE = "targets"
POWERPORT_TABLE = TARGETING_TABLE + '/PowerPort'
POWERCELL_TABLE = TARGETING_TABLE + '/PowerCell'
CONTROL_TABLE = TARGETING_TABLE + '/control'

AZIMUTH_PIXEL_DECEL_LIMIT = 160
AZIMUTH_PIXEL_TOLERANCE = 20


class FROGVision:

    # camera resolution constants
    CAM_RES_H = 320
    CAM_RES_V = 240
    FOV_RAD_H = 0.896376
    FOV_RAD_V = 0.74858

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

    @feedback(key="PowerPort_x")
    def getTargetX(self):
        targetx = self.powerport_table.getNumber("center_x", None)
        if targetx == -999:
            return None
        else:
            return targetx

    @feedback(key='PowerPort_y')
    def getTargetY(self):
        targety = self.powerport_table.getNumber("center_y", None)
        if targety == -999:
            return None
        else:
            return targety

    @feedback(key='PowerPort_angle')
    def getTargetAngle(self):

        target_x = self.getTargetX()

        if target_x:

            return ((self.RAD_PER_PIX_H) * (target_x - self.FOV_CENTER_H)) * (
                180 / math.pi
            )

        else:

            return None

    def getTargetDistance(self):
        pass

    @feedback(key='PowerPort_rotation')
    def getTargetRotation(self):

        target_x = self.getTargetX()

        if target_x:
            rotation = (
                limit(
                    target_x - self.FOV_CENTER_H,
                    -AZIMUTH_PIXEL_DECEL_LIMIT,
                    AZIMUTH_PIXEL_DECEL_LIMIT,
                )
                / AZIMUTH_PIXEL_DECEL_LIMIT
            )

            return rotation
        # (self.RAD_PER_PIX_H * (target_x - self.FOV_CENTER_H)) / (
        # self.FOV_RAD_H / 2
        # )
        else:
            return 0
