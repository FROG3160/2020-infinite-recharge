#!/usr/bin/env python3
"""
    This is a demo program showing the use of the RobotDrive class,
    specifically it contains the code necessary to operate a robot with
    a single joystick
"""
import magicbot
import wpilib
from ctre import WPI_TalonSRX
from components.drivetrain import FROGDrive
from components.driverstation import FROGStick


class FROGbot(magicbot.MagicRobot):
    """
        Initialize components here.
    """

    chassis: FROGDrive
    # loader: Loader
    # shooter: FROGShooter

    def createObjects(self):
        """Create motors and inputs"""
        # chassis components
        self.leftMaster = WPI_TalonSRX(11)
        self.rightMaster = WPI_TalonSRX(12)
        self.leftSlave = WPI_TalonSRX(13)
        self.rightSlave = WPI_TalonSRX(14)

        # controls
        self.joystick = FROGStick(0)
        self.xbox = wpilib.XboxController(1)

    def teleopInit(self):
        """Called when teleop starts; optional"""

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""

        # feed joystick to the drivetrain
        self.chassis.setVelocity(
            self.joystick.getSpeed(),
            self.joystick.getRotation()
        )


if __name__ == "__main__":
    wpilib.run(FROGbot)
