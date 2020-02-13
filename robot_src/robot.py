#!/usr/bin/env python3
"""
    This is a demo program showing the use of the RobotDrive class,
    specifically it contains the code necessary to operate a robot with
    a single joystick
"""
import magicbot
import wpilib
from ctre import WPI_TalonFX, WPI_TalonSRX
from components.drivetrain import FROGDrive
from components.driverstation import FROGStick
from components.turret import FROGTurret, Shooter


class FROGbot(magicbot.MagicRobot):
    """
        Initialize components here.
    """

    chassis: FROGDrive
    turret: FROGTurret

    shooter: Shooter

    def createObjects(self):
        """Create motors and inputs"""
        # chassis components
        self.leftMaster = WPI_TalonFX(11)
        self.rightMaster = WPI_TalonFX(12)
        self.leftSlave = WPI_TalonFX(13)
        self.rightSlave = WPI_TalonFX(14)

        self.intake = WPI_TalonSRX(21)
        self.lowerConveyor = WPI_TalonFX(22)
        self.upperConveyor = WPI_TalonFX(23)

        self.azimuth = WPI_TalonSRX(31)
        self.elevation = WPI_TalonSRX(32)
        self.flywheel = WPI_TalonFX(33)

        self.controllerWheel = WPI_TalonSRX(41)

        # controls
        self.joystick = FROGStick(0)
        self.xbox = wpilib.XboxController(1)

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.chassis.reset_encoder_values()

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""

        if self.joystick.getDebouncedButton(11) == 1:
            self.chassis.toggleControlMode()

        if self.joystick.getPOV() == 0:
            self.chassis.setPosition(36)  # tell it to roll forward 36 inches
        if self.joystick.getPOV() == 180:
            self.chassis.setPosition(-36)  # tell it to roll backward 36 inches
        # feed joystick to the drivetrain
        self.chassis.setVelocity(
            self.joystick.getSpeed(),
            self.joystick.getRotation()
        )

        if self.joystick.getRawButton(12):
            self.turret.shooter.setFlywheelPercent(self.joystick.getThrottle())

    def testInit(self):

        pass

    def testPeriodic(self):

        pass


if __name__ == "__main__":
    wpilib.run(FROGbot)
