from magicbot import tunable
from ctre import WPI_TalonSRX
from wpilib import PIDController
from wpilib.drive import DifferentialDrive
import math
from networktables import NetworkTables
from ctre import PigeonIMU
from .common import PID

# from subsystems.vision import FROGVision
# from subsystems.common import PID


class FROGGyro(PigeonIMU):

    # create a NetworkTables value and set it to 0
    heading = tunable(0)

    def __init__(self, id_or_srx):
        super().__init__(id_or_srx)

    def getCompassHeading(self):
        # update tunable from superclass and return it for use
        self.heading = super().getCompassHeading()
        return self.heading


class FROGDrive(DifferentialDrive):

    # drivetrain characteristics.  Values halved from last year since
    # we are using Falcon 500 with half the encoder resolution
    ENCODER_TICKS_PER_REV = 2048  # halved from last year
    MAX_VELOCITY = 1500  # halved from last year
    WHEEL_DIAMETER = 6
    TICKS_PER_INCH = ENCODER_TICKS_PER_REV / (math.pi * WHEEL_DIAMETER)
    TICKS_PER_ANGLE = 4000 / 180  # in degrees - halved from last year

    # PIDs for drivetrain
    VelocityPID = PID(slot=0, f=0.313)
    PositionPID = PID(slot=0, f=0.320, p=0.55)
    RotatePID = PID(slot=0, f=0.320, p=0.76)
    TurnPID = PID(p=0.035, d=0.10, i=0.001)
    PIDOutputLimit = 0.66
    TURNDEADBAND = 1.5  # in degrees

    # Motion Magic settings
    MM_ACCELERATION = 500
    MM_CRUISE_VELOCITY = 1000

    # TODO: change to tunables?  I think thesea are only needed
    # to hold values to place on network tables
    left_encoder_current = 0
    right_encoder_current = 0

    # drive motors (channels defined in robot.py)
    leftMaster: WPI_TalonSRX
    rightMaster: WPI_TalonSRX
    leftSlave: WPI_TalonSRX
    rightSlave: WPI_TalonSRX

    # attributes used by velocity drive
    command_speed = None
    command_rotation = None
    # TODO: change these to tunables?  Technically they are not needed
    # except to hold a value long enough to publish to network tables
    command_rightvel = None
    command_leftvel = None

    # attributes used by position drive
    # TODO: clean these up along with velocity drive attributes,
    # make naming consistent and consolidate if possible.
    leftPosition = None
    rightPosition = None
    positionCommanded = None

    def __init__(self):
        # init can't be used for setting up magic components because
        # they won't be available until after the contstructor has
        # finished.  Use the setup method instead.
        pass

    def assign_slaves(self):
        self.leftSlave.follow(self.leftMaster)
        self.rightSlave.follow(self.rightMaster)

    def set_motor_output(self):
        self.leftMaster.setInverted(False)
        self.leftSlave.setInverted(False)
        self.rightMaster.setInverted(True)
        self.rightSlave.setInverted(True)

    def init_nt(self):
        NetworkTables.initialize()
        self.drive_encoders = NetworkTables.getTable("drive_encoder_values")
        self.drive_command = NetworkTables.getTable("drive_commands")
        self.vision_table = NetworkTables.getTable("Vision")

    def reset_position_commands(self):
        self.leftPosition = None
        self.rightPosition = None
        self.positionCommanded = None

        # zero out velocity drive-related commands
        # TODO: see if we can refactor this
        self.command_speed = None
        self.command_rotation = None
        self.command_rightvel = None
        self.command_leftvel = None

    def init_velocity_drive(self):
        self.setPID(self.VelocityPID)

    def init_position_drive(self):
        self.setPID(self.PositionPID)
        self.reset_position_commands()

    def config_encoders(self):
        self.leftMaster.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0
        )
        self.rightMaster.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0
        )

    def reset_encoders(self):
        # resetting encoder positions to 0
        self.leftMaster.setSelectedSensorPosition(0, 0, 0)
        self.rightMaster.setSelectedSensorPosition(0, 0, 0)

    def set_encoder_direction(self):
        # Reverses the encoder direction so forward movement always
        # results in a positive increase in the encoder ticks.
        self.leftMaster.setSensorPhase(True)
        self.rightMaster.setSensorPhase(True)

    def set_motor_neutral_mode(self):
        self.leftMaster.setNeutralMode(1)
        self.rightMaster.setNeutralMode(1)

    def set_motion_magic(self):
        for motor in [self.leftMaster, self.rightMaster]:
            motor.configMotionAcceleration(self.MM_ACCELERATION, 0)
            motor.configMotionCruiseVelocity(self.MM_CRUISE_VELOCITY, 0)

    def setup(self):
        #
        self.assign_slaves()
        self.set_motor_output()
        self.init_nt()
        self.reset_position_commands()
        self.init_velocity_drive()
        self.set_encoder_direction()
        self.reset_encoders()
        self.set_motor_neutral_mode()
        self.set_motion_magic()

        # init DifferentialDrive with left and right controllers
        super().__init__(self.leftMaster, self.rightMaster)
        self.setSafetyEnabled(False)

        # TODO: might want to move these to a chassis component that
        # has the drivetrain, gyro, and vision as objects.
        self.gyro = FROGGyro(self.leftSlave)
        # self.vision = FROGVision()

    def teleInit(self):
        """method called from robot teleopInit() method to initialize
        PIDs and any other drivetrain characteristics specifically for
        teleop control."""
        self.resetPosition()
        # added in to make sure the slaves are set as followers
        # this is especially a problem if switching to test mode
        # as it removes the configuration of the slaves.
        self.initMotors(self.robot.motorConfig)

    def setPID(self, pid):
        for motor_control in (self.leftMaster, self.rightMaster):
            motor_control.config_kP(pid.slot, pid.p, 0)
            motor_control.config_kI(pid.slot, pid.i, 0)
            motor_control.config_kD(pid.slot, pid.d, 0)
            motor_control.config_kF(pid.slot, pid.f, 0)

    def velocityDrive(self, speed, rotation):
        self.command_speed = speed
        self.command_rotation = rotation

        speed = math.copysign(speed * speed, speed)
        rotation = math.copysign(rotation * rotation, rotation)

        maxInput = math.copysign(max(abs(speed), abs(rotation)), speed)
        if speed >= 0.0:

            if rotation >= 0.0:
                leftMotorSpeed = maxInput
                rightMotorSpeed = speed - rotation
            else:
                leftMotorSpeed = speed + rotation
                rightMotorSpeed = maxInput
        else:
            if rotation >= 0.0:
                leftMotorSpeed = speed + rotation
                rightMotorSpeed = maxInput
            else:
                leftMotorSpeed = maxInput
                rightMotorSpeed = speed - rotation

        # TODO: the below function would limit the commanded speed to the -1
        # to 1 range we need to determine if this is even necessary, but the
        # old function has been deprecated, so we will need to refactor if
        # needed.
        # leftMotorSpeed = wpilib.RobotDrive.limit(leftMotorSpeed)
        # rightMotorSpeed = wpilib.RobotDrive.limit(rightMotorSpeed)

        leftMotorVel = leftMotorSpeed * self.MAX_VELOCITY
        rightMotorVel = rightMotorSpeed * self.MAX_VELOCITY

        self.command_leftvel = leftMotorVel
        self.command_rightvel = rightMotorVel

        self.leftMaster.set(WPI_TalonSRX.ControlMode.Velocity, leftMotorVel)
        self.rightMaster.set(WPI_TalonSRX.ControlMode.Velocity, rightMotorVel)

    def positionDrive(self):
        """Distance is defined in inches and gets multiplied by ticks per inch
        then added to the current encoder position for each side"""
        if self.positionCommanded and self.leftPosition and self.rightPosition:
            self.leftMaster.set(WPI_TalonSRX.ControlMode.MotionMagic, self.leftPosition)
            self.rightMaster.set(
                WPI_TalonSRX.ControlMode.MotionMagic, self.rightPosition
            )

    def resetPosition(self):
        self.leftPosition = None
        self.rightPosition = None
        self.positionCommanded = None

    def setPosition(self, distance):

        if distance and not self.positionCommanded:
            self.leftPosition = (
                distance * self.TICKS_PER_INCH
            ) + self.leftMaster.getSelectedSensorPosition(0)
            self.rightPosition = (
                distance * self.TICKS_PER_INCH
            ) + self.rightMaster.getSelectedSensorPosition(0)
        self.positionCommanded = True
        self.setPID(self.PositionPID)

    def setRotate(self, angle):
        if angle:
            ticks = angle * self.TICKS_PER_ANGLE

            if angle and not self.positionCommanded:
                self.leftPosition = ticks + self.leftMaster.getSelectedSensorPosition()
                self.rightPosition = (
                    -ticks + self.rightMaster.getSelectedSensorPosition()
                )
            self.positionCommanded = True
            self.setPID(self.RotatePID)

    def setTargetAngle(self):
        target_angle = self.vision.getTargetAngle()
        # gyro_angle = self.gyro.getYaw()

        if target_angle:
            self.setRotate(target_angle)
            # self.positionDrive()
        else:
            self.vision_table.putNumber("Vision_angle", -999)

    def initTurnPID(self):
        pass

    def pidWrite(self, output):
        self.turn_output = output

    def updateNT(self):
        """update network tables with drive telemetry"""

        self.drive_encoders.putNumber(
            "left_pos", self.leftMaster.getSelectedSensorPosition(0)
        )
        self.drive_encoders.putNumber(
            "right_pos", self.rightMaster.getSelectedSensorPosition(0)
        )
        self.drive_encoders.putNumber(
            "left_vel", self.leftMaster.getSelectedSensorVelocity(0)
        )
        self.drive_encoders.putNumber(
            "right_vel", self.rightMaster.getSelectedSensorVelocity(0)
        )
        if self.command_speed:
            self.drive_command.putNumber("commanded speed", self.command_speed)
        if self.command_rotation:
            self.drive_command.putNumber("commanded rotation", self.command_rotation)
        if self.command_rightvel:
            self.drive_command.putNumber("commanded rvel", self.command_rightvel)
        if self.command_leftvel:
            self.drive_command.putNumber("commanded lvel", self.command_leftvel)
        if self.leftPosition:
            self.drive_command.putNumber("commanded lpos", self.leftPosition)
        if self.rightPosition:
            self.drive_command.putNumber("commanded rpos", self.rightPosition)

    def execute(self):
        pass
