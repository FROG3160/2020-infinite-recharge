from magicbot import tunable
from ctre import WPI_TalonFX
from ctre import ControlMode
from ctre import NeutralMode
from ctre import FeedbackDevice
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

    # Motor Control modes
    VELOCITY_MODE = ControlMode.Velocity
    POSITION_MODE = ControlMode.MotionMagic

    # Motion Magic settings
    MM_ACCELERATION = 500
    MM_CRUISE_VELOCITY = 1000

    # TODO: change to tunables?  I think thesea are only needed
    # to hold values to place on network tables
    left_encoder_current = 0
    right_encoder_current = 0

    # drive motors (channels defined in robot.py)
    leftMaster: WPI_TalonFX
    rightMaster: WPI_TalonFX
    leftSlave: WPI_TalonFX
    rightSlave: WPI_TalonFX

    # motor value for each side of the drivetrain
    left_control = None
    right_control = None
    control_mode = VELOCITY_MODE

    def __init__(self):
        # init can't be used for setting up magic components because
        # they won't be available until after the contstructor has
        # finished.  Use the setup method instead.
        pass

    def config_encoders(self):
        for controller in [self.leftMaster, self.rightMaster]:
            controller.configSelectedFeedbackSensor(
                FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0
            )

    def config_motion_magic(self):
        for motor in [self.leftMaster, self.rightMaster]:
            motor.configMotionAcceleration(self.MM_ACCELERATION, 0)
            motor.configMotionCruiseVelocity(self.MM_CRUISE_VELOCITY, 0)

    def init_nt(self):
        NetworkTables.initialize()
        self.drive_encoders = NetworkTables.getTable("drive_encoder_values")
        self.drive_command = NetworkTables.getTable("drive_commands")
        self.vision_table = NetworkTables.getTable("Vision")

    def init_position_drive(self):
        self.setPID(self.PositionPID)
        self.reset_control_values()
        self.drivemode = self.POSITION_MODE

    def init_velocity_drive(self):
        self.setPID(self.VelocityPID)
        self.reset_control_values()
        self.drivemode = self.VELOCITY_MODE

    def reset_control_values(self):
        self.left_control = None
        self.right_control = None

    def reset_encoder_values(self):
        # resetting encoder positions to 0
        self.leftMaster.setSelectedSensorPosition(0, 0, 0)
        self.rightMaster.setSelectedSensorPosition(0, 0, 0)

    def set_encoder_direction(self):
        # Reverses the encoder direction so forward movement always
        # results in a positive increase in the encoder ticks.
        self.leftMaster.setSensorPhase(True)
        self.rightMaster.setSensorPhase(True)

    def set_motor_slaves(self):
        self.leftSlave.follow(self.leftMaster)
        self.rightSlave.follow(self.rightMaster)

    def set_motor_output(self):
        self.leftMaster.setInverted(False)
        self.leftSlave.setInverted(False)
        self.rightMaster.setInverted(True)
        self.rightSlave.setInverted(True)

    def set_motor_neutral_mode(self):
        self.leftMaster.setNeutralMode(NeutralMode.Coast)
        self.rightMaster.setNeutralMode(NeutralMode.Coast)

    def setup(self):
        #
        self.set_motor_slaves()
        self.set_motor_output()
        self.set_motor_neutral_mode()

        self.config_encoders()
        self.set_encoder_direction()
        self.reset_encoder_values()

        self.config_motion_magic()

        self.init_velocity_drive()
        self.init_nt()

        # init DifferentialDrive with left and right controllers
        super().__init__(self.leftMaster, self.rightMaster)
        self.setSafetyEnabled(False)

        # TODO: might want to move these to a chassis component that
        # has the drivetrain, gyro, and vision as objects.
        self.gyro = FROGGyro(self.leftSlave)
        # self.vision = FROGVision()

    def setPID(self, pid):
        for motor_control in (self.leftMaster, self.rightMaster):
            motor_control.config_kP(pid.slot, pid.p, 0)
            motor_control.config_kI(pid.slot, pid.i, 0)
            motor_control.config_kD(pid.slot, pid.d, 0)
            motor_control.config_kF(pid.slot, pid.f, 0)

    def setVelocity(self, speed, rotation):
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

        self.left_control = leftMotorSpeed * self.MAX_VELOCITY
        self.right_control = rightMotorSpeed * self.MAX_VELOCITY
        self.control_mode = self.VELOCITY_MODE

    def setPosition(self, distance):

        if distance:
            self.left_control = (
                distance * self.TICKS_PER_INCH
            ) + self.leftMaster.getSelectedSensorPosition(0)
            self.right_control = (
                distance * self.TICKS_PER_INCH
            ) + self.rightMaster.getSelectedSensorPosition(0)
        self.control_mode = self.POSITION_MODE
        self.setPID(self.PositionPID)

    def setRotate(self, angle):
        if angle:
            ticks = angle * self.TICKS_PER_ANGLE

            self.left_control = ticks + self.leftMaster.getSelectedSensorPosition()
            self.right_control = -ticks + self.rightMaster.getSelectedSensorPosition()
            self.control_mode = ControlMode.MotionMagic
            self.setPID(self.RotatePID)

    def setTargetAngle(self):
        target_angle = self.vision.getTargetAngle()
        # gyro_angle = self.gyro.getYaw()

        if target_angle:
            self.setRotate(target_angle)
            # self.positionDrive()
        else:
            self.vision_table.putNumber("Vision_angle", -999)

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
        # set motor values

        if self.control_mode and self.left_control and self.right_control:
            self.leftMaster.set(self.control_mode, self.left_control)
            self.rightMaster.set(self.control_mode, self.right_control)