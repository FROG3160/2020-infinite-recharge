from magicbot import tunable, feedback
from ctre import (
    WPI_TalonSRX,
    ControlMode,
    NeutralMode,
    FeedbackDevice,
    TalonFXInvertType,
)
from wpilib.drive import DifferentialDrive
from math import copysign, atan2, sqrt, pi, degrees, radians
from .common import TalonPID, limit, remap
from .vision import FROGVision
from .shooter import Intake, Conveyor
from .sensors import FROGGyro

# from navx import AHRS
# from .sensors import FROGGyro

# from .sensors import FROGGyro

# from subsystems.vision import FROGVision
# from subsystems.common import PID

# drivetrain characteristics
DEADBAND = 0.1
FALCON_CPR = 4096
ENCODER_TICKS_PER_REV = 4096  # motor ticks * gear reduction
MAX_VELOCITY = 3000  # Falcons are maxing at 20k - 21k
CLOSEDLOOPRAMP = 1.5
WHEEL_DIAMETER = 6
# TICKS_PER_INCH = 1149  #
TICKS_PER_INCH = ENCODER_TICKS_PER_REV / (pi * WHEEL_DIAMETER)
TICKS_PER_ANGLE = 39270 / 180

# PIDs for drivetrain
# VelocityPID = TalonPID(slot=0, f=0.0482)
VelocityPID = TalonPID(slot=0, f=0.313)
# DiffDrivePID_calculated = TalonPID(slot=0, f=0.028998, p=0.031916)

# PositionPID = TalonPID(slot=0, f=0.003, p=0.05)
PositionPID = VelocityPID
# RotatePID = TalonPID(slot=0, f=0.320, p=0.76)
# TurnPID = TalonPID(p=0.035, d=0.10, i=0.001)
# PIDOutputLimit = 0.66


# TURNDEADBAND = 1.5  # in degrees

# Motor Control modes
VELOCITY_MODE = ControlMode.Velocity
POSITION_MODE = ControlMode.MotionMagic


# Motion Magic settings
MM_ACCELERATION = 1000  # 4000
MM_CRUISE_VELOCITY = 2000  # 8000


class FROGDrive(DifferentialDrive):

    # drive motors (channels defined in robot.py)
    leftMaster: WPI_TalonSRX
    rightMaster: WPI_TalonSRX
    leftSlave: WPI_TalonSRX
    rightSlave: WPI_TalonSRX

    vision: FROGVision
    gyro: FROGGyro
    intake: Intake
    conveyor: Conveyor

    control_mode = ControlMode.Position
    control_mode_str = tunable('None')

    commanded_left_vel = tunable(0)
    commanded_right_vel = tunable(0)
    commanded_left_pos = tunable(0)
    commanded_right_pos = tunable(0)

    def __init__(self):
        # init can't be used for setting up magic components because
        # they won't be available until after the contstructor has
        # finished.  Use the setup method instead.
        # This needs to be in place to prevent the __init__ of
        # DifferentialDrive from running until we have all components
        # available.
        pass

    def init_position_mode(self):
        self.set_PID(PositionPID)
        self.reset_commanded_pos()
        self.reset_encoders()
        self.set_control_mode(POSITION_MODE)

    def init_velocity_mode(self):
        self.set_PID(VelocityPID)
        self.reset_commanded_vel()
        self.set_control_mode(VELOCITY_MODE)

    def init_SDPID(self):
        # initializes attributes for tuning PID with SmartDashboard
        pass

    @feedback(key='LeftPosition')
    def get_current_left_pos(self):
        return self.get_encoder_position(self.leftMaster)

    @feedback(key="RightPosition")
    def get_current_right_pos(self):
        return self.get_encoder_position(self.rightMaster)

    @feedback(key='LeftVelocity')
    def get_current_left_vel(self):
        return self.get_encoder_velocity(self.leftMaster)

    @feedback(key='RightVelocity')
    def get_current_right_vel(self):
        return self.get_encoder_velocity(self.rightMaster)

    def set_control_mode(self, mode):
        self.control_mode = mode
        self.control_mode_str = mode.__repr__()

    def get_encoder_position(self, mc):
        return mc.getSelectedSensorPosition(0)

    def get_encoder_velocity(self, mc):
        return mc.getSelectedSensorVelocity(0)

    def reset_commanded_vel(self):
        self.commanded_left_vel = 0
        self.commanded_right_vel = 0

    def reset_commanded_pos(self):
        # setting the commanded position to equal the current encoder position
        # to keep it from causing the robot to move when initializing position
        # control.
        self.commanded_left_pos = 0
        self.commanded_right_pos = 0

    def reset_encoders(self):
        # resetting encoder positions to 0
        self.leftMaster.setSelectedSensorPosition(0, 0, 0)
        self.rightMaster.setSelectedSensorPosition(0, 0, 0)

    def setup(self):
        # called by MagicBot after object construction
        # use to configure objects that only need to be set up once

        # configure slaves
        self.leftSlave.follow(self.leftMaster)
        self.rightSlave.follow(self.rightMaster)
        # configure motor outputs
        self.leftMaster.setInverted(False)  # = false
        self.leftSlave.setInverted(False)
        self.rightMaster.setInverted(True)  # = true
        self.rightSlave.setInverted(True)
        # configure neutral mode
        self.leftMaster.setNeutralMode(NeutralMode.Coast)
        self.rightMaster.setNeutralMode(NeutralMode.Coast)
        self.leftMaster.configClosedloopRamp(CLOSEDLOOPRAMP)
        self.rightMaster.configClosedloopRamp(CLOSEDLOOPRAMP)
        # configure encoders
        for controller in [self.leftMaster, self.rightMaster]:
            controller.configSelectedFeedbackSensor(
                FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0
            )
        # Reverses the encoder direction so forward movement always
        # results in a positive increase in the encoder ticks.
        # Has no effect for Falcon 500 and their integrated sensors
        # self.leftMaster.setSensorPhase(True)

        # configure Motion Magic
        for motor in [self.leftMaster, self.rightMaster]:
            motor.configMotionAcceleration(MM_ACCELERATION, 0)
            motor.configMotionCruiseVelocity(MM_CRUISE_VELOCITY, 0)

        # init DifferentialDrive with left and right controllers
        super().__init__(self.leftMaster, self.rightMaster)
        self.setSafetyEnabled(False)

        # TODO: might want to move these to a chassis component that
        # has the drivetrain, gyro, and vision as objects.
        # self.gyro = FROGGyro(self.leftSlave)
        # self.vision = FROGVision()

    def set_PID(self, pid):
        for motor_control in (self.leftMaster, self.rightMaster):
            motor_control.config_kP(pid.slot, pid.p, 0)
            motor_control.config_kI(pid.slot, pid.i, 0)
            motor_control.config_kD(pid.slot, pid.d, 0)
            motor_control.config_kF(pid.slot, pid.f, 0)

    def set_velocity(self, speed, rotation):
        '''take speed and rotation values from joystick which
        are always in the -1 to 1 range.'''

        # only set left and right control values if we are in VELOCITY_MODE
        if self.control_mode == VELOCITY_MODE:
            # square the input and keep the sign of the original value
            speed = copysign(speed * speed, speed)
            rotation = copysign(rotation * rotation, rotation)

            # TODO: decide if it's redundant to keep these values in
            # attributes.  This is done to place the value on network
            # tables for testing purposes with the updateNT() method.
            self.control_speed = speed
            self.control_rotation = rotation

            # determine the left and right wheel speed from the given
            # speed and rotation controls
            maxInput = copysign(max(abs(speed), abs(rotation)), speed)
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

            self.commanded_left_vel = leftMotorSpeed * MAX_VELOCITY
            self.commanded_right_vel = rightMotorSpeed * MAX_VELOCITY

    def set_position(self, distance):
        # We only want to set the position if it's in POSITION_MODE
        if self.control_mode == POSITION_MODE and distance:
            self.commanded_left_pos = (
                distance * TICKS_PER_INCH
            ) + self.leftMaster.getSelectedSensorPosition(0)
            self.commanded_right_pos = (
                distance * TICKS_PER_INCH
            ) + self.rightMaster.getSelectedSensorPosition(0)

    def set_rotate(self, angle):
        if self.control_mode == POSITION_MODE and angle:
            ticks = angle * TICKS_PER_ANGLE

            self.commanded_left_pos = (
                ticks + self.leftMaster.getSelectedSensorPosition()
            )
            self.commanded_right_pos = (
                -ticks + self.rightMaster.getSelectedSensorPosition()
            )
            # self.set_control_mode(POSITION_MODE)
            # self.set_PID(PositionPID)

    def set_target_angle(self):
        target_angle = self.vision.getTargetAngle()
        # gyro_angle = self.gyro.getYaw()

        if target_angle:
            self.set_rotate(target_angle)
            # self.positionDrive()
        else:
            self.vision_table.putNumber("Vision_angle", -999)

    def toggle_control_mode(self):
        if self.control_mode == POSITION_MODE:
            self.init_velocity_mode()
        elif self.control_mode == VELOCITY_MODE:
            self.init_position_mode()

    def update_NT(self):
        """update network tables with drive telemetry"""
        self.get_current_left_pos()
        self.get_current_right_pos()
        self.get_current_left_vel()
        self.get_current_right_vel()

    def on_enable(self):
        # TODO: Move initialization on every enable (teleop/auto)
        # useful for resetting components to a safe/known state
        self.reset_encoders()  # this is currently run in teleop_init, too

    def driveToTarget(self):
        pc_x = self.vision.getPowerCellErrorX()
        pc_y = self.vision.getPowerCellErrorY()
        if pc_x and pc_y:
            # if the error is less that 60 pixel from the bottom,
            # we need to kick on the intake
            if pc_y < 60:
                self.intake.enable()
                self.conveyor.enable()
            else:
                self.intake.disable()
                self.conveyor.disable()
            rotate = remap(pc_x, -160, 160, -0.5, 0.5)
            speed = remap(pc_y, 0, 240, 0.3, 0.8)
            self.set_velocity(speed, rotate)
        else:
            self.intake.disable()
            self.conveyor.disable()

    def driveToHeading(self, x, y):
        # x should be right positive
        # y should be forward positive,
        print(x, y)
        if x < DEADBAND:
            x = 0
        if y < DEADBAND:
            y = 0

        # get the magnitude of the vector
        # formed by x and y for the speed:
        speed = sqrt(abs(x) ** 2 + abs(y) ** 2)

        # since the robot intake is facing "backward"
        # we are inverting X and Y for the driver
        # to move the intake towards balls.
        # TODO:  Look into inverting drivetrain motors
        # instead.
        desiredHeading = degrees(atan2(-x, -y))
        actualHeading = self.gyro.getHeading()
        # also because we "inverted" the heading for the driver
        # this calculation needs to be inverted also:
        turnDegrees = -((actualHeading - desiredHeading + 180) % 360 - 180)
        # if turnDegrees > 90 in either direction, we need to just give max
        # rotation and no forward/backward movement:

        if abs(turnDegrees) > 45:
            speed = 0
            # full rotation indirection indicated by degree heading
            rotation = copysign(0.4, turnDegrees)
        else:
            rotation = remap(turnDegrees, -45, 45, -0.5, 0.5)
        speed = remap(speed, -1, 1, -0.7, 0.7)
        self.set_velocity(speed, rotation)

    def execute(self):
        # set motor values
        if self.control_mode == VELOCITY_MODE:
            self.leftMaster.set(self.control_mode, self.commanded_left_vel)
            self.rightMaster.set(self.control_mode, self.commanded_right_vel)
        elif self.control_mode == POSITION_MODE:
            self.leftMaster.set(self.control_mode, self.commanded_left_pos)
            self.rightMaster.set(self.control_mode, self.commanded_right_pos)

        self.update_NT()
