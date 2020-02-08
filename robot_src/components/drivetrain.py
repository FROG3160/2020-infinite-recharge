from magicbot import tunable
from ctre import WPI_TalonFX, ControlMode, NeutralMode, FeedbackDevice, PigeonIMU
from wpilib.drive import DifferentialDrive
import math
from networktables import NetworkTables
from .common import PID, limit

# from subsystems.vision import FROGVision
# from subsystems.common import PID

# drivetrain characteristics.  Values are changed from last year.
# The encoders are now on the motor shaft instead of the drive shaft
# and the encoder has half the resolution of the ones from last year.
ENCODER_TICKS_PER_REV = 2048 * 10.71  # motor ticks * gear reduction
MAX_VELOCITY = 12000  # Falcons are maxing at 20k - 21k
MAX_ACCEL = MAX_VELOCITY / 50  # sampled 50 times a second makes this MAX ACCEL/sec
MAX_DECEL = MAX_ACCEL
WHEEL_DIAMETER = 6
TICKS_PER_INCH = 1149  # ENCODER_TICKS_PER_REV / (math.pi * WHEEL_DIAMETER)
TICKS_PER_ANGLE = 40000 / 180

# PIDs for drivetrain
VelocityPID = PID(slot=0, f=0.0482)
PositionPID = PID(slot=0, f=0.003, p=0.0)
RotatePID = PID(slot=0, f=0.320, p=0.76)
TurnPID = PID(p=0.035, d=0.10, i=0.001)
PIDOutputLimit = 0.66

TURNDEADBAND = 1.5  # in degrees

# Motor Control modes
VELOCITY_MODE = ControlMode.Velocity
POSITION_MODE = ControlMode.MotionMagic

# Motion Magic settings
MM_ACCELERATION = 4000
MM_CRUISE_VELOCITY = 8000


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

    # drive motors (channels defined in robot.py)
    leftMaster: WPI_TalonFX
    rightMaster: WPI_TalonFX
    leftSlave: WPI_TalonFX
    rightSlave: WPI_TalonFX

    def __init__(self):
        # init can't be used for setting up magic components because
        # they won't be available until after the contstructor has
        # finished.  Use the setup method instead.
        # This needs to be in place to prevent the __init__ of
        # DifferentialDrive from running until we have all components
        # available.
        pass

    def config_encoders(self):
        for controller in [self.leftMaster, self.rightMaster]:
            controller.configSelectedFeedbackSensor(
                FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0
            )

    def config_motion_magic(self):
        for motor in [self.leftMaster, self.rightMaster]:
            motor.configMotionAcceleration(MM_ACCELERATION, 0)
            motor.configMotionCruiseVelocity(MM_CRUISE_VELOCITY, 0)

    def init_nt(self):
        NetworkTables.initialize()
        self.drive_encoders = NetworkTables.getTable("drive_encoder_values")
        self.drive_command = NetworkTables.getTable("drive_commands")
        self.vision_table = NetworkTables.getTable("Vision")

    def init_position_mode(self):
        self.setPID(PositionPID)
        self.reset_control_values()
        self.control_mode = POSITION_MODE

    def init_velocity_mode(self):
        self.setPID(VelocityPID)
        self.left_control = 0
        self.right_control = 0
        self.control_mode = VELOCITY_MODE

    def init_SDPID(self):
        # initializes attributes for tuning PID with SmartDashboard
        pass

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
        # called by MagicBot after object construction
        self.set_motor_slaves()
        self.set_motor_output()
        self.set_motor_neutral_mode()

        self.config_encoders()
        self.set_encoder_direction()
        self.reset_encoder_values()

        self.config_motion_magic()

        self.init_velocity_mode()
        self.init_nt()

        # init DifferentialDrive with left and right controllers
        DifferentialDrive.__init__(self.leftMaster, self.rightMaster)
        self.setSafetyEnabled(False)

        # TODO: might want to move these to a chassis component that
        # has the drivetrain, gyro, and vision as objects.
        # self.gyro = FROGGyro(self.leftSlave)
        # self.vision = FROGVision()

    def setPID(self, pid):
        for motor_control in (self.leftMaster, self.rightMaster):
            motor_control.config_kP(pid.slot, pid.p, 0)
            motor_control.config_kI(pid.slot, pid.i, 0)
            motor_control.config_kD(pid.slot, pid.d, 0)
            motor_control.config_kF(pid.slot, pid.f, 0)

    def setVelocity(self, speed, rotation):
        '''take speed and rotation values from joystick which
        are always in the -1 to 1 range.'''

        # square the input and keep the sign of the original value
        speed = math.copysign(speed * speed, speed)
        rotation = math.copysign(rotation * rotation, rotation)

        # TODO: decide if it's redundant to keep these values in
        # attributes.  This is done to place the value on network
        # tables for testing purposes with the updateNT() method.
        self.control_speed = speed
        self.control_rotation = rotation

        # determine the left and right wheel speed from the given
        # speed and rotation controls
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

        # adjust the motor velocity by the requested speed,
        # limited by the max velocity allowed.
        self.left_control = self.left_control + limit(
            (leftMotorSpeed * MAX_VELOCITY - self.left_control),
            -MAX_ACCEL,
            MAX_ACCEL)
        self.right_control = self.right_control + limit(
            (rightMotorSpeed * MAX_VELOCITY - self.right_control),
            -MAX_ACCEL,
            MAX_ACCEL)

    def setPosition(self, distance):

        if distance and not self.left_control and not self.right_control:
            self.left_control = (
                distance * TICKS_PER_INCH
            ) + self.leftMaster.getSelectedSensorPosition(0)
            self.right_control = (
                distance * TICKS_PER_INCH
            ) + self.rightMaster.getSelectedSensorPosition(0)

    def setRotate(self, angle):
        if angle:
            ticks = angle * self.TICKS_PER_ANGLE

            self.left_control = ticks + self.leftMaster.getSelectedSensorPosition()
            self.right_control = -ticks + self.rightMaster.getSelectedSensorPosition()
            self.control_mode = POSITION_MODE
            self.setPID(RotatePID)

    def setTargetAngle(self):
        target_angle = self.vision.getTargetAngle()
        # gyro_angle = self.gyro.getYaw()

        if target_angle:
            self.setRotate(target_angle)
            # self.positionDrive()
        else:
            self.vision_table.putNumber("Vision_angle", -999)

    def toggleControlMode(self):
        if self.control_mode == POSITION_MODE:
            self.init_velocity_mode()
        elif self.control_mode == VELOCITY_MODE:
            self.init_position_mode()

    def updateNT(self):
        """update network tables with drive telemetry"""

        self.drive_encoders.putNumber(
            "left_pos", self.leftMaster.getSelectedSensorPosition(FeedbackDevice.IntegratedSensor)
        )
        self.drive_encoders.putNumber(
            "right_pos", self.rightMaster.getSelectedSensorPosition(FeedbackDevice.IntegratedSensor)
        )
        self.drive_encoders.putNumber(
            "left_vel", self.leftMaster.getSelectedSensorVelocity(FeedbackDevice.IntegratedSensor)
        )
        self.drive_encoders.putNumber(
            "right_vel", self.rightMaster.getSelectedSensorVelocity(FeedbackDevice.IntegratedSensor)
        )
        if self.control_speed:
            self.drive_command.putNumber("commanded speed", self.control_speed)
        if self.control_rotation:
            self.drive_command.putNumber("commanded rotation", self.control_rotation)
        if self.left_control:
            self.drive_command.putNumber("left control", self.left_control)
        if self.right_control:
            self.drive_command.putNumber("right control", self.right_control)

    def execute(self):
        # set motor values

        if self.control_mode and self.left_control and self.right_control:
            self.leftMaster.set(self.control_mode, self.left_control)
            self.rightMaster.set(self.control_mode, self.right_control)
        self.updateNT()
