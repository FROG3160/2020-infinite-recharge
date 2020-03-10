'''Intake and shooter components'''
from ctre import (
    WPI_TalonFX,
    WPI_TalonSRX,
    WPI_VictorSPX,
    FeedbackDevice,
    ControlMode,
    NeutralMode,
    TalonFXInvertType,
)
from .common import TalonPID, limit, remap
from magicbot import tunable, feedback, StateMachine, state, timed_state
from .vision import FROGVision, CAM_RES_H
from .sensors import FROGGyro, FROGdar
from math import copysign, log

# AZIMUTH_PID = TalonPID(0, p=32) #worked for Position Control
# AZIMUTH_PID = TalonPID(0, p=0, i=0, d=0, f=4.15)
AZIMUTH_PID = TalonPID(0, p=2.8, i=0, d=28, f=4.15)
ELEVATION_PID = TalonPID(0, p=32)
FLYWHEEL_PID = TalonPID(0, p=0.4, f=0.0515)

NEVEREST_CPR = 7 * 60  # motor ticks * gear reduction
FALCON_CPR = 2048

FLYWHEEL_MODE = ControlMode.Velocity
FLYWHEEL_MAX_VEL = 25000  # Falcons are maxing at 20k - 21k
FLYWHEEL_MAX_ACCEL = (
    FLYWHEEL_MAX_VEL / 50
)  # sampled 50 times a second makes this MAX ACCEL/sec
FLYWHEEL_MAX_DECEL = -FLYWHEEL_MAX_ACCEL
FLYWHEEL_INCREMENT = 500  # increment for manually adjusting speed
FLYWHEEL_VELOCITY_PORTAL = 14000
FLYWHEEL_VELOCITY_LOB = 7000
FLYWHEEL_VELOCITY_TOLERANCE = 500
FLYWHEEL_LOOP_RAMP = 0.5

# TODO: change over to Position Mode, or MM?
ELEVATION_MODE = ControlMode.PercentOutput
ELEVATION_LOW = 0
ELEVATION_HIGH = 4500


AZIMUTH_CENTER = 0
AZIMUTH_LIMIT_RIGHT = 2200
AZIMUTH_LIMIT_LEFT = -2200
AZIMUTH_MAX_SPEED = 300
AZIMUTH_COUNTS_PER_DEGREE = AZIMUTH_LIMIT_RIGHT / 90
AZIMUTH_AUTO_MOTOR_MODE = ControlMode.MotionMagic
AZIMUTH_MANUAL_MOTOR_MODE = ControlMode.PercentOutput
AZIMUTH_ENCODER_PER_H_FOV = 1180
AZIMUTH_ENCODER_PER_PIXEL = AZIMUTH_ENCODER_PER_H_FOV / CAM_RES_H
AZIMUTH_TARGET_PIXEL_TOLERANCE = 5

INTAKE_SPEED = 0.4
LOWER_CONVEYOR_SPEED = 0.5
UPPER_CONVEYOR_SPEED = 0.5


class Azimuth:
    _PID = AZIMUTH_PID
    azimuth_motor: WPI_TalonSRX
    gyro: FROGGyro
    vision: FROGVision

    def __init__(self):
        self.automatic = True
        self.motor_mode = AZIMUTH_AUTO_MOTOR_MODE
        self.azimuth_command = 0
        self.enabled = False
        # self.chassisHeading = None
        # self.targetCenterX = None

    def calcTurretSpeed(self, value):
        # use a natural log function to calculate the
        # motor percentage we need for the turret given
        # the error we are off-target
        return limit(0.277 * log(value) - 0.2339, 0, 1)

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    # read current encoder position
    @feedback(key='Position')
    def getPosition(self):
        return self.azimuth_motor.getSelectedSensorPosition(
            FeedbackDevice.IntegratedSensor
        )

    @feedback(key="Velocity")
    def getVelocity(self):
        return self.azimuth_motor.getSelectedSensorVelocity(
            FeedbackDevice.IntegratedSensor
        )

    @feedback(key="Commanded")
    def getCommanded(self):
        return self.azimuth_command

    def onTarget(self, x_error):
        return abs(x_error) <= AZIMUTH_TARGET_PIXEL_TOLERANCE

    def resetEncoder(self):
        self.azimuth_motor.setSelectedSensorPosition(AZIMUTH_CENTER, 0, 0)

    def setup(self):
        # this motor uses an attached Quad Encoder
        self.azimuth_motor.configSelectedFeedbackSensor(
            FeedbackDevice.QuadEncoder, 0, 0
        )
        self.azimuth_motor.setSensorPhase(False)
        self.azimuth_motor.setInverted(False)
        self.azimuth_motor.setNeutralMode(NeutralMode.Brake)
        # we start with the turret centered
        self.resetEncoder()
        # setting soft limits and enabling them
        self.azimuth_motor.configForwardSoftLimitThreshold(AZIMUTH_LIMIT_RIGHT, 0)
        self.azimuth_motor.configReverseSoftLimitThreshold(AZIMUTH_LIMIT_LEFT, 0)
        self.azimuth_motor.configForwardSoftLimitEnable(True, 0)
        self.azimuth_motor.configReverseSoftLimitEnable(True, 0)
        # set PID
        self._PID.configTalon(self.azimuth_motor)

    def setPosition(self, value):
        # move to the given position
        self.motor_mode = ControlMode.Position
        self.azimuth_command = value

    def setVelocity(self, rotation):
        self.motor_mode = ControlMode.Velocity
        self.azimuth_command = rotation * AZIMUTH_MAX_SPEED

    def setSpeed(self, speed):
        self.motor_mode = AZIMUTH_MANUAL_MOTOR_MODE
        # if speed != 0:
        # speed = copysign(limit(abs(speed), 0.20, 1), speed)
        # self.azimuth_command = copysign(speed * speed, speed)
        self.azimuth_command = speed

    def setManual(self):
        self.automatic = False

    def setAutomatic(self):
        self.automatic = True

    def execute(self):
        if self.enabled:
            #
            if self.automatic:
                # check if the robot is pointed the right direction
                if -90 <= self.gyro.getHeading() <= 90:
                    # if we already see the target, move to it

                    if (x_error := self.vision.getPowerPortErrorX()) :
                        if not self.onTarget(x_error):
                            # self.setVelocity(remap(x_error, -160, 160, -1, 1))
                            if x_error > 0:
                                # self.setSpeed(remap(x_error, 0, 160, 0.38, 1))
                                self.setSpeed(self.calcTurretSpeed(x_error))
                            else:
                                self.setSpeed(
                                    copysign(
                                        self.calcTurretSpeed(abs(x_error)), x_error
                                    )
                                )  # self.setPosition(x_error * AZIMUTH_ENCODER_PER_PIXEL)
                        else:
                            self.setSpeed(0)
                    # otherwise, move turret in direction of the target
                    else:
                        self.setPosition(
                            self.gyro.getHeading() * -AZIMUTH_COUNTS_PER_DEGREE
                        )
                # center the turret until we can move it to the target
                else:
                    self.setPosition(AZIMUTH_CENTER)

            self.azimuth_motor.set(self.motor_mode, self.azimuth_command)
        else:
            self.azimuth_motor.set(0)


class Conveyor:
    conveyor_motor: WPI_VictorSPX
    conveyor_command = tunable(LOWER_CONVEYOR_SPEED)

    def __init__(self):
        self.enabled = False

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    @feedback(key='Percent')
    def get_speed(self):
        return self.conveyor_motor.get()

    def setup(self):
        self.conveyor_motor.setInverted(True)
        self.conveyor_motor.setNeutralMode(NeutralMode.Brake)

    def set_speed(self, speed):
        self.conveyor_command = speed

    def execute(self):
        if self.enabled:
            self.conveyor_motor.set(ControlMode.PercentOutput, self.conveyor_command)
        else:
            self.conveyor_motor.set(0)


class Elevation:

    _PID = ELEVATION_PID
    elevation_motor: WPI_TalonSRX
    elevation_mode = ELEVATION_MODE
    elevation_command = tunable(0)

    def __init__(self):
        self.enabled = False

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    # read current encoder position
    @feedback(key='Position')
    def get_position(self):
        return self.elevation_motor.getSelectedSensorPosition(
            FeedbackDevice.IntegratedSensor
        )

    def onTarget(self):
        return True

    @feedback(key='Percent')
    def get_speed(self):
        return self.elevation_motor.get()

    def setup(self):
        # this motor uses an attached Quad Encoder
        self.elevation_motor.configSelectedFeedbackSensor(
            FeedbackDevice.QuadEncoder, 0, 0
        )
        self.elevation_motor.setSensorPhase(False)
        self.elevation_motor.setInverted(False)
        self.elevation_motor.setNeutralMode(NeutralMode.Brake)
        # high angle.  We start with the linear screw all the way down
        self.elevation_motor.setSelectedSensorPosition(ELEVATION_HIGH, 0, 0)
        # setting soft limits and enabling TalonFXInvertType
        self.elevation_motor.configForwardSoftLimitThreshold(ELEVATION_HIGH - 100, 0)
        self.elevation_motor.configReverseSoftLimitThreshold(ELEVATION_LOW + 100, 0)
        self.elevation_motor.configForwardSoftLimitEnable(True, 0)
        self.elevation_motor.configReverseSoftLimitEnable(True, 0)
        # setting the PID
        self._PID.configTalon(self.elevation_motor)

    def set_position(self, value):
        # move to the given position
        self.elevation_mode = ControlMode.Position
        self.elevation_command = value

    def set_speed(self, speed):
        self.elevation_mode = ControlMode.PercentOutput
        self.elevation_command = speed

    def execute(self):
        if self.enabled:
            self.elevation_motor.set(self.elevation_mode, self.elevation_command)
        else:
            self.elevation_motor.set(0)


class Flywheel:
    _PID = FLYWHEEL_PID
    flywheel_motor: WPI_TalonFX

    def __init__(self):
        self.enabled = False
        self._controlMode = FLYWHEEL_MODE
        # defines the two velocities we'll use for our FeedbackDevice
        self._velocityModes = ['PORTAL', 'LOB']
        # the initial velocity we'll use.
        self._velocityMode = 'PORTAL'

        self._velocity = 0

        # sets the values for the two defined velocities
        self._velocities = {}
        self._velocities['LOB'] = FLYWHEEL_VELOCITY_LOB
        self._velocities['PORTAL'] = FLYWHEEL_VELOCITY_PORTAL

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def isReady(self):
        return (
            self._velocity - FLYWHEEL_VELOCITY_TOLERANCE
            <= self.getVelocity()
            <= self._velocity + FLYWHEEL_VELOCITY_TOLERANCE
        )

    # read current encoder velocity
    @feedback(key='Velocity')
    def getVelocity(self):
        return self.flywheel_motor.getSelectedSensorVelocity(
            FeedbackDevice.IntegratedSensor
        )

    # read current controller percent vBus
    @feedback(key='PercentVBus')
    def getSpeed(self):
        return self.flywheel_motor.get()

    def setup(self):
        # Falcon500 motors use the integrated sensor
        self.flywheel_motor.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor, 0, 0
        )
        self.flywheel_motor.setSensorPhase(False)
        self.flywheel_motor.setInverted(
            TalonFXInvertType.Clockwise
        )  # = setInverted(True)
        self.flywheel_motor.setNeutralMode(NeutralMode.Coast)
        self._PID.configTalon(self.flywheel_motor)

        self.flywheel_motor.configClosedloopRamp(FLYWHEEL_LOOP_RAMP)

    def accelerate(self, vel):
        # accelerate Flywheel to the given velocity
        self._velocity = self._velocity + limit(
            (vel * FLYWHEEL_MAX_VEL - self._velocity),
            FLYWHEEL_MAX_DECEL,
            FLYWHEEL_MAX_ACCEL,
        )

    def setSpeed(self, speed):
        self._controlMode = ControlMode.PercentOutput
        self._velocity = speed

    def setVelocity(self, velocity):
        self._controlMode = ControlMode.Velocity
        self._velocity = velocity

    # adjust current velocity by defined increment
    def incrementSpeed(self):
        self._velocities[self._velocityMode] += FLYWHEEL_INCREMENT
        self.setVelocity(self._velocities[self._velocityMode])

    def decrementSpeed(self):
        self._velocities[self._velocityMode] -= FLYWHEEL_INCREMENT
        self.setVelocity(self._velocities[self._velocityMode])

    # switch between defined velocities
    def toggleVelocityMode(self):
        # True = 1, so if expression is true, second element
        # in list is selected.
        self._velocityMode = self._velocityModes[self._velocityMode == 'PORTAL']
        self._velocityTarget = self._velocities[self._velocityMode]

    def execute(self):
        if self.enabled:
            # if self._velocity != self._velocities[self._velocityMode]:
            # self.accelerate(self._velocities[self._velocityMode])
            self.flywheel_motor.set(self._controlMode, self._velocity)
        else:
            self.flywheel_motor.set(0)


class Intake:

    intake_motor: WPI_VictorSPX
    intake_command = tunable(INTAKE_SPEED)

    def __init__(self):
        self.enabled = False

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    @feedback(key='Percent')
    def get_speed(self):
        return self.intake_motor.get()

    def setup(self):
        self.intake_motor.setInverted(False)
        self.intake_motor.setNeutralMode(NeutralMode.Brake)

    def set_speed(self, speed):
        self.intake_command = speed

    def execute(self):
        if self.enabled:
            self.intake_motor.set(ControlMode.PercentOutput, self.intake_command)
        else:
            self.intake_motor.set(0)


class Loader:
    loader_motor: WPI_TalonSRX
    loader_command = tunable(0)

    def __init__(self):
        self.enabled = False
        self.override = False

    def disable(self):
        self.enabled = False
        self.override = False

    def enable(self):
        self.enabled = True

    def override(self):
        ''' Enables an override of the limit switch checks to allow
        the motor to run even without any balls in the system'''
        self.override = True

    @feedback(key='Percent')
    def get_speed(self):
        return self.loader_motor.get()

    @feedback(key='LowerSwitchEnabled')
    def get_lower_switch_str(self):
        return str(self.get_lower_switch())

    def get_lower_switch(self):
        return self.loader_motor.isRevLimitSwitchClosed()

    @feedback
    def get_upper_switch_str(self):
        return str(self.get_upper_switch())

    def get_upper_switch(self):
        return self.loader_motor.isFwdLimitSwitchClosed()

    def setup(self):
        self.loader_motor.setInverted(False)
        self.loader_motor.setNeutralMode(NeutralMode.Brake)

    def set_speed(self, speed):
        self.loader_command = speed

    def execute(self):
        if self.enabled and self.get_lower_switch() and not self.get_upper_switch():
            self.loader_motor.set(ControlMode.PercentOutput, self.loader_command)
        elif self.override:
            self.loader_motor.set(ControlMode.PercentOutput, self.loader_command)
        else:
            self.loader_motor.set(0)


class FROGShooter(StateMachine):
    azimuth: Azimuth
    elevation: Elevation
    flywheel: Flywheel

    # loader: Loader
    conveyor: Conveyor
    intake: Intake

    def __init__(self):

        super().__init__()
        self.chassis_heading = 0
        # self.vision = FROGVision()
        # self.gyro = FROGGyro()

    def fire(self):
        # start the state machine with the @state that's "first=True"
        self.engage()

    def onTarget(self):
        # check azimuth and elevation are on target.
        return self.azimuth.onTarget() and self.elevation.onTarget()

    @state(first=True)
    def targeting(self):
        self.azimuth.enable()
        # self.elevation.enable()
        # if self.onTarget():
        # self.next_state('prepareToFire')

    @state()
    def prepareToFire(self):
        if self.onTarget():
            self.flywheel.enable()
            if self.flywheel.isReady():
                pass
        else:
            self.flywheel.disable()
            self.next_state('targeting')

    @timed_state(duration=1, must_finish=True)
    def firing(self):
        # self.flywheel.enable()
        pass
