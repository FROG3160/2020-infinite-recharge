'''Intake and shooter components'''
from ctre import (
    WPI_TalonFX,
    WPI_TalonSRX,
    FeedbackDevice,
    ControlMode,
    NeutralMode,
    TalonFXInvertType,
)
from .common import TalonPID, limit
from magicbot import tunable, feedback, StateMachine, state, timed_state

AZIMUTH_PID = TalonPID(0, f=0.4)
ELEVATION_PID = TalonPID(0, f=0.4)
FLYWHEEL_PID = TalonPID(0, f=0.05)

NEVEREST_CPR = 7 * 60  # motor ticks * gear reduction
FALCON_CPR = 2048

FLYWHEEL_MODE = ControlMode.Velocity
FLYWHEEL_MAX_VEL = 25000  # Falcons are maxing at 20k - 21k
FLYWHEEL_MAX_ACCEL = (
    FLYWHEEL_MAX_VEL / 50
)  # sampled 50 times a second makes this MAX ACCEL/sec
FLYWHEEL_MAX_DECEL = -FLYWHEEL_MAX_ACCEL
FLYWHEEL_INCREMENT = 250  # increment for manually adjusting speed
FLYWHEEL_SPEED_GOAL = 14000
FLYWHEEL_SPEED_LOB = 7000
FLYWHEEL_SPEED_TOLERANCE = 500

# TODO: change over to Position Mode, or MM?
ELEVATION_MODE = ControlMode.PercentOutput
ELEVATION_LOW = 0
ELEVATION_HIGH = 0

AZIMUTH_MODE = ControlMode.PercentOutput
AZIMUTH_CENTER = 2700
AZIMUTH_LIMIT_RIGHT = 5400
AZIMUTH_LIMIT_LEFT = 0


INTAKE_SPEED = 0.2
LOWER_CONVEYOR_SPEED = 0.2
UPPER_CONVEYOR_SPEED = 0.2


class Azimuth:
    _PID = AZIMUTH_PID
    azimuth_motor: WPI_TalonSRX
    azimuth_mode = AZIMUTH_MODE
    azimuth_command = tunable(0)

    def __init__(self):
        self.enabled = False

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    # read current encoder position
    @feedback
    def getPosition(self):
        return self.azimuth_motor.getSelectedSensorPosition(
            FeedbackDevice.IntegratedSensor
        )

    @feedback
    def getSpeed(self):
        return self.azimuth_motor.get()

    def setup(self):
        # this motor uses an attached Quad Encoder
        self.azimuth_motor.configSelectedFeedbackSensor(
            FeedbackDevice.QuadEncoder, 0, 0
        )
        self.azimuth_motor.setSensorPhase(False)
        self.azimuth_motor.setInverted(False)
        self.azimuth_motor.setNeutralMode(NeutralMode.Brake)
        self.azimuth_motor.setSelectedSensorPosition(AZIMUTH_CENTER, 0, 0)
        self.azimuth_motor.configForwardSoftLimitThreshold(AZIMUTH_LIMIT_RIGHT, 0, 0)
        self.azimuth_motor.configReverseSoftLimitThreshold(AZIMUTH_LIMIT_LEFT, 0, 0)
        self.azimuth_motor.configForwardSoftLimitEnable(True, 0, 0)
        self.azimuth_motor.configReverseSoftLimitEnable(True, 0, 0)

    def setPosition(self, value):
        # move to the given position
        self.azimuth_mode = ControlMode.Position
        self.azimuth_command = value

    def setSpeed(self, speed):
        self.azimuth_mode = ControlMode.PercentOutput
        self.azimuth_command = speed

    def execute(self):
        if self.enabled:
            self.azimuth_motor.set(self.azimuth_mode, self.azimuth_command)
        else:
            self.azimuth_motor.set(0)


class Conveyor:
    conveyor_motor: WPI_TalonSRX
    conveyor_command = tunable(0)

    def __init__(self):
        self.enabled = False

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    @feedback
    def get_speed(self):
        return self.conveyor_motor.get()

    def setup(self):
        self.conveyor_motor.setInverted(False)
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
    @feedback
    def get_position(self):
        return self.elevation_motor.getSelectedSensorPosition(
            FeedbackDevice.IntegratedSensor
        )

    @feedback
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
        self._velocityTarget = FLYWHEEL_SPEED_GOAL
        self._velocity = tunable(0)

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    @feedback
    def isReady(self):
        return (
            self._velocity - FLYWHEEL_SPEED_TOLERANCE
            <= self.getVelocity()
            <= self._velocity + FLYWHEEL_SPEED_TOLERANCE
        )

    # read current encoder velocity
    @feedback
    def getVelocity(self):
        return self.flywheel_motor.getSelectedSensorVelocity(
            FeedbackDevice.IntegratedSensor
        )

    @feedback
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

    def setVelocity(self, vel):
        # run Flywheel at the given velocity
        self._velocity = self._velocity + limit(
            (vel * FLYWHEEL_MAX_VEL - self._velocity),
            FLYWHEEL_MAX_DECEL,
            FLYWHEEL_MAX_ACCEL,
        )

    def setSpeed(self, speed):
        self._controlMode = ControlMode.PercentOutput
        self._velocity = speed

    def incrementSpeed(self):
        self._velocity += FLYWHEEL_INCREMENT

    def decrementSpeed(self):
        self._velocity -= FLYWHEEL_INCREMENT

    def toggleVelocityTarget(self):
        # True = 1, so if expression is true, second element
        # in list is selected.
        self._velocityTarget = [FLYWHEEL_SPEED_GOAL, FLYWHEEL_SPEED_LOB][
            self._velocityTarget == FLYWHEEL_SPEED_GOAL
        ]

    def execute(self):
        if self.enabled:
            if self._velocity != self._velocityTarget:
                self.setVelocity(self._velocityTarget)
            self.flywheel_motor.set(self._controlMode, self._velocity)
        else:
            self.flywheel_motor.set(0)


class Intake:

    intake_motor: WPI_TalonSRX
    intake_command = tunable(0)

    def __init__(self):
        self.enabled = False

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    @feedback
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

    @feedback
    def get_speed(self):
        return self.loader_motor.get()

    @feedback
    def get_lower_switch(self):
        return self.loader_motor.isRevLimitSwitchClosed()

    @feedback
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
        elif self.
        elif self.override:
            self.loader_motor.set(ControlMode.PercentOutput, self.loader_command)
        else:
            self.loader_motor.set(0)


class FROGShooter(StateMachine):
    azimuth: Azimuth
    elevation: Elevation
    flywheel: Flywheel
    loader: Loader
    conveyor: Conveyor
    intake: Intake


    def fire(self):
        # start the state machine with the @state that's "first=True"
        self.engage()

    @state(first=True)
    def prepareToFire(self):
        self.flywheel.enable()

        if self.flywheel.isReady():
            self.next_state_now('firing')

    @timed_state(duration=1, must_finish=True)
    def firing(self):
        self.flywheel.enable()