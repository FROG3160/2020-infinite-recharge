'''Intake and shooter components'''
from ctre import (
    WPI_TalonFX,
    WPI_TalonSRX,
    FeedbackDevice,
    ControlMode,
    NeutralMode,
    TalonFXInvertType,
)
from .common import PID, limit
from magicbot import tunable, feedback

AZIMUTH_PID = PID(0, f=0.4)
ELEVATION_PID = PID(0, f=0.4)
FLYWHEEL_PID = PID(0, f=0.4)
NEVEREST_CPR = 7 * 60  # motor ticks * gear reduction
FALCON_CPR = 2048
FLYWHEEL_MAX_VEL = 25000  # Falcons are maxing at 20k - 21k
FLYWHEEL_MAX_ACCEL = (
    FLYWHEEL_MAX_VEL / 50
)  # sampled 50 times a second makes this MAX ACCEL/sec
FLYWHEEL_MAX_DECEL = FLYWHEEL_MAX_ACCEL
FLYWHEEL_INCREMENT = 0.05

# TODO: change these to velocity Mode
FLYWHEEL_MODE = ControlMode.PercentOutput
ELEVATION_MODE = ControlMode.PercentOutput
AZIMUTH_MODE = ControlMode.PercentOutput
FLYWHEEL_SPEED_GOAL = 0.7
FLYWHEEL_SPEED_LOB = 0.4

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
    def get_position(self):
        return self.azimuth_motor.getSelectedSensorPosition(
            FeedbackDevice.IntegratedSensor
        )

    @feedback
    def get_speed(self):
        return self.azimuth_motor.get()

    def setup(self):
        # this motor uses an attached Quad Encoder
        self.azimuth_motor.configSelectedFeedbackSensor(
            FeedbackDevice.QuadEncoder, 0, 0
        )
        self.azimuth_motor.setSensorPhase(False)
        self.azimuth_motor.setInverted(False)
        self.azimuth_motor.setNeutralMode(NeutralMode.Brake)

    def set_position(self, value):
        # move to the given position
        self.azimuth_mode = ControlMode.Position
        self.azimuth_command = value

    def set_speed(self, speed):
        self.azimuth_mode = ControlMode.PercentOutput
        self.azimuth_command = speed

    def execute(self):
        if self.enabled:
            self.azimuth_motor.set(self.azimuth_mode, self.azimuth_command)
        else:
            self.azimuth_motor.set(0)


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
        elif self.override:
            self.loader_motor.set(ControlMode.PercentOutput, self.loader_command)
        else:
            self.loader_motor.set(0)


class Flywheel:
    _PID = FLYWHEEL_PID
    flywheel_motor: WPI_TalonFX
    flywheel_mode = FLYWHEEL_MODE
    flywheel_command = tunable(0)

    def __init__(self):
        self.enabled = False

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    # read current encoder velocity
    @feedback
    def get_velocity(self):
        return self.flywheel_motor.getSelectedSensorVelocity(
            FeedbackDevice.IntegratedSensor
        )

    @feedback
    def get_speed(self):
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

    def set_velocity(self, vel):
        # run Flywheel at the given velocity
        self.flywheel_mode = ControlMode.Velocity
        self.flywheel_command = self.flywheel_command + limit(
            (vel * FLYWHEEL_MAX_VEL - self.flywheel_command),
            -FLYWHEEL_MAX_ACCEL,
            FLYWHEEL_MAX_DECEL,
        )

    def set_speed(self, speed):
        self.flywheel_mode = ControlMode.PercentOutput
        self.flywheel_command = speed

    def incrementFlywheelPercent(self):
        self.flywheel_command += FLYWHEEL_INCREMENT

    def decrementFlywheelPercent(self):
        self.flywheel_command -= FLYWHEEL_INCREMENT

    def toggle_firing_speed(self):
        # True = 1, so if expression is true, second element
        # in list is selected.
        self.flywheel_command = [FLYWHEEL_SPEED_GOAL, FLYWHEEL_SPEED_LOB][
            self.flywheel_command == FLYWHEEL_SPEED_GOAL
        ]

    def execute(self):
        if self.enabled:
            self.flywheel_motor.set(self.flywheel_mode, self.flywheel_command)
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


class FROGShooter:
    azimuth: Azimuth
    elevation: Elevation
    flywheel: Flywheel
    loader: Loader
    conveyor: Conveyor
    intake: Intake

    def execute(self):
        pass
