"""Intake and shooter components"""
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
from .sensors import FROGGyro, FROGdar, LimitSwitch
from math import copysign, log
from .driverstation import FROGXboxGunner, FROGXboxDriver, LEFTHAND, RIGHTHAND


# AZIMUTH_PID = TalonPID(0, p=1.3, d=3.1, f=3.1)
# AZIMUTH_PID = TalonPID(0, p=8.4, i=0.09, d=0, f=0)  #4.87
AZIMUTH_PID = TalonPID(0, p=0, i=0, d=0, f=4.4)
ELEVATION_PID = TalonPID(0, p=32)
FLYWHEEL_PID = TalonPID(0, p=0.4, f=0.0515)
LOADER_PID = TalonPID(0, p=0.05, i=0, d=0, f=0.045)

NEVEREST_CPR = 7 * 60  # motor ticks (counts) per rev. * gear reduction
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

# TODO: get new azimuth position limits and speed max
AZIMUTH_CENTER = 0
AZIMUTH_LIMIT_RIGHT = 1680
AZIMUTH_LIMIT_LEFT = -1680
AZIMUTH_MAX_SPEED = 210
# TODO: figure out minimum speed
AZIMUTH_MIN_SPEED = 25
AZIMUTH_COUNTS_PER_DEGREE = AZIMUTH_LIMIT_RIGHT / 90
AZIMUTH_AUTO_MOTOR_MODE = ControlMode.MotionMagic
AZIMUTH_MANUAL_MOTOR_MODE = ControlMode.Velocity
AZIMUTH_ENCODER_PER_H_FOV = 1180
AZIMUTH_ENCODER_PER_PIXEL = AZIMUTH_ENCODER_PER_H_FOV / CAM_RES_H
AZIMUTH_TARGET_PIXEL_TOLERANCE = 5

INTAKE_SPEED = 0.45
LOWER_CONVEYOR_SPEED = 0.25
LOADER_SPEED = 0.50


class Azimuth:
    _PID = AZIMUTH_PID
    azimuth_motor: WPI_TalonSRX

    def __init__(self):
        self.azimuth_mode = AZIMUTH_AUTO_MOTOR_MODE
        self.azimuth_command = 0
        self.enabled = False

    def setup(self):
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
        self.chassisHeading = None
        self.targetCenterX = None

    def disable(self):
        self.enabled = False

    def enable(self):
        self.stop()
        self.enabled = True

    # read current encoder position
    @feedback(key="AzimuthPosition")
    def getPosition(self):
        return self.azimuth_motor.getSelectedSensorPosition(
            FeedbackDevice.IntegratedSensor
        )

    @feedback(key="AzimuthVelocity")
    def getVelocity(self):
        return self.azimuth_motor.getSelectedSensorVelocity(
            FeedbackDevice.IntegratedSensor
        )

    @feedback(key="AzimuthCommanded")
    def getCommanded(self):
        return self.azimuth_command

    def resetEncoder(self):
        self.azimuth_motor.setSelectedSensorPosition(AZIMUTH_CENTER, 0, 0)

    def setPosition(self, value):
        # move to the given position
        self.azimuth_mode = ControlMode.Position
        self.azimuth_command = value

    def setVelocity(self, speed):
        self.azimuth_mode = ControlMode.Velocity
        self.azimuth_command = speed * AZIMUTH_MAX_SPEED

    def stop(self):
        self.setVelocity(0)

    def execute(self):
        if self.enabled:
            self.azimuth_motor.set(self.azimuth_mode, self.azimuth_command)
        else:
            self.stop()


class Elevation:
    _PID = ELEVATION_PID
    elevation_motor: WPI_TalonSRX

    def __init__(self):
        self.disable()
        # self.setup()
        self.elevation_mode = ELEVATION_MODE
        self.elevation_command = tunable(0)

    def enable(self):
        self.stop()
        self.enabled = True

    def disable(self):
        self.enabled = False

    # read current encoder position
    @feedback(key="position")
    def get_position(self):
        return self.elevation_motor.getSelectedSensorPosition(
            FeedbackDevice.IntegratedSensor
        )

    def get_speed(self):
        return self.elevation_motor.get()

    def setup(self):
        # this method exists to re-initialze the configuration
        # of the motor controller if needed.

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

    def setPosition(self, value):
        # move to the given position
        self.elevation_mode = ControlMode.Position
        self.elevation_command = value

    def setSpeed(self, speed):
        self.elevation_mode = ControlMode.PercentOutput
        self.elevation_command = speed

    def stop(self):
        self.elevation_motor.set(0)

    def execute(self):
        if self.enabled:
            self.elevation_motor.set(self.elevation_mode, self.elevation_command)
        else:
            self.stop()


class Flywheel:
    _PID = FLYWHEEL_PID
    flywheel_motor: WPI_TalonFX

    def __init__(self):
        self.enabled = False
        self._controlMode = FLYWHEEL_MODE
        # defines the two velocities we'll use for our FeedbackDevice
        self._velocityModes = ["PORTAL", "LOB"]
        # the initial velocity we'll use.
        self._velocityMode = "PORTAL"

        # sets the values for the two defined velocities
        self._velocities = {}
        self._velocities["LOB"] = FLYWHEEL_VELOCITY_LOB
        self._velocities["PORTAL"] = FLYWHEEL_VELOCITY_PORTAL
        self._velocity = self._velocities["PORTAL"]

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
    @feedback(key="velocity")
    def getVelocity(self):
        return self.flywheel_motor.getSelectedSensorVelocity(
            FeedbackDevice.IntegratedSensor
        )

    @feedback(key="commanded")
    def getCommandedVelocity(self):
        return self._velocity

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
        # use closed loop ramp to accelerate smoothly
        self.flywheel_motor.configClosedloopRamp(FLYWHEEL_LOOP_RAMP)

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
        self._velocityMode = self._velocityModes[self._velocityMode == "PORTAL"]
        self.setVelocity(self._velocities[self._velocityMode])

    def execute(self):
        if self.enabled:
            self.flywheel_motor.set(self._controlMode, self._velocity)
        else:
            self.flywheel_motor.set(0)


class Intake:

    intake_motor: WPI_VictorSPX
    intake_command = INTAKE_SPEED

    def __init__(self):
        self.enabled = False

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    @feedback(key="Percent")
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
    _PID = LOADER_PID
    loader_motor: WPI_TalonFX
    loader_command = LOADER_SPEED

    def __init__(self):
        # self.loader_motor.setInverted(False)
        # self.loader_motor.setNeutralMode(NeutralMode.Brake)
        self.enabled = False

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    @feedback(key="Speed")
    def get_speed(self):
        return self.loader_motor.get()

    # turn boolean into string to display on dashboard
    @feedback(key="LowerSwitchEnabled")
    def get_lower_switch_str(self):
        return str(self.get_lower_switch())

    def get_lower_switch(self):
        return self.loader_motor.isRevLimitSwitchClosed()

    # turn boolean into string to display on dashboard
    @feedback(key="UpperSwitchEnabled")
    def get_upper_switch_str(self):
        return str(self.get_upper_switch())

    def get_upper_switch(self):
        return self.loader_motor.isFwdLimitSwitchClosed()

    def set_speed(self, speed):
        self.loader_command = speed

    def execute(self):
        if self.enabled:
            self.loader_motor.set(ControlMode.PercentOutput, self.loader_command)
        else:
            self.loader_motor.set(0)


# low-level components
# azimuth, elevation, flywheel, intake, loader

# azimuth, elevation = targeting
# flywheel, intake, loader = shooting/firing


class Targeting:
    azimuth: Azimuth
    elevation: Elevation
    gyro: FROGGyro
    vision: FROGVision
    lidar: FROGdar

    def __init__(self):
        self.automatic = False

    def enable(self):
        self.automatic = True
        self.azimuth.enable()
        self.elevation.enable()

    def disable(self):
        self.automatic = False
        self.azimuth.disable()
        self.elevation.disable()

    def calcElevation(self):
        # get lidar distance in inches and calculate
        # position for elevation
        if (inches := self.lidar.getDistance()) :
            return 25199 * inches ** -0.476
        else:
            return None

    # move azimuth
    def moveToTarget(self):
        # check if the robot is pointed the right direction
        if -90 <= self.gyro.getHeading() <= 90:
            # if we get a value for x_error, we see the target
            # if we already see the target, turn towards it.
            # positive x_error = target to the right of center
            if (x_error := self.vision.getPowerPortErrorX()) :
                # if we are within tolerance, don't move
                if abs(x_error) > AZIMUTH_TARGET_PIXEL_TOLERANCE:
                    if x_error > 0:
                        # turn to right
                        self.azimuth.setVelocity(
                            remap(x_error, 0, CAM_RES_H / 2, 0.08, 1)
                        )
                    else:
                        # turn to left
                        self.azimuth.setVelocity(
                            remap(x_error, -CAM_RES_H / 2, 0, -1, -0.08)
                        )
                else:
                    self.azimuth.setVelocity(0)
            # We don't see the target, so move turret
            # in direction of the target
            else:
                # move the turret to point back downfield in the
                # direction of the target
                self.setPosition(self.gyro.getHeading() * -AZIMUTH_COUNTS_PER_DEGREE)
        # we are not pointed a direction that will allow
        # us to rotate toward the target.
        # center the turret until we can move it to the target
        else:
            self.azimuth.setPosition(AZIMUTH_CENTER)

    # move elevation
    def moveToRange(self):
        if (new_position := self.calcElevation()) :
            self.elevation.setPosition(new_position)
        else:
            self.elevation.stop()

    # get gyro

    # get vision

    # get lidar

    def execute(self):
        if self.automatic:
            self.moveToTarget()
            self.moveToRange()


class FROGShooter(StateMachine):
    intake: Intake
    loader: Loader
    azimuth: Azimuth
    elevation: Elevation
    flywheel: Flywheel
    drive_stick: FROGXboxDriver
    gunner_stick: FROGXboxGunner

    def __init__(self):

        super().__init__()

    def fire(self):
        # start the state machine with the @state that's "first=True"
        self.engage()

    def onTarget(self):
        # check azimuth and elevation are on target.
        return self.azimuth.onTarget() and self.elevation.onTarget()

    @state(first=True)
    def checkMagazine(self):
        pass

    @state()
    def readyToLoad(self):
        if self.drive_stick.getBumperPressed(LEFTHAND):
            self.next_state("runIntake")

    def runIntake(self, duration=1):
        self.intake.enable()
        self.conveyor.enable()

    @state()
    def prepareToFire(self):
        if self.onTarget():
            self.flywheel.enable()
            if self.flywheel.isReady():
                pass
        else:
            self.flywheel.disable()
            self.next_state("targeting")

    @timed_state(duration=1, must_finish=True)
    def firing(self):
        # self.flywheel.enable()
        pass
