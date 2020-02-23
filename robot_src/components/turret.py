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
from networktables import NetworkTables
from magicbot import tunable, feedback


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


class Shooter:

    flywheelPID = PID(0, p=0)

    azimuth: WPI_TalonSRX
    elevation: WPI_TalonSRX
    flywheel: WPI_TalonFX

    azimuth_mode = AZIMUTH_MODE
    azimuth_speed = tunable(0)

    elevation_mode = ELEVATION_MODE
    elevation_speed = tunable(0)

    flywheel_mode = FLYWHEEL_MODE
    flywheel_speed = tunable(0)

    # fx_motors = [self.flywheel]
    # srx_motors = [self.azimuth, self.elevation]

    def config_encoders(self):
        pass
        # Falcon500 motors use the integrated sensor
        # for controller in self.fx_motors:

        self.flywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)

        # These motors are using an attached Quad Encoder
        # for controller in self.srx_motors:
        # controller.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0)
        self.azimuth.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0)
        self.elevation.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0)

    def config_motors(self):
        self.flywheel.setInverted(TalonFXInvertType.Clockwise)  # = setInverted(True)
        self.azimuth.setInverted(False)
        self.elevation.setInverted(False)
        self.flywheel.setNeutralMode(NeutralMode.Coast)
        self.azimuth.setNeutralMode(NeutralMode.Brake)
        self.elevation.setNeutralMode(NeutralMode.Brake)

    def set_encoder_direction(self):
        pass

    def set_motor_output(self):
        pass

    def set_motor_neutral_mode(self):
        pass

    def setup(self):
        self.set_motor_output()
        self.set_motor_neutral_mode()

        self.config_encoders()
        self.config_motors()
        self.nt = NetworkTables.getTable("Shooter_values")

    def set_flywheel_velocity(self, vel):
        # run Flywheel at the given velocity
        self.flywheel_speed = self.flywheel_speed + limit(
            (vel * FLYWHEEL_MAX_VEL - self.flywheel_speed),
            -FLYWHEEL_MAX_ACCEL,
            FLYWHEEL_MAX_DECEL,
        )

    def setFlywheelPercent(self, speed):
        self.flywheel_speed = speed

    def incrementFlywheelPercent(self):
        self.flywheel_speed += FLYWHEEL_INCREMENT

    def decrementFlywheelPercent(self):
        self.flywheel_speed -= FLYWHEEL_INCREMENT

    def stopFlywheelPercent(self):
        self.flywheel_speed = 0

    def setElevationPercent(self, speed):
        self.elevation_speed = speed

    def setAzimuthPercent(self, speed):
        self.azimuth_speed = speed

    def toggle_firing_speed(self):
        # True = 1, so if expression is true, second element
        # in list is selected.
        self.flywheel_speed = [FLYWHEEL_SPEED_GOAL, FLYWHEEL_SPEED_LOB][
            self.flywheel_speed == FLYWHEEL_SPEED_GOAL
        ]

    def setPID(self, motor_control, pid):
        motor_control.config_kP(pid.slot, pid.p, 0)
        motor_control.config_kI(pid.slot, pid.i, 0)
        motor_control.config_kD(pid.slot, pid.d, 0)
        motor_control.config_kF(pid.slot, pid.f, 0)

    def update_nt(self, key, value):
        """update network tables with drive telemetry"""
        self.nt.putNumber(key, value)

    def execute(self):
        self.update_nt(
            'flywheel_velocity',
            self.flywheel.getSelectedSensorVelocity(FeedbackDevice.IntegratedSensor),
        )
        self.flywheel.set(self.flywheel_mode, self.flywheel_speed)
        self.elevation.set(self.elevation_mode, self.elevation_speed)


class Conveyor:
    lowerConveyor: WPI_TalonSRX
    upperConveyor: WPI_TalonSRX

    lowerConveyor_mode = ControlMode.PercentOutput
    lowerConveyor_speed = tunable(0)

    upperConveyor_mode = ControlMode.PercentOutput
    upperConveyor_speed = tunable(0)

    def enable_upper_conveyor(self):
        self.upperConveyor.set(ControlMode.PercentOutput, UPPER_CONVEYOR_SPEED)

    def disable_upper_conveyor(self):
        self.upperConveyor.set(ControlMode.PercentOutput, 0)

    @feedback
    def get_lower_switch(self):
        return self.upperConveyor.isRevLimitSwitchClosed()

    @feedback
    def get_upper_switch(self):
        return self.upperConveyor.isFwdLimitSwitchClosed()

    def execute(self):
        if self.get_lower_switch() and not self.get_upper_switch():
            pass


class Intake:

    intake: WPI_TalonSRX

    intake_mode = ControlMode.PercentOutput
    intake_speed = tunable(0)

    # fx_motors = [lowerConveyor, upperConveyor]
    # srx_motors = [intake]

    def config_encoders(self):
        pass
        # Falcon500 motors use the integrated sensor
        # for controller in self.fx_motors:
        # controller.configSelectedFeedbackSensor(
        # FeedbackDevice.IntegratedSensor, 0, 0
        # )
        ## These motors are using an attached Quad Encoder
        # for controller in self.srx_motors:
        # controller.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0)

    def config_motors(self):
        pass

    def init_nt(self):
        NetworkTables.initialize()
        self.nt = NetworkTables.getTable("Intake_values")

    def update_nt(self, control, value):
        self.nt.putNumber(control, value)

    def set_encoder_direction(self):
        pass

    def set_motor_output(self):
        pass

    def set_motor_neutral_mode(self):
        pass

    def setup(self):
        self.set_motor_output()
        self.set_motor_neutral_mode()

        self.config_encoders()
        self.config_motors()

    def set_pid(self, motor_control, pid):
        motor_control.config_kP(pid.slot, pid.p, 0)
        motor_control.config_kI(pid.slot, pid.i, 0)
        motor_control.config_kD(pid.slot, pid.d, 0)
        motor_control.config_kF(pid.slot, pid.f, 0)

    def execute(self):
        # if we have a ball at the bottom, and not one at the top,
        # go ahead and move the conveyor.
        pass


class FROGTurret:
    shooter: Shooter
    # intake: Intake()

    def execute(self):
        pass
