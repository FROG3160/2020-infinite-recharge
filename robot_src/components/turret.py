'''Intake and shooter components'''
from ctre import WPI_TalonFX, WPI_TalonSRX, FeedbackDevice, ControlMode, NeutralMode, TalonFXInvertType
from .common import PID, limit
from networktables import NetworkTables

NEVEREST_CPR = 7 * 60  # motor ticks * gear reduction
FALCON_CPR = 2048
FLYWHEEL_MAX_VEL = 25000  # Falcons are maxing at 20k - 21k
FLYWHEEL_MAX_ACCEL = FLYWHEEL_MAX_VEL / 50  # sampled 50 times a second makes this MAX ACCEL/sec
FLYWHEEL_MAX_DECEL = FLYWHEEL_MAX_ACCEL


class Shooter:

    flywheelPID = PID(0, p=0)

    azimuth = WPI_TalonSRX(31)
    elevation = WPI_TalonSRX(32)
    flywheel = WPI_TalonFX(33)

    turret_mode = ControlMode.PercentOutput
    turret_speed = 0

    hood_mode = ControlMode.PercentOutput
    hood_speed = 0

    flywheel_mode = ControlMode.PercentOutput
    flywheel_speed = 0

    fx_motors = [flywheel]
    srx_motors = [azimuth, elevation]

    def config_encoders(self):
        # Falcon500 motors use the integrated sensor
        for controller in self.fx_motors:
            controller.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor, 0, 0
            )
        # These motors are using an attached Quad Encoder
        for controller in self.srx_motors:
            controller.configSelectedFeedbackSensor(
                FeedbackDevice.QuadEncoder, 0, 0
            )

    def config_motors(self):
        self.flywheel.setInverted(TalonFXInvertType.CounterClockwise)
        self.flywheel.setNeutralMode(NeutralMode.Coast)

    def init_NT(self):
        NetworkTables.initialize()
        self.turret_encoders = NetworkTables.getTable("turret_encoder_values")

    def update_NT(self, control, value):
        self.turret_encoders.putNumber(control, value)

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

        self.init_NT()

    def setFlywheelVelocity(self, vel):
        # run Flywheel at the given velocity
        self.flywheel_speed = self.flywheel_speed + limit(
            (vel * FLYWHEEL_MAX_VEL - self.flywheel_speed),
            -FLYWHEEL_MAX_ACCEL,
            FLYWHEEL_MAX_DECEL)

    def setFlywheelPercent(self, speed):
        self.flywheel_speed = speed

    def setPID(self, motor_control, pid):
        motor_control.config_kP(pid.slot, pid.p, 0)
        motor_control.config_kI(pid.slot, pid.i, 0)
        motor_control.config_kD(pid.slot, pid.d, 0)
        motor_control.config_kF(pid.slot, pid.f, 0)

    def execute(self):
        self.update_NT(
            'flywheel_velocity',
            self.flywheel.getSelectedSensorVelocity(FeedbackDevice.IntegratedSensor)
        )
        self.flywheel.set(ControlMode.PercentOutput, self.flywheel_speed)


class Intake:

    intake = WPI_TalonSRX(21)
    lowerConveyor = WPI_TalonFX(22)
    upperConveyor = WPI_TalonFX(23)

    intake_mode = ControlMode.PercentOutput
    intake_speed = 0

    lowerConveyor_mode = ControlMode.PercentOutput
    lowerConveyor_speed = 0

    upperConveyor_mode = ControlMode.PercentOutput
    upperConveyor_speed = 0

    fx_motors = [lowerConveyor, upperConveyor]
    srx_motors = [intake]

    def config_encoders(self):
        # Falcon500 motors use the integrated sensor
        for controller in self.fx_motors:
            controller.configSelectedFeedbackSensor(
                FeedbackDevice.IntegratedSensor, 0, 0
            )
        # These motors are using an attached Quad Encoder
        for controller in self.srx_motors:
            controller.configSelectedFeedbackSensor(
                FeedbackDevice.QuadEncoder, 0, 0
            )

    def config_motors(self):
        pass

    def init_NT(self):
        NetworkTables.initialize()
        self.intake_encoders = NetworkTables.getTable("intake_encoder_values")

    def update_NT(self, control, value):
        self.intake_encoders.putNumber(control, value)

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

    def setPID(self, motor_control, pid):
        motor_control.config_kP(pid.slot, pid.p, 0)
        motor_control.config_kI(pid.slot, pid.i, 0)
        motor_control.config_kD(pid.slot, pid.d, 0)
        motor_control.config_kF(pid.slot, pid.f, 0)

    def execute(self):
        pass


class FROGTurret:
    shooter: Shooter
    # intake: Intake()

    def execute(self):
        pass
