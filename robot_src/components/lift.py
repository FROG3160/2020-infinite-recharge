import wpilib
from ctre import WPI_TalonSRX, ControlMode, FeedbackDevice, NeutralMode
from .common import TalonPID
from magicbot import tunable, feedback

LIFT_PID = TalonPID(0, p=0.3)
LIFT_MODE = ControlMode.PercentOutput
LIFT_HIGH = 0
LIFT_LOW = 0


class Lift:

    _PID = LIFT_PID
    liftLeft: WPI_TalonSRX
    liftRight: WPI_TalonSRX
    lift_mode = LIFT_MODE
    lift_command = tunable(0)

    def __init__(self):
        self.enabled = False

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    # read current encoder position
    @feedback(key='LeftPosition')
    def get_positionLeft(self):
        return self.liftLeft.getSelectedSensorPosition(FeedbackDevice.IntegratedSensor)

    @feedback(key='RightPosition')
    def get_positionRight(self):
        return self.liftRight.getSelectedSensorPosition(FeedbackDevice.IntegratedSensor)

    @feedback(key='Percent')
    def get_speed(self):
        return self.liftLeft.get()

    def setup(self):

        for motor_controller in [self.liftLeft, self.liftRight]:

            # these motors use an attached Quad Encoder
            motor_controller.configSelectedFeedbackSensor(
                FeedbackDevice.QuadEncoder, 0, 0
            )
            motor_controller.setSensorPhase(False)
            motor_controller.setInverted(False)
            motor_controller.setNeutralMode(NeutralMode.Brake)
            # starting value for the encoder
            motor_controller.setSelectedSensorPosition(LIFT_HIGH, 0, 0)
            # setting soft limits
            motor_controller.configForwardSoftLimitThreshold(LIFT_HIGH - 100, 0)
            motor_controller.configReverseSoftLimitThreshold(LIFT_LOW + 100, 0)
            motor_controller.configForwardSoftLimitEnable(True, 0)
            motor_controller.configReverseSoftLimitEnable(True, 0)
            # setting the PID
            self._PID.configTalon(motor_controller)

    def set_position(self, value):
        # move to the given position
        self.lift_mode = ControlMode.Position
        self.lift_command = value

    def set_speed(self, speed):
        self.lift_mode = ControlMode.PercentOutput
        self.lift_command = speed

    def execute(self):
        if self.enabled:
            self.liftLeft.set(self.lift_mode, self.lift_command)
            self.liftRight.set(self.lift_mode, self.lift_command)
        else:
            self.liftLeft.set(0)
            self.liftRight.set(0)
