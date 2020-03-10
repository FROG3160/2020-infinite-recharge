'''Components for managing the control panel controller'''
from ctre import (
    WPI_TalonFX,
    WPI_TalonSRX,
    FeedbackDevice,
    ControlMode,
    NeutralMode,
    TalonFXInvertType,
)
from .common import TalonPID, limit
from magicbot import tunable, feedback

CONTROLWHEEL_PID = TalonPID(0, p=0.05, f=0.04)
CONTROLWHEEL_MODE = ControlMode.Position


class ControlWheel:
    _PID = CONTROLWHEEL_PID
    controlwheel_motor: WPI_TalonSRX
    controlwheel_mode = CONTROLWHEEL_MODE
    controlwheel_command = tunable(0)

    def __init__(self):
        self.enabled = False

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    # read current encoder position
    @feedback
    def get_position(self):
        return self.controlwheel_motor.getSelectedSensorPosition(
            FeedbackDevice.IntegratedSensor
        )

    @feedback
    def get_speed(self):
        return self.controlwheel_motor.get()

    def setup(self):
        # this motor uses an attached Quad Encoder
        self.controlwheel_motor.configSelectedFeedbackSensor(
            FeedbackDevice.QuadEncoder, 0, 0
        )
        self.controlwheel_motor.setSensorPhase(False)
        self.controlwheel_motor.setInverted(False)
        self.controlwheel_motor.setNeutralMode(NeutralMode.Brake)

    def set_position(self, value):
        # move to the given position
        self.controlwheel_mode = ControlMode.Position
        self.controlwheel_command = value

    def set_speed(self, speed):
        self.controlwheel_mode = ControlMode.PercentOutput
        self.controlwheel_command = speed

    def execute(self):
        if self.enabled:
            self.controlwheel_motor.set(
                self.controlwheel_mode, self.controlwheel_command
            )
        else:
            self.controlwheel_motor.set(0)
