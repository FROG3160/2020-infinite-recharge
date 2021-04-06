from magicbot import AutonomousStateMachine, timed_state, state
import wpilib

# this is one of your components
from components.drivetrain import FROGDrive


class DriveForward(AutonomousStateMachine):

    MODE_NAME = "Back Up"
    DEFAULT = True

    # Injected from the definition in robot.py
    chassis: FROGDrive

    @timed_state(duration=1, first=True)
    def drive_backward(self):
        #self.chassis.init_position_mode()
        #self.chassis.set_position(36)
        pass
