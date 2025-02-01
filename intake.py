# from enum import Enum
from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder
from phoenix5 import TalonSRX, TalonSRXConfiguration, ControlMode
import constants

class Intake(Subsystem):
    """
    Test class for shooter prototype
    """

    def __init__(self, test_mode = False):
        super().__init__()
        self.Intake_Motor = TalonSRX(constants.INTAKE_MOTOR)
        self.Intake_Motor.configFactoryDefault()

    def drive_motor(self, speed: float):
        self.Intake_Motor.set(ControlMode.PercentOutput, speed)

    def stop_motor(self) -> None:
        self.Intake_Motor.set(ControlMode.PercentOutput, 0)

    def periodic(self) -> None:
        SmartDashboard.putNumber("Intake_Speed", self.Intake_Motor.getMotorOutputPercent)

class SetIntake(Command):
    def __init__(self, Intake: Intake):
        self._Intake = Intake

        self.addRequirements(self._Intake)

    def initialize(self):
        # self._Intake.set_Intake_location(self._location)
        pass 

    def execute(self):
        self.drive_motor(self, 0.5)
       
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool):
        self._Intake.stop_motor()
