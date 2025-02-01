# from enum import Enum
from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder
from phoenix5 import TalonSRX, TalonSRXConfiguration, ControlMode, TalonSRXControlMode
import constants

class Intake(Subsystem):
    """
    Test class for shooter prototype
    """

    def __init__(self):
        super().__init__()
        self.Intake_Motor: TalonSRX = TalonSRX(constants.INTAKE_MOTOR)
        self.Intake_Motor.configFactoryDefault()


    def drive_motor(self, speed: float):
        # self.Intake_Motor.set(ControlMode.PercentOutput, speed)
        self.Intake_Motor.set(TalonSRXControlMode.PercentOutput, speed)


    def stop_motor(self) -> None:
        self.Intake_Motor.set(ControlMode.PercentOutput, 0)

    def periodic(self) -> None:
        # SmartDashboard.putNumber("Intake_Speed", self.Intake_Motor.getMotorOutputPercent)
        pass

class SetIntake(Command):
    def __init__(self, Intake: Intake):
        self._Intake = Intake

        self.addRequirements(self._Intake)

    def initialize(self):
        # self._Intake.set_Intake_location(self._location)
        pass 

    def execute(self):
        self._Intake.drive_motor(0.2)
       
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool):
        self._Intake.stop_motor()
