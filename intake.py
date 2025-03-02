# from enum import Enum
from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder, Timer
from phoenix5 import TalonSRX, TalonSRXConfiguration, ControlMode, TalonSRXControlMode
import constants
import wpilib
from commands2.button import CommandXboxController

#===(Hardware Notes)==============================================
'''
The intake is moved using an AndyMark NeveRest motor controlled by a Talon SRX.
The motor runs at 130 RPM.

No sensors (limit switches or rotation encoders) are used.

Spinning the wheels inward is the positive direction and the SRX indicates green.

'''
#================================================================


class Intake(Subsystem):
    """
    Test class for shooter prototype
    """

    def __init__(self):
        super().__init__()
        self.Intake_Motor: TalonSRX = TalonSRX(constants.INTAKE_MOTOR)
        self.Intake_Motonr.configFactoryDefault()


    def drive_motor(self, speed: float):
        # self.Intake_Motor.set(ControlMode.PercentOutput, speed)
        self.Intake_Motor.set(TalonSRXControlMode.PercentOutput, speed)


    def stop_motor(self) -> None:
        self.Intake_Motor.set(ControlMode.PercentOutput, 0)

    def periodic(self) -> None:
        # SmartDashboard.putNumber("Intake_Speed", self.Intake_Motor.getMotorOutputPercent)
        pass

#=====================================================================

class SetIntakeManual(Command):
    def __init__(self, Intake: Intake, speed: float):
        self._Intake = Intake
        self._speed: float = speed

        self.addRequirements(self._Intake)

    def initialize(self):
        # self._Intake.set_Intake_location(self._location)
        pass 

    def execute(self):
        self._Intake.drive_motor(self._speed)
       
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool):
        self._Intake.stop_motor()

#=====================================================================

class SetIntakeSpeedandTime(Command):
    def __init__(self, Intake: Intake, speed: float, runseconds: float):
        self._Intake = Intake
        self.speed = speed
        self.runseconds = runseconds
        self._timer = Timer()
        self._timer.start()
        self.addRequirements(self._Intake)

    def initialize(self):
        self._timer.restart()
        print ("Running Intake speed: ",self.speed, " for ", self.runseconds, "seconds at "
               , wpilib.Timer.getFPGATimestamp()  )

    def execute(self):
        self._Intake.drive_motor(self.speed)
       
    def isFinished(self) -> bool:
        return self._timer.hasElapsed(self.runseconds)
    
    def end(self, interrupted: bool):
        self._Intake.stop_motor()