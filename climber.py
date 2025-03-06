# from enum import Enum
from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder, Timer
from phoenix5 import TalonSRX, TalonSRXConfiguration, ControlMode, TalonSRXControlMode
import constants
import wpilib
from commands2.button import CommandXboxController

#===(Hardware Notes)==============================================
'''
The climber is moved using an 775 motor controlled by a Talon SRX.

The climber DOES  NOT have limit switches at the top and bottom of range
motion to prevent damage.

A prototype limit switch is partially installed which could perform this 
function but is not complete.

GREAT care must be used when driving the limit switch or hardware DAMAGE will occur

'''
#================================================================


class Climber(Subsystem):
    """
    Class to control the single linear actuator used to lift the robot
    """

    def __init__(self):
        super().__init__()
        self.Climber_Motor: TalonSRX = TalonSRX(constants.INTAKE_MOTOR)
        self.Climber_Motor.configFactoryDefault()


    def drive_motor(self, speed: float):
        self.Climber_Motor.set(TalonSRXControlMode.PercentOutput, speed)


    def stop_motor(self) -> None:
        self.Climber_Motor.set(ControlMode.PercentOutput, 0)

    def periodic(self) -> None:
        SmartDashboard.putNumber("Climber_Speed", self.Climber_Motor.getMotorOutputPercent)
        pass

#=====================================================================

class SetClimberManual(Command):
    def __init__(self, Climber: Climber, speed: float):
        self._Climber = Climber
        self._speed: float = speed
        self.addRequirements(self._Climber)

    def initialize(self):
        pass 

    def execute(self):
        self._Climber.drive_motor(self._speed)
       
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool):
        self._Climber.stop_motor()

#=====================================================================

class SetClimberSpeedandTime(Command):
    def __init__(self, Climber: Climber, speed: float, runseconds: float):
        self._Climber = Climber
        self.speed = speed
        self.runseconds = runseconds
        self._timer = Timer()
        self._timer.start()
        self.addRequirements(self._Climber)

    def initialize(self):
        self._timer.restart()
        print ("Running Climber speed: ",self.speed, " for ", self.runseconds, "seconds at "
               , wpilib.Timer.getFPGATimestamp()  )

    def execute(self):
        self._Climber.drive_motor(self.speed)
       
    def isFinished(self) -> bool:
        return self._timer.hasElapsed(self.runseconds)
    
    def end(self, interrupted: bool):
        self._Climber.stop_motor()