from commands2 import Subsystem, Command, RunCommand
from wpilib import SmartDashboard, RobotBase, RobotController, DutyCycleEncoder
from phoenix5 import TalonSRX, TalonSRXConfiguration, ControlMode, TalonSRXControlMode, Faults
import constants


class WristControl(Subsystem):
    def __init__(self):
        # Initialize the TalonSRX motor for the wrist
        self.wrist_motor = TalonSRX(constants.WRIST_MOTOR)
        

        self.wrist_motor_faults:  Faults = Faults()


        # Configure the motor (if necessary, add configurations here).
        self.wrist_motor.configFactoryDefault()  # Reset to defaults
        
        # # Initialize the joystick (assuming joystick 0 is used).
        # self.joystick = Joystick(0)

    def move_wrist(self, speed: float):

        # Use the joystick input to control the motor speed.
        # Adjust the speed values as needed for your specific setup.
        self.wrist_motor.set(ControlMode.PercentOutput, speed)  # Corrected ControlMode
        SmartDashboard.putNumber("Wrist_speed", speed)



    def Wrist_at_Top(self) -> bool:
        at_top = self.wrist_motor_faults.ForwardLimitSwitch
        SmartDashboard.putBoolean("Wrist at Top", at_top)
        return at_top

    def Wrist_at_Bottom(self) -> bool:
        at_bottom = self.wrist_motor_faults().ReverseLimitSwitch
        SmartDashboard.putBoolean("Wrist at Bottom", at_bottom)
        return at_bottom


    def periodic(self):
        # You can display the current wrist motor position (if encoder is set up) on the dashboard.
        SmartDashboard.putNumber("Wrist Position", self.wrist_motor.getSelectedSensorPosition())  # Corrected method to get position
        self.wrist_motor.getFaults(self.wrist_motor_faults)

#================================================================================================

class SetWrist(Command):
    def __init__(self, Wrist: WristControl, speed: float):
        super().__init__()
        self._Wrist = Wrist
        self._speed = speed

        self.addRequirements(self._Wrist)

    def initialize(self):
        # self._Intake.set_Intake_location(self._location)
        pass 

    def execute(self):
        self._Wrist.move_wrist(self._speed)
       
    def isFinished(self) -> bool:
        return False
    
    def end(self, interrupted: bool):
        self._Wrist.move_wrist(0)
