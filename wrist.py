from commands2 import Subsystem, Command
from commands2.button import CommandXboxController
from wpilib import SmartDashboard, DutyCycleEncoder
from phoenix5 import TalonSRX, ControlMode, Faults
import constants


class WristControl(Subsystem):
    def __init__(self):
        super().__init__()
        # Initialize the TalonSRX motor for the wrist
        self.wrist_motor = TalonSRX(constants.WRIST_MOTOR)
        self.wrist_motor.configFactoryDefault()  # Reset to defaults

        # Initialize the Faults object
        self.wrist_motor_faults = Faults()

        # Initialize the absolute encoder
        self._wrist_angle = self.__configure_wrist_encoder()

        # self.controller = CommandXboxController(constants.CONTROLLER_PORT)
        # self.configureButtonBindings()

    def __configure_wrist_encoder(self) -> DutyCycleEncoder:
        wrist_encoder = DutyCycleEncoder(constants.WRIST_ANGLE_ENCODER)  # DIO port
        return wrist_encoder

    def move_wrist(self, speed: float):
        # Use the joystick input to control the motor speed.
        # Adjust the speed values as needed
        self.wrist_motor.set(ControlMode.PercentOutput, speed)
        SmartDashboard.putNumber("Wrist_speed", speed)

    def Wrist_at_Top(self) -> bool:
        self.wrist_motor.getFaults(self.wrist_motor_faults)
        at_top = self.wrist_motor_faults.ForwardLimitSwitch
        SmartDashboard.putBoolean("Wrist at Top", at_top)
        return at_top

    def Wrist_at_Bottom(self) -> bool:
        self.wrist_motor.getFaults(self.wrist_motor_faults)
        at_bottom = self.wrist_motor_faults.ReverseLimitSwitch
        SmartDashboard.putBoolean("Wrist at Bottom", at_bottom)
        return at_bottom

    def getAbsolutePosition(self) -> float:
        return 360 * self._wrist_angle.get()
    
    def periodic(self):
        if self._wrist_angle.isConnected():
            SmartDashboard.putNumber("Wrist Encoder Pos", self.getAbsolutePosition())
        SmartDashboard.putNumber("Wrist Position", self.wrist_motor.getSelectedSensorPosition())

    def move_wrist_up(self, speed: float):
        if not self.Wrist_at_Top():
            self.move_wrist(speed)
        else:
            self.move_wrist(0)

    def move_wrist_down(self, speed: float):
        if not self.Wrist_at_Bottom():
            self.move_wrist(-speed)
        else:
            self.move_wrist(0)

   
    

#================================================================================================


class SetWrist(Command):
    def __init__(self, Wrist: WristControl, speed: float):
        super().__init__()
        self._Wrist = Wrist
        self._speed = speed
        self.addRequirements(self._Wrist)

    def initialize(self):
        pass

    def execute(self):
        self._Wrist.move_wrist(self._speed)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._Wrist.move_wrist(0)




#================================================================================================



class SetWrist_Manual(Command):
    def __init__(self, Wrist: WristControl, controller: CommandXboxController):
        super().__init__()
        self._Wrist = Wrist
        self._controller = controller
        self.addRequirements(self._Wrist)

    def initialize(self):
        pass

    def execute(self):
        self._Wrist.move_wrist(self._controller.getLeftY())

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._Wrist.move_wrist(0)


#================================================================================================


class Set_Wrist_Angle(Command):
    def __init__(self, Wrist: WristControl, target_angle: float):
        super().__init__()
        self._Wrist = Wrist
        self.target_angle = target_angle
        self.addRequirements(self._Wrist)

    def initialize(self):
        pass

    def execute(self):
        current_angle = self._Wrist.getAbsolutePosition()
        if current_angle > self.target_angle:
            self._Wrist.move_wrist_down(0.2)
        else:
            self._Wrist.move_wrist_up(0.2) 
        
    def isFinished(self) -> bool:
        current_angle = self._Wrist.getAbsolutePosition()
        #abs(current_angle - self.target_angle) < 20
        print ((current_angle - self.target_angle))
        return abs(current_angle - self.target_angle) < 5



    def end(self, interrupted: bool):
        self._Wrist.move_wrist(0)
        pass



