from commands2 import Subsystem, Command
from commands2.button import CommandXboxController
from wpilib import SmartDashboard, DutyCycleEncoder, Timer, RobotBase
from phoenix5 import TalonSRX, ControlMode, Faults
import constants
import wpilib

#===(Hardware Notes)==============================================
'''
The wrist is moved using an AndyMark NeveRest motor controlled by a Talon SRX.
The motor runs at 25 RPM and has a 22 Tooth steel gear driving a steel 52 Tooth steer gear.
The 52 Tooth gear is on the wrist axle.

The wrist Talon SRX shows red when the wrist is moving upward.  (Negative direction)

The intake axle has limit switches connected directly to the Talon SRX to prevent the 
robot software from overdriving the wrist axle and breaking the robot.  The talon SRX has a 
breakout box mounted on the remote interface to access the limit switch interface.  This
board also provides red LEDs to indicate when the limit is reached.  The top LED/connector is
connected to the top limit switch.  The bottom LED/connector is connected to the bottom limit 
switch.

The wrist joint is monitored with a REV robotics absolute encoder which read degrees.
The absolute encoder is connected to the RoboRIO Digital Input/Output #0

The wrist has two main commands:
1) Manual control of the wrist
2) Autonomous mode where the desired set point angle is provided

'''
#================================================================


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
    def __init__(self, Wrist: WristControl, target_angle: float, timeout = 1):
        super().__init__()
        self._Wrist = Wrist
        self.target_angle = target_angle
        self._timeout = timeout

        self._timer = Timer()
        self._timer.start()
        self.addRequirements(self._Wrist)

    def initialize(self):
        self._timer.restart()
        print ("Moving wrist to: ",self.target_angle, "  at " , wpilib.Timer.getFPGATimestamp() )

    def execute(self):
        current_angle = self._Wrist.getAbsolutePosition()
        if current_angle > self.target_angle:
            self._Wrist.move_wrist_down(0.2)
        else:
            self._Wrist.move_wrist_up(0.2) 
        
    def isFinished(self) -> bool:
        ret = False
        if RobotBase.isSimulation():
            ret = True    # just for testing simulation

        current_angle = self._Wrist.getAbsolutePosition()
        # print ((current_angle - self.target_angle))
        if (abs(current_angle - self.target_angle) < 5):
            ret = True
        if (self._timer.hasElapsed(self._timeout)):
            ret = True
        return ret

    def end(self, interrupted: bool):
        self._Wrist.move_wrist(0)
        print ("WRIST MOVEMENT DONE at ", wpilib.Timer.getFPGATimestamp())



