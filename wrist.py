from commands2 import Subsystem, Command
from commands2.button import CommandXboxController
from wpilib import SmartDashboard, DutyCycleEncoder, Timer, RobotBase
from phoenix5 import TalonSRX, ControlMode, Faults
import constants
import wpilib
from wpimath.controller import PIDController

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

The encoder is mounted so that the 0 degree position is when the wrist is a little bit forward of vertical. 
The encoder reads about 50 degrees with the wrist all the way forward and hitting the lower limit switch.
The encoder reads about 340 degrees (-20) when the wrist is in the "Stowed" position. 
The "Stowed" position is needed to allow the intake to be within the frame of the robot at the start
of the competition.

The wrist has two main commands:
1) Set_Wrist_Angle_manual_and_auto_with_PID
2) Set_Global_Wrist_Angle

The Command "Set_Wrist_Angle_manual_and_auto_with_PID" the wrist default command.
This command reads a wrist global variable and uses this to set the wrist angle.
When the partner moves the joystick, the value outside of the deadband is detect and the
joystick is used to update the global variable.

Autonomous command "Set_Global_Wrist_Angle" is available to set the global variable.


'''
#================================================================


class WristControl(Subsystem):
    __DRIVER_DEADBAND = 0.1
    WRIST_UP_SPEED = -.3
    WRIST_DOWN_SPEED = .3
    
    def __init__(self):
        super().__init__()
        # Initialize the TalonSRX motor for the wrist
        self.wrist_motor = TalonSRX(constants.WRIST_MOTOR)
        self.wrist_motor.configFactoryDefault()  # Reset to defaults

        # Initialize the Faults object
        self.wrist_motor_faults = Faults()

        # Initialize the absolute encoder
        self._wrist_angle = self.__configure_wrist_encoder()

        self.global_target_wrist_angle = 0

        kP = 0.1
        kI = 0.0001
        kD = 0.0001
        wrist_angle_tolerance = 2  # degrees

        self.wrist_pid_controller = PIDController(kP, kI, kD)
        self.wrist_pid_controller.setTolerance(wrist_angle_tolerance)

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
        SmartDashboard.putBoolean("Wrist at Bottom", at_top)  ## Directional Errors here
        return at_top

    def Wrist_at_Bottom(self) -> bool:
        self.wrist_motor.getFaults(self.wrist_motor_faults)
        at_bottom = self.wrist_motor_faults.ReverseLimitSwitch
        SmartDashboard.putBoolean("Wrist at Top", at_bottom)
        return at_bottom
    
    def getAbsolutePosition(self) -> float:
        '''
        Return angle and keep between values of 180 and -180

        -20 is the forward limit, and 55 is the reverse limit

        We want the wrist to automatically start at 0 on autonomous init
        '''
        angle = 360 * self._wrist_angle.get()
        if angle > 180: 
            angle = angle - 360
        return angle
    
    def periodic(self):
        # if self._wrist_angle.isConnected():
        SmartDashboard.putNumber("Wrist Encoder Pos", self.getAbsolutePosition())
        SmartDashboard.putNumber("Wrist Position", self.wrist_motor.getSelectedSensorPosition())
        self.Wrist_at_Bottom()
        self.Wrist_at_Top()
        SmartDashboard.putNumber("Global Wrist Position", self.get_global_wrist_angle())


    def move_wrist_up(self, speed: float):
        # if not self.Wrist_at_Top():
        #     self.move_wrist(-speed)
        # else:
        #     self.move_wrist(0)
        self.move_wrist(-speed)

    def move_wrist_down(self, speed: float):
        # if not self.Wrist_at_Bottom():
        #     self.move_wrist(speed)
        # else:
        #     self.move_wrist(0)
        self.move_wrist(speed)

    def get_global_wrist_angle(self) -> float:
        return self.global_target_wrist_angle
    
    def set_global_wrist_angle(self, new_angle) -> None:
        self.global_target_wrist_angle = new_angle
    
    def move_wrist_with_pid(self, target_angle: float) -> bool:
        # default to return false
        ret = False

        PID_STEP = 2 # change 2 degrees at a time
        curr_angle = self.getAbsolutePosition()
        # Change the angle to negative if we're moving upward from 45 -> 0 for example
        if target_angle < curr_angle:
            PID_STEP = PID_STEP * -1
        
        target_angle = self._clamp(curr_angle + PID_STEP, max=constants.WRIST_ACCEPTABLE_LOWER_LIMIT, min=constants.WRIST_ACCEPTABLE_UPPER_LIMIT)
        angle_error = self.wrist_pid_controller.calculate(curr_angle, target_angle)

        controlled_wrist_speed = self._clamp(angle_error)   
        self.move_wrist(controlled_wrist_speed)

        return self.wrist_pid_controller.atSetpoint()

    def reset_pid_controller(self, new_angle: float) -> None:
        self.wrist_pid_controller.reset()
        self.wrist_pid_controller.setSetpoint(new_angle)

    def _deadband(self, input: float, abs_min: float) -> float:
        """
        If the value is between 0 and the abs_min value passed in,
        return 0.

        This eliminates joystick drift on input
        """
        if input < 0 and input > (abs_min * -1):
            input = 0

        if input > 0 and input < abs_min:
            input = 0

        return input
   
    def _clamp(self, input: float, max=1, min=-1) -> float:  
        """
        Clamp (limit) control from -1 to 1
        """
        if input < min:
            input = min
        elif input > max:
            input = max
        else:
            #input must be zero
            pass
        return input


#================================================================================================

class Set_Wrist_Angle_manual_and_auto_with_PID(Command):
    def __init__(self, Wrist: WristControl,  controller: CommandXboxController, useInAutonomousMode = True):
        super().__init__()
        self._Wrist = Wrist
        self.controller = controller
        self.useInAutonomousMode = useInAutonomousMode
        self.target_angle = 0

        self._done = False
        self.addRequirements(self._Wrist)

    def initialize(self):
        self._done = False
        self.target_angle = self._Wrist.get_global_wrist_angle()
        print ("Moving wrist to: ",self.target_angle, "  at " , wpilib.Timer.getFPGATimestamp() )
        self._Wrist.reset_pid_controller()

    def execute(self):
        '''
        Get current angle
        Feed into PID Loop (need to add feed forward to counter gravity)
        PID controller calculates error
        !! Need to convert error into motor control
        Clamp motor control speed
        '''

        # Determine if manual controller is active (Joystick out of deadzone)    ## Move to separate function ##
        joystick_active = False

        joystick_position = self._Wrist._deadband(self.controller.getLeftY(), 0.1)
        if ( abs(joystick_position) > 0):  
            joystick_active = True

        if joystick_active:
            self.target_angle = self._Wrist._clamp(self.target_angle + joystick_position, max=constants.WRIST_ACCEPTABLE_LOWER_LIMIT, min=constants.WRIST_ACCEPTABLE_UPPER_LIMIT)
            self._Wrist.set_global_wrist_angle(self.target_angle)

        self._done = self._Wrist.move_wrist_with_pid(self.target_angle)
        # current_angle = self._Wrist.getAbsolutePosition()
        # AngleError = self.wrist_pid_controller.calculate(current_angle, self.target_angle)

        # controlled_wrist_speed = self._Wrist._clamp(AngleError)   
        # self._Wrist.move_wrist(controlled_wrist_speed)
        # print("Target: ", self.target_angle, "  Current:  ", current_angle , "  controlled_wrist_speed: ", controlled_wrist_speed)
        
    def isFinished(self) -> bool:    # When used in autonomous mode, we want the command to end when tolerance is reached
        return_value = False
        if self.useInAutonomousMode and self._done:
            return_value = True
        return return_value
        
    def end(self, interrupted: bool):
        self._Wrist.move_wrist(0)
        print ("WRIST MOVEMENT DONE at ", wpilib.Timer.getFPGATimestamp())


#================================================================================================
class Set_Global_Wrist_Angle(Command):
    def __init__(self, Wrist: WristControl, target_angle: float):
        super().__init__()
        self._Wrist = Wrist
        self.target_angle = target_angle
        self.addRequirements(self._Wrist)

    def initialize(self):
        self._Wrist.set_global_wrist_angle(self.target_angle)
        print ("Setting Global angle: ",self.target_angle, "  at " , wpilib.Timer.getFPGATimestamp() )

    def execute(self):
        pass

    def isFinished(self) -> bool:
        True

    def end(self, interrupted: bool):
        pass


class SetWristAngleAuto(Command):
    WRIST_ANGLE_TOLERANCE = 3
    def __init__(self, Wrist: WristControl, target_angle: float):
        super().__init__()
        self._Wrist = Wrist
        self.target_angle = target_angle
        self.addRequirements(self._Wrist)

    def initialize(self):
        self._done = False
        self._Wrist.reset_pid_controller()
        self._Wrist.set_global_wrist_angle(self.target_angle)
        print ("Setting Global angle: ",self.target_angle, "  at " , wpilib.Timer.getFPGATimestamp() )

    def execute(self):
        self._done = self._Wrist.move_wrist_with_pid(self.target_angle)

    def isFinished(self) -> bool:
        return self._done

    def end(self, interrupted: bool):
        pass

#================================================================================================
# class Wrist_Dummy_Default_command_Test(Command):
#     def __init__(self, Wrist: WristControl):
#         super().__init__()
#         self._Wrist = Wrist
#         self.addRequirements(self._Wrist)

#     def initialize(self):
#         print ("Dummy Default: ", wpilib.Timer.getFPGATimestamp() )

#     def execute(self):
#         pass

#     def isFinished(self) -> bool:
#         True

#     def end(self, interrupted: bool):
#         pass
