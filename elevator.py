from typing import Self
from enum import Enum
from phoenix6 import controls
from commands2 import InstantCommand, Subsystem, Command, cmd
# from phoenix5 import (
#     TalonSRX,
#     TalonSRXControlMode,
#     TalonSRXConfiguration,
#     LimitSwitchSource,
#     Faults,
#     LimitSwitchNormal,
# )
import wpilib
#===========================================================
from phoenix6.configs import (
    TalonFXConfiguration,
    TalonFXConfigurator,
)


from phoenix6 import hardware, controls, signals
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls.follower import Follower
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue, ForwardLimitValue, ReverseLimitValue
from phoenix6.unmanaged import feed_enable
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals.spn_enums import (InvertedValue, NeutralModeValue, FeedbackSensorSourceValue)
from phoenix6 import StatusCode
from robot import CommandXboxController
from wpilib import SmartDashboard, AnalogInput, RobotBase, Timer
import constants
from constants import ElevatorPosition

#===(Hardware Notes)==============================================
'''
The elavator is driven by a Falcon 500 which uses a Talon FX controller.
The motor has a VersaPlanetary gearbox with a reduction of 16 to 1 (two 4:1 stages)
After the gearbox is a 14 Tooth sprocket driving a 36 tooth sprocket

As of Feb 21, the Falcon indicates red when the elevator is going up.
The Falcon is running in inverted mode.
The limit switch at the top of the elavator is considered the Reverse Limit Switch
The limit switch at the bottom of the elevator is considered the Forward Limit Switch

We have enabled a feature that zeros the Falcon internal rotation counter when the Forward
Limit switch is hit called "forward_limit_autoset_position_value"

???? Will the internal position counter go to zero when the elevator reaches the top????

Our Motor is running in reverse when rising so the internal rotation counter is running negative
Our function which reads the counter negates the count.

Zero is a the bottom and about 4.9 counts (big gear revolutions) is the top.

The elevator has three main commands:
1) Manual control of the elevator
2) Autonomous mode where the desired set point is provided
3) Zero the elevator.  (Bring the elevator down slowly until the bottom limit switch and zero the counter)


NEXT YEAR we should design the robot motors to have positive direction move the arm or elevator and 
have the counts go positive.  The Upper limit switch should be at the end of the forward action.

'''
#================================================================

class ELEVATOR(Subsystem):
    ELEVATOR_TOP_LIMIT = 5000.
    ELEVATOR_UP_SPEED = -0.6  # was 0.2
    ELEVATOR_DOWN_SPEED = 0.5

    def __init__(self) -> None:
        super().__init__()

#===========================================================
        self._ELEVATOR: TalonFX = self.__configure_elevatorMotor()

        self.duty_cycle_out = controls.DutyCycleOut(0, enable_foc=False)

    def __configure_elevatorMotor(self,) -> TalonFX:
        
        # Applying a new configuration will erase all other config settings since
        # we start with a blank config so each setting needs to be explicitly set
        # here in the config method
        talon = TalonFX(constants.ELEVATOR)

        # Perform any other configuration
        config: TalonFXConfiguration = TalonFXConfiguration()
        config.slot0.k_s = 0.15  # Measured .15 volts to overcome static friction
        config.slot0.k_v = 0.11  # Measured .19 for 1ps
        config.slot0.k_a = 0.01  # complete guess, not measured
        config.slot0.k_p = (
            0.18  # an error of 1 RPS should add almost .19 more to correct
        )
        config.slot0.k_i = 0
        config.slot0.k_d = 0

        # Set the left side motors to be counter clockwise positive
        config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE  #  Red going upward

        config.motor_output.neutral_mode = NeutralModeValue.BRAKE

        # This configuration item supports counting wheel rotations
        # This item sets the gear ratio between motor turns and wheel turns
        config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR
        config.feedback.sensor_to_mechanism_ratio = constants.ELEVATOR_GEAR_RATIO

        # reset the relative encoder to 0 at the bottom
        config.hardware_limit_switch.forward_limit_autoset_position_enable = True
        config.hardware_limit_switch.forward_limit_autoset_position_value = 0

        # Apply the configuration to the motors
        for i in range(6):  # Try 5 times with results returned into "ret"
            ret = talon.configurator.apply(config)

        talon.set_position(0)    #  Reset the encoder to  zero

        return talon

    def move_ELEVATOR_up(self) -> None:
        self.duty_cycle_out.output = self.ELEVATOR_UP_SPEED
        self.__set_elevator_duty_cycle()

    def move_ELEVATOR_down(self) -> None:
        self.duty_cycle_out.output = self.ELEVATOR_DOWN_SPEED
        self.__set_elevator_duty_cycle()
    
    def move_ELEVATOR_down_with_speed(self,speed:float) -> None:
        self.duty_cycle_out.output = speed
        self.__set_elevator_duty_cycle()

    def ELEVATOR_at_top(self) -> bool:
        reverse_limit = self._ELEVATOR.get_reverse_limit() 
        return (reverse_limit.value is signals.ReverseLimitValue.CLOSED_TO_GROUND)
    
    def ELEVATOR_at_bottom(self) -> bool:
        forward_limit = self._ELEVATOR.get_forward_limit()
        return(forward_limit.value is signals.ReverseLimitValue.CLOSED_TO_GROUND)
    
    def stop_ELEVATOR_motors(self) -> None:
        self.duty_cycle_out.output = 0
        self.__set_elevator_duty_cycle()

    def __set_elevator_duty_cycle(self) -> None:
        self._ELEVATOR.set_control(self.duty_cycle_out)       # FX Control Code
    
    # def getPosition(self) -> float:
    #     return self._ELEVATOR.get_position()
    
    def get_rotation_count(self) -> float:
        '''
        To make sure the limit switches all function correctly, they are wired so the 
        reverse limit is at the top of the elevator
        '''
        return -self._ELEVATOR.get_position().value_as_double

#==================================================================================
##   SOURCE:  https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/api-usage/actuator-limits.html

##   SAMPLE CODE FOR DETECTING LIMIT SWITCHES ON FALCON 500 (Talon FX)
# from phoenix6 import signals

# ##### >> May need to reverse top/bottom depending on wiring of the robot
#
#    def elevator_at_top(self) -> bool:
#    # def elevator_at_bottom(self) -> bool:
#        reverse_limit = self._elevator_motor.get_reverse_limit()
#        return (reverse_limit.value is signals.ReverseLimitValue.CLOSED_TO_GROUND)


#    def elevator_at_bottom(self) -> bool:
#    # def elevator_at_top(self) -> bool:
#        forward_limit = self._elevator_motor.get_forward_limit()
#        return (forward_limit.value is signals.ForwardLimitValue.CLOSED_TO_GROUND)

    def reset_encoder(self) -> None:
        self._ELEVATOR.set_position(0)


    def periodic(self) -> None:
        # SmartDashboard.putNumber("Elevator Encoder", self._ELEVATOR.get_position().value_as_double)
        SmartDashboard.putNumber("Elevator Encoder", self.get_rotation_count())
        position: constants.ElevatorPosition = constants.get_closest_elevator_position(self._ELEVATOR.get_position().value)
        SmartDashboard.putNumber("Elevator_Position", position.value)
        SmartDashboard.putBoolean("Elevator_At_Lower_Limit", self.ELEVATOR_at_bottom())
    
    # def elevator_position(self, position) -> None:
    #     self._curr_position = position


    def reset_Elevator(self) -> None:
        if not self.ELEVATOR_at_bottom():
         self._ELEVATOR.get_position(0.2)
        else:
         self.ELEVATOR_at_bottom(0)


#==================================================================================


    # def periodic(self) -> None:
    #     SmartDashboard.putBoolean("Elev Forward Limit", self._ELEVATOR.get_forward_limit()==1)                                 
    #     SmartDashboard.putBoolean("Elev Reverse Limit", self._ELEVATOR.get_reverse_limit()==1) 
    #     # This code does not work.        

class MoveELEVATOR(Command):
    def __init__(self, sub: ELEVATOR, speed: float, timeout = 0):
        super().__init__()

        self._speed = speed
        self._ELEVATOR = sub
        self._timeout = timeout

        self._timer = Timer()
        self._timer.start()

        self.addRequirements(self._ELEVATOR)     

    def initialize(self):
        self._timer.restart()

    def execute(self):
        ## TODO:  ADD A CHECK TO PREVENT ELEVATOR MOVEMENT UNLESS WRIST IN ACCEPTABLE POSITION 
        #         OR MOVE WRIST TO MIDDLE POSITION

        if self._speed > 0:
            self._ELEVATOR.move_ELEVATOR_up()
        elif self._speed < 0:
            self._ELEVATOR.move_ELEVATOR_down()
        else:
            self._ELEVATOR.stop_ELEVATOR_motors()


    def isFinished(self) -> bool:
        if self._timeout == 0:
            return False

        return self._timer.hasElapsed(self._timeout)

    def end(self, interrupted: bool):
        self._ELEVATOR.stop_ELEVATOR_motors()
    #==================================================================================

    #==================================================================================
class MoveELEVATORToSetPoint(Command):
    DIRECTION_UP = -1    # was 1
    DIRECTION_DOWN = 1
    def __init__(self, sub: ELEVATOR, TargetPosition: ElevatorPosition, ):
     super().__init__()

     self._ELEVATOR = sub
     self._TargetPosition = TargetPosition
     self._timer = Timer()
     self._direction = 0
     self._timer.start()
     self.addRequirements(self._ELEVATOR)     

    def initialize(self):
        ## TODO:  ADD A COMMAND TO MOVE WRIST TO 30 DEGREES

        print ("Elevator to set position: ", self._TargetPosition, "  at ", wpilib.Timer.getFPGATimestamp())
        self._timer.restart()
        self.currentposition = self._ELEVATOR.get_rotation_count()
        if self._TargetPosition.value > self.currentposition:    # trying to reverse direction (Was >)
            self._direction = MoveELEVATORToSetPoint.DIRECTION_UP
        else:
            self._direction = MoveELEVATORToSetPoint.DIRECTION_DOWN

    def execute(self):
        self.currentposition = self._ELEVATOR.get_rotation_count()
      
        if self._direction == MoveELEVATORToSetPoint.DIRECTION_UP:
            self._ELEVATOR.move_ELEVATOR_up()
        elif self._direction == MoveELEVATORToSetPoint.DIRECTION_DOWN:
            self._ELEVATOR.move_ELEVATOR_down()
        else:
            self._ELEVATOR.stop_ELEVATOR_motors()
        

    def isFinished(self) -> bool:
        ret = False
        # print ("Current: ",  self.currentposition,   "  target " , self._TargetPosition.value, 
        #        " Delta: ", self.currentposition > self._TargetPosition.value)
        if self._direction == MoveELEVATORToSetPoint.DIRECTION_UP:
            if self.currentposition > self._TargetPosition.value:
                ret = True
        elif self._direction == MoveELEVATORToSetPoint.DIRECTION_DOWN:
            if self.currentposition < self._TargetPosition.value:
                ret = True
        else:
            print ("wrong direction: {}".format(self._direction))
        return ret
        

    def end(self, interrupted: bool):
        self._ELEVATOR.stop_ELEVATOR_motors()
        print ("Elevator DONE movement at ", wpilib.Timer.getFPGATimestamp())

    
#==================================================================================

class ElevatorController:
    LEVELS = {
        "A": 50,  # Example encoder values for different levels
        "B": 100,
        "X": 150,
        "Y": 200
    }
#==================================================================================

class MoveELEVATORToZero(Command):
    def __init__(self, sub: ELEVATOR):
        super().__init__()

        self._ELEVATOR = sub

        self.addRequirements(self._ELEVATOR)     

    def initialize(self):
        self._ELEVATOR.move_ELEVATOR_down_with_speed(0.2)
        ## TODO:  ADD A COMMAND TO MOVE WRIST TO 30 DEGREES

    def execute(self):
       SmartDashboard.putNumber("Elevator_Position", self._ELEVATOR.get_rotation_count())
       pass


    def isFinished(self) -> bool:
        if self._ELEVATOR.ELEVATOR_at_bottom():
            return True

    def end(self, interrupted: bool):
        self._ELEVATOR.stop_ELEVATOR_motors()
        self._ELEVATOR.reset_encoder()
        # SmartDashboard.putNumber("Elevator Encoder", self._ELEVATOR.get_position().value_as_double)

# ======================================================================================================
class Move_Elevator_L3(Command):
    def __init__(self, Elevator: ELEVATOR ):
        self.elevator = Elevator
        self.target_position = constants.L3
        self.addRequirements(self.elevator)

class CancelElevatorMovement(InstantCommand):
    def __init__(self, elevator: ELEVATOR):
        super().__init__()
        self._elevator = elevator
        self.addRequirements(self._elevator)

    def initialize(self):
        self._elevator.stop_ELEVATOR_motors()
        