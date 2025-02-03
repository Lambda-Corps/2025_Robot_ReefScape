from typing import Self
from commands2 import Subsystem, Command, cmd
from phoenix5 import (
    TalonSRX,
    TalonSRXControlMode,
    TalonSRXConfiguration,
    LimitSwitchSource,
    Faults,
    LimitSwitchNormal,
)

#===========================================================
from phoenix6.configs import (
    TalonFXConfiguration,
    TalonFXConfigurator,
)


from phoenix6 import hardware, controls
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls.follower import Follower
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue, ForwardLimitValue, ReverseLimitValue
from phoenix6.unmanaged import feed_enable
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals.spn_enums import (InvertedValue, NeutralModeValue, FeedbackSensorSourceValue)
from phoenix6 import StatusCode

#===========================================================

from wpilib import SmartDashboard, AnalogInput, RobotBase, Timer
import constants

class ELEVATOR(Subsystem):
    ELEVATOR_TOP_LIMIT = 5000
    ELEVATOR_UP_SPEED = 0.2
    ELEVATOR_DOWN_SPEED = -0.2

    def __init__(self) -> None:
        super().__init__()

#===========================================================
# DF 1-17-2026:   Changed from a Talon SRX to Talon FX initialization sequence
        self._ELEVATOR: TalonFX = self.__configure_elevatorMotor()

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
        config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        # config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        config.motor_output.neutral_mode = NeutralModeValue.BRAKE

        # This configuration item supports counting wheel rotations
        # This item sets the gear ratio between motor turns and wheel turns
        config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR
        config.feedback.sensor_to_mechanism_ratio = 10

        # Apply the configuration to the motors
        for i in range(6):  # Try 5 times with results returned into "ret"
            ret = talon.configurator.apply(config)

        talon.set_position(0)    #  Reset the encoder to  zero

        return talon

    def move_ELEVATOR_up(self) -> None:
        output = controls.DutyCycleOut(0.0)
        self._ELEVATOR.set_control(output.with_output(self.ELEVATOR_UP_SPEED).with_limit_forward_motion(self.ELEVATOR_at_top()))

    def move_ELEVATOR_down(self) -> None:
        output = controls.DutyCycleOut(0.0)
        self._ELEVATOR.set_control(output.with_output(self.ELEVATOR_DOWN_SPEED).with_limit_reverse_motion(self.ELEVATOR_at_bottom()))

    def ELEVATOR_at_top(self) -> bool:
        return self._ELEVATOR.get_forward_limit() == ForwardLimitValue.CLOSED_TO_GROUND

        # Notes on:  StatusSignal[ForwardLimitValue]
        # https://api.ctr-electronics.com/phoenix6/release/python/autoapi/phoenix6/hardware/core/core_talon_fx/index.html

        # https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/actuator-limits.html
        # https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreTalonFX.html

    def ELEVATOR_at_bottom(self) -> bool:
        return self._ELEVATOR.get_reverse_limit() == ReverseLimitValue.CLOSED_TO_GROUND

    def stop_ELEVATOR_motors(self) -> None:
        # self._ELEVATOR.set(TalonSRXControlMode.PercentOutput, 0)   # SRX control code
        self._ELEVATOR.set_control(controls.DutyCycleOut(0))       # FX Control Code
        
    def periodic(self) -> None:
        # self._ELEVATOR.getFaults(self._left_faults)          # SRX Control code
       
        # SmartDashboard.putBoolean("EL Forward Limit", self._left_faults.ForwardLimitSwitch)  # SRX
        SmartDashboard.putBoolean("Elev Forward Limit", self.ELEVATOR_at_top())                                 
                                          
        # SmartDashboard.putBoolean("Elev Reverse Limit", self._ELEVATOR.isRevLimitSwitchClosed())
        SmartDashboard.putBoolean("Elev Reverse Limit", self.ELEVATOR_at_bottom())
        

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
    
    
    def reset_encoder(self) -> None:
        self.talon.set_position(0)


    def periodic(self) -> None:
        SmartDashboard.putNumber("Elevator_Position", self.talon.get_position().value)
        SmartDashboard.putBoolean("Elevator_At_Lower_Limit", self.talon)


##### WORK IN PROGRESS
    # def Elevator_at_bottom(self) -> bool:
    #     return self.talon.reverse_limit_switch_closed()
            
    # get_reverse_limit = self.talon.get_reverse_limit()

    def reset_Elevator(self) -> None:
        while not self.talon.get_reverse_limit():
            self.talon.set(0.2)
        self.talon.set_position(0)
