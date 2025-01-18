# import wpilib
# from wpilib import DigitalInput
# from commands2 import SubsystemBase
# from phoenix6.configs import TalonFXConfiguration
# from phoenix6.hardware.talon_fx import TalonFX
# from phoenix6.signals.spn_enums import (InvertedValue, NeutralModeValue, FeedbackSensorSourceValue)
# # from ctre import TalonFX, ControlMode
# # import ctre
# class ElevatorSubsystem(SubsystemBase):
#     def __init__(self, motor_id: int, upper_limit_channel: int, lower_limit_channel: int):
#         super().__init__()

        
#         self.__configure_elevator_drive()
        
#     def __configure_elevator_drive(self) -> None:
        
#          self._left_leader_motor = TalonFX(1)     # 1 is the CAN bus address
#        # Applying a new configuration will erase all other config settings since
#        # we start with a blank config so each setting needs to be explicitly set
#        # here in the config method
       
#          config = TalonFXConfiguration()
 

#        # Set the left side motors to be counter clockwise positive
#          config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE


#        # Set the motors to electrically stop instead of coast
#          config.motor_output.neutral_mode = NeutralModeValue.BRAKE


#        # This configuration item supports counting wheel rotations
#        # This item sets the gear ratio between motor turns and wheel turns
#          config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR
#          config.feedback.sensor_to_mechanism_ratio = 10


#        # Apply the configuration to the motors
#          for i in range(6):  # Try 5 times
#            ret = self._left_leader_motor.configurator.apply(config)


#          self._left_leader_motor.set_position(0)    #  Reset the encoder to  zero
      
#     def __configure_right_side_drive(self) -> None:
#        self._right_leader_motor = TalonFX(2)     # 2 is the CAN bus address
#        # Applying a new configuration will erase all other config settings since
#        # we start with a blank config so each setting needs to be explicitly set
#        # here in the config method
      
#        config = TalonFXConfiguration()


#        # Set the left side motors to be counter clockwise positive
#        config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE


#        # Set the motors to electrically stop instead of coast
#        config.motor_output.neutral_mode = NeutralModeValue.BRAKE


#        # This configuration item supports counting wheel rotations
#        # This item sets the gear ratio between motor turns and wheel turns
#        config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR
#        config.feedback.sensor_to_mechanism_ratio = 10


#        # Apply the configuration to the motors
#        for i in range(6):  # Try 5 times
#            ret = self._right_leader_motor.configurator.apply(config)


#        self._left_leader_motor.set_position(0)    #  Reset the encoder to  zero

#         # Falcon 5000 motor controller
#        self.motor = TalonFX(5)

#         # Limit switches
#        self.upper_limit_switch = DigitalInput(5)
#        self.lower_limit_switch = DigitalInput(6)

#         # Motor safety configuration
#        self.motor.configFactoryDefault()
#         # self.motor.setNeutralMode(ctre.NeutralMode.Brake)

#     def move(self, speed: float):
#         """
#         Moves the elevator up or down at the specified speed.

#         :param speed: Speed of the motor (-1.0 to 1.0).
#         """
#         # if speed > 0 and not self.upper_limit_switch.get():
#         #     # Prevent upward movement if the upper limit is hit
#         #     self.motor.set_control(self.speed)
#         #     # self.motor.set(ControlMode.PercentOutput, 0)
#         # elif speed < 0 and not self.lower_limit_switch.get():
#         #     # Prevent downward movement if the lower limit is hit
#         #     self.motor.set_control(self.speed)
#         # else:
#         #     # Normal movement
#         self.motor.set_control(self.speed)

#     def stop(self):
#         """
#         Stops the elevator motor.
#         """
#         self.motor.set_control(0)


#     def at_upper_limit(self):
#         """
#         Checks if the upper limit switch is triggered.
#         """
#         return not self.upper_limit_switch.get()

#     def at_lower_limit(self):
#         """
#         Checks if the lower limit switch is triggered.
#         """
#         return not self.lower_limit_switch.get()
