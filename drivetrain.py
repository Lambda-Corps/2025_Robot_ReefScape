import math
from commands2.button import CommandXboxController
import wpilib.drive
from wpilib import RobotBase, DriverStation
import wpilib.simulation
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    DifferentialDriveOdometry,
    DifferentialDriveKinematics,
    ChassisSpeeds,
    DifferentialDriveWheelSpeeds,
)
from wpimath.filter import SlewRateLimiter
from wpilib import SmartDashboard, Field2d
from commands2 import Subsystem, Command, cmd, PIDCommand
from phoenix6 import StatusCode
from phoenix6.configs import (
    TalonFXConfiguration,
    Slot0Configs,
)
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls.follower import Follower
from phoenix6.signals.spn_enums import (
    InvertedValue,
    NeutralModeValue,
    FeedbackSensorSourceValue,
)
from phoenix6.controls import (
    DutyCycleOut,
    VoltageOut,
    MotionMagicVoltage,
)
import navx
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPLTVController
from pathplannerlib.config import RobotConfig

from typing import Callable
import constants

VISION_KP = 0.012
FEEDFORWARD = 0.1
FOLLOWER_MOTORS_PRESENT = False


#===(Hardware Notes)==============================================
'''
The drivetrain uses two Kraken motors. (One on each side)
The krakens have internal rotation counter to monitor robot motion.
The RoboRIO has a NAVX gyro to monitor robot heading.

The autonomous drivetrain performance is controlled by Q and R elements 
See about line 140.

The teleOp drivetrain performance is controlled by Slew Rate Limiters.
See aboutl ine 70

'''
#================================================================


class DriveTrain(Subsystem):
    __DRIVER_DEADBAND = 0.1
    __LEVEL0_SLEW = 3  # .33 seconds to full speed
    __LEVEL1_SLEW = 2.6 # .38 seconds to full speed
    __LEVEL2_SLEW = 2.2 # .45 seconds to full speed
    __LEVEL3_SLEW = 1.8 # .55 seconds to full speed
    __LEVEL4_SLEW = 1.4 # .70 seconds to full speed
    __CLAMP_SPEED = 1.0
    __TURN_PID_SPEED = 0.3

    def __init__(self, test_mode=False) -> None:
        super().__init__()
        self._gyro: navx.AHRS = navx.AHRS.create_spi()

        # Create the output objects for the talons, currently one each for
        # the following modes: VoltageOut, PercentOutput, and MotionMagic
        self.__create_output_objects()

        # Create the PID controller setup for turning
        self.__create_turn_pid_objects()

        # Create the FF and PID for paths
        self.__create_path_pid_objects()

        # Apply all the configurations to the left and right side Talons
        self.__configure_left_side_drive()
        self.__configure_right_side_drive()

        # Create the Odometry tracker
        self._odometry: DifferentialDriveOdometry = DifferentialDriveOdometry(
            self._gyro.getRotation2d(), 0, 0
        )

        self._kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(
            constants.DT_TRACKWIDTH_METERS
        )

        SmartDashboard.putData("Navx", self._gyro)

        self._base_fwd_limiter:   SlewRateLimiter = SlewRateLimiter(self.__LEVEL0_SLEW)
        self._level1_fwd_limiter: SlewRateLimiter = SlewRateLimiter(self.__LEVEL1_SLEW)
        self._level2_fwd_limiter: SlewRateLimiter = SlewRateLimiter(self.__LEVEL2_SLEW)
        self._level3_fwd_limiter: SlewRateLimiter = SlewRateLimiter(self.__LEVEL3_SLEW)
        self._level4_fwd_limiter: SlewRateLimiter = SlewRateLimiter(self.__LEVEL4_SLEW)

        self._test_mode = test_mode
        if self._test_mode:
            SmartDashboard.putNumber("ClampSpeed", 0.3)

        # Setup the autonomous configuration for Pathplanner
        # increasing Qelems numbers, tries to drive more conservatively as the effect
        # In the math, what we're doing is weighting the error less heavily, meaning,
        # as the error gets larger don't react as much.  This makes the robot drive
        # conservatively along the path.
        # Decreasing Relems should make the motors drive less aggressively (fewer volts)
        # In the math, this is the same as increasing Q values.  Basically, think of it
        # like a car, if you limit how far you can press the gas pedal, a driver
        # has a better chance of keeping the car under control
        # Down below, in comments, there are a few candidate values that have been used
        # under testing.  Tweak, and test, to find the right ones.
        # [0.0625, 0.125, 2.5],  # <-- Q Elements
        # [0.075, 0.15, 3.1],
        # [0.09, 0.19, 3.7],
        # [0.125, 2.5, 5.0],
        # [0.19, 3.75, 7.5],
        # [2.5, 5.0, 10.0],
        # current [-5, 5],  # <-- R elements
        # [-8, 8],
        # [-10, 10],
        # [-11, 11],
        # [-12, 12],
        ltv_q_elems = [0.09, 0.19, 3.7]
        ltv_r_elems = [-9, 9]
        if RobotBase.isSimulation():
            ltv_q_elems = [0.09, 0.19, 3.7]
            ltv_r_elems = [-10, 10]

        config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.get_robot_pose,
            self.reset_odometry,
            self.get_wheel_speeds,  # Current ChassisSpeeds supplier
            lambda speeds, feedforwards: self.driveSpeeds(speeds), # Method that will drive the robot given ChassisSpeeds
            PPLTVController(ltv_q_elems, ltv_r_elems, 0.02 ),
            config,
            self.should_flip_path,  # Flip if we're on the red side
            self,  # Reference to this subsystem to set requirements
        )

        self._field = Field2d()
        SmartDashboard.putData("MyField", self._field)

        # Maintain knowledge of where the elevator position is, so we can adjust the 
        # acceleration constraints accordingly
        self._elevator_pos: constants.ElevatorPosition = constants.ElevatorPosition.LEVEL_UKNOWN


    def __configure_motion_magic(self, config: TalonFXConfiguration) -> None:
        self._mm_setpoint = 0

        self._mm_tolerance = (
            math.pi * constants.DT_WHEEL_DIAMETER
        ) / 50  # end within 1/50th of a rotation
        config.motion_magic.motion_magic_cruise_velocity = (
            10  # 1 rps = 1.5 fps, 10 rps = 15 fps
        )
        config.motion_magic.motion_magic_acceleration = (
            20  # .5 seconds to reach full speed
        )

        # Motion Magic slot will be 0
        slot_0: Slot0Configs = config.slot0
        if RobotBase.isSimulation():
            slot_0.k_p = 1.2  # 1 full wheel rotation will correct with 1.2v power
            slot_0.k_s = 0.01  # 1 percent maybe accounts for friction?
        else:
            # TODO -- Tune these on the robot
            slot_0.k_p = 12  # 1 full wheel rotation will correct with 12 volts
            slot_0.k_s = 0

    def __create_output_objects(self) -> None:
        self._left_volts_out: VoltageOut = VoltageOut(0, enable_foc=False)
        # self._left_volts_out.update_freq_hz = 0
        self._right_volts_out: VoltageOut = VoltageOut(0, enable_foc=False)
        # self._right_volts_out.update_freq_hz = 0

        self._left_percent_out: DutyCycleOut = DutyCycleOut(0, enable_foc=False)
        self._right_percent_out: DutyCycleOut = DutyCycleOut(0, enable_foc=False)

        self._mm_out: MotionMagicVoltage = MotionMagicVoltage(0, enable_foc=False)

    def __create_turn_pid_objects(self) -> None:
        self._turn_setpoint = 0
        self._turn_tolerance = 1  # within 3 degrees we'll call good enough
        if RobotBase.isSimulation():
            self._turn_pid_controller: PIDController = PIDController(0.002, 0.0, 0.0001)
            self._turn_kF = 0.049
        else:
            # These must be tuned
            self._turn_pid_controller: PIDController = PIDController(0.02, 0, 0.001)
            self._turn_kF = 0.1  # TODO Tune me

        self._turn_pid_controller.enableContinuousInput(-180, 180)
        self._turn_pid_controller.setTolerance(1)
        SmartDashboard.putData("Turn PID", self._turn_pid_controller)

    def __create_path_pid_objects(self) -> None:
        if RobotBase.isSimulation():
            self._path_left_pid_controller: PIDController = PIDController(0.002, 0, 0)
            self._path_right_pid_controller: PIDController = PIDController(0.002, 0, 0)
            self._path_feedforward: SimpleMotorFeedforwardMeters = (
                SimpleMotorFeedforwardMeters(
                    constants.DT_KS_VOLTS_SIM, constants.DT_KV_VOLTSECONDS_METER_SIM, 0
                )
            )
        else:
            self._path_left_pid_controller: PIDController = PIDController(
                0.01, 0, 0.001
            )
            self._path_right_pid_controller: PIDController = PIDController(
                0.01, 0, 0.001
            )
            self._path_feedforward: SimpleMotorFeedforwardMeters = (
                SimpleMotorFeedforwardMeters(
                    constants.DT_KS_VOLTS, constants.DT_KV_VOLTSECONDS_METER, 0
                )
            )

    def __configure_left_side_drive(self) -> None:
        self._left_leader = TalonFX(constants.DT_LEFT_LEADER)

        # Applying a new configuration will erase all other config settings since we start with a blank config
        # so each setting needs to be explicitly set here in the config method
        config = TalonFXConfiguration()

        # Set the left side motors to be counter clockwise positive
        config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        # Set the motors to electrically stop instead of coast
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # PID controls will use integrated encoder
        config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR

        config.feedback.sensor_to_mechanism_ratio = constants.DT_GEAR_RATIO

        # Configure the motion magic objects and store them as member variables
        self.__configure_motion_magic(config)

        # Setup current limits for supply and stator
        config.current_limits.supply_current_limit = 40
        config.current_limits.supply_current_limit_enable = True
        # 1 second of spikes before limiting # TODO Tune this
        config.current_limits.supply_current_threshold = 1.0
        config.current_limits.stator_current_limit = 80
        config.current_limits.stator_current_limit_enable = False

        # Apply the configuration to the motors
        for i in range(0, 6):  # Try 5 times
            ret = self._left_leader.configurator.apply(config)
            if ret == StatusCode.is_ok:
                break

        self._left_leader.set_position(0)
        
        # Setup the followers if we have them
        if FOLLOWER_MOTORS_PRESENT:
            self._left_follower = TalonFX(constants.DT_LEFT_FOLLOWER)

            for i in range(0, 6):  # Try 5 times
                ret = self._left_follower.configurator.apply(config)
                if ret == StatusCode.is_ok:
                    break

            # Set the left follower to only follow master
            follow_request = Follower(constants.DT_LEFT_LEADER, False)
            self._left_follower.set_control(follow_request)



    def __configure_right_side_drive(self) -> None:
        self._right_leader = TalonFX(constants.DT_RIGHT_LEADER)
        # Applying a new configuration will erase all other config settings since we start with a blank config
        # so each setting needs to be explicitly set here in the config method
        config = TalonFXConfiguration()

        # Set the left side motors to be counter clockwise positive
        config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        # Set the motors to electrically stop instead of coast
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # PID controls will use integrated encoder
        config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR

        # Configure the motion magic objects and store them as member variables
        self.__configure_motion_magic(config)

        config.feedback.sensor_to_mechanism_ratio = constants.DT_GEAR_RATIO

        # Setup current limits for supply and stator
        config.current_limits.supply_current_limit = 40
        config.current_limits.supply_current_limit_enable = True
        # 1 second of spikes before limiting # TODO Tune this
        config.current_limits.supply_current_threshold = 1.0
        config.current_limits.stator_current_limit = 80
        config.current_limits.stator_current_limit_enable = False

        # Apply the configuration to the motors
        for i in range(0, 6):  # Try 5 times
            ret = self._right_leader.configurator.apply(config)
            if ret == StatusCode.is_ok:
                break

        self._right_leader.set_position(0)

        if FOLLOWER_MOTORS_PRESENT:
            self._right_follower = TalonFX(constants.DT_RIGHT_FOLLOWER)

            for i in range(0, 6):  # Try 5 times
                ret = self._right_follower.configurator.apply(config)
                if ret == StatusCode.is_ok:
                    break

            # Set the right side follower to go with leader
            follow_request = Follower(constants.DT_RIGHT_LEADER, False)
            self._right_follower.set_control(follow_request)



    def configure_motion_magic(self, distance_in_inches: float) -> None:
        """
        Method to configure the motors using a MotionMagic profile.

        :param: distance_in_inches  Distance in inches to travel.
        """
        # The TalonFX configuration already sets up the SensorToMechanism ratio when
        # it accounts for the position feedback.  So, when the talon.get_position()
        # method is called, it returns the roations of the wheel's output shaft, not
        # the input shaft of the gearbox.  So, if we were to use a setpoint of 1 (1 rotation)
        # in the MotionMagic profile, it would try to turn the wheels 1 rotation, not
        # the motor.

        # Convert the distance in inches, to wheel rotations
        #                          distance_in_inches
        # wheel_rotations =   --------------------------
        #                        Pi * Wheel Diameter
        distance_in_rotations = distance_in_inches / (
            math.pi * constants.DT_WHEEL_DIAMETER
        )
        curr_right = self._right_leader.get_position().value_as_double

        self._mm_out.with_position(curr_right + distance_in_rotations).with_slot(0)

    ########################### Drivetrain Drive methods #######################
    def __get_slewrate_limited_fwd_speed(self, forward: float) -> float:
        match self._elevator_pos:
            case constants.ElevatorPosition.LEVEL_BOTTOM:
                forward = self._base_fwd_limiter.calculate(forward)
            case constants.ElevatorPosition.LEVEL_ONE:
                forward = self._level1_fwd_limiter.calculate(forward)
            case constants.ElevatorPosition.LEVEL_TWO:
                forward = self._level2_fwd_limiter.calculate(forward)
            case constants.ElevatorPosition.LEVEL_THREE:
                forward = self._level3_fwd_limiter.calculate(forward)
            case constants.ElevatorPosition.LEVEL_FOUR:
                forward = self._level4_fwd_limiter.calculate(forward)
            case constants.ElevatorPosition.LEVEL_UKNOWN:
                forward = self._base_fwd_limiter.calculate(forward)
            case _: 
                # This is the default case in the event nothing matches
                forward = self._base_fwd_limiter.calculate(forward)

        return forward
            
    def drive_teleop(self, forward: float, turn: float, percent_out=False):
        if self._test_mode:
            self.__CLAMP_SPEED = SmartDashboard.getNumber("ClampSpeed", 0.3)
        forward = self.__deadband(forward, self.__DRIVER_DEADBAND)
        turn = self.__deadband(turn, self.__DRIVER_DEADBAND)

        turn = self.__clamp(turn, self.__CLAMP_SPEED)
        forward = self.__clamp(forward, self.__CLAMP_SPEED)

        # Limit the acceleration based on the elevator position
        forward = self.__get_slewrate_limited_fwd_speed(forward)

        if percent_out:
            self.__drive_teleop_percent(forward, turn)
        else:
            self.__drive_teleop_volts(forward, turn)

    def drive_pid_turn(self, turn: float) -> None:
        if turn < 0:
            # Turning right
            if turn > -self._turn_kF:
                turn = -self._turn_kF
        elif turn > 0:
            # Turning left
            if turn < self._turn_kF:
                turn = self._turn_kF

        self.__drive_teleop_volts(0, turn)

    def __drive_teleop_volts(self, forward: float, turn: float) -> None:

        speeds = wpilib.drive.DifferentialDrive.curvatureDriveIK(forward, turn, True)

        self._left_volts_out.output = speeds.left * 12.0
        self._right_volts_out.output = speeds.right * 12.0

        self._left_leader.set_control(self._left_volts_out)
        self._right_leader.set_control(self._right_volts_out)

    def __drive_teleop_percent(self, forward: float, turn: float) -> None:

        speeds = wpilib.drive.DifferentialDrive.curvatureDriveIK(forward, turn, True)

        self._left_percent_out.output = speeds.left
        self._right_percent_out.output = speeds.right

        self._left_leader.set_control(self._left_percent_out)
        self._right_leader.set_control(self._right_percent_out)

    def drive_velocity_volts(self, left: float, right: float) -> None:
        left_pid = self._path_left_pid_controller.calculate(
            self._left_leader.get_velocity().value_as_double * constants.DT_GEAR_RATIO,
            left,
        )
        right_pid = self._path_right_pid_controller.calculate(
            self._right_leader.get_velocity().value_as_double * constants.DT_GEAR_RATIO,
            right,
        )
        self._left_volts_out.output = self._path_feedforward.calculate(left) + left_pid
        self._right_volts_out.output = (
            self._path_feedforward.calculate(right) + right_pid
        )

        if self._test_mode:
            SmartDashboard.putNumber("LeftPID", left_pid)
            SmartDashboard.putNumber("RightPID", right_pid)
            SmartDashboard.putNumber("LeftSpeed", left)
            SmartDashboard.putNumber("RightSpeed", right)

        self._left_leader.set_control(self._left_volts_out)
        self._right_leader.set_control(self._right_volts_out)

    def driveSpeeds(self, speeds: ChassisSpeeds) -> None:
        speeds: DifferentialDriveWheelSpeeds = self._kinematics.toWheelSpeeds(speeds)
        self.drive_velocity_volts(speeds.left, speeds.right)

    def drive_motion_magic(self) -> None:
        self._left_leader.set_control(Follower(self._right_leader.device_id, False))
        self._right_leader.set_control(self._mm_out)

    def turn_ccw_positive(self, speed: float) -> None:
        if RobotBase.isSimulation():
            speed = self.__clamp(speed, 0.05)
            wpilib.SmartDashboard.putNumber("TurnSpeed", speed)
        else:
            speed = self.__clamp(speed, self.__TURN_PID_SPEED)

        if speed < 0:
            # Turn CCW
            left_speed = -speed
            right_speed = speed
        elif speed > 0:
            left_speed = speed
            right_speed = -speed
        else:
            left_speed = 0
            right_speed = 0

        self._left_percent_out.output = left_speed
        self._right_percent_out.output = right_speed

        self._left_leader.set_control(self._left_percent_out)
        self._right_leader.set_control(self._right_percent_out)

    def __turn_with_pid(self) -> None:
        curr_angle = self.__get_gyro_heading()
        setpoint = self._turn_pid_controller.getSetpoint()

        pidoutput = self._turn_pid_controller.calculate(curr_angle)

        # if curr_angle > setpoint and pidoutput > 0:
        #     # We need to turn right, which is -
        #     pidoutput *= -1
        # elif curr_angle < setpoint and pidoutput < 0:
        #     # We need to turn left, which - from here
        #     pidoutput *= -1

        # Promote the value to at least the KF
        if (pidoutput < 0) and (pidoutput > -self._turn_kF):
            pidoutput = -self._turn_kF
        elif (pidoutput > 0) and (pidoutput < self._turn_kF):
            pidoutput = self._turn_kF

        if RobotBase.isSimulation():
            wpilib.SmartDashboard.putNumber("TurnPIDOut", pidoutput)
        self.turn_ccw_positive(pidoutput)

    ################## Drive train Helpers ##########################

    def __deadband(self, input: float, abs_min: float) -> float:
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

    def __clamp(self, input: float, abs_max: float) -> None:
        """
        Set a max speed for a given input
        """
        if input < 0 and input < (abs_max * -1):
            input = abs_max * -1
        elif input > 0 and input > abs_max:
            input = abs_max

        return input

    def at_mm_setpoint(self) -> bool:
        curr_right = self._right_leader.get_position().value

        return abs(self._mm_out.position - curr_right) < self._mm_tolerance

    def __config_turn_command(self, desired_angle: float) -> None:
        # self._turn_setpoint = desired_angle + self._gyro.getYaw()
        self._turn_setpoint = desired_angle
        self._turn_pid_controller.setSetpoint(self._turn_setpoint)
        self._turn_pid_controller.setTolerance(self._turn_tolerance)
        wpilib.SmartDashboard.putNumber("Turn Setpoint", self._turn_setpoint)

    def __at_turn_setpoint(self) -> bool:
        # curr_angle = self._gyro.getYaw()

        # wpilib.SmartDashboard.putBoolean(
        #     "At Setpoint", self._turn_pid_controller.atSetpoint()
        # )
        # return abs(curr_angle - self._turn_setpoint) < self._turn_tolerance
        return self._turn_pid_controller.atSetpoint()

    def getHeading(self) -> float:
        return self.__get_gyro_heading()

    def __get_gyro_heading(self) -> float:
        angle = math.fmod(-self._gyro.getAngle(), 360)

        if angle < 0:
            return angle if angle >= -180 else angle + 360
        else:
            return angle if angle <= 180 else angle - 360


    def get_robot_pose(self) -> Pose2d:
        return self._odometry.getPose()

    def get_wheel_speeds(self) -> ChassisSpeeds:
        diff_speed: DifferentialDriveWheelSpeeds = DifferentialDriveWheelSpeeds(
            self.__rps_to_mps(self._left_leader.get_velocity().value_as_double),
            self.__rps_to_mps(self._right_leader.get_velocity().value_as_double),
        )
        return self._kinematics.toChassisSpeeds(diff_speed)

    def reset_odometry(self, pose: Pose2d) -> None:
        self._odometry.resetPosition(
            self._gyro.getRotation2d(),
            0,
            0,
            pose,
        )

    def reset_encoders(self) -> None:
        self._left_leader.set_position(0)
        self._right_leader.set_position(0)

    def reset_drivetrain(self) -> None:
        self._gyro.setAngleAdjustment(0)
        self._gyro.reset()

        self.reset_encoders()

    def set_alliance_offset(self) -> None:
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self._gyro.setAngleAdjustment(180)
        else:
            self._gyro.setAngleAdjustment(0)

    ################### Periodic Updates for the Subsystems ######################

    def periodic(self) -> None:
        # SmartDashboard.putNumber("LeftEncoder", self._left_leader.get_position().value)
        # SmartDashboard.putNumber(
        #     "RightEncoder", self._right_leader.get_position().value
        # )

        pose = self._odometry.update(
            Rotation2d().fromDegrees(self.__get_gyro_heading()),
            self.__rotations_to_meters(
                self._left_leader.get_position().value_as_double
            ),
            self.__rotations_to_meters(
                self._right_leader.get_position().value_as_double
            ),
        )

        self._field.setRobotPose(pose)

        SmartDashboard.putNumber("Gyro CCW Angle", self.__get_gyro_heading())
        SmartDashboard.putNumber("LeftVelOut", self._left_volts_out.output)
        SmartDashboard.putNumber("rightVelOut", self._right_volts_out.output)

        # Get the elevator position so we can limit the acceleration
        elev_pos_val: float = SmartDashboard.getNumber("Elevator_Position", 0.0)
        self._elevator_pos = constants.get_closest_elevator_position(elev_pos_val)

    ############# Drivetrain Odometry methods ###################

    def __rotations_to_meters(self, rotations: float) -> float:
        return rotations * constants.DT_WHEEL_CIRCUMFERENCE_METERS

    def should_flip_path(self) -> bool:
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def __rps_to_mps(self, rotations: float) -> float:
        return rotations * constants.DT_WHEEL_CIRCUMFERENCE_METERS

    ################################# Drivetrain Command Methods ############################
    def mm_drive_distance(self) -> Command:
        return (
            cmd.run(lambda: self.drive_motion_magic(), self)
            .until(lambda: self.at_mm_setpoint())
            .withName("DriveMM")
        )

    def mm_drive_config(self, distance_in_inches: float) -> Command:
        return cmd.runOnce(lambda: self.configure_motion_magic(distance_in_inches))

    def configure_turn_pid(self, desired_angle: float) -> Command:
        return cmd.runOnce(lambda: self.__config_turn_command(desired_angle))

    def turn_with_pid(self) -> Command:
        return (
            cmd.run(lambda: self.__turn_with_pid(), self)
            .until(lambda: self.__at_turn_setpoint())
            .withName("TurnWithPID")
        )


class DriveMMInches(Command):
    def __init__(self, dt: DriveTrain, distance_in_inches: float) -> None:
        super().__init__()

        self._dt = dt
        self._desired_distance = distance_in_inches

        # Tell the scheduler this requires the drivetrain
        self.addRequirements(self._dt)

    def initialize(self):
        self._dt.configure_motion_magic(self._desired_distance)

    def execute(self):
        self._dt.drive_motion_magic()

    def isFinished(self) -> bool:
        return self._dt.at_mm_setpoint()

    def end(self, interrupted: bool):
        self._dt.drive_velocity_volts(0, 0)


class TeleopDriveWithVision(Command):
    def __init__(
        self,
        dt: DriveTrain,
        _yaw_getter: Callable[[], float],
        controller: CommandXboxController,
        flipped_controls=False,
    ):
        self._dt = dt
        self._yaw_getter = _yaw_getter
        self._controller = controller
        self._flipped = flipped_controls
        SmartDashboard.putNumber("VisionKP", 0.012)
        SmartDashboard.putNumber("VisionFF", 0.1)
        self.addRequirements(self._dt)

    def execute(self):
        forward = -self._controller.getLeftY()
        if self._flipped:
            # Invert the translation
            forward *= -1
        yaw: float = self._yaw_getter()
        if 1000 == yaw:
            # We didn't get a result, use the joystick
            if RobotBase.isSimulation:
                yaw = -self._controller.getRawAxis(constants.CONTROLLER_TURN_SIM)
            else:
                yaw = -self._controller.getRightX()
        else:
            yaw = self._calculate_yaw(yaw)

        SmartDashboard.putNumber("Yaw", yaw)
        self._dt.drive_teleop(forward, yaw)

    def isFinished(self) -> bool:
        # Should only run while button is held, return False
        return False

    def _calculate_yaw(self, yaw: float) -> float:
        yaw = -yaw * VISION_KP

        if yaw < 0:
            yaw = yaw - FEEDFORWARD
        elif yaw > 0:
            yaw += FEEDFORWARD

        return yaw


class TurnToAnglePID(PIDCommand):
    def __init__(self, dt: DriveTrain, angle: float, timeout=2):
        super().__init__(
            dt._turn_pid_controller,
            dt.getHeading,
            angle,
            lambda output: dt.drive_pid_turn(output),
            dt,
        )

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return self.getController().atSetpoint()
