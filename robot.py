#!/usr/bin/env python3
import math
import wpilib
from wpilib import RobotBase, DriverStation
from commands2 import (
    TimedCommandRobot,
    CommandScheduler,
    Command,
    PrintCommand,
    RunCommand,
    WaitCommand,
    cmd,
)
from commands2.button import CommandXboxController, Trigger, JoystickButton
from wpimath.geometry import Pose2d
from pathplannerlib.auto import (
    NamedCommands,
    PathPlannerAuto,
)
from phoenix6 import SignalLogger
from drivetrain import DriveTrain,  TurnToAnglePID
from intake import Intake, SetIntake
from leds import LEDSubsystem, FlashLEDCommand
####>>> from vision import VisionSystem
from elevator import ELEVATOR, MoveELEVATOR

import constants
from typing import Tuple, List


class MyRobot(TimedCommandRobot):
    """Class that defines the totality of our Robot"""

    def robotInit(self) -> None:
        """
        This method must eventually exit in order to ever have the robot
        code light turn green in DriverStation. So, this will create an
        instance of the Robot that contains all the subsystems,
        button bindings, and operator interface pieces like driver
        dashboards
        """
        # Disable the CTRE signal logger
        SignalLogger.stop()  # Disable for debugging later on

        # Setup the operator interface (typically CommandXboxController)
        self._driver_controller = CommandXboxController(
            constants.CONTROLLER_DRIVER_PORT
        )
        self._partner_controller = CommandXboxController(
            constants.CONTROLLER_PARTNER_PORT
        )

        # Remove the joystick warnings in sim
        if RobotBase.isSimulation():
            DriverStation.silenceJoystickConnectionWarning(True)

        # Instantiate any subystems
        self._drivetrain: DriveTrain = DriveTrain()
        wpilib.SmartDashboard.putData("Drivetrain", self._drivetrain)

        self._intake: Intake = Intake()
        wpilib.SmartDashboard.putData("Intake", self._intake)


        self._leds: LEDSubsystem = LEDSubsystem()

        self._ELEVATOR: ELEVATOR = ELEVATOR()
        wpilib.SmartDashboard.putData("Elevator", self._ELEVATOR)

        # self._vision: VisionSystem = VisionSystem(False, True)
        # self._vision: VisionSystem = VisionSystem(True, True)

        self.__configure_default_commands()

        self.__configure_button_bindings()

        self.__configure_autonomous_commands()

        self.__configure_led_triggers()

        self._auto_command = None
        self._current_pose = Pose2d()

    def __configure_button_bindings(self) -> None:
        # Driver controller controls first
        ####>> self._driver_controller.a().whileTrue(IntakeCommand(self._intake))

        # Left Button Note Aim
        # The WPILIB enum and our controller mapping are different.  On the Zorro
        # controller, the "right bumper" according to WPILib is actually the left
        # button that would be by a trigger
        ####>>>> self._driver_controller.rightBumper().whileTrue(
        #     TeleopDriveWithVision(
        #         self._drivetrain, self._vision.get_note_yaw, self._driver_controller
        #     ).withName("Note Driving")
        # )
        # Right Trigger April Tag
        # Create a button that maps to the proper integer number (found in driverstation)
        self._right_controller_button: JoystickButton = JoystickButton(
            self._driver_controller.getHID(), 13  # TODO -- Assign this correct number
        )
        # ####>>>self._right_controller_button.whileTrue(
        #     TeleopDriveWithVision(
        #         self._drivetrain, self._vision.get_tag_yaw, self._driver_controller
        #     ).withName("Tag Driving")
        # )

        # self._driver_controller.rightBumper().whileTrue(
        #     RunCommand(
        #         lambda: self._drivetrain.drive_teleop(
        #             self._driver_controller.getLeftY(),
        #             -self._driver_controller.getRightX(),
        #         ),
        #         self._drivetrain,
        #     ).withName("FlippedControls")
        # )

        # wpilib.SmartDashboard.putData(
        #     "Turn-90",
        #     self._drivetrain.configure_turn_pid(-90).andThen(
        #         self._drivetrain.turn_with_pid().withName("TurnTo -90"),
        #     ),
        # )

        ######################## Partner controller controls #########################

                # Right Trigger Climber Up
        self._partner_controller.rightTrigger().whileTrue(
            MoveELEVATOR(self._ELEVATOR, 0.4).withName("ElevatorUp")
        )
        # Left Trigger Climber Down
        self._partner_controller.leftTrigger().whileTrue(
            MoveELEVATOR(self._ELEVATOR, -0.4).withName("ElevatorDown")
        )

        wpilib.SmartDashboard.putData("Turn90", TurnToAnglePID(self._drivetrain, 90, 3))
        wpilib.SmartDashboard.putData(
            "Turn-90", TurnToAnglePID(self._drivetrain, -90, 3)
        )

    def __configure_default_commands(self) -> None:
        # Setup the default commands for subsystems
        if wpilib.RobotBase.isSimulation():
            # Set the Drivetrain to arcade drive by default
            self._drivetrain.setDefaultCommand(
                # A split-stick arcade command, with forward/backward controlled by the left
                # hand, and turning controlled by the right.
                RunCommand(
                    lambda: self._drivetrain.drive_teleop(
                        -self._driver_controller.getRawAxis(
                            constants.CONTROLLER_FORWARD_SIM
                        ),
                        -self._driver_controller.getRawAxis(
                            constants.CONTROLLER_TURN_SIM
                        ),
                    ),
                    self._drivetrain,
                ).withName("DefaultDrive")
            )
        else:
            self._drivetrain.setDefaultCommand(
                # A split-stick arcade command, with forward/backward controlled by the left
                # hand, and turning controlled by the right.
                RunCommand(
                    lambda: self._drivetrain.drive_teleop(
                        -self._driver_controller.getLeftY(),
                        -self._driver_controller.getRightX(),
                    ),
                    self._drivetrain,
                ).withName("DefaultDrive")
            )

        self._intake.setDefaultCommand(SetIntake(self._intake))

    def __configure_autonomous_commands(self) -> None:
        # Register the named commands used by the PathPlanner auto builder
        # These commands have to match exactly in the PathPlanner application
        # as we name them here in the registration
        NamedCommands.registerCommand(
            "RunIntake", PrintCommand("This is placeholder for a command that runs the intake")
        )

        # To configure the Autonomous routines use PathPlanner to define the auto routines
        # Then, take all of the path planner created routines and add them to the auto
        # chooser so the drive team can select the starting auto.
        self._auto_chooser: wpilib.SendableChooser = wpilib.SendableChooser()
        self._auto_chooser.setDefaultOption(
            "Print Auto String",PrintCommand("This is the auto shooter command string")
        )
        self._auto_chooser.addOption("ExampleAuto", PathPlannerAuto("ExampleAuto"))

        wpilib.SmartDashboard.putData("AutoChooser", self._auto_chooser)

    def __configure_led_triggers(self) -> None:
        pass

    def getAutonomousCommand(self) -> Command:
        return self._auto_chooser.getSelected()

    def teleopInit(self) -> None:
        if self._auto_command is not None:
            self._auto_command.cancel()

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()

    def autonomousInit(self) -> None:
        # If we're starting on the blue side, offset the Navx angle by 180
        # so 0 degrees points to the right for NWU
        self._drivetrain.set_alliance_offset()
        self._drivetrain.reset_encoders()

        # Set the proper April Tag ID target
        # self._vision.set_target_tag(ShooterPosition.SUBWOOFER_2)
        self._auto_command = self.getAutonomousCommand()

        if self._auto_command is not None:
            self._auto_command.schedule()

    def disabledPeriodic(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def testPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        return super().teleopPeriodic()
