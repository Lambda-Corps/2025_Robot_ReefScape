#!/usr/bin/env python3

#   WHEN YOU GET AN FATAL ERROR RUN: 
#   py -3 -m robotpy installer niweb disable

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
from commands2.button import CommandXboxController, Trigger, JoystickButton, POVButton
import wpilib.interfaces
from wpimath.geometry import Pose2d
from pathplannerlib.auto import (
    NamedCommands,
    PathPlannerAuto,
)
from phoenix6 import SignalLogger
from drivetrain import DriveTrain,  TurnToAnglePID, DriveTime
from intake import Intake,  SetIntakeSpeedandTime, SetIntakeManual
from climber import Climber,  SetClimberSpeedandTime, SetClimberManual
from wrist import (
        SetWristAngleAuto,
        WristControl, 
        # SetWrist, 
        # SetWrist_Manual,
        # Set_Wrist_Angle, 
        # Set_Wrist_Angle_with_PID, 
        Set_Wrist_Angle_manual_and_auto_with_PID,
        Set_Global_Wrist_Angle,
        # Wrist_Dummy_Default_command_Test,
)
from leds import LEDSubsystem, FlashLEDCommand
# from wrist import WristControl, SetWrist, Set_Wrist_Angle, Set_Wrist_Angle_manual_and_auto_with_PID
####>>> from vision import VisionSystem
from elevator import (
    ELEVATOR, 
    CancelElevatorMovement, 
    MoveELEVATOR, 
    MoveELEVATORToSetPoint, 
    MoveELEVATORToZero, 
)

import constants
from typing import Tuple, List

# import wrist


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
            constants.CONTROLLER_DRIVER_PORT        )
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

        self._climber: Climber = Climber()
        wpilib.SmartDashboard.putData("Climber", self._climber)

        self._wrist: WristControl = WristControl()
        wpilib.SmartDashboard.putData("Wrist", self._wrist)

        self._leds: LEDSubsystem = LEDSubsystem()

        self._elevator: ELEVATOR = ELEVATOR()
        wpilib.SmartDashboard.putData("Elevator", self._elevator)

        ####  Get current readings from the PDP to learn about motors   ######
        # Source:  https://github.com/robotpy/examples/blob/main/CANPDP/robot.py
        #
        # Actual readings taken down in TeleOp Periodic far below (near line 500)

        self.pdp = wpilib.PowerDistribution()
        wpilib.SmartDashboard.putData("PDP", self.pdp)
        self.frame_counter = 0

        # self._vision: VisionSystem = VisionSystem(False, True)
        # self._vision: VisionSystem = VisionSystem(True, True)

        self.__configure_default_commands()

        self.__configure_button_bindings()

        self.__configure_autonomous_commands()

        self.__configure_led_triggers()

        self._auto_command = None
        self._current_pose = Pose2d()

    def __configure_button_bindings(self) -> None:
        # ############ Driver controller controls first ##################################
        ####>> self._driver_controller.a().whileTrue(IntakeCommand(self._intake))


        # self._driver_controller.start().onTrue(
        #     MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_BOTTOM)
        # )
        self._driver_controller.start().onTrue(
            MoveELEVATORToZero(self._elevator)
        )

        self._driver_controller.a().onTrue(
            SetWristAngleAuto(self._wrist, 12.849).withTimeout(0).andThen(
                MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_SOURCE)
            )
        )



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
        # self._right_controller_button: JoystickButton = JoystickButton(
        # #     self._driver_controller.getHID(), 13  # TODO -- Assign this correct number
        # )
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
          # Right Joystick Wrist Up/Down
        # self._partner_controller.getLeftY().whileTrue(
        #     MoveELEVATOR(self._elevator, 0.4).withName("ElevatorUp")
        # )

        #=======(elevator controls)===================================
        # POV Up Elevator Up
        # self._partner_controller.povUp().whileTrue(
        #     MoveELEVATOR(self._elevator, 0.4).withName("ElevatorUp")
        # )
        # # POV Down Elevator Down
        # self._partner_controller.povDown().whileTrue(
        #     MoveELEVATOR(self._elevator, -0.4).withName("ElevatorDown")
        # )
       
        # Bumbers can also move the elevator up and down maybe if the POV is not working or is not a good option
        self._partner_controller.povUp().whileTrue(
            MoveELEVATOR(self._elevator, 0.4).withName("ElevatorUp")
        )
        self._partner_controller.povDown().whileTrue(
            MoveELEVATOR(self._elevator, -0.4).withName("ElevatorDown")
        )
        # self._partner_controller.a().onTrue(
        #     SetWristAngleAuto(self._wrist, 12.849).withTimeout(0).andThen(
        #         MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_SOURCE)
        #     )
        # )
        self._partner_controller.rightBumper().onTrue(
             SetWristAngleAuto(self._wrist, 72.681).withTimeout(0).andThen(
                MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_ONE)
            )
        )

        self._partner_controller.rightTrigger().onTrue(
            SetWristAngleAuto(self._wrist, 72.16).withTimeout(0).andThen(
            MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_TWO)
            )
        )

        self._partner_controller.leftTrigger().onTrue(
            SetWristAngleAuto(self._wrist, 64.306).withTimeout(0).andThen(
            MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_THREE)
            )
        )

        self._partner_controller.leftBumper().onTrue(
            SetWristAngleAuto(self._wrist, 45.67).withTimeout(0).andThen(
            MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_FOUR).withTimeout(2.3)
            )
        )

        # self._partner_controller.a().onTrue(
        #     SetWristAngleAuto(self._wrist, 45).withTimeout(5).andThen(
        #         MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_ONE)
        #     )
        # )
        # self._partner_controller.x().onTrue(
        #     SetWristAngleAuto(self._wrist, 45).withTimeout(5).andThen(
        #         MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_THREE)
        #     )
        # )
        # self._partner_controller.y().onTrue(
        #     SetWristAngleAuto(self._wrist, 45).withTimeout(5).andThen(
        #         MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_BOTTOM)
        #     )
        # )
        # self._partner_controller.b().onTrue(
        #     MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_TWO)
        # )
        # self._partner_controller.x().onTrue(
        #     MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_THREE)
        # )
        # self._partner_controller.y().onTrue(
        #     MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_FOUR)
        # )

        # self._partner_controller.start().onTrue(
        #     MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_BOTTOM)
        # )
        # self._partner_controller.start().onTrue(
        #     MoveELEVATORToZero(self._elevator)
        # )
        #=======(Wrist controls)===================================



        #  Third parameter is False so the command does not end.
        self._wrist.setDefaultCommand(Set_Wrist_Angle_manual_and_auto_with_PID(self._wrist, self._partner_controller, False))


        # self._driver_controller.a().onTrue(Set_Global_Wrist_Angle(self._wrist, 0))  # Example target angle
        # self._driver_controller.b().onTrue(Set_Global_Wrist_Angle(self._wrist, 20))  # Example target angle
        # self._driver_controller.x().onTrue(Set_Global_Wrist_Angle(self._wrist, 45))  # Example target angle
        # self._driver_controller.y().onTrue(Set_Global_Wrist_Angle(self._wrist, 60))  # Example target angle


        # self._driver_controller.a().whileTrue(Set_Global_Wrist_Angle(self._wrist, 0))  # Example target angle
        # self._driver_controller.b().whileTrue(Set_Global_Wrist_Angle(self._wrist, 20))  # Example target angle
        # self._driver_controller.x().whileTrue(Set_Global_Wrist_Angle(self._wrist, 45))  # Example target angle
        # self._driver_controller.y().whileTrue(Set_Global_Wrist_Angle(self._wrist, 60))  # Example target angle

        # self._wrist.setDefaultCommand(SetWrist_Manual(self._wrist, self._partner_controller))

        # self._driver_controller.a().whileTrue(Set_Wrist_Angle_with_PID(self._wrist, 0))  # Example target angle
        # self._driver_controller.b().whileTrue(Set_Wrist_Angle_with_PID(self._wrist, 20))  # Example target angle
        # self._driver_controller.x().whileTrue(Set_Wrist_Angle_with_PID(self._wrist, 45))  # Example target angle
        # self._driver_controller.y().whileTrue(Set_Wrist_Angle_with_PID(self._wrist, 60))  # Example target angle

        #=======(Intake controls)===================================

        #self._intake.setDefaultCommand(SetIntakeManual(self._intake, self._partner_controller))

        self._partner_controller.a().whileTrue(
            SetIntakeManual(self._intake, 1.0).withName("IntakeIn")
        )
        self._partner_controller.y().whileTrue(
            SetIntakeManual(self._intake, -1.0).withName("IntakeOut")
        )

        # Intake in for one second when a button is pushed
        # self._partner_controller.b().onTrue(
        #     SetIntakeSpeedandTime(self._intake, -1, 1)
        # )
        # # Intake out for one second when a button is pushed
        # self._partner_controller.a().onTrue(
        #     SetIntakeSpeedandTime(self._intake, 1, 1)
        # )


        # wpilib.SmartDashboard.putData("Turn90", TurnToAnglePID(self._drivetrain, 90, 3))
        # wpilib.SmartDashboard.putData(
        #     "Turn-90", TurnToAnglePID(self._drivetrain, -90, 3)
        # )



        #=======(Climber controls)===================================

        self._partner_controller.b().whileTrue(
            SetClimberManual(self._climber, 1.0).withName("ClimberUP")
        )
        self._partner_controller.x().whileTrue(
            SetClimberManual(self._climber, -1.0).withName("ClimberDOWN")
        )

        #=======(Drivetrain controls)===================================

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


    def __configure_autonomous_commands(self) -> None:
        # Register the named commands used by the PathPlanner auto builder
        # These commands have to match exactly in the PathPlanner application
        # as we name them here in the registration

        #===( Elevator Named commands [needed for PathPlanner])==============
        NamedCommands.registerCommand(
            "RaiseElevator", PrintCommand("RaiseElevator")
            )
        
        # NamedCommands.registerCommand(
        #     "Move_Elevator_L3",PrintCommand("Move_Elevator_L3")
        #     )
        
        NamedCommands.registerCommand(
            "ElevatorToL1", MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_ONE).withTimeout(5)
            )
 
        NamedCommands.registerCommand(
            "ElevatorToL2", MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_TWO).withTimeout(5)
            )

        NamedCommands.registerCommand(
            "ElevatorToL3", MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_THREE).withTimeout(5)
            )

        NamedCommands.registerCommand(
            "ElevatorToL4", MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_FOUR).withTimeout(5)
            )
        NamedCommands.registerCommand(
            "ElevatorToBottom", MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_BOTTOM).withTimeout(5)
            )

        #===(Wrist Named Commands)====================================
        
        NamedCommands.registerCommand(
            "RunWrist",PrintCommand("RunWrist")
            )  
        #  The wrist 0 degree position is high
        #  The wrist 90 degree position is low

        NamedCommands.registerCommand(
            "WristToHighPosition", SetWristAngleAuto(self._wrist, 0).withTimeout(3)
        )    
         
        NamedCommands.registerCommand(
            "WristToMediumPosition", SetWristAngleAuto(self._wrist, 30).withTimeout(3)
        )   
        
        NamedCommands.registerCommand(
            "WristToLowPosition", SetWristAngleAuto(self._wrist, 50).withTimeout(3)
        )        
        
        #===(Intake Named Commands)====================================

        NamedCommands.registerCommand(
            "AutoIntake", PrintCommand("AutoIntake")  # Added to quiet down startup error
            )
        
        NamedCommands.registerCommand(
            "RunIntake", PrintCommand("RunIntake")
            )
        
        NamedCommands.registerCommand(
            "IntakeInFor1Second", SetIntakeSpeedandTime(self._intake,-1,1)
            )

        NamedCommands.registerCommand(
            "IntakeOutFor1Second", SetIntakeSpeedandTime(self._intake,1,1)
            )
        
        NamedCommands.registerCommand(
            "IntakeOutFor5Seconds", SetIntakeSpeedandTime(self._intake,5,5)
        )

# ==========Drivetrain name command========================
        NamedCommands.registerCommand(
            "DriveForward6", DriveTime(self._drivetrain,0.2,0.5)
        )

        NamedCommands.registerCommand(
            "DriveBackward6", DriveTime(self._drivetrain,-0.2,0.5)
        )

        NamedCommands.registerCommand(
            "SitStill", DriveTime(self._drivetrain,0,1)
        )
        
        
        # To configure the Autonomous routines use PathPlanner to define the auto routines
        # Then, take all of the path planner created routines and add them to the auto
        # chooser so the drive team can select the starting auto.   
        self._auto_chooser: wpilib.SendableChooser = wpilib.SendableChooser()
        self._auto_chooser.setDefaultOption(
            "TestDrive",PathPlannerAuto("TestDrive")
        )
        self._auto_chooser.addOption("TestSubSystems", PathPlannerAuto("TestSubSystems"))
        # self._auto_chooser.addOption("Best Auto",PathPlannerAuto("Best Auto"))
        self._auto_chooser.addOption("Mid Auto", PathPlannerAuto("Mid Auto"))
        self._auto_chooser.addOption("StraightPath", PathPlannerAuto("StraightPath"))
        self._auto_chooser.addOption("DragonL4", PathPlannerAuto("DragonL4"))

        self._auto_chooser.addOption("BabyDragon", PathPlannerAuto("BabyDragon"))

        self._auto_chooser.addOption("NuNuL1", PathPlannerAuto("NuNuL1"))

        wpilib.SmartDashboard.putData("AutoChooser", self._auto_chooser)


    def __configure_led_triggers(self) -> None:
        pass

    def getAutonomousCommand(self) -> Command:
        return self._auto_chooser.getSelected()

    def teleopInit(self) -> None:
        self.frame_counter = 0    # Used in periodic printing of motor currents below in teleop periodic

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

        # The PDP returns the current in increments of 0.125A.
        
        ## Consider using Partner "Start"  button to control printing of current values

        # if (self._partner_controller.start().getAsBoolean()):
        if (constants.PRINT_CURRENT_MEASURMENTS):     

            if (True):     # Allow Selective Printing of specific items
                elevator_motor_current = self.pdp.getCurrent(constants.ELEVATOR_PDP_CHANNEL)
                wpilib.SmartDashboard.putNumber("elevator_motor_current", elevator_motor_current)

                self.frame_counter = self.frame_counter + 1
                if (self.frame_counter >= 1):                  # Print once every half second to the console
                    if (elevator_motor_current > 0 ):
                        print (f"Elevator_motor_current {elevator_motor_current:3.2f}   {wpilib.Timer.getFPGATimestamp():3.4f}")
                    self.frame_counter = 0

            if (False):     # Allow Selective Printing of specific items
                climber_motor_current = self.pdp.getCurrent(constants.ClIMBER_MOTOR_PDP_CHANNEL)
                wpilib.SmartDashboard.putNumber("climber_motor_current", climber_motor_current)

                self.frame_counter = self.frame_counter + 1
                if (self.frame_counter >= 25):                  # Print once every half second to the console
                    print (f"Climber_motor_current {climber_motor_current:.1f}")
                    self.frame_counter = 0


            if (False):     # Allow Selective Printing of specific items
                intake_motor_current = self.pdp.getCurrent(constants.INTAKE_MOTOR_PDP_CHANNEL)
                wpilib.SmartDashboard.putNumber("elevator_motor_current", intake_motor_current)

                self.frame_counter = self.frame_counter + 1
                if (self.frame_counter >= 25):                  # Print once every half second to the console
                    print (f"Intake_motor_current {intake_motor_current:.1f}")
                    self.frame_counter = 0


            if (False):     # Allow Selective Printing of specific items
                wrist_motor_current = self.pdp.getCurrent(constants.WRIST_MOTOR_PDP_CHANNEL)
                wpilib.SmartDashboard.putNumber("wrist_motor_current", wrist_motor_current)

                self.frame_counter = self.frame_counter + 1
                if (self.frame_counter >= 5):                  # Print five times second to the console
                    print (f"Wrist_motor_current {wrist_motor_current:.1f}")
                    self.frame_counter = 0

        return super().teleopPeriodic()
