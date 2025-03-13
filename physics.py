import wpilib.simulation as sim
from wpilib import RobotController, Mechanism2d, SmartDashboard, DriverStation

from wpilib.simulation import DifferentialDrivetrainSim, SimDeviceSim, ElevatorSim, DCMotorSim
from wpimath.kinematics import DifferentialDriveKinematics, DifferentialDriveWheelSpeeds
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations

from pyfrc.physics.core import PhysicsInterface

from phoenix6 import unmanaged, sim

import math

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot

import constants

class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller

        # Keep a reference to the motor sim state so we can update it
        self.left_talon_sim = robot._drivetrain._left_leader.sim_state
        self.right_talon_sim = robot._drivetrain._right_leader.sim_state

        self.wheel_radius = constants.DT_WHEEL_RADIUS_INCHES * constants.METERS_PER_INCH
        self.gear_ratio = constants.DT_GEAR_RATIO
        track_width = constants.DT_TRACKWIDTH_METERS

        self.drivetrain = DifferentialDrivetrainSim(
            DCMotor.krakenX60(2),            # 2 Kraken X60 on each side of the drivetrain
            self.gear_ratio,                    # drivetrain gear ratio
            2.1,                                # MOI of 2.1 kg m^2 (from CAD model)
            constants.ROBOT_MASS,               # Mass of the robot is 26.5 kg
            self.wheel_radius,                  # Robot uses 4" radius wheels
            track_width,                        # Distance between wheels is _ meters.
        )

        self.kinematics = DifferentialDriveKinematics(track_width)

        # Set the orientation of the simulated devices relative to the robot chassis.
        # WPILib expects +V to be forward. Specify orientations to match that behavior.

        # Drivetrain left devices are CCW+
        self.left_talon_sim.orientation = sim.ChassisReference.Clockwise_Positive
        # Drivetrain right devices are CW+
        self.right_talon_sim.orientation = sim.ChassisReference.CounterClockwise_Positive

        # Setup the simulation Gyro tracking
        self._sim_gyro = SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self._sim_gyro.getDouble("Yaw")

        # Add in the elevator Talon for physics simulation
        self.elevator_sim = robot._elevator._ELEVATOR.sim_state
        self.elevator_sim.orientation = sim.ChassisReference.CounterClockwise_Positive
        self.elevator_gearbox: DCMotor = DCMotor.falcon500(1)
        self.elevator: DCMotorSim = DCMotorSim(LinearSystemId.DCMotorSystem(self.elevator_gearbox, 0.02, constants.ELEVATOR_GEAR_RATIO ), self.elevator_gearbox)
        # self.elevator: ElevatorSim = ElevatorSim(
        #     self.elevator_gearbox,
        #     constants.ELEVATOR_GEAR_RATIO,
        #     constants.ELEVATOR_CARRIAGE_MASS,
        #     constants.ELEVATOR_DRUM_RADIUS_M,
        #     constants.ELEVATOR_MIN_HEIGHT_M,
        #     constants.ELEVATOR_MAX_HEIGHT_M,
        #     True,
        #     0,
        #     [0.01, 0.0]
        # )

        # # Create the mechanism visualization
        # self.elev_mech_2d = Mechanism2d(18, 55)
        # self.elev_root = self.elev_mech_2d.getRoot("Elevator Root", 10, 0)
        # self.elev_mech_upper_2d = self.elev_root.appendLigament("Elevator", self.elevator.getPositionInches(), 90)

        # SmartDashboard.putData("Elevator Sim", self.elev_mech_2d)


        # Add Wrist to Physics simulation
        # self.wrist_sim = robot._wrist.wrist_motor.getSimCollection()
        # self.wrist_sim2 = robot._wrist.wrist_motor
        # self.wrist_gearbox: DCMotor = DCMotor.andymark9015()
        # self.wrist: DCMotorSim = DCMotorSim(LinearSystemId.DCMotorSystem(self.wrist_gearbox, 0.02, constants.WRIST_GEAR_RATIO ), self.wrist_gearbox)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        # if DriverStation.isEnabled():
        #     unmanaged.feed_enable(100)

        #-------------- Give Simulation objects voltage -----------------#
        battery_v = RobotController.getBatteryVoltage()
        self.left_talon_sim.set_supply_voltage(battery_v)
        self.right_talon_sim.set_supply_voltage(battery_v)
        self.elevator_sim.set_supply_voltage(battery_v)
        # self.wrist_sim.setBusVoltage(battery_v)

        # ------------------------   Drivetrain Simulation ----------------------------#
        # CTRE simulation is low-level, so SimState inputs
        # and outputs are not affected by user-level inversion.
        # However, inputs and outputs *are* affected by the mechanical
        # orientation of the device relative to the robot chassis,
        # as specified by the `orientation` field.
        #
        # WPILib expects +V to be forward. We have already configured
        # our orientations to match this behavior.
        self.drivetrain.setInputs(self.left_talon_sim.motor_voltage, self.right_talon_sim.motor_voltage)

        # Advance the model by tm_diff
        self.drivetrain.update(tm_diff)

        self.left_talon_sim.set_raw_rotor_position(self.meters_to_rotations(self.drivetrain.getLeftPosition()))
        self.left_talon_sim.set_rotor_velocity(self.meters_to_rotations(self.drivetrain.getLeftVelocity()))
        self.right_talon_sim.set_raw_rotor_position(self.meters_to_rotations(self.drivetrain.getRightPosition()))
        self.right_talon_sim.set_rotor_velocity(self.meters_to_rotations(self.drivetrain.getRightVelocity()))

        self.physics_controller.drive(
            self.kinematics.toChassisSpeeds(
                DifferentialDriveWheelSpeeds(
                    self.drivetrain.getLeftVelocity(),
                    self.drivetrain.getRightVelocity()
                )
            ),
            tm_diff,
        )

        # Update the gyro simulation
        degrees = self.drivetrain.getHeading().degrees()

        # The drive train simulator is CCW positive, and the Navx is not.  So when we set
        # the Yaw value here, negate it to represent the real world Navx as well.
        self.navx_yaw.set(-degrees)

        # ------------------------------- Elevator Simulation -------------------------------------#
        # Update the simulator calculation
        #self.elevator.setInput(0, self.elevator_sim.supply_current)
        self.elevator.setInputVoltage(self.elevator_sim.motor_voltage)
        self.elevator.update(tm_diff)
        # self.elevator_sim.set_raw_rotor_position(self.meters_to_rotations(self.elevator.getPosition()))
        # self.elevator_sim.set_rotor_velocity(self.meters_to_rotations(self.elevator.getVelocity()))
        self.elevator_sim.set_raw_rotor_position(radiansToRotations(self.elevator.getAngularPosition())*constants.ELEVATOR_GEAR_RATIO)
        self.elevator_sim.set_rotor_velocity(radiansToRotations(self.elevator.getAngularVelocity()))
        # self.elev_mech_upper_2d.setLength(self.elevator.getPositionInches())

        # ------------------------------- Wrist Simulation -------------------------------------#
        #      Update the simulator calculation
        # self.wrist.setInputVoltage(self.elevator_sim.motor_voltage)
        # self.wrist.setInputVoltage(self.wrist_sim.setBusVoltage)
        # self.wrist.setInputVoltage(12)
        # self.wrist.update(tm_diff)
        # self.wrist_sim.setAnalogPosition(int(radiansToRotations(self.wrist.getAngularPosition())))
        # self.wrist_sim.setQuadratureRawPosition(100)



    def meters_to_rotations(self, dist: float) -> float:
        circumference = self.wheel_radius * 2.0 * math.pi
        rotations_per_meter = self.gear_ratio / circumference
        return dist * rotations_per_meter

