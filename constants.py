import math
from enum import Enum

METERS_PER_INCH = 0.0254
INCHES_PER_METER = 39.37

# Robot Physical Characteristics with dimensional units
ROBOT_MASS = 80
DT_GEAR_RATIO = (60 / 13) * (34 / 18)
DT_MOTORS_PER_SIDE = 2
DT_WHEEL_DIAMETER = 4
DT_WHEEL_RADIUS_INCHES = DT_WHEEL_DIAMETER / 2
DT_TRACKWIDTH_METERS = 0.546
DT_TICKS_PER_MOTOR_REV = int(2048)
DT_TICKS_PER_INCH = (DT_TICKS_PER_MOTOR_REV * DT_GEAR_RATIO) / (
    (2 * math.pi) * DT_WHEEL_DIAMETER
)
DT_TICKS_PER_METER = (DT_TICKS_PER_MOTOR_REV * DT_GEAR_RATIO) / (
    (2 * math.pi) * DT_WHEEL_DIAMETER
)
DT_WHEEL_CIRCUMFERENCE_METERS = (math.pi * DT_WHEEL_DIAMETER) * METERS_PER_INCH
DT_KS_VOLTS = 0.30
DT_KS_VOLTS_SIM = 0.08
DT_KV_VOLTSECONDS_METER_SIM = 0.4
DT_KV_VOLTSECONDS_METER = 1.5
DT_KV_VOLTSECONDS_SQUARED_METER = 0.2

ROBOT_WHEELBASE = 22
ROBOT_BUMPER_WIDTH = 3.25
ROBOT_WIDTH = 29 + (ROBOT_BUMPER_WIDTH * 2)
ROBOT_LENGTH = 29 + (ROBOT_BUMPER_WIDTH * 2)

# These should be tuned
# SHOOTER_KV = 0.11  # Volts * seconds/m
# SHOOTER_KA = 0.40  # Volts * seconds^2/m
# SHOOTER_KS = 0.1
# SHOOTER_GEARING = 1.0  # No reduction
# SHOOTER_MOI = 0.004572  # 1/2 MR^2 in Kgs, .1 * .45 * (4 * .0254)
# SHOOTER_WHEEL_DIAMETER = 4  # 4" compliant wheels
# SHOOTER_WHEEL_CIRCUMFERENCE_METERS = math.pi * SHOOTER_WHEEL_DIAMETER * METERS_PER_INCH


# CAN IDS
DT_LEFT_LEADER = 1
DT_RIGHT_LEADER = 2
DT_LEFT_FOLLOWER = 3
DT_RIGHT_FOLLOWER = 4
ELEVATOR = 6
INTAKE_MOTOR = 7
WRIST_MOTOR = 8


##>>
# INTAKE_ROLLER = 7
# INDEX_RIGHT = 8
# FLYWHEEL_RIGHT = 9
# LATERAL_INTAKE = 10
# FLYWHEEL_LEFT = 11
# INDEX_ROLLER = 12
# LEFT_CLIMBER = 13
# RIGHT_CLIMBER = 14


# ROBORIO Ports
# Analog
# INTAKE_BEAM_BREAK_0 = 0
# INTAKE_BEAM_BREAK_1 = 1
WRIST_ANGLE_ENCODER = 0
# DIO
# SHOOTER_ANGLE_ENCODER = 0

# PWM


# Default Robot loop period
ROBOT_PERIOD_MS = 0.020  # 50Hz, or 20 times a second

# Controller Mapping information
CONTROLLER_DRIVER_PORT = 0
CONTROLLER_PARTNER_PORT = 1
CONTROLLER_FORWARD_REAL = 1
CONTROLLER_FORWARD_SIM = 1
CONTROLLER_TURN_REAL = 4
CONTROLLER_TURN_SIM = 0

# Elevator stopping positions counting rotations of the mechanism
class ElevatorPosition(Enum):
    LEVEL_BOTTOM = 0
    LEVEL_ONE = 1
    LEVEL_TWO = 2
    LEVEL_THREE = 3
    LEVEL_FOUR = 4
    LEVEL_SOURCE = 1.809
    LEVEL_UKNOWN = 999999

def get_closest_elevator_position(value: int) -> ElevatorPosition:
    '''
    This function calculates the distance between two points and returns 
    which of the ElevatorPositions is the closest to the input number.
    
    The first step is to take the ElevatorPosition enumeration, and make
    a list representing the values.  For example, below the variable named
    enum_values is actually a python list that looks like:
          [0, 50, 100, 150, 200, 999999]
    Then for each of the values in that list (x) perform the calculation:
        abs_value( x - input_number )
    Find the lowest number (min) of all those calculations and return the
    corresponding Enum value.  
    '''
    # Convert the enum values to a list of tuples (value, enum member)
    # This pairs the data with the encoder number being the first member
    # which is accessed by array notation [0] (first member of the array)
    # and the Enum item being the second member which is accessed by the
    # array notation [1]
    enum_values = [(item.value, item) for item in ElevatorPosition]

    # Find the closest value. Closes value will be a Tuple, or grouping,
    # taking the form of (encoder int, ElevatorPosition object)
    closest_value = min(enum_values, key=lambda x: abs(x[0] - value))

    # Return only the ElevatorPosition object to the caller
    return closest_value[1]

# TODO -- This needs to be fixed for simulation accuracy
ELEVATOR_GEAR_RATIO = (16 / 1) * (42 / 14)
# ELEVATOR_GEAR_RATIO = (10/1)*(42/14)
ELEVATOR_CARRIAGE_MASS = 5 # in Kilograms
ELEVATOR_DRUM_RADIUS_M = 2 * INCHES_PER_METER # in meters
ELEVATOR_MIN_HEIGHT_M = 6 * INCHES_PER_METER # base elevator 6 inches off the ground
ELEVATOR_MAX_HEIGHT_M = 93 * INCHES_PER_METER # Top of the elevator stack


WRIST_GEAR_RATIO = 263.7 * 3 #  NeveRest ~25 RPM at motor output + reduction gears on shaft

WRIST_ACCEPTABLE_UPPER_LIMIT = -5
WRIST_ACCEPTABLE_LOWER_LIMIT = 65


CS = 175