import math

METERS_PER_INCH = 0.0254
INCHES_PER_METER = 39.37

# Robot Physical Characteristics with dimensional units
ROBOT_MASS = 70
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
ELEVATOR = 5
INTAKE_MOTOR = 8

ELEVATOR = 5
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
