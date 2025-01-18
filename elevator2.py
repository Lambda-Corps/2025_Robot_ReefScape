from commands2 import Subsystem, Command, cmd
from phoenix5 import (
    TalonSRX,
    TalonSRXControlMode,
    TalonSRXConfiguration,
    LimitSwitchSource,
    Faults,
    LimitSwitchNormal,
)
from wpilib import SmartDashboard, AnalogInput, RobotBase, Timer

import constants


class ELEVATOR(Subsystem):
    ELEVATOR_TOP_LIMIT = 5000
    ELEVATOR_UP_SPEED = 1
    ELEVATOR_DOWN_SPEED = -1

    def __init__(self) -> None:
        super().__init__()

        self._ELEVATOR: TalonSRX = TalonSRX(constants.ELEVATOR)
        self._ELEVATOR.configFactoryDefault()
        # self.ELEVATOR.configReverseLimitSwitchSource(
        #     LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0
        # )
        # self.ELEVATOR.configForwardSoftLimitThreshold(self.ELEVATOR_TOP_LIMIT)
        # self.ELEVATOR.configForwardSoftLimitEnable(True)
        self._ELEVATOR.setSensorPhase(True)
        self._ELEVATOR.setSelectedSensorPosition(0)

        
        # self._right_ELEVATOR.configReverseLimitSwitchSource(
        #     LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0
        # )
        # self._right_ELEVATOR.configForwardSoftLimitThreshold(self.ELEVATOR_TOP_LIMIT)
        # self._right_ELEVATOR.configForwardSoftLimitEnable(True)

        self._left_faults: Faults = Faults()
    

    def move_ELEVATOR_up(self) -> None:
        self._ELEVATOR.set(TalonSRXControlMode.PercentOutput, self.ELEVATOR_UP_SPEED)
    

    def move_ELEVATOR_down(self) -> None:
        self._ELEVATOR.set(
            TalonSRXControlMode.PercentOutput, self.ELEVATOR_DOWN_SPEED
        )

    def ELEVATOR_at_top(self) -> bool:
        return (
            self._left_faults.ForwardLimitSwitch
        )

    def stop_ELEVATOR_motors(self) -> None:
        self._ELEVATOR.set(TalonSRXControlMode.PercentOutput, 0)
        
    def periodic(self) -> None:
        self._ELEVATOR.getFaults(self._left_faults)
       

        SmartDashboard.putBoolean(
            "L Forward Limit", self._left_faults.ForwardLimitSwitch
        )
        
        
        SmartDashboard.putBoolean(
            "L Reverse Limit", self._ELEVATOR.isRevLimitSwitchClosed()
        )
        


class MoveELEVATOR(Command):
    def __init__(self, sub: ELEVATOR, speed: float, timeout=0):
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
