from commands2 import Command
from elevator import ElevatorSubsystem

class MoveElevatorCommand(Command):
    def __init__(self, elevator: ElevatorSubsystem, speed: float):
        super().__init__()
        self.elevator = elevator
        self.speed = speed

        # Declare subsystem dependencies
        self.addRequirements(self.elevator)

    def execute(self):
        self.elevator.move(self.speed)

    def end(self, interrupted: bool):
        self.elevator.stop()

    def isFinished(self):
        # This command runs until interrupted
        return False
