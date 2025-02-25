import commands2
from commands2 import PrintCommand 
from elevator import ELEVATOR, MoveELEVATORToSetPoint
from wrist import WristControl, Set_Global_Wrist_Angle
import constants


class TwoSubsystemCommandGroupOne(commands2.SequentialCommandGroup):
   def __init__(self, _elevator: ELEVATOR, _wrist : WristControl) -> None:
       super().__init__()
       print ("Within SequentialCommand Group 1")

       self._elevator = _elevator
       self._wrist = _wrist
       
       self.addCommands(Set_Global_Wrist_Angle(self._wrist, 0))  
       self.addCommands(MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_ONE).withTimeout(5))

       self.addCommands(PrintCommand("TwoSubsystemCommandGroupOne Done"))

class TwoSubsystemCommandGroupTwo(commands2.SequentialCommandGroup):
   def __init__(self, _elevator: ELEVATOR, _wrist : WristControl) -> None:
       super().__init__()
       print ("Within SequentialCommand Group")

       self._elevator = _elevator
       self._wrist = _wrist
       
       self.addCommands(Set_Global_Wrist_Angle(self._wrist, 0))  
       self.addCommands(MoveELEVATORToSetPoint(self._elevator,constants.ElevatorPosition.LEVEL_ONE).withTimeout(5))

       self.addCommands(PrintCommand("TwoSubsystemCommandGroup Done"))