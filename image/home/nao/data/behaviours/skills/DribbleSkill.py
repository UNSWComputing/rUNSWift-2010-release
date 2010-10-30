import ActionCommand
import Robot
import Joints
from ScanForBallSkill import ScanForBallSkill
from GotoBallSkill import GotoBallSkill

class DribbleSkill:
   def __init__(self):
      self.scanForBallSkill = ScanForBallSkill()
      self.gotoBallSkill = GotoBallSkill()

   def execute(self, b, h, l):
      if Robot.lBallLostCount() > 15:
         self.scanForBallSkill.execute(b, h, l)
         b.actionType(ActionCommand.Body.WALK)
         # debug leds
         l.leftEye(1, 0, 0)
         l.rightEye(1, 0, 0)
      else:
         self.gotoBallSkill.execute(b, h, l)
         # debug leds
         l.leftEye(0, 1, 0)
         l.rightEye(0, 1, 0)


