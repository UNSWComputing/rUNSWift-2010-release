import Joints
import Robot
import ActionCommand
import WhichPosts
from math import degrees, radians, atan2
from TrackBallSkill import TrackBallSkill
from ScanForBallSkill import ScanForBallSkill

class GotoBallSkill:
   def __init__(self, goalDistance=100):
      self.trackBallSkill = TrackBallSkill()
      self.scanForBallSkill = ScanForBallSkill()
      self.goalDistance = goalDistance

   def execute(self, b, h, l):
         if Robot.lBallLostCount() > 15 :
            self.scanForBallSkill.execute(b, h, l)
            return False
         else:
            self.trackBallSkill.execute(b, h, l)

         heading = Robot.vRrBallLocation()['heading']
         distance = Robot.vRrBallLocation()['distance']

         distance -= self.goalDistance
         b.actionType(ActionCommand.Body.FAST)
         if degrees(heading) < abs(30):
            b.turn(heading/2.0)
            b.forward(int(distance/2.5))
            b.left(0)
         else:
            b.forward(0)
            b.left(0)
            b.turn(heading/2.0)

         if distance < 15 and abs(heading) < radians(5):
            return True
         return False

