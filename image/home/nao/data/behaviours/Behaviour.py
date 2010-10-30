import Robot
import ActionCommand
import GameController
import Helpers
import sys
import SonarFilter

skill = Robot.pythonSkill()
exec "from skills.%s import %s" % (skill, skill)

skillInstance = eval(skill+"()")

def decideNextAction():
   try:
      SonarFilter.update()
      b = ActionCommand.Body()
      h = ActionCommand.Head()
      l = ActionCommand.LED()

      if Helpers.localised():
         l.leftEye(0,1,1)
      elif Helpers.unLocalised():
         l.leftEye(1,0,1)
      else:
         l.leftEye(0,0,1)
      if Robot.vNumBalls() > 0:
         l.rightEye(0,1,0)
      elif Helpers.foundBall():
         l.rightEye(1,1,0)
      else:
         l.rightEye(1,0,0)

      from math import radians
      b.actionType(ActionCommand.Body.STAND)
      skillInstance.execute(b, h, l)
      Robot.setAction(h,b,l)
      #print (b, h, l)
   except KeyboardInterrupt:
      print "###########################"
      print "##    SIGINT RECEIVED    ##"
      print "##       BY PYTHON       ##"
      print "##  ATTEMPTING SHUTDOWN  ##"
      print "###########################"
      Robot.attemptShutdown()
print "What is the air speed velocity of an unladen swallow?"
Robot.setCallback(decideNextAction)
if Robot.gcTeam()['teamColour'] == GameController.BLUE:
   colour = "blue"
else:
   colour = "red"
Robot.say("Player %d, team %d " % (Robot.player(), Robot.gcTeam()['teamNumber']) + colour)

