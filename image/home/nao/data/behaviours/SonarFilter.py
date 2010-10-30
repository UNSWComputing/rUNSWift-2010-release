import ActionCommand
from math import radians
import Robot
import WhichCamera


lastSonarValues = ([],[])
SONAR_RANGE = .6


def update():
   global lastSonarValues
   leftSonar = Robot.sensorValues()['sonar'][0]
   rightSonar = Robot.sensorValues()['sonar'][10]
   lastSonarValues[0].append(leftSonar)
   lastSonarValues[1].append(rightSonar)
   if len(lastSonarValues[0]) > 10:
      lastSonarValues[0].pop(0)
   if len(lastSonarValues[1]) > 10:
      lastSonarValues[1].pop(0)

def isOnLeft():
   leftCount = 0
   for value in lastSonarValues[0]:
      if value < SONAR_RANGE:
         leftCount += 1
   return leftCount >= 8

def isOnRight():
   rightCount = 0
   for value in lastSonarValues[1]:
      if value < SONAR_RANGE:
         rightCount += 1
   return rightCount >= 8

def isMoreOnLeft():
   leftCount = 0
   for value in lastSonarValues[0]:
      if value < SONAR_RANGE:
         leftCount += 1
   rightCount = 0
   for value in lastSonarValues[1]:
      if value < SONAR_RANGE:
         rightCount += 1
   return leftCount >= rightCount
      
