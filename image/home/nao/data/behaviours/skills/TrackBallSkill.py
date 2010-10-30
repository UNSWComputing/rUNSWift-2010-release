import ActionCommand
import WhichCamera
import WhichPosts
import Joints
import Robot
from math import degrees, radians, atan, atan2

class TrackBallSkill:
   LOST_TIME = 25

   YAW_NEAR_OFFSET = radians(1.2)
   YAW_FAR_OFFSET = radians(2.5)

   def __init__(self):
      self.lastYaw = 0.0
      self.lastYawDiff = 0.0

      self.lastPitch = 0.0

      self.lastDist = 0
      self.lastDistDiff = 0

      self.usingTopCamera = False
      Robot.setCamera(WhichCamera.BOTTOM_CAMERA)

   def execute(self, b, h, l):
      isSucceed = False

      yaw = self.lastYaw
      pitch = self.lastPitch
      yawSpeed = 0.0
      pitchSpeed = 0.0

      lostBall = Robot.lBallLostCount()
      if lostBall < self.LOST_TIME:
         ball = Robot.vRrBallLocation()
         ballD = (self.lastDist + ball['distance']) / 2

         realPitch = Robot.sensorValues()['joints']['angles'][Joints.HeadPitch]
         if realPitch > 0.0 and ballD < 850:
            self.usingTopCamera = False
         elif realPitch < radians(-10.0):
            self.usingTopCamera = True

         if self.usingTopCamera:
            Robot.setCamera(WhichCamera.TOP_CAMERA)
            yaw = self.clipYaw(ball['heading'], self.YAW_FAR_OFFSET)
            pitch = radians(72.5) - atan(ballD / 1050)
            yawSpeed = 0.1
            pitchSpeed = 0.25            
         else:
            Robot.setCamera(WhichCamera.BOTTOM_CAMERA)
            yaw = self.clipYaw(ball['heading'], self.YAW_NEAR_OFFSET)
            pitch = radians(29.5) - atan(ballD / 1050)
            yawSpeed = 0.2
            pitchSpeed = 0.25

         self.lastDistDiff = ballD - self.lastDist
         self.lastDist = ballD

         if pitch < radians(-38.5):
            pitch = radians(-38.5)

         if lostBall > 10:
            yaw += (3.5 * self.lastYawDiff)
            yawSpeed = 0.5

            ballD = self.lastDist + (7.0 * self.lastDistDiff)
            if ballD > 1000:
               Robot.setCamera(WhichCamera.TOP_CAMERA)
               pitch = radians(72.5) - atan(ballD / 1050)
            else:
               Robot.setCamera(WhichCamera.BOTTOM_CAMERA)
               pitch = radians(29.5) - atan(ballD / 1050)
            pitchSpeed = 0.5

         self.lastPitch = pitch

         isSucceed = True

      Robot.setCamera(WhichCamera.BOTTOM_CAMERA)
      h.yaw(yaw)
      h.pitch(pitch)
      h.yawSpeed(yawSpeed)
      h.pitchSpeed(pitchSpeed)
      h.isRelative(False)
      return isSucceed


   def clipYaw(self, alpha, offset):
      lastY = self.lastYaw

      if abs(alpha) > abs(self.lastYaw) + offset or abs(alpha) < abs(self.lastYaw) - offset:
         self.lastYaw = alpha

      if (self.lastYaw - lastY) != 0:
         self.lastYawDiff = self.lastYaw - lastY

      return self.lastYaw

