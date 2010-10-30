import Joints
import Robot
import ActionCommand
import WhichPosts
import WhichCamera
from math import degrees, radians, atan2

# set up a set of waypoints for the head to track to which is
# executed in sequence
class ScanForBallSkill:
   BOTTOM = radians(25.0)
   MIDDLE = radians(-3.0)
   TOP = radians(17.0)
   YAW_MAX = radians(80.0)

   S_FEET = 0
   S_BOTTOM = 1
   S_MIDDLE = 2
   S_TOP = 3
   state = S_FEET
   
   def __init__(self):
      self.waypoints = []
      self.waypoints.append((self.BOTTOM, -self.YAW_MAX, 0.5, WhichCamera.BOTTOM_CAMERA))  # bottom right
      self.waypoints.append((self.BOTTOM, self.YAW_MAX, 0.4, WhichCamera.BOTTOM_CAMERA))  # bottom left
      self.waypoints.append((self.MIDDLE, -self.YAW_MAX, 0.4, WhichCamera.BOTTOM_CAMERA)) # middle right
      self.waypoints.append((self.TOP, self.YAW_MAX, 0.15, WhichCamera.TOP_CAMERA))     # top left

   def reset(self):
      self.state = self.S_FEET

   def execute(self, b, h, l):
      Robot.setCamera(self.waypoints[self.state][3])
      if Robot.lBallLostCount() <= 15:
         return True

      h.pitch(self.waypoints[self.state][0])
      h.yaw(self.waypoints[self.state][1] * 1.0)
      h.yawSpeed(self.waypoints[self.state][2])
      h.isRelative(False)
      h.pitchSpeed(.2)

      realPitch = Robot.sensorValues()['joints']['angles'][Joints.HeadPitch]
      realYaw = Robot.sensorValues()['joints']['angles'][Joints.HeadYaw]

      if abs(realPitch - h._pitch) < radians(10) and abs(realYaw - h._yaw) < radians(10):
         self.state = (self.state + 1) % len(self.waypoints)
         if self.state == 0:
            return False
      return True
