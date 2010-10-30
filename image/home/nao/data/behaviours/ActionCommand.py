from TypeSafety import accepts

# see utils/ActionCommand.hpp for documentation
class Body:
   NONE = 0
   STAND = 1
   WALK = 2
   SLOW = 3
   FAST = 4
   WAVE = 5
   AL = 6
   GETUP_FRONT = 7
   GETUP_BACK = 8
   KICK = 9
   INITIAL = 10
   DEAD = 11
   REF_PICKUP = 12
   SQUAT = 13
   SQUAT_FORWARD = 14
   OPEN_FEET = 15
   THROW_IN = 16
   GOALIE_SIT = 17
   NUM_ACTION_TYPES = 18
   def __init__(self):
      self._actionType = Body.NONE
      self._forward = 0
      self._left = 0
      self._turn = 0.0
      self._power = 1.0
   def __repr__(self):
      return ' '.join(map(repr,[self._actionType, (self._forward, self._left, self._turn,), self._power]))
   @accepts(int)
   def actionType(self, actionType):
      self._actionType = actionType
   @accepts(int)
   def forward(self, forward):
      self._forward = forward
   @accepts(int)
   def left(self, left):
      self._left = left
   @accepts(float)
   def turn(self, turn):
      self._turn = turn
   @accepts(float)
   def power(self, power):
      self._power = power

class Head:
   def __init__(self):
      self._yaw = 0.0
      self._pitch = 0.0
      self._isRelative = False
      self._yawSpeed = 0.0
      self._pitchSpeed = 0.0
   def __repr__(self):
      return ' '.join(map(repr,[(self._yaw, self._pitch,), self._isRelative, (self._yawSpeed, self._pitchSpeed,)]))
   @accepts(float)
   def yaw(self, yaw):
      self._yaw = yaw
   @accepts(float)
   def pitch(self, pitch):
      self._pitch = pitch
   @accepts(bool)
   def isRelative(self, isRelative):
      self._isRelative = isRelative
   @accepts(float)
   def yawSpeed(self, yawSpeed):
      self._yawSpeed = yawSpeed
   @accepts(float)
   def pitchSpeed(self, pitchSpeed):
      self._pitchSpeed = pitchSpeed

class LED:
   def __init__(self):
      self._leftEar = 0x0
      self._rightEar = 0x0
      self._leftEye = (0, 0, 0)
      self._rightEye = (0, 0, 0)
      self._chestButton = (0, 0, 0)
      self._leftFoot = (0, 0, 0)
      self._rightFoot = (0, 0, 0)
   def __repr__(self):
      return ' '.join(map(repr,[self._leftEar, self._rightEar, self._leftEye, self._rightEye, self._chestButton, self._leftFoot, self._rightFoot]))
   @accepts(int)
   def leftEar(self, leftEar):
      self._leftEar = leftEar
   @accepts(int)
   def rightEar(self, rightEar):
      self._rightEar = rightEar
   @accepts(int, int, int)
   def leftEye(self, r, g, b):
      self._leftEye = (r, g, b)
   @accepts(int, int, int)
   def rightEye(self, r, g, b):
      self._rightEye = (r, g, b)
   @accepts(int, int, int)
   def chestButton(self, r, g, b):
      self._chestButton = (r, g, b)
   @accepts(int, int, int)
   def leftFoot(self, r, g, b):
      self._leftFoot = (r, g, b)
   @accepts(int, int, int)
   def rightFoot(self, r, g, b):
      self._rightFoot = (r, g, b)

