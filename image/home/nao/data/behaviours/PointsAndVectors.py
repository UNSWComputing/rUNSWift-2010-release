import math

class Vector:
   @staticmethod
   def FromAngleLength(angle, length):
      v = Vector(math.cos(angle), math.sin(angle))
      return v*length

   @staticmethod
   def FromTwoPoints(a, b):
      return b - a

   def __init__(self, x, y):
      self.x = x
      self.y = y

   def __mul__(self, scalar):
      return Vector(self.x * scalar, self.y * scalar)

   def __rmul__(self, scalar):
      return Vector(self.x * scalar, self.y * scalar)

   def __imul__(self, scalar):
      self.x *= scalar
      self.y *= scalar

   def __div__(self, scalar):
      return Vector(self.x / scalar, self.y / scalar)

   def __rdiv__(self, scalar):
      return Vector(self.x / scalar, self.y / scalar)

   def __idiv__(self, scalar):
      self.x /= scalar
      self.y /= scalar

   def __add__(self, other):
      return Vector(self.x + other.x, self.y + other.y)

   def __iadd__(self, other):
      self.x += other.x
      self.y += other.y

   def __sub__(self, other):
      return Vector(self.x - other.x, self.y - other.y)

   def __isub__(self, other):
      self.x -= other.x
      self.y -= other.y

   def rotate90CCW(self):
      return Vector(-self.y, self.x)

   def rotate90CW(self):
      return Vector(self.y, -self.x)

   def abs(self):
      return math.sqrt(self.x*self.x+self.y*self.y)

   def normalise(self):
      return self / self.abs()

   def direction(self):
      return math.atan2(self.y, self.x)

   def angleToClockwise(self, other):
      theta = self.direction() - other.direction()
      if (theta < 0):
         theta += 2*math.pi
      return theta

   def angleToCounterClockwise(self, other):
      theta = other.direction() - self.direction() 
      if (theta < 0):
         theta += 2*math.pi
      return theta

   def __repr__(self):
      return "Vector(%f, %f)" % (self.x, self.y)

Point = Vector

