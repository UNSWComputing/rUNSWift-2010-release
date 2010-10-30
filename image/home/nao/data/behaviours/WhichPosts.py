import GameController
NONE = 0
BLUE_LEFT = 1
BLUE_RIGHT = 2
BLUE_BOTH = 3
BLUE_EITHER = 4
YELLOW_LEFT = 5
YELLOW_RIGHT = 6
YELLOW_BOTH = 7
YELLOW_EITHER = 8

def isEnemyPost(which, team):
   if team == GameController.BLUE:
      if which >= 5:
         return True
   else:
      if which < 5 and which >= 1:
         return True
   return False
def both(which):
   if which == BLUE_BOTH or which == YELLOW_BOTH:
      return True
   return False
