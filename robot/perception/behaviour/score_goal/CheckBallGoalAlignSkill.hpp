/*
Copyright 2010 The University of New South Wales (UNSW).

This file is part of the 2010 team rUNSWift RoboCup entry. You may
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version as
modified below. As the original licensors, we add the following
conditions to that license:

In paragraph 2.b), the phrase "distribute or publish" should be
interpreted to include entry into a competition, and hence the source
of any derived work entered into a competition must be made available
to all parties involved in that competition under the terms of this
license.

In addition, if the authors of a derived work publish any conference
proceedings, journal articles or other academic papers describing that
derived work, then appropriate academic citations to the original work
must be included in that publication.

This rUNSWift source is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with this source code; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#pragma once
#include <vector>
#include "perception/behaviour/Skill.hpp"
#include "utils/Timer.hpp"
#include "utils/RRCoord.hpp"

/* This skill checks to see if the goals are
 * to the left and right of the robot
 * It assumes the ball is at the robots feet
 */
class CheckBallGoalAlignSkill : Skill {
   public:
   CheckBallGoalAlignSkill() : isAligning(false), finishAlignTime(0),
                               seenBlue(false), seenYellow(false) {}
   void execute(SkillParams *params, ActionCommand::All *actions);
   /* Used to tell the main decision tree not to
    * do anything else as we are currently trying to
    * check if we are aligned with the goals
    */
   bool getIsLiningUp();
   /* Checks if this routine found that the robot
    * was algined with the ball recently
    */
   bool getIsAligned();

   /* Call this when you want to see weather you
    * should act on an alignment.
    * This will be true for a period after an 
    * alignment finishes
    */
   bool hasJustCheckedAlignment();

   /* This is the modules guess at which way the user should
    * rotate in order to turn around the ball and face the goal
    */
   int getTurnDirection();

   void startNewAlignment();

   private:
      void buildGoalVector(SkillParams *params);
      void determineIfAligned();
      void removeOutlierGoals();
      Timer timer;
      bool isAligning;
      bool isAligned;
      float finishAlignTime;
      bool hasSeenTwoGoals;
      enum AlignStage {
         START_SCAN,
         SCAN
      };
      AlignStage stage;
      static const float POST_ALIGN_THRESHOLD = 3000;
      static const float SCAN_PITCH = -20/57.0;
      static const float START_SCAN_YAW = 115/57.0;
      static const float END_SCAN_YAW = -115/57.0;
      static const float SCAN_SPEED = 0.4;
      /* Variables to keep track of which goals have been seen */
      std::vector<RRCoord> goalPositions;
      bool seenBlue, seenYellow;
};

