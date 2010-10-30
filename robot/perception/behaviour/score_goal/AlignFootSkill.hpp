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
#include <deque>
#include "perception/behaviour/Skill.hpp"
#include "utils/Timer.hpp"
#include "utils/RRCoord.hpp"

/* This should only be used when the ball
 * is close (<30cm). And the ball is in view
 * of the camera.
 * This routine moves slowly so as to not rock
 * too much and thus position itself accurately.
 */
class AlignFootSkill : Skill {
   public:
   AlignFootSkill() : lastPauseToStabalizeTime(0) {}
   void execute(SkillParams *params, ActionCommand::All *actions);
   /* Returns true when the ball is in the optimal position for
      Kicking
   */
   bool isAligned();
   private:
      static const float GOAL_HEADING = 9/57.0;
      static const float GOAL_DISTANCE = 200;
      float lastAlignTime;
      Timer timer;
      float lastPauseToStabalizeTime;
      static const float STABALIZE_TIME = 1000;
      static const float ALIGN_ALIVE_TIME = 1000;
      /* Keeping a vector of the last 10 RRCoords 
       * Using this to detect when robot is unstable.
       */
      std::deque<RRCoord> ballPositions;
      bool isRobotUnstable();
};

