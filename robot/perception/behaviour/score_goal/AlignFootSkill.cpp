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

#include "perception/behaviour/score_goal/AlignFootSkill.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "utils/log.hpp"
#include "utils/basic_maths.hpp"
#include "utils/Timer.hpp"

bool AlignFootSkill::isAligned() {
   return timer.elapsed_ms() - lastAlignTime < ALIGN_ALIVE_TIME;
}

void AlignFootSkill::execute(SkillParams *params, ActionCommand::All *actions) {
   llog(INFO) << "Heading: "
              << RAD2DEG(params->vRrBallLocation.heading) << std::endl;
   float heading = params->vRrBallLocation.heading;
   float distanceToGo = params->vRrBallLocation.distance;
   ballPositions.push_back(params->vRrBallLocation);

   /* Update ball Position history */
   if (ballPositions.size() > 10) ballPositions.pop_front();
   llog(INFO) << "Distance left: " << distanceToGo << std::endl;
   bool isUnstable = isRobotUnstable();
   if (isUnstable) {
      lastPauseToStabalizeTime = timer.elapsed_ms();
      llog(INFO) << "Unstable!! Pause for a while" << std::endl;
   }
   float sgn = (distanceToGo-GOAL_DISTANCE)/ABS(distanceToGo-GOAL_DISTANCE);
   float hsgn = (heading)/ABS(heading);
   if (timer.elapsed_ms() - lastPauseToStabalizeTime < STABALIZE_TIME) return;
   if (ABS(heading - GOAL_HEADING) > DEG2RAD(1)) {
      actions->body = ActionCommand::Body(ActionCommand::Body::WALK,
                                          0, 0, hsgn*2);
   } else if (ABS(distanceToGo-GOAL_DISTANCE) > 35) {
      llog(INFO) << "Move: " << sgn*130 << std::endl;
      actions->body = ActionCommand::Body(ActionCommand::Body::WALK,
                                          sgn*130, 0, 0);
   } else {
      lastAlignTime = timer.elapsed_ms();
   }
}

bool AlignFootSkill::isRobotUnstable() {
   float min, max;
   min = max = ballPositions[0].distance;
   for (int i = 1; i < ballPositions.size(); i++) {
      if (ballPositions[i].distance < min)
         min = ballPositions[i].distance;
      if (ballPositions[i].distance > max)
         max = ballPositions[i].distance;
   }
   return (max - min > 110);
}
