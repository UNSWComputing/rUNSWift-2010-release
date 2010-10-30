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

#include <cmath>
#include "perception/behaviour/lc/LocalisationChallengeSkill.hpp"
#include "perception/behaviour/lc/LocaliseSkill.hpp"
#include "perception/behaviour/lc/WalkToPointSkill.hpp"
#include "utils/basic_maths.hpp"
#include "utils/log.hpp"

using namespace std;

LocalisationChallengeSkill::LocalisationChallengeSkill() {
   localise = (Skill *) new LocaliseSkill();
   walkToPoint = (Skill *) new WalkToPointSkill();
   stop = false;
   localised = false;
}

void LocalisationChallengeSkill::execute(
      SkillParams *params, ActionCommand::All *actions) {
   llog(DEBUG2) << "x-variance = " << params->robotPos.var[0] << endl;
   llog(DEBUG2) << "y-variance = " << params->robotPos.var[1] << endl;
   int goalX = params->targetx;
   int goalY = params->targety;
   int x = robotPos.x;
   int y = robotPos.y;
   float theta = robotPos.theta;
   float t = 1000.0;
   float oldDist = sqrt(ABS(SQUARE(goalY - y) + SQUARE(goalX - x)));
   float dist = ABS(SQUARE(goalY - params->robotPos.y)
              + SQUARE(goalX - params->robotPos.x));
   llog(DEBUG3) << "stop = " << stop << endl;
   llog(DEBUG3) << "dist = " << dist << endl;

   if (stop) {
      actions->leds.leftFoot = ActionCommand::LED::rgb(1, 0, 0);
      actions->leds.rightFoot = ActionCommand::LED::rgb(1, 0, 0);
   }

   int walkTime = 5 + MIN(25, (oldDist/1000) * 10);
   llog(DEBUG3) << "Walk time: " << walkTime << std::endl;
   // Scan the head to find goals.
   localise->execute(params, actions);
   if (localised && !stop && timer.elapsed() < walkTime) {
      llog(DEBUG3) << "state 1" << endl;
      float absAngle = atan2(goalY - y, goalX - x);

      llog(INFO) << "AbsAngle: " << RAD2DEG(absAngle) << std::endl;
      llog(INFO) << "Robot Angle: " << RAD2DEG(theta)
         << std::endl;
      walkToPoint->execute(params, actions);
      actions->body.actionType = ActionCommand::Body::WALK;
      float rangle = absAngle - theta;
      llog(INFO) << "Relative angle: " <<
         RAD2DEG(rangle) << std::endl;
      actions->body.forward = 60*cos(rangle);
      actions->body.left = 0;  // 60*sin(rangle);
      actions->body.turn = rangle;  // 0
   } else if (isTurning) {
      actions->body.actionType = ActionCommand::Body::WALK;
      actions->body.forward = 0;
      actions->body.left = 0;
      actions->body.turn = DEG2RAD(20.0f);
      llog(DEBUG3) << "Turning" << std::endl;
      if (turnTimer.elapsed() > 30) {
         isTurning = false;
      }
   } else if (params->robotPos.var[0] > 600000 &&
            params->robotPos.var[1] > 600000) {
      isTurning = true;
      turnTimer.restart();
      llog(DEBUG3) << "Start turning" << std::endl;
   } else {
      llog(DEBUG3) << "state 2" << endl;
      if (timer.elapsed() > walkTime + 4 && dist < SQUARE(150) &&
         params->robotPos.var[0] < 37000.0 &&
         params->robotPos.var[1] < 37000.0) {
         llog(DEBUG3) << "state 2A" << endl;
         stop = true;
      }
      if (timer.elapsed() > walkTime + 5 &&
          params->robotPos.var[0] < 37000.0 &&
            params->robotPos.var[1] < 37000.0) {
         llog(DEBUG3) << "state 2B" << endl;
         robotPos = params->robotPos;
         localised = true;
         timer.restart();
      }
   }
}

