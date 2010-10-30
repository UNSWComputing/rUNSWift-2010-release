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

#include "perception/behaviour/score_goal/LineUpBallGoalSkill.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "utils/log.hpp"
#include "utils/basic_maths.hpp"

bool LineUpBallGoalSkill::getIsLiningUp() {
   if (stage == BALL_AT_FEET || lastLineUpTime > 1000) {
      return false;
   }
   return true;
}
void LineUpBallGoalSkill::execute(SkillParams *params,
                          ActionCommand::All *actions) {
   float heading = params->vRrBallLocation.heading;
   int turnDirection = heading/ABS(heading);

   const float *angles = params->sensors.joints.angles;
   float headYaw = angles[Joints::HeadYaw];
   float headPitch = angles[Joints::HeadPitch];

   llog(INFO) << "Line up: " << stage << std::endl;
   if (timer.elapsed_ms() - lastLineUpTime > 1000) {
      stage = BALL_AT_FEET;
   }
   lastLineUpTime = timer.elapsed_ms();
   switch (stage) {
      case BALL_AT_FEET:
         actions->head = ActionCommand::Head(0,
                                             20,
                                             false,
                                             .3,
                                             1.0);
         if (ABS(headYaw) < 1 && ABS(headPitch -20) < 1) {
            if (params->vNumBalls == 0) {
               // dunno yet
               llog(DEBUG1) << "Ball not at feet" << std::endl;
            } else {
               stage = CHECK_GOALS_START;
               llog(DEBUG1) << "-> Ball at feet." << std::endl;
            }
         }
         break;
      case CHECK_GOALS_START:
         actions->head = ActionCommand::Head(60,
                                             -10,
                                             false,
                                             .3,
                                             1.0);
         if (ABS(headYaw - (60))) {
            stage = CHECK_GOALS;
         }
         break;
      case CHECK_GOALS:
         actions->head = ActionCommand::Head(-60,
                                             -10,
                                             false,
                                             .3,
                                             1.0);
         if (ABS(headYaw - (-60))) {
            stage = ROTATE_ABOUT_BALL;
            startRotateTime = timer.elapsed_ms();
         }
         break;
      case ROTATE_ABOUT_BALL:
         actions->body = ActionCommand::Body(ActionCommand::Body::WALK,
                                             0, 100, DEG2RAD(-15));
         if (timer.elapsed_ms() - startRotateTime > 2000) {
            stage = BALL_AT_FEET;
         }
         break;
   }

   if (ABS(heading) > DEG2RAD(5) &&
       timer.elapsed_ms() - lastSideStepTime > 100) {
      llog(INFO) << "Rotate to align: "
                 << heading << std::endl;
      actions->body = ActionCommand::Body(ActionCommand::Body::WALK,
                                          0, 0, -heading);
   } else if (ABS(heading) <= DEG2RAD(5)) {
      llog(INFO) << "Side step" << std::endl;
      actions->body = ActionCommand::Body(ActionCommand::Body::WALK,
                                          0, 100, -heading/1.5);
      lastSideStepTime = timer.elapsed_ms();
   } else {
      actions->body = ActionCommand::Body(ActionCommand::Body::WALK,
                                          0, 100, -heading/1.5);
   }
}

