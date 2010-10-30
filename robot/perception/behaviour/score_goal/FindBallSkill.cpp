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

#include "perception/behaviour/score_goal/FindBallSkill.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "utils/log.hpp"
#include "utils/body.hpp"
#include "utils/basic_maths.hpp"

void FindBallSkill::execute(SkillParams *params, ActionCommand::All *actions) {
   const float *angles = params->sensors.joints.angles;
   float headYaw = angles[Joints::HeadYaw];
   float headPitch = angles[Joints::HeadPitch];

   llog(INFO) << "Trying to find ball..." << std::endl;
   llog(INFO) << "Stage: " << stage <<  std::endl
              << "Yaw1: " << RAD2DEG(headYaw) << std::endl
              << "Pitch1: " << RAD2DEG(headPitch) << std::endl;

   if (timer.elapsed_ms() - lastScanTime > NEW_SCAN_TIME) {
      stage = AT_FEET_CHECK;
   }
   lastScanTime = timer.elapsed_ms();
   actions->body.actionType = ActionCommand::Body::STAND;
   switch (stage) {
      case AT_FEET_CHECK:
         actions->head = ActionCommand::Head(0,
                                             BOTTOM_SCAN_PITCH,
                                             false,
                                             SCAN_SPEED,
                                             0.5);
         if (ABS(headYaw) < DEG2RAD(5) &&
            ABS(headPitch - BOTTOM_SCAN_PITCH) < DEG2RAD(5)) {
            stage = LOW_START;
         }
         break;
      case LOW_START:
         actions->head = ActionCommand::Head(START_SCAN_YAW,
                                             BOTTOM_SCAN_PITCH,
                                             false,
                                             SCAN_SPEED,
                                             0.5);
         if (ABS(headYaw - START_SCAN_YAW) < DEG2RAD(5) &&
            ABS(headPitch - BOTTOM_SCAN_PITCH) < DEG2RAD(5)) {
            stage = LOW_END;
         }
         break;
      case LOW_END:
         actions->head = ActionCommand::Head(END_SCAN_YAW,
                                             BOTTOM_SCAN_PITCH,
                                             false,
                                             SCAN_SPEED,
                                             0.5);
         if (ABS(headYaw - END_SCAN_YAW) < DEG2RAD(5) &&
            ABS(headPitch - BOTTOM_SCAN_PITCH) < DEG2RAD(5)) {
            stage = MIDDLE_END;
            startTurnTime = timer.elapsed_ms();
         }
         break;
      case MIDDLE_END:
         actions->head = ActionCommand::Head(START_SCAN_YAW,
                                             MIDDLE_SCAN_PITCH,
                                             false,
                                             SCAN_SPEED,
                                             0.5);
         if (ABS(headYaw - START_SCAN_YAW) < DEG2RAD(5) &&
            ABS(headPitch - MIDDLE_SCAN_PITCH) < DEG2RAD(5)) {
            stage = TOP_END;
         }
         break;
      case TOP_END:
         actions->head = ActionCommand::Head(END_SCAN_YAW,
                                             TOP_SCAN_PITCH,
                                             false,
                                             SCAN_SPEED,
                                             0.5);
         if (ABS(headYaw - END_SCAN_YAW) < DEG2RAD(5) &&
            ABS(headPitch - TOP_SCAN_PITCH) < DEG2RAD(5)) {
            stage = TURN;
         }
         break;
      case TURN:
         actions->body = ActionCommand::Body(ActionCommand::Body::WALK,
                                             0, 0, RAD2DEG(15));
         actions->head = ActionCommand::Head(0,
                                             0,
                                             false,
                                             SCAN_SPEED,
                                             0.5);
         llog(DEBUG1) << "Turn" << std::endl;
         if (timer.elapsed_ms() - startTurnTime > 7000) {
            stage = AT_FEET_CHECK;
         }
         break;
   }
}

