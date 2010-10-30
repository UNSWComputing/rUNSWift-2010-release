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

#include "perception/behaviour/lc/LocaliseSkill.hpp"
#include "utils/basic_maths.hpp"
#include "utils/log.hpp"

LocaliseSkill::LocaliseSkill() {
   stage = HIGH_LEFT;
   seenBoth = false;
}

void LocaliseSkill::execute(
      SkillParams *params, ActionCommand::All *actions) {
   float headYaw = params->sensors.joints.angles[Joints::HeadYaw];
   float headPitch = params->sensors.joints.angles[Joints::HeadPitch];


   float headSpeed = 0.4;
   if (params->posts.which != pNONE) {
      timer.restart();
      if (params->posts.which == pBLUE_BOTH ||
          params->posts.which == pYELLOW_BOTH) {
          seenBoth = true;
      } else {
         seenBoth = false;
      }
   }
   if (timer.elapsed_ms() < 30 * 10) {
      if (seenBoth) {
         headSpeed = 0;
      } else {
         headSpeed = 0.04;
      }
   }
   if (timer.elapsed_ms() > 4000) {
      headSpeed = 0.1;
   }
   actions->head.isRelative = false;
   actions->head.yawSpeed = headSpeed;
   actions->head.pitchSpeed = headSpeed;
   switch (stage) {
      case HIGH_LEFT :
         actions->head.pitch = DEG2RAD(-30.0);
         actions->head.yaw = DEG2RAD(75.0);
         if (ABS(headYaw-actions->head.yaw) < DEG2RAD(5))
            stage = HIGH_RIGHT;
         break;
      case HIGH_RIGHT :
         actions->head.pitch = DEG2RAD(-30.0);
         actions->head.yaw = DEG2RAD(-75.0);
         if (ABS(headYaw-actions->head.yaw) < DEG2RAD(5))
            stage = LOW_LEFT;
         break;
      case LOW_LEFT :
         actions->head.pitch = DEG2RAD(-30.0);
         actions->head.yaw = DEG2RAD(75.0);
         if (ABS(headYaw-actions->head.yaw) < DEG2RAD(5))
            stage = HIGH_RIGHT;
         break;
      case LOW_RIGHT :
         actions->head.pitch = DEG2RAD(0.0);
         actions->head.yaw = DEG2RAD(-75.0);
         if (ABS(headYaw-actions->head.yaw) < DEG2RAD(5))
            stage = HIGH_LEFT;
         break;
   }
   actions->body.actionType = ActionCommand::Body::STAND;
   actions->body.forward = 0;
   actions->body.left = 0;
   actions->body.turn = 0;
   llog(DEBUG3) << "Stand" << std::endl;
}

