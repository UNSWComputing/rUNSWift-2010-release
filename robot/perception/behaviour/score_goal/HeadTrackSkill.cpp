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

#include "perception/behaviour/score_goal/HeadTrackSkill.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "utils/body.hpp"
#include "utils/log.hpp"
#include "utils/basic_maths.hpp"

void HeadTrackSkill::execute(SkillParams *params, ActionCommand::All *actions) {
   if (params->vNumBalls != 0) {
      const float *angles = params->sensors.joints.angles;
      float headPan = angles[Joints::HeadYaw];
      float ballHeading = params->vRrBallLocation.heading;
      float ballRHead = ballHeading + headPan;
      float dPitch = IMAGE_ROWS/2 - params->ballLocation.second;
      float dYaw = (IMAGE_COLS/2 - params->ballLocation.first);
      if (ABS(dPitch) < 100) {
         dPitch = 0;
      }
      if (ABS(dYaw) < 100) {
         dYaw = 0;
      }
      // float dYaw = ballRHead;
      float yawSpeed = (ABS(dYaw)/DEG2RAD(41)) * .7;
      yawSpeed = 1;
      // if(ABS(dYaw) > DEG2RAD(20)) yawSpeed = .8;
      llog(INFO) << dYaw << " with speed " << yawSpeed << std::endl;
      actions->head = ActionCommand::Head(dYaw/6000,
            -dPitch/10000.0f, true, 1.0f, 1.0f);
   } else {
      actions->head = ActionCommand::Head(0, 0, true, 1.0f, 1.0f);
   }
}

