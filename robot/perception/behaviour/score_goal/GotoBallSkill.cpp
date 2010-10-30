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

#include "perception/behaviour/score_goal/GotoBallSkill.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "utils/log.hpp"
#include "utils/basic_maths.hpp"

void GotoBallSkill::execute(SkillParams *params, ActionCommand::All *actions) {
   llog(INFO) << params->vRrBallLocation.heading << std::endl;
   float heading = params->vRrBallLocation.heading;
   float degreeTolerence = 20;
   if (ABS(heading) > DEG2RAD(degreeTolerence)) {
      actions->body = ActionCommand::Body(ActionCommand::Body::WALK, 0, 0,
            params->vRrBallLocation.heading/2);
   } else {
      actions->body = ActionCommand::Body(ActionCommand::Body::WALK, 150, 0, 0);
   }
}

