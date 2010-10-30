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

#include "perception/behaviour/StartStopSkill.hpp"
#include "utils/log.hpp"

StartStopSkill::StartStopSkill() {
   current.body = ActionCommand::Body::STAND;
   current.leds.leftEye = ActionCommand::LED::rgb();
   current.leds.rightEye = ActionCommand::LED::rgb();
   current.leds.chestButton = ActionCommand::LED::rgb();
}

void StartStopSkill::execute(SkillParams* skill, ActionCommand::All* actions,
      WhichCamera *camera, bool *usePF) {
   if (timer.elapsed_ms() >= 1500) {
      timer.restart();
      if (current.body.actionType == ActionCommand::Body::STAND) {
         current.body = ActionCommand::Body(ActionCommand::Body::WALK, 100);
         current.leds.leftEye = ActionCommand::LED::rgb(1);
         current.leds.rightEye = ActionCommand::LED::rgb(1);
         current.leds.chestButton = ActionCommand::LED::rgb(1, 1, 1);
      } else {
         current.body = ActionCommand::Body::STAND;
         current.leds.leftEye = ActionCommand::LED::rgb();
         current.leds.rightEye = ActionCommand::LED::rgb();
         current.leds.chestButton = ActionCommand::LED::rgb();
      }
   }
   *actions = current;
}

StartStopSkill::~StartStopSkill() { }
