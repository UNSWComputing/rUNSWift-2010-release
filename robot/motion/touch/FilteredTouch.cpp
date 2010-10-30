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

#include "motion/touch/FilteredTouch.hpp"
#include "utils/log.hpp"
#include "utils/speech.hpp"

FilteredTouch::FilteredTouch(Touch* t)
   : touch(t),
     init(true) {
   llog(INFO) << "FilteredTouch constructed" << std::endl;
}

FilteredTouch::~FilteredTouch() {
   llog(INFO) << "FilteredTouch destroyed" << std::endl;
}

SensorValues FilteredTouch::getSensors() {
   SensorValues update = touch->getSensors();
   struct timeval tv;
   static bool said = false;
   gettimeofday(&tv, NULL);
   if (tv.tv_sec % 5 == 0 && !said) {
      if (update.sonar[0] < Sonar::MIN)
         SAY("sonar error");
   }
   said = (tv.tv_sec % 5 == 0);
   if (init) {
      init = false;
      state = update;
      for (uint8_t i = 0; i < Sonar::NUMBER_OF_READINGS; ++i) {
         if (state.sonar[i] >= Sonar::INVALID || state.sonar[i] < Sonar::MIN)
            state.sonar[i] = 10.0f;
      }

   } else {
      state.joints = update.joints;
      uint8_t i;
      for (i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
         state.sensors[i] = update.sensors[i];
      for (i = 0; i < Sonar::NUMBER_OF_READINGS; ++i) {
         if (update.sonar[i] >= Sonar::INVALID)
            update.sonar[i] = 10.0f;
         if (update.sonar[i] >= Sonar::MIN)
            state.sonar[i] += (update.sonar[i] - state.sonar[i]) * 1.0;
      }
      llog(VERBOSE) << state.sonar[0] << " " << state.sonar[10]  << std::endl;
   }
   return state;
}

bool FilteredTouch::getStanding() {
   return touch->getStanding();
}

ButtonPresses FilteredTouch::getButtons() {
   return touch->getButtons();
}
