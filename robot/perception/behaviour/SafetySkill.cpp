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

#include "perception/behaviour/SafetySkill.hpp"
#include "utils/basic_maths.hpp"
#include "utils/log.hpp"

SafetySkill::SafetySkill(Skill* s) {
   skill = s;
   filtered_fsr_sum = 5.0;  // assume we start standing
   if (s == NULL)
      llog(FATAL) << "SafetySkill: s == NULL" << std::endl;
   llog(INFO) << "SafetySkill constructed" << std::endl;
}

SafetySkill::~SafetySkill() {
   llog(INFO) << "SafetySkill destroyed" << std::endl;
}

void SafetySkill::execute(SkillParams* params, ActionCommand::All* actions,
      WhichCamera *camera, bool *usePF) {
   SensorValues& s = params->sensors;
   float fsr_sum = s.sensors[Sensors::LFoot_FSR_FrontLeft]
                 + s.sensors[Sensors::LFoot_FSR_FrontRight]
                 + s.sensors[Sensors::LFoot_FSR_RearLeft]
                 + s.sensors[Sensors::LFoot_FSR_RearRight]
                 + s.sensors[Sensors::RFoot_FSR_FrontLeft]
                 + s.sensors[Sensors::RFoot_FSR_FrontRight]
                 + s.sensors[Sensors::RFoot_FSR_RearLeft]
                 + s.sensors[Sensors::RFoot_FSR_RearRight];
   filtered_fsr_sum = filtered_fsr_sum + 0.2*(fsr_sum - filtered_fsr_sum);
   skill->execute(params, actions, camera, usePF);
   static int blink = 0;
   blink = !blink;
   // if lying face down
   if (s.sensors[Sensors::InertialSensor_AccX] < -FALLEN) {
      actions->leds.leftEye = ActionCommand::LED::rgb(blink, 0);
      actions->leds.rightEye = ActionCommand::LED::rgb(blink, 0);
      actions->body = ActionCommand::Body::GETUP_BACK;
   } else if (s.sensors[Sensors::InertialSensor_AccX] > FALLEN) {
      actions->leds.leftEye = ActionCommand::LED::rgb(blink, 0);
      actions->leds.rightEye = ActionCommand::LED::rgb(blink, 0);
      actions->body = ActionCommand::Body::GETUP_FRONT;
   // about to fall
   } else if (ABS(s.sensors[Sensors::InertialSensor_AccX]) > FALLING ||
      ABS(s.sensors[Sensors::InertialSensor_AccY]) > FALLING) {
      actions->leds.leftEye = ActionCommand::LED::rgb(blink, blink);
      actions->leds.rightEye = ActionCommand::LED::rgb(blink, blink);
      actions->body = ActionCommand::Body::DEAD;
   // if ref picks us up
   } else if (filtered_fsr_sum < MIN_STANDING_WEIGHT) {
      actions->leds.leftEye = ActionCommand::LED::rgb(0, blink);
      actions->leds.rightEye = ActionCommand::LED::rgb(0, blink);
      actions->body = ActionCommand::Body::REF_PICKUP;
   }
}
