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

#include "motion/generator/KickGenerator.hpp"
#include <cmath>
#include "utils/angles.hpp"
#include "utils/basic_maths.hpp"
#include "utils/clip.hpp"
#include "utils/log.hpp"

using boost::program_options::variables_map;
using namespace Joints;
using namespace Sensors;

// Times are in cycles, but stored as floats for conveniance
const float KickGenerator::times[NUMBER_OF_STATES] = {
   0,
   50,
   20,
   50,
   0,  // LINEUP is not timed, but dynamic!
   100,
   100,
   // 100,
   100,
   100
};

const std::string KickGenerator::name[NUMBER_OF_STATES] = {
   "INACTIVE",
   "LEAN",
   "PAUSE1",
   "LIFT",
   "LINEUP",
   "KICK",
   "UN LINEUP",
   // "UN LIFT",
   "UN LEAN",
   "PAUSE2"
};

inline bool MOVE(JointValues& j, JointCode joint,
                 float target, float speed=1.0) {
   float max_dist = Radians::MaxSpeed[joint] * ABS(speed);
   if (ABS(target - j.angles[joint]) < max_dist) {
      j.angles[joint] = target;
      return true;
   } else if (target < j.angles[joint]) {
      j.angles[joint] -= max_dist;
   } else {
      j.angles[joint] += max_dist;
   }
   return false;
}

inline void INTERP(JointValues& j, JointCode joint,
                   JointValues& from, float to, float step) {
   j.angles[joint] = from.angles[joint] * (1 - step) + to * step;
}

KickGenerator::KickGenerator()
   : state(INACTIVE),
     t(0) {
   llog(INFO) << "KickGenerator constructed" << std::endl;
}

KickGenerator::~KickGenerator() {
   llog(INFO) << "KickGenerator destroyed" << std::endl;
}

JointValues KickGenerator::makeJoints(ActionCommand::All* request,
                                      Odometry* odometry,
                                      const SensorValues &sensors) {
   static JointValues j = sensors.joints;
   for (uint8_t i = 0; i < NUMBER_OF_JOINTS; ++i)
      j.stiffnesses[i] = 1.0;

   if (state == INACTIVE && request->body.left &&
       ABS(request->body.left) <= 125) {
      j = sensors.joints;
      left_leg = (request->body.left > 0);
      state = LEAN;
   }

   static float leg_angle = 0;
   static float leg_length = 225;
   static float leg_twist = 0;

   if (state != KICK && request->body.actionType == ActionCommand::Body::KICK) {
      leg_angle = atan2(ABS(request->body.left) - 50, 225);
      leg_length = sqrt(SQUARE(ABS(request->body.left) - 50)
                      + SQUARE(225));
      leg_twist = (left_leg?-1:1) * request->body.turn;
   }

   static JointValues from;
   bool next;
   switch (state) {
   case LEAN:
   // case UNLIFT:
      if (t == 0) {
         from = j;
         llog(DEBUG1) << name[state] << std::endl;
      }
      INTERP(j, LShoulderPitch, from, DEG2RAD(90), t/times[state]);
      INTERP(j, LShoulderRoll, from, DEG2RAD(15), t/times[state]);
      INTERP(j, LElbowYaw, from, DEG2RAD(0), t/times[state]);
      INTERP(j, LElbowRoll, from, DEG2RAD(0), t/times[state]);

      INTERP(j, RShoulderPitch, from, DEG2RAD(90), t/times[state]);
      INTERP(j, RShoulderRoll, from, DEG2RAD(-15), t/times[state]);
      INTERP(j, RElbowYaw, from, DEG2RAD(0), t/times[state]);
      INTERP(j, RElbowRoll, from, DEG2RAD(0), t/times[state]);

      INTERP(j, LHipYawPitch, from, DEG2RAD(0), t/times[state]);
      INTERP(j, LHipRoll, from,
             (left_leg?-1:1) * DEG2RAD(22), t/times[state]);
      INTERP(j, LHipPitch, from,
             left_leg?DEG2RAD(0):DEG2RAD(-40), t/times[state]);
      INTERP(j, LKneePitch, from,
             left_leg?DEG2RAD(0):DEG2RAD(70), t/times[state]);
      INTERP(j, LAnklePitch, from,
             left_leg?DEG2RAD(0):DEG2RAD(-30), t/times[state]);
      INTERP(j, LAnkleRoll, from,
             left_leg?DEG2RAD(-6):DEG2RAD(6), t/times[state]);

      INTERP(j, RHipRoll, from,
             (left_leg?-1:1) * DEG2RAD(22), t/times[state]);
      INTERP(j, RHipPitch, from,
             left_leg?DEG2RAD(-40):DEG2RAD(0), t/times[state]);
      INTERP(j, RKneePitch, from,
             left_leg?DEG2RAD(70):DEG2RAD(0), t/times[state]);
      INTERP(j, RAnklePitch, from,
             left_leg?DEG2RAD(-30):DEG2RAD(0), t/times[state]);
      INTERP(j, RAnkleRoll, from,
             left_leg?DEG2RAD(-6):DEG2RAD(6), t/times[state]);
      if (t > times[state]) {
         state = (KickState)(state + 1);
         t = 0;
      } else {
         t++;
      }
      break;
   case LIFT:
   case UNLINEUP:
      if (t == 0) {
         from = j;
         llog(DEBUG1) << name[state] << std::endl;
      }
      INTERP(j, LShoulderPitch, from, DEG2RAD(90), t/times[state]);
      INTERP(j, LShoulderRoll, from, DEG2RAD(15), t/times[state]);
      INTERP(j, LElbowYaw, from, DEG2RAD(0), t/times[state]);
      INTERP(j, LElbowRoll, from, DEG2RAD(0), t/times[state]);

      INTERP(j, RShoulderPitch, from, DEG2RAD(90), t/times[state]);
      INTERP(j, RShoulderRoll, from, DEG2RAD(-15), t/times[state]);
      INTERP(j, RElbowYaw, from, DEG2RAD(0), t/times[state]);
      INTERP(j, RElbowRoll, from, DEG2RAD(0), t/times[state]);

      INTERP(j, LHipYawPitch, from, DEG2RAD(0), t/times[state]);
      INTERP(j, LHipRoll, from,
             (left_leg?-1:1) * DEG2RAD(22), t/times[state]);
      INTERP(j, LHipPitch, from,
             left_leg?DEG2RAD(-30):DEG2RAD(-40), t/times[state]);
      INTERP(j, LKneePitch, from,
             left_leg?DEG2RAD(60):DEG2RAD(80), t/times[state]);
      INTERP(j, LAnklePitch, from,
             left_leg?DEG2RAD(-30):DEG2RAD(-40), t/times[state]);
      if (left_leg) {
         INTERP(j, LAnkleRoll, from, DEG2RAD(0), t/times[state]);
      } else {
         INTERP(j, LAnkleRoll, from, DEG2RAD(6), t/times[state]);
      }

      INTERP(j, RHipRoll, from,
             (left_leg?-1:1) * DEG2RAD(22), t/times[state]);
      INTERP(j, RHipPitch, from,
             left_leg?DEG2RAD(-40):DEG2RAD(-30), t/times[state]);
      INTERP(j, RKneePitch, from,
             left_leg?DEG2RAD(80):DEG2RAD(60), t/times[state]);
      INTERP(j, RAnklePitch, from,
             left_leg?DEG2RAD(-40):DEG2RAD(-30), t/times[state]);
      if (left_leg) {
         INTERP(j, RAnkleRoll, from, DEG2RAD(-6), t/times[state]);
      } else {
         INTERP(j, RAnkleRoll, from, DEG2RAD(0), t/times[state]);
      }

      if (t > times[state]) {
         state = (KickState)(state + 1);
         t = 0;
      } else {
         t++;
      }
      break;
   case LINEUP:
      if (t == 0) {
         from = j;
         llog(DEBUG1) << name[state] << std::endl;
      }
      next = true;
      j.angles[LShoulderPitch] = DEG2RAD(90);
      j.angles[LShoulderRoll] = DEG2RAD(15);
      j.angles[LElbowYaw] = DEG2RAD(0);
      j.angles[LElbowRoll] = DEG2RAD(0);

      j.angles[RShoulderPitch] = DEG2RAD(90);
      j.angles[RShoulderRoll] = DEG2RAD(-15);
      j.angles[RElbowYaw] = DEG2RAD(0);
      j.angles[RElbowRoll] = DEG2RAD(0);

      next &= MOVE(j, LHipYawPitch, leg_twist, 0.3);

      if (left_leg) {
         j.angles[RHipRoll] = DEG2RAD(-22);
         j.angles[RHipPitch] = DEG2RAD(-40 + leg_twist);
         j.angles[RKneePitch] = DEG2RAD(80);
         j.angles[RAnklePitch] = DEG2RAD(-40);
         next &= MOVE(j, RAnkleRoll, -(DEG2RAD(6) + leg_angle/4), 0.3);

         next &= MOVE(j, LHipRoll, DEG2RAD(-22) + leg_angle, 0.3);
         next &= MOVE(j, LAnkleRoll, -leg_angle, 0.3);
         next &= MOVE(j, LHipPitch,
                      DEG2RAD(-20 + leg_twist +
                              30*(leg_length - 225)/20.0f), 0.3);
         next &= MOVE(j, LKneePitch,
                      DEG2RAD(60 - 60*(leg_length - 225)/20.0f), 0.3);
         next &= MOVE(j, LAnklePitch,
                      DEG2RAD(-40 + 30*(leg_length - 225)/20.0f), 0.3);
      } else {
         j.angles[LHipRoll] = DEG2RAD(22);
         j.angles[LHipPitch] = DEG2RAD(-40 + leg_twist);
         j.angles[LKneePitch] = DEG2RAD(80);
         j.angles[LAnklePitch] = DEG2RAD(-40);
         next &= MOVE(j, LAnkleRoll, DEG2RAD(6) + leg_angle/4, 0.3);

         next &= MOVE(j, RHipRoll, DEG2RAD(22) - leg_angle, 0.3);
         next &= MOVE(j, RAnkleRoll, leg_angle, 0.3);
         next &= MOVE(j, RHipPitch,
                      DEG2RAD(-20 + leg_twist +
                              30*(leg_length - 225)/20.0f), 0.3);
         next &= MOVE(j, RKneePitch,
                      DEG2RAD(60 - 60*(leg_length - 225)/20.0f), 0.3);
         next &= MOVE(j, RAnklePitch,
                      DEG2RAD(-40 + 30*(leg_length - 225)/20.0f), 0.3);
      }
      if (next) {
         state = (KickState)(state + 1);
         t = 0;
      } else {
         t++;
      }
      break;
   case KICK:
      if (t == 0) {
         from = j;
         llog(DEBUG1) << name[state] << std::endl;
      }
      j.angles[LShoulderPitch] = DEG2RAD(90);
      j.angles[LShoulderRoll] = DEG2RAD(15);
      j.angles[LElbowYaw] = DEG2RAD(0);
      j.angles[LElbowRoll] = DEG2RAD(0);

      j.angles[RShoulderPitch] = DEG2RAD(90);
      j.angles[RShoulderRoll] = DEG2RAD(-15);
      j.angles[RElbowYaw] = DEG2RAD(0);
      j.angles[RElbowRoll] = DEG2RAD(0);

      j.angles[LHipYawPitch] = leg_twist;

      if (left_leg) {
         j.angles[RHipRoll] = DEG2RAD(-22);
         j.angles[RHipPitch] = DEG2RAD(-40 + leg_twist);
         j.angles[RKneePitch] = DEG2RAD(80);
         j.angles[RAnklePitch] = DEG2RAD(-40);
         MOVE(j, RAnkleRoll, -(DEG2RAD(6) + leg_angle/4), 0.3);

         MOVE(j, LHipRoll, DEG2RAD(-22) + leg_angle, 0.3);
         MOVE(j, LAnkleRoll, -leg_angle, 0.3);
         MOVE(j, LHipPitch,
              DEG2RAD(-60 + leg_twist + 30*(leg_length - 225)/20.0f), 1.0);
         MOVE(j, LKneePitch, DEG2RAD(30), 0.3);
         MOVE(j, LAnklePitch,
                 DEG2RAD(0 + 30*(leg_length - 225)/20.0f), 1.0);
      } else {
         j.angles[LHipRoll] = DEG2RAD(22);
         j.angles[LHipPitch] = DEG2RAD(-40 + leg_twist);
         j.angles[LKneePitch] = DEG2RAD(80);
         j.angles[LAnklePitch] = DEG2RAD(-40);
         MOVE(j, LAnkleRoll, DEG2RAD(6) + leg_angle/4, 0.3);

         MOVE(j, RHipRoll, DEG2RAD(22) - leg_angle, 0.3);
         MOVE(j, RAnkleRoll, leg_angle, 0.3);
         MOVE(j, RHipPitch,
              DEG2RAD(-60 + leg_twist + 30*(leg_length - 225)/20.0f), 1.0);
         MOVE(j, RKneePitch, DEG2RAD(30), 0.3);
         MOVE(j, RAnklePitch,
                 DEG2RAD(0 + 30*(leg_length - 225)/20.0f), 1.0);
      }
      if (t > times[state]) {
         state = (KickState)(state + 1);
         t = 0;
      } else {
         t++;
      }
      break;
   case UNLEAN:
      if (t == 0) {
         from = j;
         llog(DEBUG1) << name[state] << std::endl;
      }
      INTERP(j, LShoulderPitch, from, DEG2RAD(90), t/times[state]);
      INTERP(j, LShoulderRoll, from, DEG2RAD(15), t/times[state]);
      INTERP(j, LElbowYaw, from, DEG2RAD(0), t/times[state]);
      INTERP(j, LElbowRoll, from, DEG2RAD(0), t/times[state]);

      INTERP(j, RShoulderPitch, from, DEG2RAD(90), t/times[state]);
      INTERP(j, RShoulderRoll, from, DEG2RAD(-15), t/times[state]);
      INTERP(j, RElbowYaw, from, DEG2RAD(0), t/times[state]);
      INTERP(j, RElbowRoll, from, DEG2RAD(0), t/times[state]);

      INTERP(j, LHipYawPitch, from, DEG2RAD(0), t/times[state]);
      INTERP(j, LHipRoll, from, DEG2RAD(0), t/times[state]);
      INTERP(j, LHipPitch, from, DEG2RAD(-30), t/times[state]);
      INTERP(j, LKneePitch, from, DEG2RAD(60), t/times[state]);
      INTERP(j, LAnklePitch, from, DEG2RAD(-30), t/times[state]);
      INTERP(j, LAnkleRoll, from, DEG2RAD(0), t/times[state]);

      INTERP(j, RHipRoll, from, DEG2RAD(0), t/times[state]);
      INTERP(j, RHipPitch, from, DEG2RAD(-30), t/times[state]);
      INTERP(j, RKneePitch, from, DEG2RAD(60), t/times[state]);
      INTERP(j, RAnklePitch, from, DEG2RAD(-30), t/times[state]);
      INTERP(j, RAnkleRoll, from, DEG2RAD(0), t/times[state]);
      if (t > times[state]) {
         state = (KickState)(state + 1);
         t = 0;
      } else {
         t++;
      }
      break;
   case PAUSE1:
   case PAUSE2:
      if (t > times[state]) {
         state = (KickState)(state + 1);
         t = 0;
      } else {
         t++;
      }
      break;
   case INACTIVE:
   case NUMBER_OF_STATES:
      break;
   }
   if (state == NUMBER_OF_STATES) state = INACTIVE;
   return j;
}

bool KickGenerator::isActive() {
   return state != INACTIVE;
}

void KickGenerator::reset() {
   t = 0;
   state = INACTIVE;
}

void KickGenerator::readOptions(variables_map& config) {}
