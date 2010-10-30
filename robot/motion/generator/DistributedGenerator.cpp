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

#include "motion/generator/DistributedGenerator.hpp"
#include "motion/generator/ActionGenerator.hpp"
#include "motion/generator/ALWalkGenerator.hpp"
#include "motion/generator/DeadGenerator.hpp"
#include "motion/generator/HeadGenerator.hpp"
#include "motion/generator/KickGenerator.hpp"
#include "motion/generator/NullGenerator.hpp"
#include "motion/generator/RefPickupGenerator.hpp"
#include "motion/generator/StandGenerator.hpp"
#include "motion/generator/SlowWalkGenerator.hpp"
#include "motion/generator/FastWalkGenerator.hpp"
#include "motion/generator/WaveWalkGenerator.hpp"
#include "utils/body.hpp"
#include "utils/log.hpp"

using ActionCommand::Body;
using boost::program_options::variables_map;

DistributedGenerator::DistributedGenerator() : current_generator(Body::NONE) {
   headGenerator = (Generator*)(new HeadGenerator());
   if (!headGenerator)
      llog(FATAL) << "headGenerator is NULL!" << std::endl;

   bodyGenerators[Body::NONE] = (Generator*)(new NullGenerator());
   if (!bodyGenerators[Body::NONE])
      llog(FATAL) << "bodyGenerators[NONE] is NULL!" << std::endl;

   bodyGenerators[Body::STAND] = (Generator*)(new StandGenerator());
   if (!bodyGenerators[Body::STAND])
      llog(FATAL) << "bodyGenerators[STAND] is NULL!" << std::endl;

   bodyGenerators[Body::SLOW] = (Generator*)(new SlowWalkGenerator());
   if (!bodyGenerators[Body::SLOW])
      llog(FATAL) << "bodyGenerators[SLOW] is NULL!" << std::endl;

   bodyGenerators[Body::FAST] = (Generator*)(new FastWalkGenerator());
   if (!bodyGenerators[Body::FAST])
      llog(FATAL) << "bodyGenerators[FAST] is NULL!" << std::endl;

   bodyGenerators[Body::WAVE] = (Generator*)(new WaveWalkGenerator());
   if (!bodyGenerators[Body::WAVE])
      llog(FATAL) << "bodyGenerators[WAVE] is NULL!" << std::endl;

   bodyGenerators[Body::AL] = (Generator*)(new ALWalkGenerator());
   if (!bodyGenerators[Body::AL])
      llog(FATAL) << "bodyGenerators[AL] is NULL!" << std::endl;

   bodyGenerators[Body::WALK] = bodyGenerators[Body::AL];

   bodyGenerators[Body::GETUP_FRONT] = (Generator*)
                                 (new ActionGenerator("getupFront"));
   if (!bodyGenerators[Body::GETUP_FRONT])
      llog(FATAL) << "bodyGenerators[GETUP_FRONT] is NULL!" << std::endl;

   bodyGenerators[Body::GETUP_BACK] = (Generator*)
                                 (new ActionGenerator("getupBack"));
   if (!bodyGenerators[Body::GETUP_BACK])
      llog(FATAL) << "bodyGenerators[GETUP_BACK] is NULL!" << std::endl;

   bodyGenerators[Body::KICK] = (Generator*)
                                 (new KickGenerator());
   if (!bodyGenerators[Body::KICK])
      llog(FATAL) << "bodyGenerators[KICK] is NULL!" << std::endl;

   bodyGenerators[Body::INITIAL] = (Generator*)
                                 (new ActionGenerator("initial"));
   if (!bodyGenerators[Body::INITIAL])
      llog(FATAL) << "bodyGenerators[INITIAL] is NULL!" << std::endl;

   bodyGenerators[Body::DEAD] = (Generator*)(new DeadGenerator());
   if (!bodyGenerators[Body::DEAD])
      llog(FATAL) << "bodyGenerators[DEAD] is NULL!" << std::endl;

   bodyGenerators[Body::REF_PICKUP] = (Generator*)(new RefPickupGenerator());
   if (!bodyGenerators[Body::REF_PICKUP])
      llog(FATAL) << "bodyGenerators[REF_PICKUP] is NULL!" << std::endl;

   // these are the testing poses
   bodyGenerators[Body::SQUAT] = (Generator*)
                                 (new ActionGenerator("squat"));
   if (!bodyGenerators[Body::SQUAT])
      llog(FATAL) << "bodyGenerators[SQUAT] is NULL!" << std::endl;

   bodyGenerators[Body::SQUAT_FORWARD] = (Generator*)
                                 (new ActionGenerator("squatForward"));
   if (!bodyGenerators[Body::SQUAT_FORWARD])
      llog(FATAL) << "bodyGenerators[SQUAT_FORWARD] is NULL!" << std::endl;

   bodyGenerators[Body::OPEN_FEET] = (Generator*)
                                 (new ActionGenerator("openFeet"));
   if (!bodyGenerators[Body::OPEN_FEET])
      llog(FATAL) << "bodyGenerators[OPEN_FEET] is NULL!" << std::endl;

   bodyGenerators[Body::THROW_IN] = (Generator*)
                                 (new ActionGenerator("throwIn"));
   if (!bodyGenerators[Body::THROW_IN])
      llog(FATAL) << "bodyGenerators[THROW_IN] is NULL!" << std::endl;

   bodyGenerators[Body::GOALIE_SIT] = (Generator*)
                                 (new ActionGenerator("goalieSit"));
   if (!bodyGenerators[Body::GOALIE_SIT])
      llog(FATAL) << "bodyGenerators[GOALIE_SIT] is NULL!" << std::endl;

   llog(INFO) << "DistributedGenerator constructed" << std::endl;
}

DistributedGenerator::~DistributedGenerator() {
   delete headGenerator;
   for (uint8_t i = 0; i < Body::NUM_ACTION_TYPES; ++i)
      if (bodyGenerators[i]) {
         delete bodyGenerators[i];
         for (uint8_t j = i + 1; j < Body::NUM_ACTION_TYPES; ++j)
            if (bodyGenerators[j] == bodyGenerators[i])
               bodyGenerators[j] = NULL;
      }
   llog(INFO) << "DistributedGenerator destroyed" << std::endl;
}

JointValues DistributedGenerator::makeJoints(ActionCommand::All* request,
                                             Odometry* odometry,
                                             const SensorValues &sensors) {
   JointValues fromBody;
   bool usesHead;
   if (ActionCommand::priorities[request->body.actionType] >
       ActionCommand::priorities[current_generator])
      reset();
   if (!bodyGenerators[current_generator]->isActive()) {
      current_generator = request->body.actionType;
   } else if (bodyGenerators[current_generator]->isActive() &&
              bodyGenerators[current_generator] !=
              bodyGenerators[request->body.actionType]) {
      bodyGenerators[current_generator]->stop();
   }
   switch (current_generator) {
   case Body::NONE:             usesHead = false; break;
   case Body::STAND:            usesHead = false; break;
   case Body::SLOW:             usesHead = false; break;
   case Body::FAST:             usesHead = false; break;
   case Body::WAVE:             usesHead = false; break;
   case Body::AL:               usesHead = false; break;
   case Body::WALK:             usesHead = false; break;
   case Body::GETUP_FRONT:      usesHead = true;  break;
   case Body::GETUP_BACK:       usesHead = true;  break;
   case Body::INITIAL:          usesHead = true;  break;
   case Body::KICK:             usesHead = false; break;
   case Body::DEAD:             usesHead = true;  break;
   case Body::REF_PICKUP:       usesHead = false; break;
   case Body::SQUAT:            usesHead = false; break;
   case Body::SQUAT_FORWARD:    usesHead = false; break;
   case Body::OPEN_FEET:        usesHead = true;  break;
   case Body::THROW_IN:         usesHead = false; break;
   case Body::GOALIE_SIT:       usesHead = false; break;
   case Body::NUM_ACTION_TYPES: usesHead = false; break;
   }
   fromBody = bodyGenerators[current_generator]->
              makeJoints(request, odometry, sensors);
   request->body.actionType = current_generator;
   if (!usesHead) {
      JointValues fromHead = headGenerator->
                             makeJoints(request, odometry, sensors);
      for (uint8_t i = Joints::HeadYaw; i <= Joints::HeadPitch; ++i) {
         fromBody.angles[i] = fromHead.angles[i];
         fromBody.stiffnesses[i] = fromHead.stiffnesses[i];
      }
   }

   return fromBody;
}

bool DistributedGenerator::isActive() { return true; }

void DistributedGenerator::reset() {
   for (uint8_t i = 0; i < Body::NUM_ACTION_TYPES; ++i)
      bodyGenerators[i]->reset();
   headGenerator->reset();
   current_generator = ActionCommand::Body::NONE;
}

void DistributedGenerator::readOptions(variables_map& config) {
   std::string w = config["motion.walk"].as<std::string>();
   if (w == "Slow")
      bodyGenerators[Body::WALK] = bodyGenerators[Body::SLOW];
   else if (w == "Fast")
      bodyGenerators[Body::WALK] = bodyGenerators[Body::FAST];
   else if (w == "Wave")
      bodyGenerators[Body::WALK] = bodyGenerators[Body::WAVE];
   else if (w == "AL")
      bodyGenerators[Body::WALK] = bodyGenerators[Body::AL];
   for (uint8_t i = 0; i < Body::NUM_ACTION_TYPES; ++i)
      bodyGenerators[i]->readOptions(config);
   headGenerator->readOptions(config);
}
