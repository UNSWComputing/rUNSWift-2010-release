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

#include "motion/MotionAdapter.hpp"

#include <boost/bind.hpp>

#include "motion/touch/AgentTouch.hpp"
#include "motion/effector/AgentEffector.hpp"
#include "motion/touch/NullTouch.hpp"
#include "motion/touch/FilteredTouch.hpp"
#include "motion/effector/NullEffector.hpp"
#include "motion/generator/ClippedGenerator.hpp"
#include "motion/generator/DistributedGenerator.hpp"
#include "blackboard/Blackboard.hpp"
#include "utils/log.hpp"
#include "utils/body.hpp"
#include "utils/incapacitated.hpp"
#include "utils/ActionCommand.hpp"
#include "utils/JointValues.hpp"
#include "utils/SensorValues.hpp"
#include "utils/Timer.hpp"

using namespace std;
const std::string MotionAdapter::name("Motion");

void construct(Touch** touch, std::string name) {
   if (name == "Agent") *touch = (Touch*) new AgentTouch();
   if (name == "Null")  *touch = (Touch*) new NullTouch();
   if (*touch == NULL)
      llog(FATAL) << "MotionAdapter: NULL " + name + "Touch" << endl;
}

void construct(Effector** effector, std::string name) {
   if (name == "Agent") *effector = (Effector*) new AgentEffector();
   if (name == "Null")  *effector = (Effector*) new NullEffector();
   if (*effector == NULL)
      llog(FATAL) << "MotionAdapter: NULL " + name + "Effector" << endl;
}

MotionAdapter::MotionAdapter()
   : uptime(0) {
   llog(INFO) << "Constructing MotionAdapter" << endl;

   // We only construct the NullTouch/Generators, the rest are done on demand

   touches["Null"] = (Touch*)(new NullTouch());
   if (touches["Null"] == NULL)
      llog(FATAL) << "MotionAdapter: NULL NullTouch" << endl;
   touches["Agent"] = (Touch*)(NULL);
   touch = (Touch*) new FilteredTouch(touches["Null"]);

   generator = (Generator*) new ClippedGenerator(
      (Generator*) new DistributedGenerator());
   if (generator == NULL)
      llog(FATAL) << "MotionAdapter: NULL Generator" << endl;

   effectors["Null"] = (Effector*)(new NullEffector());
   if (effectors["Null"] == NULL)
      llog(FATAL) << "MotionAdapter: NULL NullEffector" << endl;
   effectors["Agent"] = (Effector*)(NULL);
   effector = effectors["Null"];

   readOptions();

   if (!readFrom(thread, configCallbacks).count(name))
      llog(WARNING) << "Possible concurrency bug.  Initialize my key in map" <<
                       endl;
   boost::function<void()> ro = boost::bind(&MotionAdapter::readOptions, this);
   writeTo(thread, configCallbacks[name], ro);

   llog(INFO) << "MotionAdapater constructed" << std::endl;
}

MotionAdapter::~MotionAdapter() {
   llog(INFO) << "Destroying MotionAdapter" << endl;

   writeTo(thread, configCallbacks[name], boost::function<void()>());

   for (std::map<std::string, Touch*>::iterator it = touches.begin();
        it != touches.end(); it++)
      delete it->second;
   delete generator;
   for (std::map<std::string, Effector*>::iterator it = effectors.begin();
        it != effectors.end(); it++)
      delete it->second;
}

void MotionAdapter::readOptions() {
   std::string e = (blackboard->config)["motion.effector"].as<string>();
   std::string t = (blackboard->config)["motion.touch"].as<string>();
   if (touches.count(t)) {
      if (touches[t] == NULL) construct(&touches[t], t);
      touch = (Touch*) new FilteredTouch(touches[t]);
   }
   if (effectors.count(e)) {
      if (effectors[e] == NULL) construct(&effectors[e], e);
      effector = effectors[e];
   }
   generator->readOptions(blackboard->config);
}

void MotionAdapter::tick() {
   Timer t;

   SensorValues sensors = touch->getSensors();
   bool standing = touch->getStanding();
   ButtonPresses buttons = touch->getButtons();
   llog(VERBOSE) << "touch->getSensors took "
                 << t.elapsed_ms() << "ms" << std::endl;
   t.restart();

   if (standing) {
      uptime = 0.0f;
   } else {
      uptime += 0.01f;
   }

   acquireLock(behaviourmotion);
   writeTo(motion, uptime, uptime);
   writeTo(motion, sensors, sensors);
   ActionCommand::All request = readFrom(behaviour, request);
   releaseLock(behaviourmotion);
   if (isIncapacitated(request.body.actionType))
      uptime = 0.0f;
   buttons |= readFrom(motion, buttons);
   writeTo(motion, buttons, buttons);
   Odometry odo = readFrom(motion, odometry);

   llog(VERBOSE) << "writeTo / readFrom took "
                 << t.elapsed_ms() << "ms" << std::endl;
   t.restart();

   if (standing) {
      generator->reset();
      request.body = ActionCommand::Body::INITIAL;
      odo.clear();
   }

   JointValues joints = generator->makeJoints(&request, &odo, sensors);
   writeTo(motion, active, request);
   writeTo(motion, odometry, odo);
   llog(VERBOSE) << "generator->makeJoints took "
                 << t.elapsed_ms() << "ms" << std::endl;
   t.restart();

   effector->actuate(joints, request.leds, request.sonar);
   llog(VERBOSE) << "effector->actuate took "
                 << t.elapsed_ms() << "ms" << std::endl;
}

