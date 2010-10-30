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

#include <pthread.h>
#include <ctime>
#include "perception/PerceptionThread.hpp"
#include "blackboard/Blackboard.hpp"
#include "utils/log.hpp"
#include "boost/lexical_cast.hpp"

using namespace std;

const std::string PerceptionThread::name("Perception");

PerceptionThread::PerceptionThread() {
   releaseLock(vision);
   uint8_t const* currentFrame = readFrom(vision, currentFrame);
   if (currentFrame != NULL) {
      string file = "/home/nao/crashframe-" +
         boost::lexical_cast<string>(time(NULL)) + ".yuv";
      FILE *errorFrameFile = fopen(file.c_str(), "w");
      fwrite(currentFrame, 640*480*2, 1, errorFrameFile);
      fclose(errorFrameFile);
      file = "/usr/bin/tail -n 200 /var/volatile/runswift/vision.log > "
         + file + ".log";
      system(file.c_str());
   }
}

PerceptionThread::~PerceptionThread() {
}

void PerceptionThread::tick() {
   llog(DEBUG1) << "Perception.. ticking away" << endl;
   Timer t1;
   Timer t;
   kinematicsAdapter.tick();
   uint32_t kinematics = t.elapsed_us();
   llog(VERBOSE) << "kinematics tick took " << kinematics << endl;
   t.restart();
   if (blackboard->config["debug.vision"].as<bool>()) {
      visionAdapter.tick();
   }
   uint32_t vision = t.elapsed_us();
   llog(VERBOSE) << "vision tick took " << vision << endl;
   t.restart();
   localisationAdapter.tick();
   uint32_t localisation = t.elapsed_us();
   llog(VERBOSE) << "localisation tick took " << localisation << endl;
   t.restart();
   if (blackboard->config["debug.behaviour"].as<bool>()) {
      behaviourAdapter.tick();
   }
   uint32_t behaviour = t.elapsed_us();
   llog(VERBOSE) << "behaviour tick took " << behaviour << endl;
   uint32_t total = t1.elapsed_us();
   llog(VERBOSE) << "perception took " << total << endl;

   writeTo(perception, kinematics, kinematics);
   writeTo(perception, vision, vision);
   writeTo(perception, localisation, localisation);
   writeTo(perception, behaviour, behaviour);
   writeTo(perception, total, total);
}

