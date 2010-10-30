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

#include "motion/generator/ALWalkGenerator.hpp"
#include <cmath>
#include "libagent/AgentData.hpp"
#include "utils/angles.hpp"
#include "utils/clip.hpp"
#include "utils/log.hpp"

using boost::program_options::variables_map;
using namespace std;

ALWalkGenerator::ALWalkGenerator()
   : running(false),
     stop_req(false),
     height(0.31f),
     bend(DEG2RAD(30.0f)) {
   llog(INFO) << "ALWalkGenerator constructed" << std::endl;
}

ALWalkGenerator::~ALWalkGenerator() {
   llog(INFO) << "ALWalkGenerator destroyed" << std::endl;
}

JointValues ALWalkGenerator::makeJoints(ActionCommand::All* request,
                                        Odometry* odometry,
                                        const SensorValues &sensors) {
   running = sensors.AL_isActive;
   JointValues j = sensors.joints;
   j.AL_command = AL_ON;
   j.AL_x = CLIP((float)request->body.forward/40.0f, -1.0f, 1.0f);
   j.AL_y = CLIP((float)request->body.left/40.0f, -1.0f, 1.0f);
   j.AL_theta = CLIP((float)(request->body.turn/DEG2RAD(40.0f)), -1.0f, 1.0f);
   j.AL_height = height;
   j.AL_bend = bend;
   j.AL_frequency = 1.0f;
   j.AL_stop = stop_req;
   stop_req = false;
   // Odometry magic numbers out of ALDocs. This may be considered by some
   // to be an ugly hack, but I contend this whole generator is an ugly hack :P
   // -- stuartr
   // I concur
   // -- davidc
   *odometry = *odometry + Odometry(j.AL_x * 0.95f, j.AL_y * 0.45f,
                                    j.AL_theta * DEG2RAD(0.95f));
   return j;
}

bool ALWalkGenerator::isActive() {
   return running;
}

void ALWalkGenerator::reset() {
   running = false;
}

void ALWalkGenerator::stop() {
   stop_req = true;
}

void ALWalkGenerator::readOptions(variables_map& config) {
   bend = config["walk.b"].as<float>();
   static const float a = -5248;
   static const float b = 2047;
   static float c = -77 - bend*2;
   height = (-b - sqrt(b*b - 4*a*c))/(2*a);
   bend = DEG2RAD(bend);
   llog(INFO) << "Successfully changed AL walk options" << std::endl;
}
