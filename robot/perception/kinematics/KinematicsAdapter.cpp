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

#include "KinematicsAdapter.hpp"
#include "blackboard/Blackboard.hpp"
#include "utils/log.hpp"
#include "perception/vision/RobotCamera.hpp"

using namespace std;

const std::string KinematicsAdapter::name("Kinematics");

KinematicsAdapter::KinematicsAdapter() {
}

KinematicsAdapter::~KinematicsAdapter() {
}

void KinematicsAdapter::tick() {
   llog(DEBUG3) << "Kinematics.. ticking away" << endl;
   t.restart();
   kinematics.sensorValues = readFrom(motion, sensors);
   kinematics.cameraOffsetXTop = readFrom(kinematics, cameraOffsetXTop);
   kinematics.cameraOffsetYTop = readFrom(kinematics, cameraOffsetYTop);
   kinematics.cameraOffsetXBottom = readFrom(kinematics, cameraOffsetXBottom);
   kinematics.cameraOffsetYBottom = readFrom(kinematics, cameraOffsetYBottom);
   kinematics.bodyPitchOffset = readFrom(kinematics, bodyPitchOffset);
   kinematics.whichCamera = readFrom(behaviour, whichCamera);
   kinematics.updateDHChain();
   Pose pose = kinematics.getPose();
   llog(DEBUG3) << "Kinematics took: " << t.elapsed_us() << " us" << endl;
   llog(DEBUG3) << "Result is: " << pose.cameraToWorldTransform << std::endl;

   RRCoord coord = pose.imageToRobotRelative(320, 240, 0);
   llog(INFO) << "exp3: " << coord.distance << endl;
   writeTo(kinematics, pose, pose);
}

