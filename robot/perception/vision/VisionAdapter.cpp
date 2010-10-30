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

#include "VisionAdapter.hpp"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/bind.hpp>
#include <pthread.h>
#include <vector>
#include "blackboard/Blackboard.hpp"
#include "utils/log.hpp"
#include "perception/kinematics/Pose.hpp"

using namespace std;
using namespace boost::algorithm;

const std::string VisionAdapter::name("Vision");

VisionAdapter::VisionAdapter()
   : V((blackboard->config)["vision.dumpframes"].as<bool>(),
     (blackboard->config)["vision.dumprate"].as<int>(),
     (blackboard->config)["vision.dumpfile"].as<string>(),
     (blackboard->config)["debug.vision"].as<bool>()) {
   writeTo(vision, saliency, (Colour*)V.saliency);

   readOptions();

   if (!readFrom(thread, configCallbacks).count(name))
      llog(WARNING) << "Possible concurrency bug.  Initialize my key in map" <<
            endl;
   boost::function<void()> ro = boost::bind(&VisionAdapter::readOptions, this);
   writeTo(thread, configCallbacks[name], ro);
}

VisionAdapter::~VisionAdapter() {
}

void VisionAdapter::tick() {
   timer.restart();

   llog(VERBOSE) << "Vision.. ticking away" << endl;
   Timer t;

   // Read current Pose from blakboard
   V.convRR.pose = readFrom(kinematics, pose);

   // Read whichCamera from blackboard
   V.whichCamera = readFrom(behaviour, whichCamera);
   V.getFrame();
   writeTo(vision, currentFrame, V.currentFrame);
   V.convRR.findEndScanValues();
   llog(VERBOSE) << "getFrame() took " << t.elapsed_us() << " us" << endl;
   t.restart();
   acquireLock(vision);
   llog(VERBOSE) << "acquireLock(vision) took " << t.elapsed_us()
      << " us" << endl;
   t.restart();
   V.saliencyScan();
   llog(VERBOSE) << "saliencyScan() took " << t.elapsed_us() << " us" << endl;
   t.restart();
   SensorValues values = readFrom(motion, sensors);
   V.convRR.updateAngles(values);
   pthread_yield();
   V.processFrame();
   llog(VERBOSE) << "processFrame() took " << t.elapsed_us() << " us" << endl;
   t.restart();

   writeTo(vision, numEdges, (int)V.fieldEdgeDetection.numEdgeLines);
   if (V.fieldEdgeDetection.numEdgeLines != 0) {
      writeArray(vision, edges, V.fieldEdgeDetection.RRedgeLines);
      writeArray(vision, frameEdges, V.fieldEdgeDetection.edgeLines);
   }

   writeTo(vision, posts, V.goalDetection.typeOfPost);
   writeArray(vision, post, V.goalDetection.posts);
   writeArray(vision, postCoords, V.goalDetection.postCoords);
   // Temporary writes to the blackboard
   writeArray(vision, distanceProjection, V.goalDetection.distanceProjection);
   writeArray(vision, distanceGoalPostWidth,
         V.goalDetection.distanceGoalPostWidth);
   writeTo(vision, distanceGoalSep, V.goalDetection.distanceGoalSep);
   writeArray(vision, canSeeBottom, V.goalDetection.canSeeBottom);

   writeTo(vision, numBalls, V.ballDetection.numBalls);
   if (V.ballDetection.numBalls) {
      writeArray(vision, ball, V.ballDetection.ballLoc);
      writeTo(vision, ballInCameraCoords, V.ballDetection.ballCentre);
      writeTo(vision, ballRadius, V.ballDetection.radius);
   }
   writeTo(vision, numFieldLinePoints,
         V.fieldLineDetection.numFieldLinePoints);
   writeArray(vision, fieldLinePoints, V.fieldLineDetection.fieldLinePoints);

   writeTo(vision, numRobots, V.robotDetection.numRobots);
   writeArray(vision, robotTypes, V.robotDetection.robotTypes);
   writeArray(vision, robotLocations, V.robotDetection.robotLocations);
   writeArray(vision, canSeeBottomRobot, V.robotDetection.canSeeBottomRobot);
   writeArray(vision, robotImageCoords, V.robotDetection.robotImageCoords);
   releaseLock(vision);
   llog(VERBOSE) << "Blackboard write took " << t.elapsed_us() << " us" << endl;
   llog(VERBOSE) << "Vision took " << timer.elapsed_us() << "us" << endl;
   if (timer.elapsed_us() > 33666) {
      llog(VERBOSE) << "WARNING: Vision took too long!" << endl;
   }
}

void VisionAdapter::readOptions() {
   const string &e =
         (blackboard->config)["vision.camera_controls"].as<string>();
   vector<string> vs;
   split(vs, e, is_any_of(",;"));
   for (vector<string>::const_iterator ci = vs.begin(); ci != vs.end(); ++ci) {
      vector<string> nv;
      split(nv, *ci, is_any_of(":"));
      if (nv.size() != 2)
         llog(ERROR) << "controls should be control_id:value" << endl;
      else
         V.camera->setControl(strtoul(nv[0].c_str(), NULL, 0),
                              strtol(nv[1].c_str(), NULL, 0));
   }
}
