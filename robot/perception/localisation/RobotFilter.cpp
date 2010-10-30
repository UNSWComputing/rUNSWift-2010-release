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

#include "RobotFilter.hpp"
#include "utils/angles.hpp"

using namespace std;

#define MAX_ROBOT_LOST_COUNT 250
#define PROCESS_VAR 10
#define MAX_OBS_HYPOTH_DIFF DEG2RAD(15.0)

RobotFilter::RobotFilter() {
}

void RobotFilter::update() {
   Odometry deltaOdometry = odometry - prev_odometry;
   float deltaF = deltaOdometry.forward;
   float deltaS = deltaOdometry.left;
   float deltaT = deltaOdometry.turn;
   float deltaX = deltaF*cos(robotPos.theta) - deltaS*sin(robotPos.theta);
   float deltaY = deltaF*sin(robotPos.theta) + deltaS*cos(robotPos.theta);
   prev_odometry = odometry;

   list<RobotObstacle>::iterator i;

   // remove old hypotheses
   i = robots.begin();
   while (i != robots.end()) {
      if (i->lostCount > MAX_ROBOT_LOST_COUNT) {
         i = robots.erase(i);
      } else {
         ++i;
      }
   }

   for (i = robots.begin(); i != robots.end(); ++i) {
      // odometry update
      RRCoord &pos = i->pos;
      int &lostCount = i->lostCount;
      float alpha = NORMALISE(robotPos.theta + pos.heading);
      float robotx = pos.distance*cos(alpha) + robotPos.x - deltaX;
      float roboty = pos.distance*sin(alpha) + robotPos.y - deltaY;
      pos.heading = NORMALISE(pos.heading - deltaT);
      pos.distance = sqrt(SQUARE(robotx - robotPos.x)
            + SQUARE(roboty - robotPos.y));
      // lost count update
      lostCount++;
      // process variance update
      pos.var[X] = SQUARE(sqrt(pos.var[X]) + PROCESS_VAR);
      pos.var[Y] = SQUARE(sqrt(pos.var[Y]) + PROCESS_VAR);
   }

   // process new observations
   for (int j = 0; j < obsNumRobots; ++j) {
      float min_diff = MAX_OBS_HYPOTH_DIFF;
      list<RobotObstacle>::iterator best_match;
      for (i = robots.begin(); i != robots.end(); ++i) {
         RRCoord &pos = i->pos;
         RobotType &type = i->type;
         const float alpha = ABS(obsRobotLocations[j].heading - pos.heading);
         if ((alpha < min_diff) && ((obsRobotTypes[j] == type) ||
                                   (obsRobotTypes[j] == UNKNOWN_ROBOT))) {
            min_diff = alpha;
            best_match = i;
         }
      }
      if (min_diff < MAX_OBS_HYPOTH_DIFF) {
         llog(INFO) << "updating an existing robot hypothesis" << endl;
         // update hypothesis
         if (best_match->type == UNKNOWN_ROBOT) {
            best_match->type = obsRobotTypes[j];
         }
         // pos & var update
         float k = best_match->pos.var[X] /
         (best_match->pos.var[X] + obsRobotLocations[j].var[DIST]);
         best_match->lostCount = 0;
         best_match->seenCount++;
         if (obsCanSeeBottom[j]) {
            best_match->pos.distance = best_match->pos.distance +
               k * (obsRobotLocations[j].distance - best_match->pos.distance);
            best_match->pos.var[DIST] = (1-k) * best_match->pos.var[DIST];
         }
         best_match->pos.heading = NORMALISE(best_match->pos.heading +
            k * (obsRobotLocations[j].heading - best_match->pos.heading));
         best_match->pos.var[THETA] = (1-k) * best_match->pos.var[THETA];
      } else {
         llog(INFO) << "generating a new robot hypothesis" << endl;
         // generate new hypothesis
         if (obsCanSeeBottom[j]) {
            RobotObstacle newHypoth;
            newHypoth.lostCount = 0;
            newHypoth.seenCount = 1;
            newHypoth.pos = obsRobotLocations[j];
            newHypoth.type = obsRobotTypes[j];
            robots.push_back(newHypoth);
         }
      }
   }
}

