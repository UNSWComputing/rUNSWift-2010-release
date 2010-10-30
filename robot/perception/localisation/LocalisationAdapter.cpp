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

#include <vector>
#include "perception/localisation/LocalisationAdapter.hpp"
#include "blackboard/Blackboard.hpp"
#include "utils/log.hpp"

using namespace std;

const std::string LocalisationAdapter::name("Localisation");

LocalisationAdapter::LocalisationAdapter() {
   string mapFileName = "/home/nao/data/map.data";
   fieldLineLocalisation.setupMap(mapFileName.c_str());
   kFilter = new KFilter();
   kFilter->fieldLineLocalisation = &fieldLineLocalisation;
   pFilter = new PFilter();
   pFilter->fieldLineLocalisation = &fieldLineLocalisation;
   usingPF = true;
}

LocalisationAdapter::~LocalisationAdapter() {
}

void LocalisationAdapter::tick() {
   llog(DEBUG1) << "Localisation.. ticking away" << endl;

   if (readFrom(behaviour, usePF)) {
      usingPF = true;
   }
   if (usingPF) {
      populateLocaliser(pFilter);
      pFilter->localise();

      if (pFilter->numRobotPos == 1) {
         kFilter->ranLastCycle = false;
         usingPF = false;
      }
      writeTo(localisation, numRobotPos, pFilter->numRobotPos);
      writeTo(localisation, robotPos, pFilter->robotPos);
   }

   populateLocaliser(kFilter);
   kFilter->localise();
   writeTo(localisation, numRobotPos, kFilter->numRobotPos);
   writeTo(localisation, robotPos, kFilter->robotPos);
   writeTo(localisation, kidnapFactor, kFilter->kidnapFactor);
   writeTo(localisation, usingPF, usingPF);

   /* run ball filter */
   B.playerNumber = readFrom(gameController, player_number);
   readArray(receiver, data, B.teamData);
   B.robotPos = kFilter->robotPos[0];
   B.obsNumBalls = readFrom(vision, numBalls);
   B.odometry = readFrom(motion, odometry);
   if (B.obsNumBalls > 0) {
      B.obsBall = readFrom(vision, ball[0]);
   }
   B.update();
   writeTo(localisation, egoBallPosAbs, B.egoBallPosAbs);
   writeTo(localisation, teamBallPosAbs, B.teamBallPosAbs);
   writeTo(localisation, ballPosRr, B.ballPosRr);
   writeTo(localisation, ballLostCount, B.lostCount);

   /* run robot filter */
   R.odometry = readFrom(motion, odometry);
   R.obsNumRobots = readFrom(vision, numRobots);
   readArray(vision, robotLocations, R.obsRobotLocations);
   readArray(vision, robotTypes, R.obsRobotTypes);
   readArray(vision, canSeeBottomRobot, R.obsCanSeeBottom);
   R.robotPos = kFilter->robotPos[0];
   R.update();
   writeTo(localisation, robotObstacles, R.robots);

   S.update(readFrom(motion, sensors));
   writeTo(localisation, sonarLostCount, S.lostCount);
}

void LocalisationAdapter::populateLocaliser(Localiser *L) {
   /** clear all data arrays */
   memset(L->obsPosts, 0, sizeof(RRCoord) * MAX_POSTS);
   memset(L->obsEdges, 0, sizeof(Line) * MAX_FIELD_EDGES);
   /** get data from blackboard */
   L->team_red = readFrom(gameController, team_red);
   const std::vector<AbsCoord> &oldPos = readFrom(localisation, robotPos);
   if (oldPos.size() > 0) {
      L->robotPos = oldPos;
   } else {
      AbsCoord startPos;
      startPos.x = 0;
      startPos.y = 0;
      startPos.theta = 0.0f;
      startPos.var[0] = SQUARE(300.0);
      startPos.var[1] = SQUARE(300.0);
      startPos.var[2] = SQUARE(DEG2RAD(10.0f));
      std::vector<AbsCoord> startVec;
      startVec.push_back(startPos);
      L->robotPos = startVec;
   }
   L->acActive = readFrom(motion, active);
   L->odometry = readFrom(motion, odometry);
   L->obsNumEdges = readFrom(vision, numEdges);
   readArray(vision, edges, L->obsEdges);
   L->fieldLineLocalisation->numFieldLinePoints =
      readFrom(vision, numFieldLinePoints);
   readArray(vision, fieldLinePoints,
         L->fieldLineLocalisation->fieldLinePoints);
   L->obsWhichPosts = readFrom(vision, posts);
   readArray(vision, post, L->obsPosts);
}
