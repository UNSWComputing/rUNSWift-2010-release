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

#include <limits>
#include "LocalisationDefs.hpp"
#include "BallFilter.hpp"
#include "utils/log.hpp"
#include "utils/incapacitated.hpp"

using namespace std;

BallFilter::BallFilter() {
   lostCount = numeric_limits<uint16_t>::max();
   egoBallPosAbs.x = 0;
   egoBallPosAbs.y = 0;
   egoBallPosAbs.var[X] = SQUARE(FULL_FIELD_LENGTH);
   egoBallPosAbs.var[Y] = SQUARE(FULL_FIELD_WIDTH);
   ballPosRr.heading = 0.0;
   ballPosRr.distance = 0.0;
}

void BallFilter::update() {
   rrPosUpdate();
   egoAbsPosUpdate();
   teamAbsPosUpdate();
}

void BallFilter::rrPosUpdate() {
   Odometry deltaOdometry = odometry - prev_odometry;
   float deltaF = deltaOdometry.forward;
   float deltaS = deltaOdometry.left;
   float deltaT = deltaOdometry.turn;
   float deltaX = deltaF*cos(robotPos.theta) - deltaS*sin(robotPos.theta);
   float deltaY = deltaF*sin(robotPos.theta) + deltaS*cos(robotPos.theta);
   prev_odometry = odometry;

   float alpha = NORMALISE(robotPos.theta + ballPosRr.heading);
   float ballx = ballPosRr.distance*cos(alpha) + robotPos.x - deltaX;
   float bally = ballPosRr.distance*sin(alpha) + robotPos.y - deltaY;
   ballPosRr.heading = NORMALISE(ballPosRr.heading - deltaT);
   ballPosRr.distance = sqrt(SQUARE(ballx - robotPos.x)
                           + SQUARE(bally - robotPos.y));

   float k = 0.6;  //  increased gain!!
   if (obsNumBalls > 0) {
      lostCount = 0;
      /* update RR ball pos */
      ballPosRr.distance = ballPosRr.distance +
         k * (obsBall.distance - ballPosRr.distance);
      ballPosRr.heading = NORMALISE(ballPosRr.heading +
         k * (obsBall.heading - ballPosRr.heading));
   } else {
      lostCount++;
   }
}

void BallFilter::egoAbsPosUpdate() {
   /* update self filtered ABS ball pos */

   // TODO(davidc) remove evil constants

   /* process update */
   egoBallPosAbs.var[X] = SQUARE(sqrt(egoBallPosAbs.var[X]) + 25);
   egoBallPosAbs.var[Y] = SQUARE(sqrt(egoBallPosAbs.var[Y]) + 25);
   if (obsNumBalls > 0) {
      float alpha = NORMALISE(robotPos.theta + obsBall.heading);
      AbsCoord obsAbsBall;
      obsAbsBall.x = obsBall.distance*cos(alpha) + robotPos.x;
      obsAbsBall.y = obsBall.distance*sin(alpha) + robotPos.y;
      float k = egoBallPosAbs.var[X] /
         (egoBallPosAbs.var[X] + obsBall.var[DIST]);
      egoBallPosAbs.x = egoBallPosAbs.x + k * (obsAbsBall.x - egoBallPosAbs.x);
      egoBallPosAbs.y = egoBallPosAbs.y + k * (obsAbsBall.y - egoBallPosAbs.y);
      egoBallPosAbs.var[X] = (1 - k) * egoBallPosAbs.var[X];
      egoBallPosAbs.var[Y] = egoBallPosAbs.var[X];
   }
}

void BallFilter::teamAbsPosUpdate() {
   /* update team ABS ball pos */

   /* update from own egoBallPos */
   teamBallPosAbs.x = egoBallPosAbs.x;
   teamBallPosAbs.y = egoBallPosAbs.y;
   teamBallPosAbs.var[X] = egoBallPosAbs.var[X] + robotPos.var[X];
   teamBallPosAbs.var[Y] = egoBallPosAbs.var[Y] + robotPos.var[Y];

   /* update from team-mates' egoBallPos */
   for (int player = 1; player <= 3; player++) {
      /* check player num is valid */
      if (player != playerNumber &&
            !isIncapacitated(player)) {
         const AbsCoord &friendRobotPos = teamData[player-1].robotPos;
         const AbsCoord &friendAbsBall = teamData[player-1].egoBallPosAbs;
         float kx = teamBallPosAbs.var[X] /
            (teamBallPosAbs.var[X] +
             friendAbsBall.var[X] +
             friendRobotPos.var[X]);
         float ky = teamBallPosAbs.var[Y] /
            (teamBallPosAbs.var[Y] +
             friendAbsBall.var[Y] +
             friendRobotPos.var[Y]);
         teamBallPosAbs.x = teamBallPosAbs.x +
            kx * (friendAbsBall.x - teamBallPosAbs.x);
         teamBallPosAbs.y = teamBallPosAbs.y +
            ky * (friendAbsBall.y - teamBallPosAbs.y);
         teamBallPosAbs.var[X] = (1 - kx) * teamBallPosAbs.var[X];
         teamBallPosAbs.var[Y] = (1 - ky) * teamBallPosAbs.var[Y];
      }
   }
}

