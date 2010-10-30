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

#include <QPoint>
#include <cmath>
#include <cstring>
#include <boost/shared_ptr.hpp>
#include "readers/localisationReader.hpp"
#include "../../robot/blackboard/Blackboard.hpp"
#include "perception/localisation/LocalisationDefs.hpp"
#include "perception/localisation/PFilter.hpp"
#include "../inverseVision/inverseVision.hpp"
#include "utils/basic_maths.hpp"
#include "progopts.hpp"

using namespace boost;

LocalisationReader::LocalisationReader()  {
   isAlive = true;
}

LocalisationReader::~LocalisationReader() {
}

QPoint LocalisationReader::nextPosition(float theta) {
   return QPoint(1900*sin(2*theta), 1000 * cos(theta));
}

void LocalisationReader::run() {
   Frame frame;
   QPoint realPosition(1000, 1000);
   QPoint ballPosition(1000, 1000);
   float realHeading = 0;
   int currentFrame = 0;
   emit showMessage(tr("Started localisation debug mode."), 0);
   int posX,posY;
   posX = 0;
   posY = 0;
   float theta = 0;
   float ballTheta = 1;
   bool debug = false;
   int count = 0;
   AbsCoord *myRobotPos = NULL;
   RRCoord *myBallPos = NULL;
   int i = 0;
   while (isAlive && count < 3) {
      QPoint newPosition = nextPosition(theta);
      // QPoint newPosition = realPosition;
      // newPosition.rx() += 100;
      // newPosition.ry() -= 100;
      // newPosition.rx() = 1500;
      // newPosition.ry() = -500;
      QPoint difference = newPosition - realPosition;

      realHeading = atan2(difference.y(), difference.x());
      // realHeading = (theta);
      realPosition = newPosition;

      ballPosition = nextPosition(ballTheta);
      ballPosition.rx() = 0;
      ballPosition.ry() = 0;

      // realPosition.rx() = 2105;
      // realPosition.ry() = 1650;
      // realHeading = DEG2RAD(-45);
      //switch(i % 4) {
         //case 0:
            //realHeading = DEG2RAD(45);
            //i++;
            //break;
         //case 1:
            //realHeading = DEG2RAD(135);
            //i++;
            //break;
         //case 2:
            //realHeading = DEG2RAD(-135);
            //i++;
            //break;
         //case 3:
            //realHeading = DEG2RAD(-45);
            //break;
         //default:
            //realHeading = 0;
            //break;
      //}
      //realHeading -= M_PI/4;

      theta += 0.02;
      ballTheta += 0.2;
      //theta += .05;
      //step position
      posY+=0.5;
      // localization code goes here
      Blackboard *blackboard = new Blackboard(config);
      InverseVision::calculate(&localisation, realPosition, realHeading, &ballPosition, 1);

      localisation.localise();
      if (ABS(localisation.robotPos[0].theta -
               realHeading) >= M_PI) {
         debug = true;
      }

      writeTo(vision, posts, localisation.obsWhichPosts);
      writeArray(vision, post, localisation.obsPosts);

      writeTo(vision, numEdges, localisation.obsNumEdges);
      writeArray(vision, edges, localisation.obsEdges);

      writeTo(localisation, numRobotPos, localisation.numRobotPos);
      writeTo(localisation, robotPos, localisation.robotPos);

      frame.blackboard = blackboard;
      frame.isLocalizationDebugging = true;
      frame.actualHeading = realHeading;
      frame.actualPosition = realPosition;
      frame.actualBallPos = ballPosition;
      naoData.appendFrame(frame);

      if (!naoData.getIsPaused() && naoData.getCurrentFrameIndex() <
            naoData.getFramesTotal() - 1) {
         naoData.nextFrame();
         emit newNaoData(&naoData);
      } else if (naoData.getFramesTotal() != 0) {
         emit newNaoData(&naoData);
      }
      currentFrame = naoData.getCurrentFrameIndex();
      msleep(500);
      if (debug) {
         // count++; // uncomment this to stop off-nao updating the robot info
      }
      std::cerr << "localisationReader end this frame" << std::endl;
   }
}

void LocalisationReader::stopMediaTrigger() {
}
