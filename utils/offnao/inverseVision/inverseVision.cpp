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

#include <cmath>
#include <vector>
#include <QPoint>
#include "inverseVision.hpp"
#include "utils/basic_maths.hpp"
#include <stdlib.h>

int InverseVision::yellowPostsSeen;
int InverseVision::bluePostsSeen;

using namespace std;

void InverseVision::calculate(PFilter *localisation, QPoint position, float heading, QPoint *ball, int numBalls) {
   yellowPostsSeen = 0;
   bluePostsSeen = 0;

   localisation->obsWhichPosts = pNONE;
   /* The order of these posts are important! As left has to be stored first for
    * seeing boths posts! (protocol between vision and localisation emulated)*/
   viewPost(localisation, QPoint(GOAL_POST_ABS_X_DIST, GOAL_POST_ABS_Y_DIST), position, heading);
   viewPost(localisation, QPoint(GOAL_POST_ABS_X_DIST, -GOAL_POST_ABS_Y_DIST), position, heading);
   viewPost(localisation, QPoint(-GOAL_POST_ABS_X_DIST, -GOAL_POST_ABS_Y_DIST), position, heading);
   viewPost(localisation, QPoint(-GOAL_POST_ABS_X_DIST, GOAL_POST_ABS_Y_DIST), position, heading);

   localisation->obsNumEdges = 0;
   float O_FIELD_WIDTH = FULL_FIELD_WIDTH;  // - 500;
   float O_FIELD_LENGTH = FULL_FIELD_LENGTH;  // - 500;
   viewEdge(localisation, QPoint(O_FIELD_LENGTH/2, O_FIELD_WIDTH/2),
         QPoint(O_FIELD_LENGTH/2, -O_FIELD_WIDTH/2), position, heading);
   viewEdge(localisation, QPoint(O_FIELD_LENGTH/2, O_FIELD_WIDTH/2),
         QPoint(-O_FIELD_LENGTH/2, O_FIELD_WIDTH/2), position, heading);
   viewEdge(localisation, QPoint(-O_FIELD_LENGTH/2, -O_FIELD_WIDTH/2),
         QPoint(-O_FIELD_LENGTH/2, O_FIELD_WIDTH/2), position, heading);
   viewEdge(localisation, QPoint(-O_FIELD_LENGTH/2, -O_FIELD_WIDTH/2),
         QPoint(O_FIELD_LENGTH/2, -O_FIELD_WIDTH/2), position, heading);

   int i;
      localisation->fieldLineLocalisation.numFieldLinePoints = 0;   
}

bool InverseVision::viewEdge(PFilter *localisation, QPoint startPos,
                     QPoint endPos, QPoint position, float heading) {
   // I need to find 2 points on the line that are within the field of view
   // Turns out its just easiest to check at small incrememnts along the line
   // till I find 2 such points and then use these for the lines
   QPoint lineDir = endPos - startPos;
   lineDir /= 50;  // check 50 points along the line
   QPoint point = startPos;
   std::vector<QPoint> points;
   for(int i = 0; i < 50;i++) {
      // std::cout << "point: " << point.x() << ", " << point.y() << std::endl;
      if(isPointInFOV(point, position, heading)) {
         // transform point to be in robot relative coordinate system
         // this could be wrong....
         QPoint p;
         QPoint posDiff = point - position;
         float pRRtheta;
         if (posDiff.y() == 0) {
            pRRtheta = 0;
         } else {
            pRRtheta = atan2(posDiff.y(), posDiff.x()) - heading;
         }
         float pRRdist = sqrt(SQUARE(posDiff.x()) + SQUARE(posDiff.y()));
         p.rx() = pRRdist * cos(pRRtheta);
         p.ry() = pRRdist * sin(pRRtheta);
         // p.rx() = (point.x() - position.x()) * cos(-heading) - (point.y() - position.y()) * sin(-heading);
         // p.ry() = (point.x() - position.x()) * sin(-heading) + (point.y() - position.y()) * cos(-heading);
         // p.rx() = point.x() * cos(-heading) - point.y() * sin(-heading) - position.x();
         // p.ry() = point.x() * sin(-heading) + point.y() * cos(-heading) - position.y();
         points.push_back(p);
         // std::cerr << "Real ABS Point: " << point.x() << " " << point.y() << std::endl;
         if(points.size() >= 2) break;
      }
      point += lineDir;
   }
   if(points.size() >= 2) {
      // create the line!
      // first find Ax + by + C = 0 form
      QPoint dir = points[1] - points[0];
      // std::cerr << "Real Point 1: " << points[0].x() << " " << points[0].y() << std::endl;
      // std::cerr << "Real Point 2: " << points[1].x() << " " << points[1].y() << std::endl;
      Line l(points[0].x(), points[0].y(), points[1].x(), points[1].y());
      l.var = 0;
      localisation->obsEdges[localisation->obsNumEdges] = l;
      localisation->obsNumEdges++;
      // std::cout << "Line: " << l.t1 << "x " << l.t2 << "y " << l.t3 << std::endl;
      return true;
   }
   return false;
}

bool InverseVision::isPointInFOV(QPoint point, QPoint position, float heading) {
   QPoint posDif = point - position;
   float angle = std::atan2(posDif.y(), posDif.x());
   // std::cout << "posDif: " << posDif.x() << ", " << posDif.y() << std::endl;
   // std::cout << "angle (1): " << angle << std::endl;
   angle -= heading;
   if (angle < -M_PI) angle += M_PI*2;
   if (angle >= M_PI) angle -= M_PI*2;
   // std::cout << "angle (2): " << angle << " " << RAD2DEG(angle) << std::endl;
   return abs(RAD2DEG(angle)) < 25;
}

bool InverseVision::viewPost(PFilter *localisation, QPoint postPos,
                             QPoint position, float heading) {
   // first get angle from where I am to the post
   QPoint posDif = postPos - position;
   float angle = std::atan2(posDif.y(), posDif.x());
   angle -= heading;
   if (angle < -M_PI) angle += M_PI*2;
   if (angle >= M_PI) angle -= M_PI*2;
   // check if in field of view
   if (abs(RAD2DEG(angle)) < 25) {

      // if so add the relative coords to the blackboard
      float length = sqrt(pow(posDif.x(),2) + pow(posDif.y(),2));
      int index = yellowPostsSeen + bluePostsSeen;
      localisation->obsPosts[index].distance = length;
      localisation->obsPosts[index].heading = angle;
      /* perfect observation set everything to 0 */
      localisation->obsPosts[index].var[0] = 0;
      localisation->obsPosts[index].var[1] = 0;
      // std::cerr << "Distance: " << length << ", Heading: " << angle << ", Post: " <<
      //          yellowPostsSeen + bluePostsSeen << std::endl;
      if (postPos.x() <= 0) {
         if(bluePostsSeen) {
            localisation->obsWhichPosts = pBLUE_BOTH;
         } else if (postPos.y() < 0) {
            localisation->obsWhichPosts = pBLUE_LEFT;
         } else {
            localisation->obsWhichPosts = pBLUE_RIGHT;
         }
         bluePostsSeen++;
      } else {
         if (yellowPostsSeen) {
            localisation->obsWhichPosts = pYELLOW_BOTH;
         } else if (postPos.y() < 0) {
            localisation->obsWhichPosts = pYELLOW_RIGHT;
         } else {
            localisation->obsWhichPosts = pYELLOW_LEFT;
         }
         yellowPostsSeen++;
      }
   }

   return false;
}


void InverseVision::addNoise(PFilter *localisation) {
    for (int i = 0; i < 2; i++) {
        localisation->obsPosts[i].distance += 
           randRange(-localisation->obsPosts[i].distance/10.0,
              localisation->obsPosts[i].distance/10.0);
        localisation->obsPosts[i].heading += randRange(-1000, 1000)/10000.0;

        /* use actual covariances in vision atm if there exits noise */
        localisation->obsPosts[i].var[0] = SQUARE(800*localisation->obsPosts[i].distance/2000.0);  // SQUARE(750);
        localisation->obsPosts[i].var[1] = SQUARE(DEG2RAD(5));
    }
    /* randomly set seeing a single post to be either */
    if (randRange(0, 99) < 50) {
       if (localisation->obsWhichPosts == pBLUE_LEFT || localisation->obsWhichPosts == pBLUE_RIGHT) {
          localisation->obsWhichPosts = pBLUE_EITHER;
          localisation->obsPosts[0].var[1] = SQUARE(200*DEG2RAD(40)/MAX(100, localisation->obsPosts[0].distance));

       } else if (localisation->obsWhichPosts == pYELLOW_LEFT || localisation->obsWhichPosts == pYELLOW_RIGHT) {
          localisation->obsWhichPosts = pYELLOW_EITHER;
          localisation->obsPosts[0].var[1] = SQUARE(200*DEG2RAD(40)/MAX(100, localisation->obsPosts[0].distance));

       }

    }

    // for (int i = 0; i < localisation.obsNumEdges; i++) {
    // float dist = abs((x2-x1)*y1 - (y2-y1)*x1)/
    // sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    // localisation->obsEdges[i].var = 0.3 * randRange(0, 1) + 0.7 * dist;
    // }
}

int InverseVision::randRange(int min, int max) {
    if(min-max == 0) return 0;
    return min + rand() % (max-min);
}
