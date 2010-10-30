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

#include "fieldView.hpp"
#include <QColor>
#include <QBrush>
#include <QImage>
#include <QRectF>
#include <QPainter>
#include "perception/vision/yuv.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/Vision.hpp"
#include "perception/vision/RobotRegion.hpp"
#include "perception/localisation/LocalisationDefs.hpp"
#include "utils/log.hpp"
#include "utils/incapacitated.hpp"
#include "blackboard/Blackboard.hpp"
#include "progopts.hpp"

using namespace std;

FieldView::FieldView() : image(":/images/spl_field.svg") {
   renderPixmap = new QPixmap(640, 480);

   imagePixmap = QPixmap(640, 480);
   QRectF target(0.0, 0.0, 640.0, 480.0);
   QRectF source(0.0, 0.0, 740, 540.0);
   QPainter painter(&imagePixmap);
   painter.drawImage(target, image, source);

   renderPixmap->fill(Qt::darkGray);
   setPixmap(*renderPixmap);
   setMinimumSize(640, 480);
   setMaximumSize(640, 480);
   setAlignment(Qt::AlignTop);
}

FieldView::~FieldView() {
}

void FieldView::drawLocalizationDebugOverlay(Frame frame, QPainter *painter) {
   int ourTeam = OFFNAO_TEAM;
   if (frame.isLocalizationDebugging) {
      QPoint posImage = this->fieldToImageCoords(frame.actualPosition);

      double heading = RAD2DEG(frame.actualHeading);
      /* take into account rotate() goes clockwise */
      teamConversion(posImage, heading, ourTeam);
      // if (ourTeam == TEAM_BLUE) {
      //     heading = -heading;
      // } else {
      //     heading = 180 - heading;
      // }
      painter->translate(posImage.x(), posImage.y());
      painter->rotate(heading);
      painter->setBrush(QBrush(QColor(255, 255, 255)));
      painter->drawEllipse(QPoint(0, 0), 15, 15);
      QPen pen = QPen(QColor(0, 0, 0));
      pen.setWidth(3);
      painter->setPen(pen); /* reset the pen */
      painter->drawLine(QPoint(0, 0), QPoint(20, 0));
      pen.setWidth(1);
      painter->setPen(pen); /* reset the pen */
      painter->rotate(-heading);
      painter->translate(-posImage.x(), -posImage.y());

      QPoint b(frame.actualBallPos);
      double h;
      teamConversion(b, h, ourTeam);
      QPoint posBall = this->fieldToImageCoords(b);


      /* draw ball */
      painter->setBrush(QBrush(QColor(255, 255, 255)));
      painter->drawEllipse(posBall.x()-5, posBall.y()-5, 10, 10);
   }
}

void FieldView::redraw(NaoData *naoData) {
   QRectF target(0.0, 0.0, 640.0, 480.0);
   QRectF source(0.0, 0.0, 740, 540.0);

   *renderPixmap = imagePixmap;
   QPainter painter(renderPixmap);
   // painter.drawImage(target, image, source);
   if (!naoData) {
      setPixmap(*renderPixmap);
      return;
   }

   Blackboard *blackboard = naoData->getCurrentFrame().blackboard;
   ::blackboard = blackboard;
   if (blackboard) {
      /* get the team info */
      int ourTeam = readFrom(gameController, team_red);
      drawLocalizationDebugOverlay(naoData->getCurrentFrame(), &painter);
      // for now assume robot is at (0,0) with heading of 0
      /* Get a blackboard */
      int numRobotPos;

      if (naoData != NULL && naoData->getFramesTotal() > 0) {
         int numBalls = readFrom(vision, numBalls);
         RRCoord ball[MAX_BALLS];

         readArray(vision, ball, ball);
         for (int i = 0; i < numBalls; i++) {
            /* find ball pos */
            QPoint absBall = robotRelativeToImage(ball[i].distance,
                                                  ball[i].heading);
            painter.setPen(QColor(1, 0, 0));
            /* draw ball */
            painter.setBrush(QBrush(QColor(255, 165, 0)));
            painter.drawEllipse(absBall.x()-5, absBall.y()-5, 10, 10);
            /* draws the standard deviation */
            int rx = sqrt(ball[i].var[0])/FIELD_TO_IMAGE_SCALE;
            int ry = sqrt(ball[i].var[1])/FIELD_TO_IMAGE_SCALE;
            painter.setPen(QPen(QColor(255, 255, 0)));
            painter.setBrush(QBrush(Qt::NoBrush));
            painter.drawEllipse(absBall, rx, ry);
         }

         AbsCoord ego = readFrom(localisation, egoBallPosAbs);
         AbsCoord team = readFrom(localisation, teamBallPosAbs);
         uint32_t lostCount = readFrom(localisation, ballLostCount);
         if (lostCount < 300) {
            // draw ego ball
            painter.setPen(QColor(1, 0, 0));
            /* draw ball */
            painter.setBrush(QBrush(QColor(255, 127, 0)));
            QPoint p = QPoint(ego.x, ego.y);
            double uselessVariable = 0;
            teamConversion(p, uselessVariable, ourTeam);
            QPoint e = fieldToImageCoords(p);
            painter.drawEllipse(e.x()-5, e.y()-5, 10, 10);
            /* draws the standard deviation */
            int rx = sqrt(ego.var[0])/FIELD_TO_IMAGE_SCALE;
            int ry = sqrt(ego.var[1])/FIELD_TO_IMAGE_SCALE;
            painter.setPen(QPen(QColor(255, 255, 0)));
            painter.setBrush(QBrush(Qt::NoBrush));
            painter.drawEllipse(e, rx, ry);
         }
         // draw team ball
         painter.setPen(QColor(1, 0, 0));
         /* draw ball */
         painter.setBrush(QBrush(QColor(205, 133, 0)));
         QPoint p = QPoint(team.x, team.y);
         double uselessVariable = 0;
         teamConversion(p, uselessVariable, ourTeam);
         QPoint t = fieldToImageCoords(p);
         painter.drawEllipse(t.x()-5, t.y()-5, 10, 10);
         /* draws the standard deviation */
         int rx = sqrt(team.var[0])/FIELD_TO_IMAGE_SCALE;
         int ry = sqrt(team.var[1])/FIELD_TO_IMAGE_SCALE;
         painter.setPen(QPen(QColor(255, 255, 0)));
         painter.setBrush(QBrush(Qt::NoBrush));
         painter.drawEllipse(t, rx, ry);

         numRobotPos = readFrom(localisation, numRobotPos);
         const vector<AbsCoord> &absCoord = readFrom(localisation, robotPos);
         bool usingPF = readFrom(localisation, usingPF);


         // when using KF, want numRobotPos
         // when not streaming particles, want absCoord.size()
         for (int i = 0; i < MIN(numRobotPos, absCoord.size()); i++) {
            if(naoData->getCurrentFrame().isLocalizationDebugging) {
               this->robotPos = naoData->getCurrentFrame().actualPosition;
               this->robotHeading = RAD2DEG(naoData->getCurrentFrame().actualHeading);
               this->ourTeam = OFFNAO_TEAM;
               // absCord.x = robotPos.x();
               // absCord.y = robotPos.y();
               // absCord.theta = robotHeading;
            } else {
               this->robotPos = QPoint(absCoord[i].x, absCoord[i].y);
               this->robotHeading = RAD2DEG(absCoord[i].theta);
               this->ourTeam = ourTeam;
            }
            teamConversion(robotPos, robotHeading, ourTeam);

            // find transformation
            QPoint t = fieldToImageCoords(robotPos);

            /* draw me */
            QPoint absPoint(absCoord[i].x, absCoord[i].y);
            double heading = RAD2DEG((absCoord[i].theta));
            teamConversion(absPoint, heading, ourTeam);
            QPoint posImage = this->fieldToImageCoords(absPoint);


            if(!isnan(heading)) {
               /* take into account rotate() goes clockwise */

               // std::cerr << "drawing [" << i << "] robot" << std::endl;

               painter.setBrush(QBrush(QColor(0, 0, 255)));
               painter.translate(posImage.x(), posImage.y());
               painter.rotate(heading);
               /* draws the standard deviation */
               int rx = sqrt(absCoord[i].var[0])/FIELD_TO_IMAGE_SCALE;
               int ry = sqrt(absCoord[i].var[1])/FIELD_TO_IMAGE_SCALE;
               painter.setPen(QPen(QColor(255, 255, 0)));
               painter.setBrush(QBrush(Qt::NoBrush));
               painter.rotate(-heading);
               painter.drawEllipse(QPoint(0, 0), rx, ry);
               painter.rotate(heading);
               // eventually display variance of heading here

               /* draw the actual robot pos */
               if (!usingPF) {
                  painter.setBrush(QBrush(QColor(0, 0, 0)));
               } else {
                  if (i == 0) {
                     painter.setBrush(QBrush(QColor(255, 0, 0)));
                  } else {
                     painter.setBrush(QBrush(QColor(255, 128, 255)));
                  }
               }
               painter.drawEllipse(QPoint(0, 0), 10, 10);
               QPen pen = QPen(QColor(255, 255, 0));
               pen.setWidth(3);
               painter.setPen(pen); /* reset the pen */
               painter.drawLine(QPoint(0, 0), QPoint(18, 0));
               painter.setPen(QPen(QColor(0, 0, 0))); /* reset the pen */
               painter.rotate(-heading);
               painter.translate(-posImage.x(), -posImage.y());

               if (i == 0) {
                  drawPosts(naoData, &painter);
                  drawEdges(naoData, &painter);
                  drawFieldLines(naoData, &painter);
                  drawOtherRobots(naoData, &painter);
               }
            }
         }
      }

      // Draw shared data (from inter-robot comms
      for (int player=1; player<=3; ++player) {
         // cerr << "got a blackwith with recv_mask: " << (int)recv_mask << endl;
         if (!isIncapacitated(player)) {
            drawBroadcastData(readFrom(receiver, data[player-1]), &painter, ourTeam, player);
         }
      }
   }
   setPixmap(*renderPixmap);
   return;
}

QPoint FieldView::robotRelativeToImage(int distance, float heading) {
   QPoint absBall = this->robotPos;
   // if (ourTeam == TEAM_RED) robotHeading += 180;

   absBall.setX(absBall.x() + distance * cos(DEG2RAD(-robotHeading) + heading));
   absBall.setY(absBall.y() + distance * sin(DEG2RAD(-robotHeading) + heading));
   // std::cerr << "HEADING: " << RAD2DEG(heading) << std::endl;
   return this->fieldToImageCoords(absBall);
}

void FieldView::drawOtherRobots(NaoData *naoData, QPainter *p) {
   Blackboard *blackboard = naoData->getCurrentFrame().blackboard;
   const list<RobotObstacle> &ros = readFrom(localisation, robotObstacles);

   for (list<RobotObstacle>::const_iterator roi = ros.begin(); roi != ros.end();
        ++roi) {
      if (roi->type == RED_ROBOT) {
         p->setBrush(QBrush(QColor(255, 128, 128)));
      } else if (roi->type == BLUE_ROBOT) {
         p->setBrush(QBrush(QColor(128, 128, 255)));
      } else {
         p->setBrush(QBrush(QColor(128, 128, 128)));
      }
      QPoint loc = robotRelativeToImage(roi->pos.distance, roi->pos.heading);
      p->drawEllipse(loc.x() - 8, loc.y() - 8, 16, 16);
   }
}

void FieldView::drawFieldLines(NaoData *naoData, QPainter *p) {
   Blackboard *blackboard = naoData->getCurrentFrame().blackboard;
   int numFieldLinePoints = readFrom(vision, numFieldLinePoints);
   RRCoord fieldLinePoints[MAX_FIELD_LINE_POINTS];
   readArray(vision, fieldLinePoints, fieldLinePoints);

   p->setBrush(QBrush(QColor(10, 10, 150)));
   for (int k = 0; k < numFieldLinePoints; k++) {
      QPoint loc = robotRelativeToImage(fieldLinePoints[k].distance,
            fieldLinePoints[k].heading);
      p->drawEllipse(loc.x() - 1, loc.y() - 1, 2, 2);
   }
}

void FieldView::drawEdges(NaoData *naoData, QPainter *p) {
   Blackboard *blackboard = naoData->getCurrentFrame().blackboard;
   int numEdges;
   /** Lines representing the field edges */
   Line edges[MAX_FIELD_EDGES];
   numEdges = readFrom(vision, numEdges);
   readArray(vision, edges, edges);
   // std::cerr << "numEdges: " << numEdges << std::endl;
   p->setPen(QPen(QColor(255, 255, 0)));
   for(int i = 0; i < numEdges; i++) {
      drawEdge(naoData, p, edges[i]);
      p->setPen(QPen(QColor(255, 0, 0)));
   }
   p->setPen(QPen(QColor(0, 0, 0)));
}

void FieldView::drawEdge(NaoData *naoData, QPainter *p, Line line) {

   // Find y value of RR line at x = 0 and x = 639
   // QPoint p1(0, ((float)-line.t3)/line.t2);
   // QPoint p2(639, (float)(639*line.t1 + line.t3)/-(line.t2));

   QPoint p1(line.x1_, line.y1_);
   QPoint p2(line.x2_, line.y2_);

   // cerr << "line betweeen RR (" << line.x1_ << "," << line.y1_ << ") and (" << line.x2_ << "," << line.y2_ << ")" << endl;

   int dist1 = sqrt(SQUARE(p1.x()) + SQUARE(p1.y()));
   int dist2 = sqrt(SQUARE(p2.x()) + SQUARE(p2.y()));
   float heading1 = atan2(p1.y(), p1.x());
   float heading2 = atan2(p2.y(), p2.x());

   /*
   QPoint p1abs = robotRelativeToImage(dist1, -heading1);
   QPoint p2abs = robotRelativeToImage(dist2, -heading2);
   */
   QPoint p1abs (p1.x()*cos(DEG2RAD(-robotHeading)) - p1.y()*sin(DEG2RAD(-robotHeading)) + robotPos.x(),
                 p1.x()*sin(DEG2RAD(-robotHeading)) + p1.y()*cos(DEG2RAD(-robotHeading)) + robotPos.y());

   QPoint p2abs (p2.x()*cos(DEG2RAD(-robotHeading)) - p2.y()*sin(DEG2RAD(-robotHeading)) + robotPos.x(),
                 p2.x()*sin(DEG2RAD(-robotHeading)) + p2.y()*cos(DEG2RAD(-robotHeading)) + robotPos.y());


   QPoint p1img = fieldToImageCoords(p1abs);
   QPoint p2img = fieldToImageCoords(p2abs);

   p->drawLine(p1img, p2img);
   // QPoint FieldView::robotRelativeToImage(int distance, float heading)

   /*
   // find 2 points on the line
   // Y-intercept
   QPoint p1(0, (-line.t3 * 1.0)/line.t2);
   // std::cerr << "RR Point is: " << p1.x() << " " << p1.y() << std::endl;

   // X-intercept
   QPoint p2((-line.t3 * 1.0)/line.t1, 0);
   // std::cerr << "RR Point is: " << p2.x() << " " << p2.y() << std::endl;

   // Convert back to world coordinates.
   p1 = robotRelativeToAbsolute(p1, robotPos, robotHeading);
   p2 = robotRelativeToAbsolute(p2, robotPos, robotHeading);
   // std::cerr << "ABS Point is: " << p1.x() << " " << p1.y() << std::endl;
   // std::cerr << "ABS Point is: " << p2.x() << " " << p2.y() << std::endl;

   p1 = fieldToImageCoords(p1);
   p2 = fieldToImageCoords(p2);

   QPoint d = p1 - p2;
   p1 += 10 * d;
   p2 -= 10 * d;
   // std::cerr << "Point is: " << p1.x() << " " << p1.y() << std::endl;
   // std::cerr << "Point is: " << p2.x() << " " << p2.y() << std::endl;

   p->drawLine(p1, p2);
   */
}

void FieldView::drawPosts(NaoData *naoData, QPainter *p) {
   /* Currently is reporting blue and yellow around the wrong way so
    * I have written my code to account for this
    */
   Blackboard *blackboard = naoData->getCurrentFrame().blackboard;
   if (blackboard) {
      WhichPosts posts = readFrom(vision, posts);
      RRCoord post[MAX_POSTS];
      readArray(vision, post, post);
      int i  = 0;
      switch (posts) {
         case pBLUE_BOTH:
            p->setBrush(QBrush(QColor(0, 0, 255)));
            drawPost(robotRelativeToImage(post[i].distance,
                     post[i].heading), p);
            i++;
         case pBLUE_EITHER:
         case pBLUE_RIGHT:
         case pBLUE_LEFT:
            p->setBrush(QBrush(QColor(0,0,255)));
            drawPost(robotRelativeToImage(post[i].distance, post[i].heading),p);
            break;
         case pYELLOW_BOTH:
            p->setBrush(QBrush(QColor(255, 255, 0)));
            drawPost(robotRelativeToImage(post[i].distance,
                     post[i].heading), p);
            i++;
         case pYELLOW_EITHER:
         case pYELLOW_RIGHT:
         case pYELLOW_LEFT:
            p->setBrush(QBrush(QColor(255, 255, 0)));
            drawPost(robotRelativeToImage(
                     post[i].distance, post[i].heading), p);
      }
   }
}

void FieldView::drawPost(QPoint pos, QPainter *p) {
   p->drawEllipse(QPoint(pos.x(), pos.y()), 10, 10);
}

void FieldView::drawBroadcastData(const BroadcastData &bcData, QPainter *p, int ourTeam, int n) {
   QPoint absPoint(bcData.robotPos.x, bcData.robotPos.y);
   double heading = RAD2DEG(bcData.robotPos.theta);
   teamConversion(absPoint, heading, ourTeam);
   QPoint t = fieldToImageCoords(absPoint);
   p->translate(t.x(), t.y());
   /* draws the standard deviation */
   int rx = sqrt(bcData.robotPos.var[0])/FIELD_TO_IMAGE_SCALE;
   int ry = sqrt(bcData.robotPos.var[1])/FIELD_TO_IMAGE_SCALE;
   p->setBrush(QBrush(Qt::NoBrush));
   p->drawEllipse(QPoint(0, 0), rx, ry);
   /* draw the robot pos */
   if (n == 1)
      p->setBrush(QBrush(QColor(255, 0, 0)));
   else if (n == 2)
      p->setBrush(QBrush(QColor(0, 255, 0)));
   else if (n == 3)
      p->setBrush(QBrush(QColor(0, 0, 255)));
   else
      p->setBrush(QBrush(QColor(255, 255, 255)));
   p->drawEllipse(QPoint(0, 0), 6, 6);
   /* draw the heading */
   p->rotate(heading);
   QPen pen = QPen(QColor(0, 0, 0));
   pen.setWidth(1);
   p->setPen(pen);
   p->drawLine(QPoint(0, 0), QPoint(15, 0));
   p->rotate(-heading);
   /* reset the pen */
   p->setPen(QPen(QColor(0, 0, 0)));
   p->translate(-t.x(), -t.y());
}

void FieldView::teamConversion(QPoint &p, double &heading, int ourTeam) {
   if (ourTeam == TEAM_BLUE) {
      heading = -heading;
   } else {
      heading = 180 - heading;
      p.rx() *= -1;
      p.ry() *= -1;
   }
}

QPoint FieldView::fieldToImageCoords(QPoint p) {
   // p is in mm and the centre is (0,0)
   p = QPoint(p.x()/FIELD_TO_IMAGE_SCALE * 640/740.0, -p.y()/FIELD_TO_IMAGE_SCALE * 640/740.0);
   p += QPoint(this->width()/2, this->height()/2);
   return p;
}

/* p is rr, position believe to be abosolute coordinate. */
QPoint FieldView::robotRelativeToAbsolute(QPoint p, QPoint position, float heading) {
   QPoint abs;
   abs.rx() = p.x()*cos(heading) - p.y()*sin(heading) + position.x();
   abs.ry() = p.x()*sin(heading) + p.y()*cos(heading) + position.y();

   /*
   QPoint posDiff = p;
   float pRRtheta;
   if (posDiff.y() == 0) {
      pRRtheta = 0;
   } else {
      pRRtheta = atan2(posDiff.y(), posDiff.x());
   }
   pRRtheta += heading;
   float pRRdist = sqrt(pow(posDiff.x(), 2) + pow(posDiff.y(), 2));
   abs.rx() = pRRdist * cos(pRRtheta) + position.x();
   abs.ry() = pRRdist * sin(pRRtheta) + position.y();
   */
   return abs;
}
