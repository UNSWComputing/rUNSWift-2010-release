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

#include "tab.hpp"

#include <QPainter>
using namespace std;

QRgb Tab::getRGB(unsigned int col, unsigned int row, const uint8_t *yuv) {
   return Classifier::yuv2rgb(gety(yuv, row, col),
                      getu(yuv, row, col), getv(yuv, row, col));
}

void Tab::drawAllOverlaysGeneric(QPaintDevice *image, Vision *vision,
                                 float scale) {
   drawOverlaysGeneric(image, &(vision->fieldEdgeDetection.edgePoints),
         vision->fieldEdgeDetection.edgeLines,
         vision->fieldEdgeDetection.numEdgeLines,
         vision->goalDetection.postCoords,
         vision->goalDetection.numPosts,
         vision->goalDetection.typeOfPost, 
         vision->regionBuilder.regions,
         vision->regionBuilder.numRegions,
         vision->ballDetection.radius,
         vision->ballDetection.ballCentre,
         vision->ballDetection.ballEdgePoints,
         vision->ballDetection.numBallEdgePoints,
         vision->robotDetection.robotRegions,
         vision->robotDetection.numRobotRegions,
         vision->regionBuilder.lineRegions,
         vision->regionBuilder.numLineRegions,
         scale);
}

void Tab::drawOverlaysGeneric(QPaintDevice *image,
      vector<pair<uint16_t, uint16_t> > *edgePoints,
      Line edgeLines[2],
      uint8_t numEdgeLines,
      uint16_t *goals,
      uint16_t numPosts,
      WhichPosts posts,
      ImageRegion *regions,
      uint16_t numRegions,
      uint16_t radius,
      pair<uint16_t, uint16_t> ballCentre,
      pair<uint16_t, uint16_t> *ballEdgePoints,
      uint16_t numBallEdgePoints,
      RobotRegion **robotRegions,
      uint16_t numRobotRegions,
      ImageRegion **lineRegions,
      uint16_t numLineRegions,
      float scale) {
   QPainter painter(image);
   if (edgePoints != 0) {
      painter.setPen(QColor(1, 0, 0));
      painter.setBrush(QBrush(QColor(255, 255, 0)));
      vector<pair<uint16_t, uint16_t> >::const_iterator p;
      for (p = edgePoints->begin(); p != edgePoints->end(); ++p) {
         painter.drawEllipse(QPoint((*p).first*scale, (*p).second)*scale, 2, 2);
      }
   }

   if (edgeLines != 0) {
      painter.setPen(QColor(255, 255, 0));
      for (int i = 0; i < numEdgeLines; ++i) {
         painter.drawLine(
               0, -1*(float)edgeLines[i].t3/edgeLines[i].t2 * scale,
               (IMAGE_COLS-1)*scale,
               scale*((-1*(float)edgeLines[i].t3
                - ((float)edgeLines[i].t1)*(IMAGE_COLS-1))
               /edgeLines[i].t2));
               painter.setPen(QColor(255, 0, 0));
      }
   }

   if (numPosts != 0 && goals != 0) {
      if (posts >= pYELLOW_LEFT) {
         painter.setPen(QColor(255, 255, 0));
      } else {
         painter.setPen(QColor(0, 255, 255));
      }
      for (uint16_t p = 0; p < numPosts*4; p += 4) {
         QLineF lines[4] = {
            QLineF(scale*goals[p], scale*goals[p+1], scale*goals[p+2],
                   scale*goals[p+1]),
            QLineF(scale*goals[p+2], scale*goals[p+1], scale*goals[p+2],
                   scale*goals[p+3]),
            QLineF(scale*goals[p+2], scale*goals[p+3], scale*goals[p],
                   scale*goals[p+3]),
            QLineF(scale*goals[p], scale*goals[p+3], scale*goals[p],
                   scale*goals[p+1])
         };
         painter.drawLines(lines, 4);
      }
   }

   if (radius != 0) {
      painter.setPen(QColor(0, 0, 0));
      painter.setBrush(QBrush(Qt::NoBrush));
      painter.drawEllipse(QPoint(scale * ballCentre.first,
              scale * ballCentre.second),
              (int)(scale * radius), (int)(scale * radius));
      painter.setPen(QColor(255, 255, 255));
      int i;
      for (i = 0; i < numBallEdgePoints; i++) {
         painter.drawEllipse(QPoint(ballEdgePoints[i].first,
                  ballEdgePoints[i].second), 1, 1);
      }
      painter.setPen(QColor(0,255,0));
      /*if (numBallEdgePoints != 0) {
         for (; i < numBallEdgePoints + NUM_CENTRE_REPEATS; i++) {
            painter.drawEllipse(QPoint(ballEdgePoints[i].first,
                     ballEdgePoints[i].second), 1, 1);
         }
      }*/
      /*painter.drawEllipse(QPoint(scale*ballCentre.first,
                          scale*ballCentre.second),
                          (int)(scale*radius), (int)(scale*radius));*/
   }

   if (numRobotRegions != 0) {
      painter.setPen(QColor(255, 255, 255));
      for (uint16_t p = 0; p < numRobotRegions; p++) {

         if (robotRegions[p]->type == RED_ROBOT) {
            painter.setPen(QColor(255, 0, 0));
         } else if (robotRegions[p]->type == BLUE_ROBOT) {
            painter.setPen(QColor(0, 0, 255));
         } else {
            painter.setPen(QColor(0, 50, 0));
         }

         QLineF lines[4] = {
            QLineF(robotRegions[p]->leftMost * SALIENCY_DENSITY * scale, 
                  robotRegions[p]->bottomMost * SALIENCY_DENSITY * scale, 
                  robotRegions[p]->leftMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->topMost * SALIENCY_DENSITY * scale),
            QLineF(robotRegions[p]->leftMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->topMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->rightMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->topMost * SALIENCY_DENSITY * scale),
            QLineF(robotRegions[p]->rightMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->topMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->rightMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->bottomMost * SALIENCY_DENSITY * scale),
            QLineF(robotRegions[p]->leftMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->bottomMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->rightMost * SALIENCY_DENSITY * scale,
                  robotRegions[p]->bottomMost * SALIENCY_DENSITY * scale)
            };
            painter.drawLines(lines, 4);
      }
   }

   if (numLineRegions != 0 && numRegions == 0) {
      painter.setPen(QColor(255, 0, 255));
      for (uint16_t p = 0; p < numLineRegions; p++) {
         for (uint16_t i = 0; i < lineRegions[p]->numScanPoints; i++) {
            painter.drawEllipse(
                 lineRegions[p]->startScans[i].first * SALIENCY_DENSITY,
                  lineRegions[p]->startScans[i].second * SALIENCY_DENSITY,
                  2, 2
                  );
            painter.drawEllipse(
                  lineRegions[p]->endScans[i].first * SALIENCY_DENSITY,
                  lineRegions[p]->endScans[i].second * SALIENCY_DENSITY,
                  2, 2
                  );
         }
      }
   }

   if (numRegions != 0) {
      for (uint16_t p = 0; p < numRegions; p++) {
         RegionType status = regions[p].classification;
         if (status == rBALL) {
            painter.setPen(QColor(128, 20, 20));
         } else if (status == rROBOT) {
            painter.setPen(QColor(255, 255, 255));
         } else if (status == rMAYBE_ROBOT) {
            painter.setPen(QColor(255, 255, 0));
         } else if (status == rFIELD_LINE) {
            painter.setPen(QColor(255, 0, 255));
         } else if (status == rCONNECTED_ROBOT) {
            painter.setPen(QColor(0, 255, 0));
         } else {
            painter.setPen(QColor(0, 255, 255));
         }
         if (!(regions[p].deleted)) {
            /*painter.drawEllipse(
                  regions[p].centreGravity.first * SALIENCY_DENSITY,
                  regions[p].centreGravity.second * SALIENCY_DENSITY,
                  2, 2
                  );*/
            // Draw the field lines
            // if (status == rFIELD_LINE) {
               for (uint16_t i = 0; i < regions[p].numScanPoints; i++) {
                  painter.drawEllipse(
                        regions[p].startScans[i].first * SALIENCY_DENSITY,
                        regions[p].startScans[i].second * SALIENCY_DENSITY,
                        2, 2
                        );
                  painter.drawEllipse(
                        regions[p].endScans[i].first * SALIENCY_DENSITY,
                        regions[p].endScans[i].second * SALIENCY_DENSITY,
                        2, 2
                        );
               }
            // }

            QLineF lines[4] = {
               QLineF(regions[p].leftMost * SALIENCY_DENSITY, 
                     regions[p].bottomMost.second * SALIENCY_DENSITY, 
                     regions[p].leftMost * SALIENCY_DENSITY,
                     regions[p].topMost.second * SALIENCY_DENSITY),
               QLineF(regions[p].leftMost * SALIENCY_DENSITY,
                     regions[p].topMost.second * SALIENCY_DENSITY,
                     regions[p].rightMost * SALIENCY_DENSITY,
                     regions[p].topMost.second * SALIENCY_DENSITY),
               QLineF(regions[p].rightMost * SALIENCY_DENSITY,
                     regions[p].topMost.second * SALIENCY_DENSITY,
                     regions[p].rightMost * SALIENCY_DENSITY,
                     regions[p].bottomMost.second * SALIENCY_DENSITY),
               QLineF(regions[p].leftMost * SALIENCY_DENSITY,
                     regions[p].bottomMost.second * SALIENCY_DENSITY,
                     regions[p].rightMost * SALIENCY_DENSITY,
                     regions[p].bottomMost.second * SALIENCY_DENSITY)
            };
            painter.drawLines(lines, 4);
         }
      }
   }
}

