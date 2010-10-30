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

#include <QMenu>
#include <QMenuBar>
#include <QDebug>
#include <QBitmap>

#include <utility>
#include <iostream>
#include "overviewTab.hpp"
#include "../../robot/perception/vision/yuv.hpp"
#include "../../robot/perception/vision/rle.hpp"
#include "../../robot/perception/kinematics/Pose.hpp"
#include "blackboard/Blackboard.hpp"

using namespace std;

OverviewTab::OverviewTab(QTabWidget *parent, QMenuBar *menuBar,
      Vision *vision, Classifier *classifier) : blackboard(0) {
   initMenu(menuBar);
   init();
   this->vision = vision;
   this->classifier = classifier;
   memset(saliency, 0, 160*120*sizeof(Colour));
   this->parent = parent;
}


void OverviewTab::initMenu(QMenuBar *) {
}

void OverviewTab::init() {
   layout = new QGridLayout(this);
   setLayout(layout);
   layout->setAlignment(layout, Qt::AlignTop);

   layout->setHorizontalSpacing(5);
   layout->setHorizontalSpacing(5);


   layout->addWidget(&fieldView, 0, 0, 2, 1);

   /* draw the field with nothing on it */
   fieldView.redraw(NULL);


   imagePixmap = QPixmap(640.0/2, 480.0/2);
   imagePixmap.fill(Qt::darkGray);
   camLabel  = new QLabel();
   camLabel->setPixmap(imagePixmap);
   camLabel->setMinimumSize(640.0/2, 480.0/2);
   camLabel->setMaximumSize(640.0/2, 480.0/2);
   layout->addWidget(camLabel, 0, 1, 1, 2);

   layout->addWidget(&variableView, 1, 1, 1, 2);
}





void OverviewTab::redraw() {
   if (currentFrame || saliency) {
      QImage *image;
      if (currentFrame && !vision->getIsColourCalibrationLoaded()) {
         image = new QImage(IMAGE_COLS, IMAGE_ROWS, QImage::Format_RGB32);
      } else {
         image = new QImage(IMAGE_COLS/SALIENCY_DENSITY,
                            IMAGE_ROWS/SALIENCY_DENSITY,
                            QImage::Format_RGB32);
      }
      drawImage(image);
      
      imagePixmap = QPixmap::fromImage(
                     image->scaled(IMAGE_COLS/SALIENCY_DENSITY*2,
                               IMAGE_ROWS/SALIENCY_DENSITY*2));
      drawOverlays(&imagePixmap);
      //imagePixmap = QPixmap::fromImage(*image);
      delete image;
   } else {
      imagePixmap = QPixmap(IMAGE_COLS, IMAGE_ROWS);
      imagePixmap.fill(Qt::darkRed);
   }
   camLabel->setPixmap(imagePixmap);
}

void OverviewTab::drawOverlays(QPixmap *image) {
   if (!blackboard) return;

   vector<pair<uint16_t, uint16_t> > *edgePoints = 0;
   // Line *edgeLines = 0;
   uint16_t goals[MAX_POSTS*4];
   uint16_t numPosts = 0;
   WhichPosts posts = pNONE;
   ImageRegion *regions = 0;
   uint16_t numRegions = 0;
   uint16_t radius = 0;
   uint8_t numBalls = 0;
   std::pair<uint16_t, uint16_t> ballCentre;
   std::pair<uint16_t, uint16_t> *ballEdgePoints = NULL;
   uint16_t numBallEdgePoints = 0;
   uint16_t numEdges = readFrom(vision, numEdges);

   Line edgeLines[MAX_FIELD_EDGES];
   readArray(vision, frameEdges, edgeLines);
   radius = readFrom(vision, ballRadius);
   numBalls = readFrom(vision, numBalls);
   ballCentre = readFrom(vision, ballInCameraCoords);
   if (numBalls == 0) {
      radius = 0;
   }

   readArray(vision, postCoords, goals);
   posts = readFrom(vision, posts);
   if (posts == pNONE) {
      numPosts = 0;
   } else if (posts == pYELLOW_BOTH || posts == pBLUE_BOTH) {
      numPosts = 2;
   } else {
      numPosts = 1;
   }

   RobotRegion rReg[MAX_NUM_ROBOTS];
   RobotRegion *robotRegions[MAX_NUM_ROBOTS];
   RobotType rTypes[MAX_NUM_ROBOTS];
   uint16_t rImageCoords[MAX_NUM_ROBOTS * 4];
   readArray(vision, robotImageCoords, rImageCoords);
   readArray(vision, robotTypes, rTypes);
   uint16_t numRobotRegions = readFrom(vision, numRobots);
   for (int i = 0; i < numRobotRegions; i++) {
      rReg[i].topMost = rImageCoords[i*4];
      rReg[i].rightMost = rImageCoords[i*4+1];
      rReg[i].bottomMost = rImageCoords[i*4+2];
      rReg[i].leftMost = rImageCoords[i*4+3];
      rReg[i].type = rTypes[i];
      robotRegions[i] = &(rReg[i]);
   }

   // Not implemented yet
   ImageRegion **lineRegions = NULL;
   uint16_t numLineRegions = 0;


   drawOverlaysGeneric(image,
         edgePoints,
         edgeLines,
         numEdges,
         goals,
         numPosts,
         posts,
         regions,
         numRegions,
         radius,
         ballCentre,
         ballEdgePoints,
         numBallEdgePoints,
         robotRegions,
         numRobotRegions,
         lineRegions,
         numLineRegions,
         2.0/(SALIENCY_DENSITY));

   QPainter painter(image);
   const Pose &pose = readFrom(kinematics, pose);
   const std::pair<int, int> horizon = pose.getHorizon();
   painter.setBrush(QBrush(QColor(255, 255, 255)));
   painter.drawLine(0,horizon.first/SALIENCY_DENSITY*2,
         640/SALIENCY_DENSITY*2,
         horizon.second/SALIENCY_DENSITY*2);

   //draw body exclusion points
   painter.setBrush(QBrush(QColor(255, 255, 255)));
   float scale = 2.0/SALIENCY_DENSITY;
   const int16_t *points = pose.getExclusionArray();
   for (int i = 0; i < Pose::EXCLUSION_RESOLUTION; i++) {
       painter.drawEllipse(QPoint(scale*640 * i*1.0/Pose::EXCLUSION_RESOLUTION,
                         scale*points[i]), 2, 2);
   }

   return;
}


void OverviewTab::drawImage(QImage *image) {
   if (currentFrame) {
      if (vision->getIsColourCalibrationLoaded()) {
         vision->currentFrame = currentFrame;
         vision->saliencyScan();
         vision->processFrame();

         fieldView.redraw(NULL);

         for (unsigned int row = 0; row <
                 IMAGE_ROWS/SALIENCY_DENSITY; ++row) {
            for (unsigned int col = 0; col <
                 IMAGE_COLS/SALIENCY_DENSITY; ++col) {
               image->setPixel(col, row,
               QColor(CPLANE_COLOURS[vision->saliency[col][row]]).rgb());
            }
         }
      } else {
         // display normal image
         for (unsigned int row = 0; row < IMAGE_ROWS; ++row) {
            for (unsigned int col = 0; col < IMAGE_COLS; ++col) {
               image->setPixel(col, row,  getRGB(col, row, currentFrame));
            }
         }
      }
   } else {
      for (unsigned int row = 0; row < IMAGE_ROWS/SALIENCY_DENSITY; ++row) {
         for (unsigned int col = 0; col < IMAGE_COLS/SALIENCY_DENSITY; ++col) {
            if (saliency[col][row] < 0 || saliency[col][row] > 9) {
               std::cerr << "Bad pixel at " << col << " " << row << std::endl;
            } else {
               image->setPixel(col, row,
               QColor(CPLANE_COLOURS[saliency[col][row]]).rgb());
            }
         }
      }
   }
}

// TODO(brockw): see if this can be genericized into tab.cpp, so it's not in
// every tab
void OverviewTab::newNaoData(NaoData *naoData) {
   if (!naoData || !naoData->getCurrentFrame().blackboard) {  // clean up display, as read is finished
      imagePixmap.fill(Qt::darkGray);
      camLabel->setPixmap(imagePixmap);
   } else if (naoData->getFramesTotal() != 0) {
      blackboard = (naoData->getCurrentFrame().blackboard);
      currentFrame = readFrom(vision, currentFrame);
      if (!currentFrame) {
         if (readFrom(vision, saliency))
            memcpy(saliency, readFrom(vision, saliency),
                   120*160*sizeof(Colour));
      }
      if (parent->currentIndex() == parent->indexOf(this)) {
         redraw();
         fieldView.redraw(naoData);
         variableView.redraw(naoData);
      }
   }
}

