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
#include <QFileDialog>
#include <QDebug>
#include <QPainter>
#include <QMouseEvent>
#include <vector>
#include <iostream>
#include <utility>
#include "visionTab.hpp"
#include "../../robot/perception/vision/yuv.hpp"
#include "blackboard/Blackboard.hpp"

using namespace std;

VisionTab::VisionTab(QTabWidget *parent,
      QMenuBar *menuBar, Vision *vision) : parent(parent) {
   initMenu(menuBar);
   init();
   this->vision = vision;
   currentFrame = NULL;
}


void VisionTab::initMenu(QMenuBar *menuBar) {
   visionMenu = new QMenu("Vision");
   menuBar->addMenu(visionMenu);

   loadNnmcAct = new QAction(tr("Load NNMC"), visionMenu);
   visionMenu->addAction(loadNnmcAct);

   // connect the actions
   connect(loadNnmcAct, SIGNAL(triggered()), this, SLOT(loadNnmc()));
}

void VisionTab::init() {
   layout = new QGridLayout();
   this->setLayout(layout);

   imagePixmap = QPixmap(640, 480);
   imagePixmap.fill(Qt::darkGray);
   camLabel  = new QLabel();
   camLabel->setPixmap(imagePixmap);

   layout->addWidget(camLabel, 0, 0, 1, 1);

   optionsLayout = new QVBoxLayout();
   layout->addLayout(optionsLayout, 0, 1, 1, 1, Qt::AlignTop);

   checkEdgePoints = new QCheckBox("Edge points", this);
   checkEdgeLines = new QCheckBox("Edge lines", this);
   checkGoals = new QCheckBox("Goals", this);
   checkBall = new QCheckBox("Ball", this);
   checkRobots = new QCheckBox("Robots", this);
   checkFieldLines = new QCheckBox("Field lines", this);
   checkRegions = new QCheckBox("Regions", this);

   checkEdgePoints->setChecked(true);
   checkEdgeLines->setChecked(true);
   checkFieldLines->setChecked(true);
   checkGoals->setChecked(true);
   checkBall->setChecked(true);
   checkRobots->setChecked(true);
   checkRegions->setChecked(false);

   optionsLayout->addWidget(checkEdgePoints);
   optionsLayout->addWidget(checkEdgeLines);
   optionsLayout->addWidget(checkGoals);
   optionsLayout->addWidget(checkBall);
   optionsLayout->addWidget(checkRobots);
   optionsLayout->addWidget(checkFieldLines);
   optionsLayout->addWidget(checkRegions);

   connect(checkEdgePoints, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkEdgeLines, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkFieldLines, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkGoals, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkBall, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkRobots, SIGNAL(stateChanged(int)),
                    this, SLOT(redrawSlot()));
   connect(checkRegions, SIGNAL(stateChanged(int)),
                     this, SLOT(redrawSlot()));


   this->setMouseTracking(true);
   camLabel->setMouseTracking(true);
   camLabel->setAlignment(Qt::AlignTop);
   camLabel->setMinimumSize(IMAGE_COLS, IMAGE_ROWS);
   camLabel->setMaximumSize(IMAGE_COLS, IMAGE_ROWS);
   camLabel->installEventFilter(this);
}

void VisionTab::mouseMoveEvent(QMouseEvent * event) {
   mousePosition = event->pos();
   mousePosition -= camLabel->pos();
   if (currentFrame && mousePosition.x() >= 0 &&
                    mousePosition.x() < IMAGE_COLS &&
         mousePosition.y() >= 0 && mousePosition.y() < IMAGE_ROWS) {
      stringstream message;

      message << "(" << mousePosition.x() << "," << mousePosition.y() << ")";
      QColor colour = QColor(this->getRGB(mousePosition.x(),
                      mousePosition.y(), currentFrame));

      message << " - (" << colour.red() << ", " <<
              colour.green() << ", " << colour.blue() << ")";
      emit showMessage(QString(message.str().c_str()), 0);
   }
}

void VisionTab::redraw() {
   if (currentFrame) {
      lastRendering =  QImage(IMAGE_COLS, IMAGE_ROWS, QImage::Format_RGB32);
      drawImage(&lastRendering);
      if (vision->getIsColourCalibrationLoaded()) {
         drawOverlays(&lastRendering);
      }
      imagePixmap = QPixmap(QPixmap::fromImage(
                    lastRendering.scaled(IMAGE_COLS, IMAGE_ROWS)));

   } else {
      imagePixmap = QPixmap(IMAGE_COLS, IMAGE_ROWS);
      imagePixmap.fill(Qt::darkGray);
   }

   camLabel->setPixmap(imagePixmap);
}



void VisionTab::drawImage(QImage *image) {
   for (unsigned int row = 0; row < IMAGE_ROWS; ++row) {
      for (unsigned int col = 0; col < IMAGE_COLS; ++col) {
         image->setPixel(col, row, getRGB(col, row, currentFrame));
      }
   }
}

void VisionTab::drawOverlays(QImage *image) {
   vision->currentFrame = currentFrame;
   vision ->saliencyScan();
   vision ->processFrame();


   vector<pair<uint16_t, uint16_t> > *edgePoints = 0;
   Line *edgeLines = 0;
   uint16_t *goals = 0;
   uint16_t numPosts = 0;
   WhichPosts posts = pNONE;
   ImageRegion *regions = 0;
   uint16_t numRegions = 0;
   uint16_t radius = 0;
   std::pair<uint16_t, uint16_t> ballCentre;
   std::pair<uint16_t, uint16_t> *ballEdgePoints = 0;
   uint16_t numBallEdgePoints = 0;
   RobotRegion **robotRegions = NULL;
   uint16_t numRobotRegions = 0;
   ImageRegion **lineRegions = NULL;
   uint16_t numLineRegions = 0;


   if (checkEdgePoints->isChecked()) {
      edgePoints = &(vision->fieldEdgeDetection.edgePoints);
   }

   if (checkEdgeLines->isChecked()) {
      edgeLines = vision->fieldEdgeDetection.edgeLines;
   }

   if (checkGoals->isChecked()) {
      goals = vision->goalDetection.postCoords;
      numPosts = vision->goalDetection.numPosts;
      posts = vision->goalDetection.typeOfPost;
   }

   if (checkBall->isChecked()) {
      radius = vision->ballDetection.radius;
      ballCentre = vision->ballDetection.ballCentre;
      ballEdgePoints = vision->ballDetection.ballEdgePoints;
      numBallEdgePoints = vision->ballDetection.numBallEdgePoints;
   }

   if (checkFieldLines->isChecked()) {
      lineRegions = vision->regionBuilder.lineRegions;
      numLineRegions = vision->regionBuilder.numLineRegions;
   }

   if (checkRegions->isChecked()) {
      regions = vision->regionBuilder.regions;
      numRegions = vision->regionBuilder.numRegions;
   }

   if (checkRobots->isChecked()) {
      robotRegions = vision->robotDetection.robotRegions;
      numRobotRegions = vision->robotDetection.numRobotRegions;
   }

   drawOverlaysGeneric(image,
         edgePoints,
         edgeLines,
         vision->fieldEdgeDetection.numEdgeLines,
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
         1);
   return;
}

// TODO(brockw): see if this can be genericized into tab.cpp, so it's not in
// every tab
void VisionTab::newNaoData(NaoData *naoData) {
   if (!naoData || !naoData->getCurrentFrame().blackboard) {
      imagePixmap.fill(Qt::darkGray);
      camLabel->setPixmap(imagePixmap);
      currentFrame = NULL;
   } else {
      Blackboard *blackboard = naoData->getCurrentFrame().blackboard;
      if ((currentFrame = readFrom(vision, currentFrame)) != NULL)
         if (parent->currentIndex() == parent->indexOf(this))
            redraw();
   }
}

void VisionTab::loadNnmc() {
   QString fileName = QFileDialog::getOpenFileName(this, "Load Nnmc File");
   emit showMessage("Loading Nnmc file...", 0);
   if (fileName != "") {
      vision->loadColourCalibration(fileName.toStdString().c_str());
      //  calibrationLoaded = true;
   }
   emit showMessage("Finished loading Nnmc file.", 5000);
   this->redraw();
}

void VisionTab::loadNnmcFile(const char *f) {
   vision->loadColourCalibration(f);
   emit showMessage("Finished loading Nnmc file.", 5000);
   this->redraw();
}

void VisionTab::redrawSlot() {
   redraw();
}

bool VisionTab::eventFilter(QObject *object, QEvent *event) {
   if ((object == camLabel) && (event->type() == QEvent::MouseButtonPress)) {
      return classifyMouseEvent(static_cast<QMouseEvent*>(event));
   } else {
      return false;
   }
}

bool VisionTab::classifyMouseEvent(QMouseEvent *e) {
   if (e->button() == Qt::RightButton) {
      QString fileName = QFileDialog::getSaveFileName(this, "Save image");
      if (fileName != "") {
         lastRendering.save(fileName);
      }
   }
   return true;
}
