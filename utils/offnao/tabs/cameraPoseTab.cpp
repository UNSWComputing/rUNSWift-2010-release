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
#include <qpushbutton.h>
#include <QFileDialog>
#include <QDebug>
#include <QPainter>
#include <QLine>
#include <QLabel>
#include <QPoint>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <QRadioButton>
#include <vector>
#include <iostream>
#include <utility>
#include <sstream>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "cameraPoseTab.hpp"
#include "utils/matrix_helpers.hpp"
#include "../../robot/perception/vision/yuv.hpp"
#include "../../robot/utils/basic_maths.hpp"
#include "../../robot/utils/SPLDefs.hpp"
#include "blackboard/Blackboard.hpp"

using namespace std;
using namespace boost::numeric::ublas;

CameraPoseTab::CameraPoseTab(QTabWidget *parent,
      QMenuBar *menuBar, Vision *vision) {
   initMenu(menuBar);
   init();
   this->vision = vision;
   this->parent = parent;

   // set up vector of lines
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2, FIELD_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2, -FIELD_WIDTH/2, 0));

   fieldLines.push_back(createPoint(-FIELD_LINE_WIDTH/2, FIELD_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LINE_WIDTH/2, -FIELD_WIDTH/2, 0));


   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH +
            FIELD_LINE_WIDTH/2, GOAL_BOX_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH +
            FIELD_LINE_WIDTH/2, -GOAL_BOX_WIDTH/2, 0));

   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH -
            FIELD_LINE_WIDTH/2, GOAL_BOX_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH -
            FIELD_LINE_WIDTH/2, -GOAL_BOX_WIDTH/2, 0));
   // lines that connect the goal box to the edge of field
   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH,
            GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LENGTH/2,
            GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH,
            GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LENGTH/2,
            GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));


   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH,
            -GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LENGTH/2,
            -GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(FIELD_LENGTH/2 - GOAL_BOX_LENGTH,
            -GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LENGTH/2,
            -GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));


   // other box
   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH -
            FIELD_LINE_WIDTH/2, GOAL_BOX_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH +
            FIELD_LINE_WIDTH/2, -GOAL_BOX_WIDTH/2, 0));

   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH -
            FIELD_LINE_WIDTH/2, GOAL_BOX_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH -
            FIELD_LINE_WIDTH/2, -GOAL_BOX_WIDTH/2, 0));


   // other box: lines that connect the goal box to the edge of field
   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH,
            GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LENGTH/2,
            GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH,
            GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LENGTH/2,
            GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));


   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH,
            -GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LENGTH/2,
            -GOAL_BOX_WIDTH/2 + FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(-FIELD_LENGTH/2 + GOAL_BOX_LENGTH,
            -GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LENGTH/2,
            -GOAL_BOX_WIDTH/2 - FIELD_LINE_WIDTH/2, 0));


   // middle cross
   fieldLines.push_back(createPoint(MARKER_CENTER_X - FIELD_LINE_WIDTH,
            FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(MARKER_CENTER_X + FIELD_LINE_WIDTH,
            FIELD_LINE_WIDTH/2, 0));

   fieldLines.push_back(createPoint(MARKER_CENTER_X - FIELD_LINE_WIDTH,
            -FIELD_LINE_WIDTH/2, 0));
   fieldLines.push_back(createPoint(MARKER_CENTER_X + FIELD_LINE_WIDTH,
            -FIELD_LINE_WIDTH/2, 0));



   fieldLines.push_back(createPoint(MARKER_CENTER_X - FIELD_LINE_WIDTH/2,
            -FIELD_LINE_WIDTH, 0));
   fieldLines.push_back(createPoint(MARKER_CENTER_X - FIELD_LINE_WIDTH/2,
            +FIELD_LINE_WIDTH, 0));

   fieldLines.push_back(createPoint(MARKER_CENTER_X + FIELD_LINE_WIDTH/2,
            -FIELD_LINE_WIDTH, 0));
   fieldLines.push_back(createPoint(MARKER_CENTER_X + FIELD_LINE_WIDTH/2,
            +FIELD_LINE_WIDTH, 0));


   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, FIELD_WIDTH/2, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, -FIELD_WIDTH/2, 0));

   fieldLines.push_back(createPoint(-FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, FIELD_WIDTH/2, 0));
   fieldLines.push_back(createPoint(-FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, -FIELD_WIDTH/2, 0));

   //side lines
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, FIELD_WIDTH/2 + FIELD_LINE_WIDTH, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, FIELD_WIDTH/2 + FIELD_LINE_WIDTH, 0));
   
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, FIELD_WIDTH/2 - FIELD_LINE_WIDTH, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, FIELD_WIDTH/2 - FIELD_LINE_WIDTH, 0));

   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, -FIELD_WIDTH/2 + FIELD_LINE_WIDTH, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, -FIELD_WIDTH/2 + FIELD_LINE_WIDTH, 0));
   
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 + FIELD_LENGTH/2, -FIELD_WIDTH/2 - FIELD_LINE_WIDTH, 0));
   fieldLines.push_back(createPoint(FIELD_LINE_WIDTH/2 - FIELD_LENGTH/2, -FIELD_WIDTH/2 - FIELD_LINE_WIDTH, 0));


   // generate centre circle
   int PARTS = 20;
   for(int i = 0; i < PARTS; i++) {
      float angle = (i * 360 / PARTS) * M_PI/180.0;
      float n_angle = ((i+1) * 360 / PARTS) * M_PI/180.0;

      int sx = cos(angle) * (CENTER_CIRCLE_DIAMETER/2.0 +
            FIELD_LINE_WIDTH/2);
      int sy = sin(angle) * (CENTER_CIRCLE_DIAMETER/2.0 +
            FIELD_LINE_WIDTH/2);

      int nx = cos(n_angle) * (CENTER_CIRCLE_DIAMETER/2.0 +
            FIELD_LINE_WIDTH/2);
      int ny = sin(n_angle) * (CENTER_CIRCLE_DIAMETER/2.0 +
            FIELD_LINE_WIDTH/2);
      fieldLines.push_back(createPoint(sx, sy, 0));
      fieldLines.push_back(createPoint(nx, ny, 0));

      sx = cos(angle) * (CENTER_CIRCLE_DIAMETER/2.0 -
            FIELD_LINE_WIDTH/2);
      sy = sin(angle) * (CENTER_CIRCLE_DIAMETER/2.0 -
            FIELD_LINE_WIDTH/2);

      nx = cos(n_angle) * (CENTER_CIRCLE_DIAMETER/2.0 -
            FIELD_LINE_WIDTH/2);
      ny = sin(n_angle) * (CENTER_CIRCLE_DIAMETER/2.0 -
            FIELD_LINE_WIDTH/2);
      fieldLines.push_back(createPoint(sx, sy, 0));
      fieldLines.push_back(createPoint(nx, ny, 0));
   }
}


void CameraPoseTab::initMenu(QMenuBar *menuBar) {
}

void CameraPoseTab::init() {
   layout = new QGridLayout();
   this->setLayout(layout);

   imagePixmap = QPixmap(640, 480);
   imagePixmap.fill(Qt::darkGray);
   camLabel  = new QLabel();
   camLabel->setPixmap(imagePixmap);

   layout->addWidget(camLabel, 0, 0, 1, 1);

   optionsLayout = new QGridLayout();
   layout->addLayout(optionsLayout, 0, 1, 1, 1, Qt::AlignTop);

   instructionString += "1) Place robot on penalty spot\n";
   instructionString += "2) Face robots feet and head towards far goal\n";
   instructionString += "3) Check image to see if field lines match\n";
   instructionString += "4) Adjust offset if they do not match\n";
   instructionString += "5) Update config file for robot when they match\n";

   QLabel *instructionsBox = new QLabel(instructionString);
   optionsLayout->addWidget(instructionsBox, 0, 0, 1, 1);

   QGridLayout *configureLayout = new QGridLayout();
   offsetXLabel = new QLineEdit("0.0");
   offsetYLabel = new QLineEdit("0.0");
   offsetBodyPitchLabel = new QLineEdit("0.0");
   offsetXLabel->setMaximumSize(50, 30);
   offsetYLabel->setMaximumSize(50, 30);
   offsetBodyPitchLabel->setMaximumSize(50, 30);
   QLabel *xLabel = new QLabel("X Offset");
   QLabel *yLabel = new QLabel("Y Offset");
   QLabel *bPitchLabel = new QLabel("Body Pitch Offset");
   QPushButton *sendButton = new QPushButton("send");
   radio1 = new QRadioButton(tr("Calibrate from &penalty spot"));
   radio2 = new QRadioButton(tr("Calibrate from &goal line"));
   radio1->setChecked(true);
   QVBoxLayout *vbox = new QVBoxLayout;
   vbox->addWidget(radio1);
   vbox->addWidget(radio2);
   // QPushButton *decButton = new QPushButton("-");
   // decButton->setMaximumSize(30,30);
   // incButton->setMaximumSize(30,30);

   //configureLayout->addWidget(decButton, 0, 0, 1, 1);
   configureLayout->addWidget(xLabel, 0, 0, 1, 1);
   configureLayout->addWidget(yLabel, 1, 0, 1, 1);
   configureLayout->addWidget(bPitchLabel, 2, 0, 1, 1);
   configureLayout->addWidget(offsetXLabel, 0, 1, 1, 1);
   configureLayout->addWidget(offsetYLabel, 1, 1, 1, 1);
   configureLayout->addWidget(offsetBodyPitchLabel, 2, 1, 1, 1);
   configureLayout->addWidget(sendButton, 3, 0, 1, 2);
   configureLayout->addLayout(vbox, 4, 0, 1, 2, Qt::AlignTop);
   optionsLayout->addLayout(configureLayout, 1, 0, 1, 1,
                            Qt::AlignLeft);

   connect(sendButton, SIGNAL(clicked()), this, SLOT(updateOffset()));
   // connect(decButton, SIGNAL(clicked()), this, SLOT(decOffset()));

   camLabel->setAlignment(Qt::AlignTop);
   camLabel->setMinimumSize(IMAGE_COLS, IMAGE_ROWS);
   camLabel->setMaximumSize(IMAGE_COLS, IMAGE_ROWS);
   camLabel->installEventFilter(this);

   currentOffsetX = 0.0;
   currentOffsetY = 0.0;
   currentBodyPitchOffset = 0.0;
   stringstream ss;
   ss << currentOffsetX;
   offsetXLabel->setText(QString(ss.str().c_str()));
   
   stringstream ss2;
   ss2 << currentOffsetY;
   offsetYLabel->setText(QString(ss2.str().c_str()));
   
   stringstream ss3;
   ss3 << currentBodyPitchOffset;
   offsetYLabel->setText(QString(ss3.str().c_str()));
}

void CameraPoseTab::redraw() {
   lastRendering =  QImage(IMAGE_COLS/SALIENCY_DENSITY,
                           IMAGE_ROWS/SALIENCY_DENSITY, QImage::Format_RGB32);
   drawImage(&lastRendering);
   imagePixmap = QPixmap(QPixmap::fromImage(
            lastRendering.scaled(IMAGE_COLS, IMAGE_ROWS)));
   drawOverlays(&imagePixmap);

   camLabel->setPixmap(imagePixmap);

   // currentOffset = readFrom(kinematics, cameraOffset);
}



void CameraPoseTab::drawImage(QImage *image) {
   for (unsigned int row = 0; row < IMAGE_ROWS/SALIENCY_DENSITY; ++row) {
      for (unsigned int col = 0; col < IMAGE_COLS/SALIENCY_DENSITY; ++col) {
         image->setPixel(col, row,
               QColor(CPLANE_COLOURS[saliency[col][row]]).rgb());
      }
   }

}

boost::numeric::ublas::matrix<float> CameraPoseTab::createPoint(float a,
      float b,
      float c) {
   matrix<float> point(4, 1);
   point(0, 0) = a;
   point(1, 0) = b;
   point(2, 0) = c;
   point(3, 0) = 1;
   return point;                                                 
}

void CameraPoseTab::drawOverlays(QPixmap *pixmap) {
   QPainter painter(pixmap);

   // set up robot abs position and heading
   //QPoint position(0, 0);
   QPoint position(-MARKER_CENTER_X, 0);
   if (!radio1->isChecked()) {
      position = QPoint(-3050, 0);
   }
   float heading = DEG2RAD(0);

   if (!blackboard) return;


   SensorValues sensorValues;
   for (int i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
      sensorValues.joints.angles[i] = 0;
   }
   sensorValues.joints.angles[Joints::HeadPitch] = DEG2RAD(-10);
   if(blackboard) sensorValues = readFrom(motion, sensors);
   kinematics.setSensorValues(sensorValues);
   Kinematics::Chain foot = kinematics.determineSupportChain();

   kinematics.updateDHChain();
   matrix<float> c2w = kinematics.createCameraToWorldTransform
                                       (foot);

   Pose pose = readFrom(kinematics, pose);
   c2w = pose.getC2wTransform();
   matrix<float> w2c = c2w;

   invertMatrix(c2w, w2c);
   matrix<float> ctest(4, 1);
   ctest(0, 0) = 0;
   ctest(1, 0) = 0;
   ctest(2, 0) = 0;
   ctest(3, 0) = 1;

   // add on translation to position
   matrix<float> pmatrix(4, 4);
   pmatrix(0, 0) = 1;
   pmatrix(0, 1) = 0;
   pmatrix(0, 2) = 0;
   pmatrix(0, 3) = -position.x();

   pmatrix(1, 0) = 0;
   pmatrix(1, 1) = 1;
   pmatrix(1, 2) = 0;
   pmatrix(1, 3) = -position.y();

   pmatrix(2, 0) = 0;
   pmatrix(2, 1) = 0;
   pmatrix(2, 2) = 1;
   pmatrix(2, 3) = 0;

   pmatrix(3, 0) = 0;
   pmatrix(3, 1) = 0;
   pmatrix(3, 2) = 0;
   pmatrix(3, 3) = 1;

   matrix<float> rmatrix = rotateZMatrix(heading);
   w2c = prod(w2c, rmatrix);
   w2c = prod(w2c, pmatrix);

   matrix<float> cmatrix = w2c;
   

   // finally set up projection matrix
   float ex = 0;
   float ey = 0;
   // float ez = 1.0/tan(DEG2RAD(34.8/2));
   float ez = 1.0/tan(DEG2RAD(46.4/2));
   matrix<float> projection(4, 4);
   projection(0, 0) = 1;
   projection(0, 1) = 0;
   projection(0, 2) = 0;
   projection(0, 3) = -ex;

   projection(1, 0) = 0;
   projection(1, 1) = 1;
   projection(1, 2) = 0;
   projection(1, 3) = -ey;

   projection(2, 0) = 0;
   projection(2, 1) = 0;
   projection(2, 2) = 1;
   projection(2, 3) = 0;

   projection(3, 0) = 0;
   projection(3, 1) = 0;
   projection(3, 2) = 1.0/ez;
   projection(3, 3) = 0;

   matrix<float> transform = prod(projection, w2c);
   /*
      matrix<float> pixel = prod(transform, point);
      std::cout << "Point: " << pixel << std::endl;
      pixel(0, 0) /= pixel(3, 0);
      pixel(1, 0) /= pixel(3, 0);
      pixel(2, 0) = 0;
      pixel(3, 0) = 1;
      std::cout << "Image coord (-1 1) : " << pixel << std::endl;

   // really need to define these somewhere
   float ar = 34.8/46.4;
   float xscale = 640/2;
   float yscale = 480/2;
   pixel(0, 0) = (pixel(0, 0)) * xscale + xscale;
   pixel(1, 0) = (pixel(1, 0)) * yscale + yscale;
   std::cout << "Image coord: " << pixel << std::endl;
    */
   std::vector<matrix<float> > imageLines;
   for(int i = 0; i < fieldLines.size(); i++) {
      // std::cout << fieldLines[i] << ": " << prod(cmatrix,fieldLines[i]) << std::endl;
      matrix<float> pixel = prod(transform, fieldLines[i]);
      // std::cout << fieldLines[i] << ": " << pixel << std::endl;
      // std::cout << "Point: " << pixel << std::endl;
      pixel(0, 0) /= ABS(pixel(3, 0));
      pixel(1, 0) /= ABS(pixel(3, 0));
      pixel(2, 0) = 0;
      // std::cout << "Image coord (-1 1) : " << pixel << std::endl;

      // really need to define these somewhere
      float ar = 34.8/46.4;
      float xscale = 640/2;
      float yscale = 480/2;
      pixel(0, 0) = (pixel(0, 0)) * xscale + xscale;
      pixel(1, 0) = (pixel(1, 0)) * xscale + yscale;
      // std::cout << "Image coord: " << pixel << std::endl;

      imageLines.push_back(pixel);
   }
   for(int i = 0; i < fieldLines.size(); i += 2) {
      if (imageLines[i](3, 0) >= 0 && imageLines[i+1](3, 0) >= 0) {
         painter.drawLine(imageLines[i](0, 0),
               imageLines[i](1, 0),
               imageLines[i+1](0, 0),
               imageLines[i+1](1, 0));
            }
   }

}

// TODO(brockw): see if this can be genericized into tab.cpp, so it's not in
// every tab
void CameraPoseTab::newNaoData(NaoData *naoData) {
   if (!naoData || !naoData->getCurrentFrame().blackboard) {
      imagePixmap.fill(Qt::darkGray);
      camLabel->setPixmap(imagePixmap);
      blackboard = NULL;
   } else {
      if (parent->currentIndex() == parent->indexOf(this)) {
         blackboard = naoData->getCurrentFrame().blackboard;
         if (readFrom(vision, saliency)) {
            memcpy(saliency, readFrom(vision, saliency),
                  120*160*sizeof(Colour));
            redraw();
         }
      }
   }
}

void CameraPoseTab::redrawSlot() {
   redraw();
}

void CameraPoseTab::updateOffset() {
   if (blackboard) {
      stringstream ss;
      ss << ("--kinematics.cameraoffsetXbottom=");
      ss << (offsetXLabel->text()).toStdString();
      emit sendCommandToRobot(QString(ss.str().c_str()));
      
      stringstream ss1;
      ss1 << ("--kinematics.cameraoffsetXtop=");
      ss1 << (offsetXLabel->text()).toStdString();
      emit sendCommandToRobot(QString(ss1.str().c_str()));
      
      
      stringstream ss2;
      ss2 << ("--kinematics.cameraoffsetYtop=");
      ss2 << (offsetYLabel->text()).toStdString();
      emit sendCommandToRobot(QString(ss2.str().c_str()));
      
      stringstream ss3;
      ss3 << ("--kinematics.cameraoffsetYbottom=");
      ss3 << (offsetYLabel->text()).toStdString();
      emit sendCommandToRobot(QString(ss3.str().c_str()));
      
      stringstream ss4;
      ss4 << ("--kinematics.bodyPitchOffset=");
      ss4 << (offsetBodyPitchLabel->text()).toStdString();
      emit sendCommandToRobot(QString(ss4.str().c_str()));
   }
}

void CameraPoseTab::incOffset() {
}

void CameraPoseTab::decOffset() {
}
