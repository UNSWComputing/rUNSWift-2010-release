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


#include <QDebug>
#include <QFile>
#include <QMenu>
#include <QMenuBar>
#include <QInputDialog>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>
#include <sstream>

#include "../../robot/perception/vision/yuv.hpp"
#include "calibrationTab.hpp"
#include <boost/shared_ptr.hpp>
#include "blackboard/Blackboard.hpp"

using namespace std;

static const char* OverlayNames[] = {
   "none",
   "selected",
   "selected+unclassified",
   "all colours",
   "all + unclassified"
};

/* A handy helper function */
void inRange(int &n, int min, int max) {
   if (n < min) {
      n = min;
   }
   if (n > max) {
      n = max;
   }
}

CalibrationTab::CalibrationTab(QTabWidget *parent, QMenuBar *menuBar,
                               Vision *vision, Classifier *classifier) :
   zoomLevel(1), zoomLog(0),  prevMouseX(0), prevMouseY(0),
             prevZoomLevel(1), naoData(0), qdwPointCloud(tr("Point Cloud")) {
   initMenu(menuBar);
   init();
   this->vision = vision;
   this->classifier = classifier;
   this->parent = parent;
}

void CalibrationTab::initMenu(QMenuBar *menuBar) {
   calibrationMenu = new QMenu("&Calibration");
   menuBar->addMenu(calibrationMenu);

   newKernelAct = new QAction(tr("&New Kernel"), calibrationMenu);
   loadKernelAct = new QAction(tr("&Load Kernel"), calibrationMenu);
   saveAsKernelAct = new QAction(tr("&Save Kernel"), calibrationMenu);
   saveAsNNMCAct = new QAction(tr("Save &NNMC"), calibrationMenu);
   undoAct  = new QAction(tr("&Undo Act"), calibrationMenu);



   calibrationMenu->addAction(newKernelAct);
   calibrationMenu->addAction(loadKernelAct);
   calibrationMenu->addAction(saveAsKernelAct);
   calibrationMenu->addSeparator();
   calibrationMenu->addAction(saveAsNNMCAct);
   calibrationMenu->addAction(tr("&Push NNMC To Nao"), this, SLOT(pushNnmc()),
         QKeySequence(tr("Ctrl+P", "File|Print")));

   // connect the actions
   connect(newKernelAct, SIGNAL(triggered()), this, SLOT(newKernel()));
   connect(loadKernelAct, SIGNAL(triggered()), this, SLOT(loadKernel()));
   connect(saveAsKernelAct, SIGNAL(triggered()), this, SLOT(saveKernel()));
   connect(saveAsNNMCAct, SIGNAL(triggered()), this, SLOT(saveNnmc()));
   connect(undoAct, SIGNAL(triggered()), this, SLOT(undoAction()));
}

void CalibrationTab::init() {
   // Camera Image
   imagePixmap = QPixmap(640, 480);
   imagePixmap.fill(Qt::darkGray);
   camLabel  = new QLabel();
   camLabel->setPixmap(imagePixmap);

   // Colour Picker & Overlays
   checkboxLayout = new QVBoxLayout();
   colourGroup = new QButtonGroup();

   colourBox = new QGroupBox(tr("Colour"));
   colourGroupLayout = new QVBoxLayout();
   colourBox->setLayout(colourGroupLayout);
   checkboxLayout->addWidget(colourBox);

   for (int i = 0; i < cNUM_COLOURS; ++i) {
      if (i == 1) {
         colours[i] = new QRadioButton(QString("goal and robot blue"), colourBox);
         colourGroupLayout->addWidget(colours[i]);
         colourGroup->addButton(colours[i], i);
      } else if (i == 3) {
      } else {
         colours[i] = new QRadioButton(QString(ColourNames[i]), colourBox);
         colourGroupLayout->addWidget(colours[i]);
         colourGroup->addButton(colours[i], i);
      }
   }
   colours[0]->setChecked(true);

   overlayGroup = new QButtonGroup();
   overlayGroupLayout = new QVBoxLayout();
   overlayBox = new QGroupBox(tr("Overlays"));
   overlayBox->setLayout(overlayGroupLayout);
   checkboxLayout->addWidget(overlayBox);

   for (int i = 0; i < oNUM_OVERLAYS; ++i) {
      overlays[i] = new QRadioButton(QString(OverlayNames[i]), overlayBox);
      overlayGroupLayout->addWidget(overlays[i]);
      overlayGroup->addButton(overlays[i], i);
   }
   overlays[0]->setChecked(true);

   checkVisionOverlay = new QCheckBox("vision module", this);
   checkVisionOverlay->setChecked(false);
   overlayGroupLayout->addWidget(checkVisionOverlay);

   undo = new QPushButton(QString("Undo"));
   checkboxLayout->addWidget(undo);

   checkAutoWeight = new QCheckBox(
                  "Auto-Weight Guassians (good for fine-tuning)", this);
   checkAutoWeight->setChecked(false);
   checkboxLayout->addWidget(checkAutoWeight);

   checkboxLayout->addStretch(1);

   qrbControls[0] = new QRadioButton(tr("&Selected"));
   qrbControls[1] = new QRadioButton(tr("&All"));
   qrbControls[1]->setChecked(true);
   qhblControls.addWidget(qrbControls[0]);
   qhblControls.addWidget(qrbControls[1]);
   qhblControls.addStretch(1);
   qglPointCloud.addWidget(&pointCloud, 0, 0, 1, 1);
   qglPointCloud.addLayout(&qhblControls, 1, 0, 1, 1);
   qwPointCloud.setLayout(&qglPointCloud);
   qdwPointCloud.setAllowedAreas(Qt::BottomDockWidgetArea);
   qdwPointCloud.setWidget(&qwPointCloud);

   // Lay them all out
   layout = new QGridLayout();
   layout->setAlignment(layout, Qt::AlignTop);
   this->setLayout(layout);
   layout->addWidget(camLabel, 0, 0, 1, 1);
   layout->addWidget(&qdwPointCloud, 1, 0, 1, 1);
   layout->addLayout(checkboxLayout, 0, 1, 1, 1);
   camLabel->setAlignment(Qt::AlignTop);
   camLabel->setMinimumSize(IMAGE_COLS, IMAGE_ROWS);
   camLabel->setMaximumSize(IMAGE_COLS, IMAGE_ROWS);

   // Set up signals
   connect(colourGroup, SIGNAL(buttonReleased(int)), this,
                               SLOT(setColour(int)));
   connect(overlayGroup, SIGNAL(buttonReleased(int)), this,
                               SLOT(setOverlay(int)));
   connect(undo, SIGNAL(released()), this,
                               SLOT(undoAction()));
   connect(checkVisionOverlay, SIGNAL(stateChanged(int)), this,
                               SLOT(redrawSlot()));
   connect(qrbControls[0], SIGNAL(toggled(bool)), this, SLOT(redrawSlot()));

   // Even filters
   camLabel->installEventFilter(this);

   // Data
   colour = (Colour) 0;
   overlay = (Overlay) 0;

   dumpFile = NULL;
   currentFrame = NULL;

   setMouseTracking(true);
   camLabel->setMouseTracking(true);
   camLabel->setCursor(QCursor(Qt::BlankCursor));

   progressDialog = new QProgressDialog(this);
   connect(progressDialog, SIGNAL(canceled()), this, SLOT(cancelUpload()));
}


void CalibrationTab::newKernel() {
   classifier->newClassificationFile();
   redraw();
}


void CalibrationTab::loadKernel() {
   QString fileName = QFileDialog::getOpenFileName(this, "Load Kernel File");
   emit showMessage("Loading kernel file...", 0);
   if (fileName != NULL) {
      classifier->loadClassificationFile(fileName.toStdString());
   }
   emit showMessage("Kernel file loaded.", 5000);
   this->redraw();
}

void CalibrationTab::loadKernelFile(std::string f) {
   emit showMessage("Loading kernel file...", 0);
   classifier->loadClassificationFile(f);
   emit showMessage("Kernel file loaded.", 5000);
   this->redraw();
}

void CalibrationTab::saveKernel() {
   if (classifier->classificationFileOpened()) {
      cout << "In save kernel" << endl;
      cout << this << endl;
      QString fileName = QFileDialog::getSaveFileName(this, "Save Kernel File");
      if (fileName != NULL) {
         classifier->saveClassificationFile(fileName.toStdString());
      }
   } else {
      QMessageBox::warning(this, "Cannot save Kernel", "Kernel file not open");
   }
}


void CalibrationTab::saveNnmc() {
   if (classifier->classificationFileOpened()) {
      QString fileName = QFileDialog::getSaveFileName(this, "Save NNMC File");
      if (fileName != NULL) {
         classifier->saveNnmc(fileName.toStdString());
      }
   } else {
      QMessageBox::warning(this, "Cannot save NNMC", "Kernel file not open");
   }
}

void CalibrationTab::pushNnmc() {
   if (classifier->classificationFileOpened()) {
      QString fileName = "/tmp/nnmc.cal";
      classifier->saveNnmc(fileName.toStdString());
#ifndef QT_NO_CURSOR
      setCursor(Qt::WaitCursor);
#endif
      QStringList items;
      items << tr("scout.local") << tr("heavy.local") << tr("demo-man.local") <<
              tr("sniper.local") << tr("spy.local") << tr("soldier.local") <<
              tr("medic.local") << tr("engineer.local") << tr("localhost");
      items.sort();

      bool ok;
      QString item = QInputDialog::getItem(this, tr(""),
                                           tr("Connect to:"), items, 0, false, &ok);

      if (!ok) {
          return;
      }

      ftp = new QFtp(this);
      connect(ftp, SIGNAL(commandFinished(int, bool)),
            this, SLOT(ftpCommandFinished(int, bool)));
      connect(ftp, SIGNAL(dataTransferProgress(qint64, qint64)),
            this, SLOT(updateDataTransferProgress(qint64, qint64)));

      ftp->setProxy("roborouter.ai.cse.unsw.edu.au", 2121);
      ftp->connectToHost(item);
      ftp->login("root", "");
      ftp->cd("/etc/runswift");

      emit showMessage(tr("Connecting to FTP server %1...").arg("soldier"));
      nnmcFile = new QFile(fileName);
      if (!nnmcFile->open(QIODevice::ReadOnly)) {
         QMessageBox::information(this, tr("FTP"),
               tr("Unable to load the file %1: %2.").
               arg(fileName).arg(nnmcFile->errorString()));
         delete nnmcFile;
         return;
      }

      ftp->put(nnmcFile, "nnmc.cal");

      progressDialog->setLabelText(tr("Uploading %1...").arg(fileName));
      progressDialog->exec();
   } else {
      QMessageBox::warning(this, "Cannot save NNMC", "Kernel file not open");
   }
}

void CalibrationTab::giveUp() {
   exit(0);
}


void CalibrationTab::setColour(int radio_id) {
   this->colour = (Colour) radio_id;
   redraw();
}


void CalibrationTab::setOverlay(int radio_id) {
   this->overlay = (Overlay) radio_id;
   redraw();
}

void CalibrationTab::undoAction() {
   if (classifier->canUndo()) {
      classifier->undo();
      redraw();
   }
}

QPixmap CalibrationTab::drawCrosshair() {
   QPixmap preRender = QPixmap::fromImage(lastRendering);
   if (!naoData || naoData->getIsPaused() == false || !currentFrame) {
      camLabel->setCursor(QCursor());
      return preRender;
   }

   camLabel->setCursor(QCursor(Qt::BlankCursor));

   QPainter p(&preRender);
   p.setBrush(QBrush(QColor(255, 255, 255)));
   p.drawLine(mousePosition.x(), 0, mousePosition.x(),
      mousePosition.y()-2);
   p.drawLine(mousePosition.x(), mousePosition.y()+2,
      mousePosition.x(), IMAGE_ROWS);
   p.drawLine(0, mousePosition.y(), mousePosition.x()-2, mousePosition.y());
   p.drawLine(mousePosition.x()+2, mousePosition.y(),
              IMAGE_COLS, mousePosition.y());

   if (mousePosition.x() >= 0 && mousePosition.x() < IMAGE_COLS &&
         mousePosition.y() >= 0 && mousePosition.y() < IMAGE_ROWS) {
      stringstream message;
      QPoint translatedMousePos = translateToZoomedImageCoords(
                                 QPoint(mousePosition.x(), mousePosition.y()));

      message << "(" << translatedMousePos.x() <<
                 "," << translatedMousePos.y() << ")";
      QColor colour = QColor(this->getRGB(translatedMousePos.x(),
                             translatedMousePos.y(), currentFrame));

      message << " - (" << colour.red() << ", " << colour.green()
              << ", " << colour.blue() << ")";
      emit showMessage(QString(message.str().c_str()), 0);
   }
   return preRender;
}

void CalibrationTab::redraw() {
   lastRendering = QImage(IMAGE_COLS, IMAGE_ROWS, QImage::Format_RGB32);

   if (currentFrame) {
      QImage *image = new QImage(IMAGE_COLS, IMAGE_ROWS, QImage::Format_RGB32);

      drawImage(image);
      drawOverlays(image);
      drawPointCloud();

      QTransform prevTransform;
      prevTransform = prevTransform.translate(+prevMouseX, +prevMouseY);
      prevTransform = prevTransform.scale(1.0/prevZoomLevel, 1.0/prevZoomLevel);
      prevTransform = prevTransform.translate(-prevMouseX, -prevMouseY);

      QRectF realMouseCoords = prevTransform.mapRect(
                               QRectF(mouseX, mouseY, 0, 0));
      mouseX = realMouseCoords.left();
      mouseY = realMouseCoords.top();

      QTransform transform;
      transform = transform.translate(+mouseX, +mouseY);
      transform = transform.scale(1.0/zoomLevel, 1.0/zoomLevel);
      transform = transform.translate(-mouseX, -mouseY);
      imagePixmap = QPixmap(QPixmap::fromImage(
                           image->scaled(IMAGE_COLS, IMAGE_ROWS)));
      QPainter painter(&lastRendering);
      QRectF bound = transform.mapRect(QRectF(0, 0, IMAGE_COLS, IMAGE_ROWS));

      painter.drawImage(QRectF(0, 0, IMAGE_COLS, IMAGE_ROWS),
                        imagePixmap.toImage(), bound);


      prevZoomLevel = zoomLevel;
      prevMouseX = mouseX;
      prevMouseY = mouseY;
      delete image;
   } else {
      imagePixmap = QPixmap(IMAGE_COLS, IMAGE_ROWS);
      imagePixmap.fill(Qt::darkGray);
      lastRendering = imagePixmap.toImage();
   }

   QPixmap preRender = drawCrosshair();
   camLabel->setPixmap(preRender);
}

void CalibrationTab::mouseMoveEvent(QMouseEvent* event) {
   mousePosition = event->pos();
   mousePosition -= camLabel->pos();

   if (currentFrame) {
      QPixmap preRender = drawCrosshair();
      camLabel->setPixmap(preRender);
   } else {
      camLabel->setCursor(QCursor());
   }
}

void CalibrationTab::drawImage(QImage *image) {
   for (unsigned int row = 0; row < IMAGE_ROWS; ++row) {
      for (unsigned int col = 0; col < IMAGE_COLS; ++col) {
         image->setPixel(col, row, getRGB(col, row, currentFrame));
      }
   }
}


void CalibrationTab::drawOverlays(QImage *image) {
   uint8_t y, u, v;
   switch (overlay) {
      case oNONE:
         break;
      case oSELECTED:
         for (int i = 0; i < IMAGE_COLS; ++i) {
            for (int j = 0; j < IMAGE_ROWS; ++j) {
               y = gety(currentFrame, j, i);
               u = getu(currentFrame, j, i);
               v = getv(currentFrame, j, i);
               if (classifier->getClassifiedColour(y, u, v) == colour) {
                  QColor classifiedColour = QColor(CPLANE_COLOURS[
                        classifier->getClassifiedColour(y, u, v)]);
                  image->setPixel(i, j, classifiedColour.rgb());
               }
            }
         }
         break;
      case oSELECTED_UNCLASSIFIED:
         for (int i = 0; i < IMAGE_COLS; ++i) {
            for (int j = 0; j < IMAGE_ROWS; ++j) {
               y = gety(currentFrame, j, i);
               u = getu(currentFrame, j, i);
               v = getv(currentFrame, j, i);
               if (classifier->getClassifiedColour(y, u, v) == cUNCLASSIFIED ||
                        classifier->getClassifiedColour(y, u, v) == colour) {
                  image->setPixel(i, j, QColor(CPLANE_COLOURS[
                        classifier->getClassifiedColour(y, u, v)]).rgb());
               }
            }
         }
         break;
      case oALL:
         for (int i = 0; i < IMAGE_COLS; ++i) {
            for (int j = 0; j < IMAGE_ROWS; ++j) {
               y = gety(currentFrame, j, i);
               u = getu(currentFrame, j, i);
               v = getv(currentFrame, j, i);
               if (classifier->getClassifiedColour(y, u, v) != cUNCLASSIFIED) {
                  image->setPixel(i, j, QColor(CPLANE_COLOURS[
                     classifier->getClassifiedColour(y, u, v)]).rgb());
               }
            }
         }
         break;
      case oALL_UNCLASSIFIED:
         for (int i = 0; i < IMAGE_COLS; ++i) {
            for (int j = 0; j < IMAGE_ROWS; ++j) {
               y = gety(currentFrame, j, i);
               u = getu(currentFrame, j, i);
               v = getv(currentFrame, j, i);
               image->setPixel(i, j, QColor(
               CPLANE_COLOURS[classifier->getClassifiedColour(y, u, v)]).rgb());
            }
         }
         break;
      default:
         QMessageBox::warning(this, "Error",
                     "Somebody broke the code, check the overlays");
   }

   if (checkVisionOverlay->isChecked()) {
      vision->currentFrame = currentFrame;
      static boost::shared_array<uint8_t> ptr(reinterpret_cast<uint8_t*>(classifier->getNnmcPointer()));
      vision->nnmc = ptr;
      // vision->nnmc = classifier->getNnmcPointer();
      vision->isColourCalibrationLoaded = true;
      vision ->saliencyScan();
      vision ->processFrame();
      QPainter painter(image);
      painter.setPen(QColor(1, 0, 0));
      painter.setBrush(QBrush(QColor(255, 255, 0)));

      vector<pair<uint16_t, uint16_t> >::const_iterator p;
      for (p = vision->fieldEdgeDetection.edgePoints.begin();
            p != vision->fieldEdgeDetection.edgePoints.end(); ++p) {
         painter.drawEllipse(QPoint((*p).first, (*p).second), 2, 2);
      }

      for (int i = 0; i < vision->fieldEdgeDetection.numEdgeLines; ++i) {
         painter.setPen(QColor(255, 255, 0));
         painter.drawLine(
               0,
               -1*(float)vision->fieldEdgeDetection.edgeLines[i].t3/
                  vision->fieldEdgeDetection.edgeLines[i].t2,
               IMAGE_COLS-1,
               (-1*(float)vision->fieldEdgeDetection.edgeLines[i].t3
                  - ((float)vision->fieldEdgeDetection.edgeLines[i].t1)*
                  (IMAGE_COLS-1))
                  /vision->fieldEdgeDetection.edgeLines[i].t2);
      }
   }
}

void CalibrationTab::drawPointCloud() {
   pointCloud.points.resize(0);
   if (classifier->classificationFileOpened()) {
      bool selectedOnly = qrbControls[0]->isChecked();
      static const int SPACING = 3;
      for(int y = 0; y < 256; y += SPACING)
         for(int u = 0; u < 256; u += SPACING)
            for(int v = 0; v < 256; v += SPACING) {
               Colour colour = classifier->getClassifiedColour(y, u, v);
               if(colour != cUNCLASSIFIED &&
                  (!selectedOnly || colour == this->colour)) {
                  QColor classifiedColour = QColor(CPLANE_COLOURS[colour]);
                  int r, g, b;
                  classifiedColour.getRgb(&r, &g, &b);
                  pointCloud.points.push_back(make_pair(
                              qglviewer::Vec(r / 255.0, g / 255.0, b / 255.0),
                              qglviewer::Vec(y / 255.0, u / 255.0, v / 255.0)));
               }
            }
   }
}

bool CalibrationTab::eventFilter(QObject *object, QEvent *event) {
   if ((object == camLabel) && (event->type() == QEvent::MouseButtonPress)) {
      return classifyMouseEvent(static_cast<QMouseEvent*>(event));
   } else if ((object == camLabel) && (event->type() == QEvent::Wheel)) {
      return classifyWheelEvent(static_cast<QWheelEvent*>(event));
   } else {
      return false;
   }
}

bool::CalibrationTab::classifyWheelEvent(QWheelEvent *e) {
   int oldZoomLog = zoomLog;
   zoomLog += e->delta()/100;

   if (zoomLog < 0) zoomLog = 0;

   if (zoomLog > ZOOM_LIMIT) zoomLog = ZOOM_LIMIT;

   zoomLevel = pow(2, zoomLog);


   mouseX = e->x();
   mouseY = e->y();

   if (zoomLog != oldZoomLog)  redraw();
   return true;
}

bool CalibrationTab::classifyMouseEvent(QMouseEvent *e) {
   uint8_t y = 0, u = 0, v = 0;
   int imageX, imageY;

   if (e->button() == Qt::RightButton) {
      QString fileName = QFileDialog::getSaveFileName(this, "Save image");
      if (fileName != "") {
         QImage(this->lastRendering).save(fileName);
      }
   }

   if (e->button() != Qt::LeftButton) {
      // Classify on left click only
      return false;
   }
   if (!classifier->classificationFileOpened()) {
      QMessageBox::warning(this, "Error", "No Kernel file, can't classify.");
      return false;
   }
   if (!currentFrame) {
      QMessageBox::warning(this, "Error",
            "No frame, what are you classifying?");
      return false;
   }
   QPoint translatedMousePos = translateToZoomedImageCoords(
                                       QPoint(e->x(), e->y()));
   imageX = translatedMousePos.x();
   imageY = translatedMousePos.y();



   classifier->beginAction();
   y = gety(currentFrame, imageY, imageX);
   u = getu(currentFrame, imageY, imageX);
   v = getv(currentFrame, imageY, imageX);

   // update radii
   float weight = 1;
   int yRadius = 5, uRadius = 10, vRadius = 10;
   if (checkAutoWeight->isChecked()) {
      float weights[CMAX];
      classifier->colourInfo(y, u, v, weights);
      float totalWeight = 0.0;
      for (int i = 0; i < CMAX; i++) {
         totalWeight += weights[i];
      }
      yRadius = static_cast<int>(10.0 / (1.0 + totalWeight));
      uRadius = static_cast<int>(20.0 / (1.0 + totalWeight));
      vRadius = static_cast<int>(20.0 / (1.0 + totalWeight));
      inRange(yRadius, 1, 10);
      inRange(uRadius, 1, 20);
      inRange(vRadius, 1, 20);
      weight = 1.0 + (totalWeight / 10.0);
   }

   // classify!
   if (!classifier->isMostlyClassified(y, u, v, colour)) {
      classifier->classify(y, u, v, weight, colour, yRadius,
                           uRadius, vRadius, false);
   }
   classifier->endAction();
   redraw();
   return true;
}

QPoint CalibrationTab::translateToZoomedImageCoords(QPoint point) {
   QTransform prevTransform;
   prevTransform = prevTransform.translate(+prevMouseX, +prevMouseY);
   prevTransform = prevTransform.scale(1.0/prevZoomLevel, 1.0/prevZoomLevel);
   prevTransform = prevTransform.translate(-prevMouseX, -prevMouseY);
   QRectF newMousePos = prevTransform.mapRect(QRectF(point.x(),
                                             point.y(), 0, 0));
   return QPoint(newMousePos.left(), newMousePos.top());
}

// TODO(brockw): see if this can be genericized into tab.cpp, so it's not in
// every tab
void CalibrationTab::newNaoData(NaoData *naoData) {
   if (!naoData || !naoData->getCurrentFrame().blackboard) {
      imagePixmap.fill(Qt::darkGray);
      camLabel->setPixmap(imagePixmap);
      currentFrame = NULL;
   } else {
      this->naoData = naoData;
      Blackboard *blackboard = naoData->getCurrentFrame().blackboard;
      if ((currentFrame = readFrom(vision, currentFrame)) != NULL)
         if (parent->currentIndex() == parent->indexOf(this))
            redraw();
   }
}

void CalibrationTab::redrawSlot() {
   redraw();
}

void CalibrationTab::ftpCommandFinished(int, bool error) {
#ifndef QT_NO_CURSOR
   setCursor(Qt::ArrowCursor);
#endif

   bool closeFtp = false;

   if (ftp->currentCommand() == QFtp::ConnectToHost) {
      if (error) {
         QMessageBox::information(this, tr("FTP"),
               tr("Unable to connect to the FTP server "
                  "at %1. Please check that the host "
                  "name is correct.").arg("soldier"));
         closeFtp = true;
      } else {
         emit showMessage(tr("Logged onto %1.").arg("soldier"));
      }
   } else if (ftp->currentCommand() == QFtp::Put) {
      if (error) {
         emit showMessage(tr("Canceled upload of %1.").
               arg(nnmcFile->fileName()));
         nnmcFile->close();
         nnmcFile->remove();
      } else {
         emit showMessage(tr("Uploaded %1 to /etc/runswift.").
               arg(nnmcFile->fileName()));
         nnmcFile->close();
      }
      delete nnmcFile;
      progressDialog->hide();
      closeFtp = true;
   }
   if (closeFtp) {
      ftp->abort();
      ftp->deleteLater();
      ftp = 0;
#ifndef QT_NO_CURSOR
      setCursor(Qt::ArrowCursor);
#endif
   }
}

void CalibrationTab::updateDataTransferProgress(qint64 readBytes,
      qint64 totalBytes) {
   progressDialog->setMaximum(totalBytes);
   progressDialog->setValue(readBytes);
}

void CalibrationTab::cancelUpload() {
   ftp->abort();
}
