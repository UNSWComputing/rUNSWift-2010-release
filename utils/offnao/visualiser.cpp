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

#include <boost/shared_ptr.hpp>
#include <QDebug>
#include <QInputDialog>
#include <string>
#include <iostream>
#include <fstream>
#include "visualiser.hpp"
#include "ui_visualiser.h"
#include "utils/log.hpp"
#include <stdlib.h>
#include <time.h>
#include "progopts.hpp"

using namespace boost;
using std::string;

Visualiser::Visualiser(QWidget *parent)
   : QMainWindow(parent), ui(new Ui::Visualiser),
   reader(0), naoData(0) {
      // call initLogs so that vision et al don't segfault when they llog
      initLogs(config["debug.log"].as<string>(),
               config["debug.logpath"].as<string>(),
               true);

      srand(time(0));
      extern bool offNao;
      offNao = true;
      vision = new Vision();
      classifier = new Classifier();

      ui->setupUi(this);
      ui->centralWidget->setMinimumSize(1000, 600);

      initMenu();

      // add the media panel to the bottom of the window
      mediaPanel = new MediaPanel(ui->centralWidget);


      // set up tab holder
      tabs = new QTabWidget(ui->centralWidget);
      tabs->setMinimumSize(600, 450);

      overviewTab =  new OverviewTab(tabs, ui->menuBar, vision, classifier);
      calibrationTab =  new CalibrationTab(tabs, ui->menuBar,
                            vision, classifier);
      visionTab = new VisionTab(tabs, ui->menuBar, vision);
      cameraTab = new CameraTab(tabs, ui->menuBar, vision);
      sensorTab = new SensorTab(tabs, ui->menuBar, vision);
      cameraPoseTab = new CameraPoseTab(tabs, ui->menuBar, vision);
      graphTab = new GraphTab(tabs, ui->menuBar, vision);
      jointsTab = new JointsTab(tabs, ui->menuBar, vision);
      zmpTab = new ZMPTab(tabs, ui->menuBar, vision);
      tabVector.push_back(overviewTab);
      tabVector.push_back(calibrationTab);
      tabVector.push_back(visionTab);
      tabVector.push_back(cameraTab);
      tabVector.push_back(sensorTab);
      tabVector.push_back(cameraPoseTab);
      // tabVector.push_back(graphTab);  //  hidding the graph tab as its not working
      // tabVector.push_back(jointsTab);
      // tabVector.push_back(zmpTab);


      /* Set up the tabs */
      tabs->addTab(overviewTab, "Overview");
      connect(overviewTab,
              SIGNAL(showMessage(QString, int)), ui->statusBar,
              SLOT(showMessage(QString, int)));
      tabs->addTab(calibrationTab, "Calibration");
      connect(calibrationTab,
              SIGNAL(showMessage(QString, int)), ui->statusBar,
              SLOT(showMessage(QString, int)));

      tabs->addTab(visionTab, "Vision");
      connect(visionTab, SIGNAL(showMessage(QString, int)), ui->statusBar,
              SLOT(showMessage(QString, int)));

      tabs->addTab(cameraTab, "Camera");
      connect(cameraTab, SIGNAL(showMessage(QString, int)), ui->statusBar,
              SLOT(showMessage(QString, int)));

      tabs->addTab(sensorTab, "Sensor");
      connect(sensorTab, SIGNAL(showMessage(QString, int)), ui->statusBar,
              SLOT(showMessage(QString, int)));

      tabs->addTab(cameraPoseTab, "Camera Pose");
      connect(cameraPoseTab, SIGNAL(showMessage(QString, int)), ui->statusBar,
              SLOT(showMessage(QString, int)));

      // tabs->addTab(graphTab, "Graph");
      // connect(graphTab, SIGNAL(showMessage(QString, int)), ui->statusBar,
      //         SLOT(showMessage(QString, int)));

      // tabs->addTab(zmpTab, "ZMP");
      // connect(zmpTab, SIGNAL(showMessage(QString, int)), ui->statusBar,
      //         SLOT(showMessage(QString, int)));

      // tabs->addTab(jointsTab, "Joints");
      // connect(jointsTab, SIGNAL(showMessage(QString, int)), ui->statusBar,
      //        SLOT(showMessage(QString, int)));

      /* Used to redraw tabs when they are in focus */
      connect(tabs, SIGNAL(currentChanged(int)), this,
              SLOT(currentTabChanged(int)));

      ui->rootLayout->addWidget(mediaPanel, 1, 0, 1, 1);
      ui->rootLayout->addWidget(tabs, 0, 0, 1, 1);
   }

Visualiser::~Visualiser() {
   delete ui;
   for (int i = 0; i > tabVector.size(); i++) {
      delete tabVector[i];
   }
}

void Visualiser::newNaoData(NaoData *naoData) {
   this->naoData = naoData;
}

// sets up the generic menu's
void Visualiser::initMenu() {
   fileMenu = new QMenu("&File");
   ui->menuBar->addMenu(fileMenu);

   // set up file menu
   loadAct = new QAction(tr("&Load"), fileMenu);
   saveAsAct = new QAction(tr("&Save As"), fileMenu);

   exitAct = new QAction(tr("&Quit"), fileMenu);


   fileMenu->addAction(tr("Connect to &Nao"), this, SLOT(connectToNao()),
         QKeySequence(tr("Ctrl+N", "File|New")));
   fileMenu->addAction(tr("Load Camera &Dump"), this, SLOT(loadDump()),
         QKeySequence(tr("Ctrl+O", "File|Open")));
   fileMenu->addAction(tr("Load &Localisation"), this,
                       SLOT(loadLocalisationReader()),
         QKeySequence(tr("Ctrl+L", "File|Open")));
   fileMenu->addAction(tr("Load &Default Files"), this,
                        SLOT(loadDefaultFiles()),
         QKeySequence(tr("Ctrl+D", "File|Open")));
   // This is networkReader specific, but I'm not sure where to put it
   fileMenu->addAction(tr("Send Command Line &String"), this,
                       SLOT(commandLineString()),
                            QKeySequence(tr("F5", "Refresh")));
   fileMenu->addAction(tr("&Close Current Reader"), this,
         SLOT(disconnectFromNao()),
         QKeySequence(tr("Ctrl+W", "File|Close")));

   fileMenu->addAction(tr("&Quit"), this,
         SLOT(close()),
         QKeySequence(tr("Ctrl+Q", "Close")));

}


void Visualiser::close() {
    exit(0);
}

/* Starts the dump reader  */
void Visualiser::loadDump() {
   if (reader) {
      QMessageBox::warning(this, "Close current session!",
      "Please close current session from the file menu.");
      return;
   }

   QString fileName = QFileDialog::getOpenFileName(this, "Open Dump File");
   if (!fileName.isEmpty()) {
      reader = new DumpReader(fileName);
      setUpReaderSignals(reader);
      reader->start();
   }
}

void Visualiser::loadDefaultFiles() {
   if (reader) {
      QMessageBox::warning(this, "Close current session!",
      "Please close current session from the file menu.");
      return;
   }
   std::string line;
   std::ifstream f("../../../../def.txt");
   if (f.is_open()) {
      std::getline(f, line);
      std::ifstream fout(line.c_str());
      if (fout) {
         fout.close();
         QString name(line.c_str());
         reader = new DumpReader(name);
         setUpReaderSignals(reader);
         reader->start();
      } else {
         std::cout << "Could not find dump file" << std::endl;
      }

      std::getline(f,line);
      std::ifstream f2(line.c_str());
      if (f2) {
         f2.close();
         calibrationTab->loadKernelFile(line);
      } else {
         std::cout << "Could not find the kernel file" << std::endl;
      }

      std::getline(f,line);
      std::ifstream f3(line.c_str());
      if (f3) {
         f3.close();
         visionTab->loadNnmcFile(line.c_str());
      } else {
         std::cout << "Could not find the nnmc file" << std::endl;
      }

   }
   f.close();
}

/*
 * Starts the localizationReader. This is used for testing/dubuging localization
 */
void Visualiser::loadLocalisationReader() {
   if (reader) {
      QMessageBox::warning(this, "Close current session!",
      "Please close current session from the file menu.");
      return;
   }

   reader = new LocalisationReader();
   setUpReaderSignals(reader);
   reader->start();
}

/*
 * Not yet used. Probably will be for the network reader.
 */
void Visualiser::startRecording(QAction *action) {
}

void Visualiser::currentTabChanged(int tabNumber) {
   emit refreshNaoData();
}

void Visualiser::connectToNao() {
   if (reader != 0) {
      QMessageBox::warning(this, "Close current session!",
         "Please close current session from the file menu.");
      return;
   }

   /*QStringList items;
   items << tr("scout.local") << tr("heavy.local") << tr("demo-man.local") <<
      tr("sniper.local") << tr("spy.local") << tr("soldier.local") <<
      tr("medic.local") << tr("engineer.local") << tr("pyro.local") <<
      tr("localhost");
   items.sort();*/

   bool ok;
   //QString item = QInputDialog::getItem(this, tr(""),
   //      tr("Connect to:"), items, 0, false, &ok);
   QString item = QInputDialog::getText(this, tr(""),
         tr("Connect to:"),QLineEdit::Normal, "", &ok);
         
   if ( ok && !item.isEmpty() ) {
      // display box asking what user wants to receive
      QStringList items;
      items << tr("No Image") << tr("Saliency") << tr("Raw Image") << tr("Particles");

      bool ok;
      QString item2 = QInputDialog::getItem(this, tr(""),
            tr("Stream: "), items, 0, false, &ok);

      OffNaoMask_t mask = ALL_MASKS;
      if (ok && !item2.isEmpty()) {
         if (item2 == QString("No Image")) {
            mask = (BLACKBOARD_MASK);
         } else if (item2 == QString("Saliency")) {
            mask = (BLACKBOARD_MASK | SALIENCY_MASK);
         } else if (item2 == QString("Particles")) {
            mask = (BLACKBOARD_MASK | PARTICLE_FILTER_MASK);
         } else {
            mask = (ALL_MASKS);
         }

         NetworkReader *reader = new NetworkReader(item, mask);
         connect(this, SIGNAL(sendCommandLineString(const QString&)), reader,
               SLOT(sendCommandLineString(QString)));

         connect(cameraPoseTab, SIGNAL(sendCommandToRobot(QString)),
                  reader, SLOT(sendCommandLineString(QString)));
         connect(cameraTab, SIGNAL(sendCommandToRobot(QString)),
                 reader, SLOT(sendCommandLineString(QString)));
         this->reader = reader;
         setUpReaderSignals(this->reader);
         this->reader->start();
      }
   }
}

// This is networkReader specific, but I'm not sure where to put it
void Visualiser::commandLineString() {
   QString item = QInputDialog::getText(NULL, "Send String",
                                           tr("Command Line String:"));
   emit sendCommandLineString(item);
}

void Visualiser::disconnectFromNao() {
   if (reader && reader->isFinished() == false) {
      connect(reader, SIGNAL(finished()), this, SLOT(disconnectFromNao()));
      reader->finishUp();
      qDebug("Try to destroy reader. Wait for thread to exit.");
      readerClosed();
   } else if (reader) {
      delete reader;
      reader = 0;
      qDebug("Finished destroying reader");
      ui->statusBar->showMessage(QString("Reader destroyed."));
   }
}

void Visualiser::setUpReaderSignals(Reader *reader) {

   for (int i = 0; i < tabVector.size(); i++) {
      connect(reader, SIGNAL(newNaoData(NaoData*)), tabVector[i],
            SLOT(newNaoData(NaoData *)));
      connect(this, SIGNAL(readerClosed()), tabVector[i],
            SLOT(readerClosed()));
   }

   connect(reader, SIGNAL(newNaoData(NaoData*)), mediaPanel,
         SLOT(newNaoData(NaoData *)));

   connect(mediaPanel->forwardAct, SIGNAL(triggered()), reader,
         SLOT(forwardMediaTrigger()));
   connect(mediaPanel->backAct, SIGNAL(triggered()), reader,
         SLOT(backwardMediaTrigger()));
   connect(mediaPanel->playAct, SIGNAL(triggered()), reader,
         SLOT(playMediaTrigger()));
   connect(mediaPanel->pauseAct, SIGNAL(triggered()), reader,
         SLOT(pauseMediaTrigger()));
   connect(mediaPanel->stopAct, SIGNAL(triggered()), reader,
         SLOT(stopMediaTrigger()));
   connect(mediaPanel->recordAct, SIGNAL(triggered()), reader,
         SLOT(recordMediaTrigger()));
   connect(mediaPanel->frameSlider, SIGNAL(valueChanged(int)),
         reader, SLOT(sliderMoved(int)));

   connect(this, SIGNAL(refreshNaoData()), reader, SLOT(refreshNaoData()));
   connect(mediaPanel->frameSlider, SIGNAL(sliderReleased()), reader,
         SLOT(refreshNaoData()));
   connect(reader, SIGNAL(showMessage(QString, int)), ui->statusBar,
         SLOT(showMessage(QString, int)));
}
