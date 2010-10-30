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

#pragma once

#include <QtGui/QMainWindow>
#include <QTreeWidget>
#include <QTabWidget>
#include <QLabel>
#include <QFileDialog>

#include <vector>

#include "naoData.hpp"
#include "mediaPanel.hpp"

#include "readers/reader.hpp"
#include "readers/diskReader.hpp"
#include "readers/networkReader.hpp"
#include "readers/dumpReader.hpp"
#include "readers/localisationReader.hpp"

#include "tabs/calibrationTab.hpp"
#include "tabs/cameraTab.hpp"
#include "tabs/classifier.hpp"
#include "tabs/overviewTab.hpp"
#include "tabs/visionTab.hpp"
#include "tabs/sensorTab.hpp"
#include "tabs/cameraPoseTab.hpp"
#include "tabs/graphTab.hpp"
#include "tabs/jointsTab.hpp"
#include "tabs/zmpTab.hpp"

#include "perception/vision/yuv.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/Vision.hpp"
#include "perception/localisation/LocalisationDefs.hpp"
#include "perception/localisation/PFilter.hpp"

#include "utils/log.hpp"

class Tab;

namespace Ui {
   class Visualiser;
}

/*
 * This is the main window. This holds data at the root level, such as
 * tabs, status bar and the file menubar.
 */
class Visualiser : public QMainWindow {
   Q_OBJECT

   public:
      explicit Visualiser(QWidget *parent = 0);
      ~Visualiser();

      public slots:
         /* This slot is received when the reader aquires more data   */
         void newNaoData(NaoData *naoData);

      /* Temp function for demo purposes.  */
      void startRecording(QAction* action);

      /* This loads a YUV dump. Typically this is called from the file menu.*/
      void loadDump();

      /* This loads a series of default files from a config file */
      void loadDefaultFiles();

      /* This loads the localization debugging tool */
      void loadLocalisationReader();

      /* Called when a tab is changed. Currently is used to refresh
       * the current tab when you first switch to it.
       */
      void currentTabChanged(int tab);

      void connectToNao();

      void disconnectFromNao();

      /**
       * sends a command line option to the nao
       */
      void commandLineString();


      void close();

   signals:
      /* Used to tell a widget to refresh after a certain event occurs.
       * Currently is used to redraw a tab when we switch to it
       */
      void refreshNaoData();

      /**
       * sends a command line option to the nao
       */
      void sendCommandLineString(const QString &);

      void readerClosed();

   private:
      Ui::Visualiser *ui;

      std::vector<Tab*> tabVector;

      /* The reader. This could be any of the readers subclasses. This means that in the end the gui doesn't really
         care weather the data it is receiving is coming from disk or over wireless */
      Reader *reader;

      /* A pointer to the naoData, currently this is unused */
      NaoData *naoData;

      QTabWidget *tabs;

      /* Tabs */
      CalibrationTab *calibrationTab;
      OverviewTab *overviewTab;
      VisionTab *visionTab;
      CameraTab *cameraTab;
      SensorTab *sensorTab;
      CameraPoseTab *cameraPoseTab;
      GraphTab *graphTab;
      JointsTab *jointsTab;
      ZMPTab *zmpTab;

      /* initializes the general menu's. i.e. menus that are
       * not specific to a tab.
       */
      void initMenu();


      /* Variables for the file menu */
      QMenu *fileMenu;
      QAction *loadAct;
      QAction *saveAsAct;
      QAction *exitAct;

      /* This holds the global media panel consisting of
       * record/play/stop/pause buttons
       */
      MediaPanel *mediaPanel;

      Vision *vision;
      Classifier *classifier;
      PFilter *localisation;

      void setUpReaderSignals(Reader *reader);
};
