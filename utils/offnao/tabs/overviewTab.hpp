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

#include <QTabWidget>

#include <QMenuBar>
#include <QWidget>
#include <QObject>
#include <QEvent>
#include <QGridLayout>
#include <QPixmap>
#include <QLabel>
#include <QPainter>
#include <QColor>
#include <QTreeWidgetItem>
#include <QTreeWidget>

#include <cstdio>
#include <deque>

#include "tabs/tab.hpp"
#include "tabs/variableView.hpp"
#include "classifier.hpp"
#include "perception/vision/yuv.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/Vision.hpp"
#include "utils/log.hpp"
#include "fieldView.hpp"
#include "mediaPanel.hpp"

class Blackboard;

/*
 * This is the default/initial tab that shows on entering Off-Nao.
 * As the name suggests it gives an overview of most of the important
 * aspects of the nao. Good for general gameplay watching etc.
 *
 * Also will be used for localization debugging unless the localization
 * people need more info.
 */
class OverviewTab : public Tab {
   Q_OBJECT
   public:
      OverviewTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision,
                  Classifier *classifier);
      QPixmap *renderPixmap;
      QLabel *renderWidget;
   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      void redraw();

      /*  Draw the image on top of a pixmap */
      void drawImage(QImage *image);
      void drawOverlays(QPixmap *pixmap);


      QGridLayout *layout;

      /* These variables are used to present the debug variables from the nao */
      VariableView variableView;
      FieldView fieldView;



      /* Variables for vision */
      QPixmap imagePixmap;
      QLabel *camLabel;

      // Data
      Colour saliency[160][120];
      Blackboard *blackboard;
   public slots:
      void newNaoData(NaoData *naoData);
};

