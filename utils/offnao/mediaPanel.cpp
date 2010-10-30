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

#include <QtGui>
#include <QDebug>
#include <QSvgRenderer>
#include <QObject>
#include "mediaPanel.hpp"

   MediaPanel::MediaPanel(QWidget *parent)
: QWidget(parent), isPlay(true), isRecord(true) {
   layout = new QGridLayout();
   this->setLayout(layout);
   // layout->addWidget(new QLabel(tr("hi")),0,0,1,1);
   this->setMinimumSize(600, 50);

   recordButton = new QToolButton();
   recordAct = new QAction(recordButton);
   recordAct->setIcon(QIcon(tr(":/icons/media_icons/media-record.svg")));
   recordButton->setMinimumSize(BUTTON_WIDTH, BUTTON_HEIGHT);
   recordButton->setMaximumSize(BUTTON_WIDTH, BUTTON_HEIGHT);
   layout->addWidget(recordButton, 0, 0, 1, 1);
   recordButton->setDefaultAction(recordAct);

   stopButton = new QToolButton();
   stopAct = new QAction(stopButton);
   stopAct->setIcon(QIcon(tr(":/icons/media_icons/media-playback-stop.svg")));
   stopButton->setMinimumSize(BUTTON_WIDTH, BUTTON_HEIGHT);
   stopButton->setMaximumSize(BUTTON_WIDTH, BUTTON_HEIGHT);
   // layout->addWidget(stopButton, 0, 1, 1, 1);
   stopButton->setDefaultAction(stopAct);

   pauseButton = new QToolButton();
   pauseAct = new QAction(pauseButton);
   pauseAct->setIcon(QIcon(tr(":/icons/media_icons/media-playback-pause.svg")));
   pauseButton->setMinimumSize(BUTTON_WIDTH, BUTTON_HEIGHT);
   pauseButton->setMaximumSize(BUTTON_WIDTH, BUTTON_HEIGHT);
   // layout->addWidget(pauseButton, 0, 2, 1, 1);
   pauseButton->setDefaultAction(pauseAct);


   playButton = new QToolButton();
   playAct = new QAction(playButton);
   playAct->setIcon(QIcon(tr(":/icons/media_icons/media-playback-start.svg")));
   playButton->setMinimumSize(BUTTON_WIDTH, BUTTON_HEIGHT);
   playButton->setMaximumSize(BUTTON_WIDTH, BUTTON_HEIGHT);
   layout->addWidget(playButton, 0, 1, 1, 1);
   playButton->setDefaultAction(playAct);



   backButton = new QToolButton();
   backAct = new QAction(backButton);
   backAct->setIcon(QIcon(tr(":/icons/media_icons/media-skip-backward.svg")));
   backButton->setMinimumSize(BUTTON_WIDTH, BUTTON_HEIGHT);
   backButton->setMaximumSize(BUTTON_WIDTH, BUTTON_HEIGHT);
   layout->addWidget(backButton, 0, 2, 1, 1);
   backButton->setDefaultAction(backAct);


   forwardButton = new QToolButton();
   forwardAct = new QAction(forwardButton);
   forwardAct->setIcon(QIcon(tr(":/icons/media_icons/media-skip-forward.svg")));
   forwardButton->setMinimumSize(BUTTON_WIDTH, BUTTON_HEIGHT);
   forwardButton->setMaximumSize(BUTTON_WIDTH, BUTTON_HEIGHT);
   layout->addWidget(forwardButton, 0, 3, 1, 1);
   forwardButton->setDefaultAction(forwardAct);



   frameSlider = new QSlider(this);
   frameSlider->setOrientation(Qt::Horizontal);

   layout->addWidget(frameSlider, 0, 4, 1, 1);
   connect(playButton, SIGNAL(triggered(QAction*)), this,
           SLOT(togglePausePlay(QAction*)));
   connect(recordButton, SIGNAL(triggered(QAction*)), this,
           SLOT(toggleRecordStop(QAction*)));
}
void MediaPanel::setIsPlay(bool b) {
   if (b != isPlay) togglePausePlay(NULL);
   isPlay = b;
}

void MediaPanel::setIsRecord(bool b) {
   if (b != isRecord) toggleRecordStop(NULL);
   isRecord = b;
}
void MediaPanel::togglePausePlay(QAction* action) {
   isPlay = !isPlay;
   if (isPlay) {
      playButton->removeAction(pauseAct);
      playButton->setDefaultAction(playAct);
   } else {
      playButton->removeAction(playAct);
      playButton->setDefaultAction(pauseAct);
   }
}

void MediaPanel::toggleRecordStop(QAction* action) {
   isRecord = !isRecord;
   if (isRecord) {
      recordButton->removeAction(stopAct);
      recordButton->setDefaultAction(recordAct);
      setIsPlay(true);
   } else {
      recordButton->removeAction(recordAct);
      recordButton->setDefaultAction(stopAct);
      setIsPlay(false);
   }
}


void MediaPanel::newNaoData(NaoData *naoData) {
   if (naoData) {
      frameSlider->setTracking(false);
      frameSlider->setSliderPosition(
                  round(100*(1.0*naoData->getCurrentFrameIndex())/
                                 naoData->getFramesTotal()));
      frameSlider->setTracking(true);
   }
}
