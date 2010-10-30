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

#include <QObject>
#include <QWidget>
#include <QMenuBar>
#include <QString>
#include <QRgb>
#include <QPaintDevice>
#include <QTabWidget>

#include <vector>
#include <utility>

#include "perception/vision/yuv.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/Vision.hpp"
#include "classifier.hpp"
#include "utils/log.hpp"

#include "naoData.hpp"

class Tab : public QWidget {
   Q_OBJECT

   public:
      Tab() : vision(0), classifier(0), currentFrame(0) {}

   protected:
      /* Returns the RGB value of the pixel at row, col */
      virtual QRgb getRGB(unsigned int col, unsigned int row,
                          const uint8_t *yuv);

      QTabWidget *parent;

      /*  vision module from libsoccer */
      Vision *vision;
      Classifier *classifier;

      /* Current working image
       * If you working with vision you need a frame.
       */
      const uint8_t *currentFrame;

      /*
       * Generic draw overlays function
       * Supply Null to any of the arguments if you do not wish to draw
       * a particular overlay.
       */
      void drawOverlaysGeneric(QPaintDevice *image,
            std::vector<std::pair<uint16_t, uint16_t> > *edgePoints,
            Line edgeLines[2],
            uint8_t numEdgeLines,
            uint16_t *goals,
            uint16_t numPosts,
            WhichPosts posts,
            ImageRegion *regions,
            uint16_t numRegions,
            uint16_t radius,
            std::pair<uint16_t, uint16_t> ballCentre,
            std::pair<uint16_t, uint16_t> *ballEdgePoints,
            uint16_t numBallEdgePoints,
            RobotRegion **robotRegions,
            uint16_t numRobotRegions,
            ImageRegion **lineRegions,
            uint16_t numLineRegions,
            float scale);

      void drawAllOverlaysGeneric(QPaintDevice *image, Vision *vision, float scale);
   signals:
      virtual void showMessage(const QString &, int timeout = 0);

   public slots:
      virtual void newNaoData(NaoData *naoData) = 0;
      virtual void readerClosed() {}
};
