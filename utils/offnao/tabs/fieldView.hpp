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

#include <QLabel>
#include <QPixmap>
#include <QPoint>
#include <QPainter>
#include "perception/vision/yuv.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/Vision.hpp"
#include "utils/log.hpp"
#include "naoData.hpp"
#include "frame.hpp"
#include "utils/Line.hpp"
#include "utils/BroadcastData.hpp"

class FieldView: public QLabel {
   public:
      FieldView();
      ~FieldView();

      void redraw(NaoData *naoData);

   private:
      QImage image;
      QPoint robotPos;
      QPixmap imagePixmap;
      double robotHeading;
      int ourTeam;
      static const float FIELD_TO_IMAGE_SCALE = 10;
      void teamConversion(QPoint &p, double &heading, int ourTeam);
      QPixmap *renderPixmap;
      QPoint fieldToImageCoords(QPoint p);
      void drawPosts(NaoData *naoData, QPainter *p);
      void drawPost(QPoint pos, QPainter *p);
      void drawEdges(NaoData *naoData, QPainter *p);
      void drawEdge(NaoData *naoData, QPainter *p, Line line);
      void drawFieldLines(NaoData *naoData, QPainter *p);
      void drawOtherRobots(NaoData *naoData, QPainter *p);
      QPoint robotRelativeToImage(int distance, float heading);
      QPoint robotRelativeToAbsolute(QPoint p, QPoint position, float heading);
      void drawLocalizationDebugOverlay(Frame frame, QPainter *painter);
      void drawBroadcastData(const BroadcastData &bcData, QPainter *p, int ourTeam, int n);
};

