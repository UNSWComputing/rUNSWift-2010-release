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

#include "perception/vision/VisionDefs.hpp"
#include "utils/RRCoord.hpp"
#include "utils/AbsCoord.hpp"

#define GREATEST_DIST_FROM_LINE 1150
#define MIN_NUM_FIELD_LINE_POINTS 10

class FieldLineLocalisation {
   public:

      int numFieldLinePoints;
      RRCoord fieldLinePoints[MAX_FIELD_LINE_POINTS];

      /**
       * Returns true if successful
       **/
      bool setupMap(const char *mapFileName);

      AbsCoord localiseFromFieldLines(const AbsCoord &robotPos, bool &newData);

      /**
       * Finds the problability of being in the position given by x, y, and
       * orien according to field lines
       **/
      inline float probabilityOfPosition(int x, int y, float orien);

   private:

      int mapWidth;
      int mapHeight;
      int resolution;
      float *fieldMap;

      // Telescopic search params
      int maxLevel;
      int rangeX[3];
      int rangeY[3];
      float rangeAngle[3];
      int incX[3];
      int incY[3];
      float incAngle[3];

      float cos_a[720];
      float sin_a[720];


      int counterTest;

      void calculateProbabilities(int *mapElements);

      AbsCoord performTelescopicSearch(int level, int maxLevel,
            AbsCoord centre);
};
