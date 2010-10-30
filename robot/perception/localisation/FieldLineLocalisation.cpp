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

#include "perception/localisation/FieldLineLocalisation.hpp"
#include <fstream>
#include "utils/log.hpp"
#include "utils/Timer.hpp"

using namespace std;

bool FieldLineLocalisation::setupMap(const char *mapFileName) {
   ifstream data;
   data.open(mapFileName, ios::in | ios::binary);
   if (data.is_open()) {
      // First, read the header
      int arraySize;
      data.read((char *)(&arraySize), sizeof(arraySize));
      data.read((char *)(&mapWidth), sizeof(mapWidth));
      data.read((char *)(&mapHeight), sizeof(mapHeight));
      data.read((char *)(&resolution), sizeof(resolution));

      // Now read the rest of the file in
      int headerLength = sizeof(arraySize) + sizeof(mapWidth) +
         sizeof(mapHeight) + sizeof(resolution);
      data.seekg(headerLength, ios::beg);
      int *mapElements = new int[arraySize];
      for (int i = 0; i < arraySize; i++) {
         data.read((char *)(&mapElements[i]), sizeof(mapElements[i]));
      }
      data.close();
      calculateProbabilities(mapElements);
      delete[] mapElements;
      return true;
   } else {
      llog(FATAL) << "ERROR: could not open the map file" << endl;
      throw runtime_error("Error opening the map file");
      fieldMap = NULL;
      return false;
   }
}

void FieldLineLocalisation::calculateProbabilities(int *mapElements) {
   fieldMap = new float[mapWidth*mapHeight];

   for (int y = 0; y < mapHeight; y++) {
      for (int x = 0; x < mapWidth; x++) {
         int coord = y * mapWidth + x;
         if (mapElements[coord] == -1) {
            mapElements[coord] = 0;
         }
         fieldMap[coord] = mapElements[coord] * mapElements[coord] * resolution
            * resolution;
      }
   }

   for (int i = 0; i < 720; i++) {
      cos_a[i] = cos(DEG2RAD(i - 360));
      sin_a[i] = sin(DEG2RAD(i - 360));
   }
}

AbsCoord FieldLineLocalisation::localiseFromFieldLines(
      const AbsCoord &robotPos, bool &newData) {
   if (numFieldLinePoints < MIN_NUM_FIELD_LINE_POINTS) {
      llog(VERBOSE) << "Not enough field line points for localisation" << endl;
      newData = false;
      return robotPos;
   }
   // Get the range of possible positions from the robotPos struct
   float xVar = 200;
   float yVar = 200;
   float thetaVar = DEG2RAD(5.0);
   // Set up the range and increment arrays for the telescopic search
   maxLevel = 2;
   rangeX[0] = (int)xVar;
   rangeY[0] = (int)yVar;
   rangeAngle[0] = (float)thetaVar;
   incX[0] = rangeX[0];
   incY[0] = rangeY[0];
   incAngle[0] = rangeAngle[0];
   if (incX[0] == 0) {
      incX[0] = 1;
   }
   if (incY[0] == 0) {
      incY[0] = 1;
   }
   rangeX[1] = incX[0]/2;
   rangeY[1] = incY[0]/2;
   rangeAngle[1] = incAngle[0]/2.0f;
   incX[1] = rangeX[1]/2;
   incY[1] = rangeY[1]/2;
   incAngle[1] = rangeAngle[1];
   if (incX[1] == 0) {
      incX[1] = 1;
   }
   if (incY[1] == 0) {
      incY[1] = 1;
   }
   if (fieldMap != NULL) {
      counterTest = 0;
      Timer timer;
      timer.restart();
      AbsCoord retVal =  performTelescopicSearch(0, maxLevel, robotPos);
      retVal.var[1] = retVal.var[0];
      float angleVar = sqrt(retVal.var[0])/3000;
      if (angleVar > 1) {
         angleVar = 1;
      }
      retVal.var[2] = SQUARE(angleVar*M_PI);
      newData = true;
      return retVal;
   } else {
      newData = false;
      return robotPos;
   }
}

inline float FieldLineLocalisation::probabilityOfPosition(int x, int y,
      float orien) {
   float prob = 0;
   int count = 0;

   counterTest++;
   for (int numPoints = 0; numPoints < numFieldLinePoints; numPoints++) {
      int xcoord = fieldLinePoints[numPoints].distance * cos_a[(int)RAD2DEG(
            orien + fieldLinePoints[numPoints].heading) + 360] + x;
      int ycoord = fieldLinePoints[numPoints].distance * sin_a[(int)RAD2DEG(
            orien + fieldLinePoints[numPoints].heading) + 360] + y;

      // The map has cm resolution, so to find the index in the map
      // we need to divide by 10
      xcoord /= resolution;
      ycoord /= resolution;
      xcoord += mapWidth/2;
      ycoord += mapHeight/2;

      if (xcoord >= 0 && xcoord < mapWidth && ycoord >= 0 &&
            ycoord < mapHeight) {
         prob += fieldMap[ycoord * mapWidth + xcoord];
      } else {
         prob += GREATEST_DIST_FROM_LINE * GREATEST_DIST_FROM_LINE;
      }
      count++;
   }
   return prob/((float)count);
}

AbsCoord FieldLineLocalisation::performTelescopicSearch(int level,
      int maxLevel, AbsCoord centre) {
   if (level == maxLevel) {
      return centre;
   }

   AbsCoord maxLoc;
   maxLoc.x = 0;
   maxLoc.y = 0;
   maxLoc.theta = 0;
   // float maxProb = 0;
   float maxProb = 6400 * 6400;
   for (int y = -rangeY[level]; y <= rangeY[level]; y += incY[level]) {
      for (int x = -rangeX[level]; x <= rangeX[level]; x += incX[level]) {
         for (float i = -rangeAngle[level]; i <= rangeAngle[level];
               i += incAngle[level]) {
            // Ensure that the angle is in the allowable range
            float heading = i + centre.theta;
            int multiplier = heading / (M_PI * 2);
            heading = heading - ((M_PI * 2) * multiplier);
            if (heading >= M_PI) {
               heading = M_PI * 2 + heading;
            } else if (heading < -M_PI) {
               heading = M_PI * 2 + heading;
            }
            float prob = probabilityOfPosition(x + centre.x, y + centre.y,
                  heading);
            // Introduce a bit of intertia into the probabilities
            // according to how much the angle is different from the
            // robot's current heading.
            float numAngleInc = ABS(i) / incAngle[level];
            prob *= ((numAngleInc) + 1);
            if (prob < maxProb || (prob <= maxProb && y == 0 && x == 0 &&
                     i == 0)) {
               maxProb = prob;
               maxLoc.x = x + centre.x;
               maxLoc.y = y + centre.y;
               maxLoc.theta = heading;
               maxLoc.var[0] = prob;
            }
         }
      }
   }
   return performTelescopicSearch(level + 1, maxLevel, maxLoc);
}
