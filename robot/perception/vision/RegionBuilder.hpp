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

#include <vector>
#include "perception/vision/ImageRegion.hpp"
#include "perception/vision/RobotRegion.hpp"
#include "perception/vision/VisionConstants.hpp"

class RegionBuilder {
   public:
      /**
       * A store of all the regions in the image
       * MAYBE REMOVED as once the regions have been properly analysed,
       * there might not be much point in having it
       **/
      ImageRegion regions[MAX_NUM_REGIONS];
      /**
       * The number of regions found in the image
       **/
      uint16_t numRegions;

      ImageRegion *ballRegions[MAX_NUM_REGIONS];
      uint16_t numPossibleBallRegions;

      ImageRegion *lineRegions[MAX_NUM_REGIONS];
      uint16_t numLineRegions;

      RobotRegion robotRegions[MAX_NUM_REGIONS];
      uint16_t numRobotRegions;

      explicit RegionBuilder();

      ~RegionBuilder();

      /**
       * Scan through the saliency image and build regions
       **/
      void buildRegions(Colour saliency[IMAGE_COLS/SALIENCY_DENSITY]
            [IMAGE_ROWS/SALIENCY_DENSITY], uint32_t *startOfScan,
            int *endOfScan);

      /**
       * Classify the regions into various types, such as balls, lines
       * and robot, for further processing via their respective classes
       **/
      void analyseRegions(uint32_t *startOfScan);

   private:

      /**
       * Two array holders used to store the last column and current
       * column region pointers.
       **/
      ImageRegion *holder1[IMAGE_ROWS/SALIENCY_DENSITY];
      ImageRegion *holder2[IMAGE_ROWS/SALIENCY_DENSITY];
      /**
       * The number of regions found in the last column scanned
       **/
      uint16_t numLastColRegions;
      /**
       * The number of regions found in the current column
       **/
      uint16_t numCurColRegions;
      /**
       * Pointers to the regions found in the previous column
       **/
      ImageRegion **lastColRegions;
      /**
       * Pointers to the regions found in the current column
       **/
      ImageRegion **curColRegions;

      /**
       * Tne number of regions so far examined in the last column
       **/
      uint16_t lastColCounter;

      /**
       * Add a scan line to an existing region or create a new region
       **/
      inline void addScanToRegions(uint16_t startScan, uint16_t endScan,
            uint16_t numOrange, uint16_t numRRed, uint16_t numRBlue,
            uint16_t numWhite, uint16_t i, uint16_t fieldEdge,
            uint16_t bottomMostRobot, uint16_t topMostRobot);

      /**
       * Pointer to the robot region currently being expanded
       **/
      RobotRegion *currentRobot;

      /**
       * Create a new robot region based off the region provided
       **/
      inline void newRobotRegion(RobotType type, ImageRegion *region);

      /**
       * Add the region information to the currentRobot
       **/ 
      inline void addToRobotRegion(ImageRegion *region);

      /**
       * Delete a robot region from consieration
       **/
      inline void deleteRobotRegion(int index);
};
