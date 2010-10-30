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

#include <utility>
#include "perception/vision/RobotRegion.hpp"
#include "perception/vision/VisionConstants.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/ImageToRR.hpp"
#include "perception/vision/ImageRegion.hpp"

class RobotDetection {
   public:

      RobotRegion *robotRegions[MAX_NUM_REGIONS];
      uint16_t numRobotRegions;

      /**
       * Data to write onto the blackboard
       **/
      RobotType robotTypes[MAX_NUM_ROBOTS];
      RRCoord robotLocations[MAX_NUM_ROBOTS];
      uint16_t numRobots;
      bool canSeeBottomRobot[MAX_NUM_ROBOTS];
      uint16_t robotImageCoords[MAX_NUM_ROBOTS * 4];

      void findRobots(RobotRegion *robotRegions, uint16_t numRobotRegions,
            uint32_t *startOfScan, Colour saliency
            [IMAGE_COLS/SALIENCY_DENSITY][IMAGE_ROWS/SALIENCY_DENSITY]);

      void sanityCheckRobots(uint16_t numGoalPosts, uint16_t *postCoords,
            CameraToRR *convRR, ImageRegion *bestBallRegion, uint16_t radius,
            std::pair<uint16_t, uint16_t> ballCentre);

   private:
      RobotRegion *removedRobots[MAX_NUM_ROBOTS];
      uint16_t numRemovedRobots;

      void findRobotType(RobotRegion *robotRegion, Colour saliency
            [IMAGE_COLS/SALIENCY_DENSITY][IMAGE_ROWS/SALIENCY_DENSITY],
            uint32_t *startOfScan);

      inline bool isColourInGoals(uint16_t numGoalPosts,
            uint16_t *postCoords, RobotRegion *robotRegion);

      bool isColourInBall(RobotRegion *robotRegion,
            ImageRegion *bestBallRegion, uint16_t radius,
            std::pair<uint16_t, uint16_t> ballCentre);

      void addRobotToList(RobotRegion *newRobot);

      inline void addRobotToRemovedList(RobotRegion *robot);

      void combineRobots(RobotRegion *current, RobotRegion *newRobot);
};
