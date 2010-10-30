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
#include "perception/vision/ImageToRR.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/VisionConstants.hpp"
#include "perception/vision/RobotRegion.hpp"

class Vision;

class GoalDetection {
   public:

      /**
       * The type and colour of the posts seen in the image
       **/
      WhichPosts typeOfPost;
      /**
       * The robot relative coords of each post seen in the
       * image. Used for the blackboard
       **/
      RRCoord posts[MAX_POSTS];
      /**
       * The number of posts recorded in the posts array
       **/
      uint16_t numPosts;
      /**
       * The image coords of the top, left, right and bottom of each
       * post (each post takes up four adjacent entries in the array).
       * Used by off-nao to draw the posts
       **/
      uint16_t postCoords[MAX_POSTS * 4];

      /** Temp values to store diffrent distances to the blackboard **/
      float distanceProjection[MAX_POSTS];
      float distanceGoalPostWidth[MAX_POSTS];
      float distanceGoalSep;
      bool canSeeBottom[MAX_POSTS];

      /**
       * Find goals in the image and write the locations to the
       * blackboard
       **/
      void findGoals(
            XHistogram xhistogram[IMAGE_COLS/SALIENCY_DENSITY][cNUM_COLOURS],
            YHistogram yhistogram[IMAGE_ROWS/SALIENCY_DENSITY][cNUM_COLOURS],
            CameraToRR *convRR, uint32_t *startScanCoords, Vision *vision,
            int *endOfScan, uint16_t numRobotRegions,
            RobotRegion **robotRegions, const::std::pair<int, int> &horizon);

   private:
      struct PostAspects {
         bool canSeeLeft;
         bool canSeeRight;
         bool canSeeTop;
         bool canSeeBottom;
      };
      /**
       * Stores what parts of each post can be seen so that distance
       * calculations can decide what method is best to use
       **/
      struct PostAspects postAspects[MAX_POSTS];

      /**
       * Scan the field for goals of the colour goalColour
       * and process the image
       **/
      void findColouredGoals(Colour goalColour,
            XHistogram xhistogram[IMAGE_COLS/SALIENCY_DENSITY][cNUM_COLOURS],
            YHistogram yhistogram[IMAGE_ROWS/SALIENCY_DENSITY][cNUM_COLOURS],
            uint32_t *startScanCoords, Vision *vision, int *endOfScan,
            uint16_t numRobotRegions, RobotRegion **robotRegions);

      /**
       * Finds the maximum point in the y histogram. Returns -1 is seen
       * no significant maximums
       **/
      int16_t findYHistMaxPoint(Colour goalColour,
            YHistogram yhistogram[IMAGE_COLS/SALIENCY_DENSITY][cNUM_COLOURS]);

      /**
       * Finds the location of local maximums in the x histogram. Returns
       * the number of local maximums
       **/
      int16_t findXHistMaxPoints(Colour goalColour,
            XHistogram xhistogram[IMAGE_ROWS/SALIENCY_DENSITY][cNUM_COLOURS],
            uint16_t xLoc[MAX_POSTS * 3]);

      /**
       * Find the coordinate of the bottom of the post
       **/
      inline uint16_t findBottomOfPost(Colour goalColour, uint16_t xLoc,
            uint16_t yLoc, Vision *vision);

      /**
       * Find the coordinate of the top of the post
       **/
      inline uint16_t findTopOfPost(Colour goalColour, uint16_t xLoc,
            uint16_t yLoc, Vision *vision);

      /**
       * Find the left and right coordinate of the post. Returns
       * false if an error was encountered
       **/
      inline bool findWidthOfPost(Colour goalColour, uint16_t xMid,
            uint16_t minyVal, uint16_t maxyVal, uint16_t *minxVal,
            uint16_t *maxxVal, Vision *vision);

      /**
       * Do some sanity checks to see if should accept the goal.
       * Returns true if the goal should be accepted
       **/
      inline bool performSanityChecks(uint16_t minyVal, uint16_t maxyVal,
            uint16_t minxVal, uint16_t maxxVal, uint32_t *startScanCoords,
            int *endOfScan, uint16_t numRobotRegions,
            RobotRegion **robotRegions, bool *aboveRobot);
      /**
       * Find if the pixels being examined are edge pixels
       **/
      inline bool isGoalEdge(uint16_t i1, uint16_t j1,
            uint16_t i2, uint16_t j2, Vision *vision);

      /**
       * Sets what type of goal post is seen
       **/
      void findTypeOfPost(Colour goalColour,
            XHistogram xhistogram[IMAGE_ROWS/SALIENCY_DENSITY][cNUM_COLOURS]);
};
