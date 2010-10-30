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

#include "perception/vision/GoalDetection.hpp"

#include <vector>
#include <algorithm>
#include <utility>
#include "perception/vision/VisionClassifiedColour.hpp"
#include "utils/log.hpp"
#include "utils/basic_maths.hpp"
#include "utils/SPLDefs.hpp"
#include "utils/Timer.hpp"

#define GOAL_EDGE_THRESHOLD 14

using namespace std;

void GoalDetection::findGoals(
      XHistogram xhistogram[IMAGE_COLS/SALIENCY_DENSITY][cNUM_COLOURS],
      YHistogram yhistogram[IMAGE_COLS/SALIENCY_DENSITY][cNUM_COLOURS],
      CameraToRR *convRR, uint32_t *startScanCoords, Vision *vision,
      int *endOfScan, uint16_t numRobotRegions, RobotRegion **robotRegions,
      const std::pair<int, int> &horizon) {
   numPosts = 0;
   // cout << "**start**" << endl;
   findColouredGoals(cGOAL_YELLOW, xhistogram, yhistogram, startScanCoords,
         vision, endOfScan, numRobotRegions, robotRegions);
   if (numPosts == 0) {
      findColouredGoals(cGOAL_BLUE, xhistogram, yhistogram, startScanCoords,
            vision, endOfScan, numRobotRegions, robotRegions);
      if (numPosts == 0) {
         typeOfPost = pNONE;
      }
   } else {
      typeOfPost = (WhichPosts)(typeOfPost + NUMBER_ADD_TO_BLUE);
   }
   float distances[MAX_POSTS];
   uint16_t count = 0;
   float gradientHor = (horizon.second - horizon.first) / IMAGE_COLS;
   int16_t interceptHor = horizon.first;
   for (uint16_t n = 0; n < numPosts; n++, count += 4) {
      // Determine how far the goals are away from the robot based on what
      // parts of the goals can be seen
      uint16_t middle = (postCoords[count] + postCoords[count + 2]) >> 1;
      uint16_t horPoint = MAX(0, gradientHor * middle + interceptHor);
      posts[n] = convRR->convertToRR(middle, postCoords[count+3], false);
      if (horPoint >= postCoords[count + 3]) {
         if (posts[n].heading >= 0) {
            posts[n].heading -= M_PI;
         } else {
            posts[n].heading += M_PI;
         }
      } else if (horPoint + 100 >= postCoords[count + 3]) {
         RRCoord test = convRR->convertToRR(middle,
                  postCoords[count + 3] + 100, false);
         if (ABS(test.heading - posts[n].heading) > M_PI/4) {
            posts[n].heading = test.heading;
         }
      }
      distances[n] = convRR->pixelSeparationToDistance(
               postCoords[count + 2] - postCoords[count], GOAL_POST_DIAMETER);
      // DEBUG
      distanceProjection[n] = posts[n].distance;
      distanceGoalPostWidth[n] = distances[n];
      // END DEBUG
      if (postAspects[n].canSeeLeft && postAspects[n].canSeeRight) {
         posts[n].var[1] = convRR->calculateHeadingVariance(5);
      } else {
         posts[n].var[1] = convRR->calculateHeadingVariance(10);
      }
   }
   // Choose which method of calculating distance should be used
   bool distanceSet = false;
   if (typeOfPost == pYELLOW_BOTH || typeOfPost == pBLUE_BOTH) {
      if (postAspects[0].canSeeBottom &&
            postAspects[1].canSeeBottom) {
         distanceSet = true;
         llog(VERBOSE) << "Goal detection using KINEMATICS" << std::endl;
         llog(VERBOSE) << "Goal detection: bottom = " << postCoords[3]
                       << " " << postCoords[7] << std::endl;
         llog(VERBOSE) << "Goal detection: distance = " <<
                        posts[0].distance << std::endl;
         posts[0].var[0] =
            convRR->calculateDistanceVariance(posts[0].distance, 20, 200);
         posts[1].var[0] =
            convRR->calculateDistanceVariance(posts[1].distance, 20, 200);
      } else if (postAspects[0].canSeeLeft && postAspects[0].canSeeRight &&
            postAspects[1].canSeeLeft && postAspects[1].canSeeRight) {
         llog(VERBOSE) << "Goal detection using GO POST WIDTH" << std::endl;
         distanceSet = true;
         posts[0].distance = distances[0];
         posts[1].distance = distances[1];
         posts[0].var[0] =
            convRR->calculateDistanceVariance(distances[0], 5, 100);
         posts[1].var[0] =
            convRR->calculateDistanceVariance(distances[1], 5, 100);
      }
   }

   for (uint16_t i = 0; i < numPosts && !distanceSet; i++) {
      if (postAspects[i].canSeeBottom) {
         posts[i].var[0] =
            convRR->calculateDistanceVariance(posts[i].distance, 20, 100);
      } else if (postAspects[i].canSeeLeft && postAspects[i].canSeeRight) {
         posts[i].distance = distances[i];
         posts[i].var[0] =
            convRR->calculateDistanceVariance(distances[i], 5, 100);
      } else {
         // This shouldn't happen, but just in case...
         posts[i].var[0] =
            convRR->calculateDistanceVariance(posts[i].distance, 1, 100000);
      }
   }
}

void GoalDetection::findColouredGoals(Colour goalColour,
      XHistogram xhistogram[IMAGE_COLS/SALIENCY_DENSITY][cNUM_COLOURS],
      YHistogram yhistogram[IMAGE_ROWS/SALIENCY_DENSITY][cNUM_COLOURS],
      uint32_t *startScanCoords, Vision *vision, int *endOfScan,
      uint16_t numRobotRegions, RobotRegion **robotRegions) {
   int16_t yLoc = findYHistMaxPoint(goalColour, yhistogram);
   if (yLoc < 0) {
      // Did not find a suitable goal maximum
      return;
   }

   uint16_t xLoc[MAX_POSTS * 3];
   int16_t numXLoc = findXHistMaxPoints(goalColour, xhistogram, xLoc);
   if (numXLoc == 0) {
      // Did not find any significant maximums
      return;
   }

   for (uint16_t p = 0; p < numXLoc && numPosts < MAX_POSTS; p++) {
      uint16_t minyVal;
      uint16_t maxyVal;
      uint16_t minxVal = 0;
      uint16_t maxxVal = 0;
      maxyVal = findBottomOfPost(goalColour, xLoc[p], yLoc, vision);
      minyVal = findTopOfPost(goalColour, xLoc[p], yLoc, vision);
      if (minyVal > maxyVal || maxyVal - minyVal < MINIMUM_GOAL_HEIGHT) {
         // Haven't been able to get good results, so skip this possible post
         continue;
      }
      if (!findWidthOfPost(goalColour, xLoc[p], minyVal,
               maxyVal, &minxVal, &maxxVal, vision)) {
         // Didn't get sufficient information from left to right scans, so
         // discard as a possible post
         continue;
      }

      bool aboveRobot = false;
      if (performSanityChecks(minyVal, maxyVal, minxVal, maxxVal,
               startScanCoords, endOfScan, numRobotRegions, robotRegions,
               &aboveRobot)) {
         postCoords[numPosts << 2] = minxVal;
         postCoords[(numPosts << 2) + 1] = minyVal;
         postCoords[(numPosts << 2) + 2] = maxxVal;
         postCoords[(numPosts << 2) + 3] = maxyVal;
         if (maxxVal < IMAGE_COLS - 1) {
            postAspects[numPosts].canSeeRight = true;
         } else {
            postAspects[numPosts].canSeeRight = false;
         }
         if (minxVal > 0) {
            postAspects[numPosts].canSeeLeft = true;
         } else {
            postAspects[numPosts].canSeeLeft = false;
         }
         if (minyVal > 0) {
            postAspects[numPosts].canSeeTop = true;
         } else {
            postAspects[numPosts].canSeeTop = false;
         }
         if (maxyVal < IMAGE_ROWS - SALIENCY_DENSITY) {
            uint16_t midX = ((maxxVal + minxVal) >> 1) / SALIENCY_DENSITY;
            if (maxyVal + 2 > endOfScan[midX] * SALIENCY_DENSITY) {
               postAspects[numPosts].canSeeBottom = false;
            } else if (aboveRobot) {
               postAspects[numPosts].canSeeBottom = false;
            } else {
               postAspects[numPosts].canSeeBottom = true;
            }
            if (aboveRobot) {
               canSeeBottom[numPosts] = false;
            } else {
               canSeeBottom[numPosts] = true;
            }
         } else {
            postAspects[numPosts].canSeeBottom = false;
            canSeeBottom[numPosts] = false;
         }
         if (postAspects[numPosts].canSeeBottom ||
               (postAspects[numPosts].canSeeRight &&
                postAspects[numPosts].canSeeLeft)) {
            numPosts++;
         } else {
            // cout << "Not adding goal...." << endl;
         }
      }
   }
   findTypeOfPost(goalColour, xhistogram);
}

int16_t GoalDetection::findYHistMaxPoint(Colour goalColour,
            YHistogram yhistogram[IMAGE_COLS/SALIENCY_DENSITY][cNUM_COLOURS]) {
   uint16_t maxVal = 0;
   int16_t maxLoc = 0;
   uint16_t seenCount = 0;
   uint16_t maxSeen = IMAGE_ROWS;
   for (uint16_t i = 0; i < IMAGE_ROWS/SALIENCY_DENSITY; i++) {
      if (yhistogram[i][goalColour] >= Y_HIST_MAX_LIMIT && maxSeen
            == IMAGE_ROWS) {
         seenCount++;
         if (seenCount > 8) {
            maxSeen = i * SALIENCY_DENSITY;
         }
      } else {
         seenCount = 0;
      }
      if (yhistogram[i][goalColour] >= Y_HIST_MAX_LIMIT &&
            yhistogram[i][goalColour] > maxVal) {
         maxVal = yhistogram[i][goalColour];
         maxLoc = i;
      }
   }
   if (maxVal == 0) {
      return - 1;
   } else {
      return MIN(maxLoc * SALIENCY_DENSITY, (int)maxSeen);
   }
}

int16_t GoalDetection::findXHistMaxPoints(Colour goalColour,
            XHistogram xhistogram[IMAGE_COLS/SALIENCY_DENSITY][cNUM_COLOURS],
            uint16_t xLoc[MAX_POSTS * 3]) {
   int16_t numXLoc = 0;
   uint16_t maxVal = 0;
   uint16_t maxLoc = 0;
   bool foundMax = false;
   uint16_t minAllowed = 0;

   for (uint16_t i = 0; i < IMAGE_COLS/SALIENCY_DENSITY &&
         numXLoc < MAX_POSTS * 3; i++) {
      if (xhistogram[i][goalColour] > maxVal &&
            xhistogram[i][goalColour] > X_HIST_MAX_LIMIT &&
            xhistogram[i][goalColour] > minAllowed) {
         maxVal = xhistogram[i][goalColour];
         maxLoc = i;
         foundMax = true;
      } else if (foundMax &&
            xhistogram[i][goalColour] < maxVal/HIST_MIN_LIMIT) {
         minAllowed = maxVal/HIST_MIN_LIMIT;
         foundMax = false;
         maxVal = 0;
         xLoc[numXLoc] = maxLoc;
         numXLoc++;
      }
   }
      // If have found lots of posts, presumably something bad has happened
   if (numXLoc == MAX_POSTS * 3) {
      return numXLoc;
   }
   if (foundMax) {
      xLoc[numXLoc] = maxLoc;
      numXLoc++;
   }
   for (int p = 0; p < numXLoc; p++) {
      uint16_t cutoff = xhistogram[xLoc[p]][goalColour] / 2;
      uint16_t middle;
      int i;
      for (i = xLoc[p] + 1; i < IMAGE_COLS / SALIENCY_DENSITY &&
            xhistogram[i][goalColour] > cutoff; i++) {
      }
      middle = i;
      for (i = xLoc[p] - 1; i >= 0 && xhistogram[i][goalColour] > cutoff;
            i--) {
      }
      xLoc[p] = ((middle + i) >> 1) * SALIENCY_DENSITY;
   }
   return numXLoc;
}

inline uint16_t GoalDetection::findBottomOfPost(Colour goalColour,
      uint16_t xLoc, uint16_t yLoc, Vision *vision) {
   int16_t startY;
   if (yLoc >= IMAGE_ROWS - SALIENCY_DENSITY) {
      startY = IMAGE_ROWS - SALIENCY_DENSITY - 2;
   } else {
      startY = yLoc + 3;
   }
   uint16_t maxyVal = 0;
   Colour c;
   for (int16_t i = xLoc - 8; i <= xLoc + 8; i += 4) {
      if (i >= 0 && i < IMAGE_COLS) {
         bool found = false;
         uint16_t lastSeenDown = 0;
         bool foundEdge = false;
         for (int16_t j = startY; j < IMAGE_ROWS - 1; j++) {
            if (!found) {
               c = vision->getColour(j, i);
               if (c == goalColour) {
                  found = true;
               }
            } else if (vision->getColour(j, i) == cFIELD_GREEN) {
               lastSeenDown = j;
               break;
            } else if (vision->getColour(j, i) == cWHITE) {
               lastSeenDown = j;
               break;
            } else if (foundEdge) {
               if (j - lastSeenDown <= 3) {
               } else if (j - lastSeenDown > 6) {
                  break;
               } else if (vision->getColour(j, i) == goalColour) {
                  lastSeenDown = j;
                  foundEdge = false;
               }
            } else if (isGoalEdge(i, j, i, j + 1, vision)) {
               lastSeenDown = j;
               foundEdge = true;
            }
         }
         if (lastSeenDown > maxyVal) {
            maxyVal = lastSeenDown;
         }
      }
   }
   return maxyVal;
}

inline uint16_t GoalDetection::findTopOfPost(Colour goalColour, uint16_t xLoc,
      uint16_t yLoc, Vision *vision) {
   uint16_t lastSeenUp = IMAGE_ROWS;
   uint16_t minyVal = IMAGE_ROWS;
   Colour c;
   for (int16_t i = xLoc - 12; i <= xLoc + 12; i += 6) {
      if (i >= 0 && i < IMAGE_COLS) {
         /* If the maximum point from the histograms is over the goals
            scan up from this point to find the top of the posts. However,
            sometimes the maximum point can be above the goal post,
            so in this case you need to scan downwards */
         int j;
         if (vision->getColour(yLoc, xLoc) == goalColour) {
            PixelValues start = vision->getPixelValues(yLoc, xLoc);
            lastSeenUp = yLoc;
            for (j = yLoc; j >= 1; j--) {
               PixelValues cur = vision->getPixelValues(j, i);
               if (isGoalEdge(i, j, i, j - 1, vision)) {
                  break;
               } else if (ABS(start.y - cur.y) + ABS(start.u - cur.u) +
                     ABS(start.v - cur.v) > 5 * GOAL_EDGE_THRESHOLD) {
                  break;
               } else {
                  lastSeenUp = j;
               }
            }
         } else {
            for (j = yLoc; j < IMAGE_ROWS; j++) {
               c = vision->getColour(j, i);
               if (c == goalColour) {
                  break;
               }
            }
            lastSeenUp = j;
         }
         if (lastSeenUp < minyVal) {
            minyVal = lastSeenUp;
         }
      }
   }
   return minyVal;
}

inline bool GoalDetection::findWidthOfPost(Colour goalColour, uint16_t xMid,
            uint16_t minyVal, uint16_t maxyVal, uint16_t *minxVal,
            uint16_t *maxxVal, Vision *vision) {
   /* Scan a few times horizontally to find the width of the goals.
      Start the scan from the middle of the points that found the
      top and bottom of the goal posts and these points should
      hopefully be nicely inside the goal post */
   uint16_t midy = (minyVal + maxyVal) >> 1;
   uint16_t startOffset = ((midy + minyVal) >> 1) - minyVal;
   uint16_t dec = startOffset >> 1;
   if (dec == 0) {
      dec = 1;
   }
   std::vector<uint16_t> minPoints;
   std::vector<uint16_t> maxPoints;
   uint16_t badCount = 0;
   for (uint16_t j = midy + startOffset; j >= midy - startOffset;
         j -= dec) {
      uint16_t lastSeenLeft;
      uint16_t lastSeenRight;
      uint16_t numSeen = 0;
      uint16_t i;
      bool colourFound = false;
      PixelValues start = vision->getPixelValues(j, xMid);
      for (i = xMid; i >= 1; i--) {
         PixelValues cur = vision->getPixelValues(j, i);
         if (!colourFound && vision->getColour(j, i) == goalColour) {
            colourFound = true;
         }
         if (isGoalEdge(i, j, i - 1, j, vision)) {
            break;
         } else if (ABS(start.y - cur.y) + ABS(start.u - cur.u) +
            ABS(start.v - cur.v) > 4 * GOAL_EDGE_THRESHOLD) {
            break;
         }
      }
      lastSeenLeft = i;
      for (i = xMid; i < IMAGE_COLS - 1; i++) {
         PixelValues cur = vision->getPixelValues(j, i);
         if (!colourFound && vision->getColour(j, i) == goalColour) {
            colourFound = true;
         }
         if (isGoalEdge(i, j, i + 1, j, vision)) {
            break;
         } else if (ABS(start.y - cur.y) + ABS(start.u - cur.u) +
            ABS(start.v - cur.v) > 4 * GOAL_EDGE_THRESHOLD) {
            break;
         }
      }
      lastSeenRight = i;
      if (!colourFound) {
         badCount++;
      } else {
         numSeen = lastSeenRight - lastSeenLeft;
         if (numSeen > MINIMUM_GOAL_WIDTH) {
            minPoints.push_back(lastSeenLeft);
            maxPoints.push_back(lastSeenRight);
         }
      }
   }
   if (badCount > 1) {
      return false;
   }
   /* Get the median of each of these scans */
   sort(minPoints.begin(), minPoints.end());
   sort(maxPoints.begin(), maxPoints.end());
   uint16_t medianIndex = minPoints.size() >> 1;
   if (minPoints.size() < 1) {
      return false;
   } else {
      *minxVal = minPoints[medianIndex];
      *maxxVal = maxPoints[medianIndex];
      return true;
   }
}

inline bool GoalDetection::performSanityChecks(uint16_t minyVal,
      uint16_t maxyVal, uint16_t minxVal, uint16_t maxxVal,
      uint32_t *startScanCoords, int *endOfScan, uint16_t numRobotRegions,
      RobotRegion **robotRegions, bool *aboveRobot) {
   /**
    * Sanity check to remove goals not near the field. We may want to
    * remove this in the future as it will throw away goals partially
    * hidden by robots
    **/
   uint16_t midX = ((maxxVal + minxVal) >> 1) / SALIENCY_DENSITY;
   bool isOnField = false;
   if (maxyVal + SALIENCY_DENSITY > IMAGE_ROWS) {
      isOnField = true;
   } else if ((int)(startScanCoords[midX] *
               SALIENCY_DENSITY) < maxyVal + 12) {
      isOnField = true;
   } else if (maxyVal + SALIENCY_DENSITY > endOfScan[midX] *
         SALIENCY_DENSITY) {
      isOnField = true;
   }
   if (maxxVal - minxVal <= MINIMUM_GOAL_WIDTH) {
      return false;
   }
   if (maxyVal - minyVal <= MINIMUM_GOAL_HEIGHT) {
      return false;
   }
   if (MINIMUM_GOAL_RATIO * (maxxVal - minxVal) >= (maxyVal - minyVal)) {
      if (minyVal < 4 && MINIMUM_GOAL_PARTIAL_RATIO *
            (maxxVal - minxVal) < (maxyVal - minyVal)) {
      } else {
         return false;
      }
   }
   if (startScanCoords[midX] *
         SALIENCY_DENSITY + 2 < minyVal) {
      return false;
   }
   if (numPosts == 1 && postCoords[2] >= minxVal) {
      return false;
   }
   *aboveRobot = false;
   if (!isOnField) {
      bool foundRobot = false;
      uint16_t leftPost = minxVal / SALIENCY_DENSITY;
      uint16_t rightPost = maxxVal / SALIENCY_DENSITY;
      for (uint16_t p = 0; p < numRobotRegions; p++) {
         if (robotRegions[p]->type != UNKNOWN_ROBOT &&
               robotRegions[p]->leftMost < rightPost &&
               robotRegions[p]->rightMost > leftPost) {
            foundRobot = true;
            *aboveRobot = true;
         }
      }
      if (!foundRobot) {
         return false;
      }
   }
   if (minyVal > 2 && maxyVal + SALIENCY_DENSITY <
         endOfScan[midX] * SALIENCY_DENSITY &&
         maxyVal - minyVal < MINIMUM_TOTAL_GOAL_HEIGHT && !(*aboveRobot)) {
      return false;
   }
   return true;
}

inline bool GoalDetection::isGoalEdge(uint16_t i1, uint16_t j1,
      uint16_t i2, uint16_t j2, Vision *vision) {
   PixelValues p1 = vision->getPixelValues(j1, i1);
   PixelValues p2 = vision->getPixelValues(j2, i2);
   int calc = ABS(p1.y - p2.y) + ABS(p1.u - p2.u) + ABS(p1.v - p2.v);
   // int calc = ABS(p1.v - p2.v);
   if (calc > GOAL_EDGE_THRESHOLD) {
      return true;
   } else {
      return false;
   }
}

void GoalDetection::findTypeOfPost(Colour goalColour,
     XHistogram xhistogram[IMAGE_ROWS/SALIENCY_DENSITY][cNUM_COLOURS]) {
   if (numPosts == 1 && postAspects[0].canSeeTop) {
      // Work out which post can be seen
      uint16_t midX = ((postCoords[0] + postCoords[2]) >> 1) /
            SALIENCY_DENSITY;
      int16_t min = midX;
      int16_t max = midX;
      int16_t countUp;
      int16_t countDown;
      bool stopUp = false;
      bool stopDown = false;
      for (countUp = midX; countUp < IMAGE_COLS/SALIENCY_DENSITY; countUp++) {
         if (!stopUp && xhistogram[countUp][goalColour] >
               HIST_GOAL_END_THRESH) {
            max = countUp;
         } else if (countUp - max > HIST_SKIP_ALLOWANCE) {
            stopUp = true;
         }
      }
      for (countDown = midX; countDown >= 0; countDown--) {
         if (!stopDown && xhistogram[countDown][goalColour] >
               HIST_GOAL_END_THRESH) {
            min = countDown;
         } else if (min - countDown > HIST_SKIP_ALLOWANCE) {
            stopDown = true;
         }
      }
      // cout << min << " " << midX << " " << max << endl;
      int16_t width = (postCoords[2] - postCoords[0]) / SALIENCY_DENSITY;
      int16_t left = postCoords[0] / SALIENCY_DENSITY;
      int16_t right = postCoords[2] / SALIENCY_DENSITY;
      if (!stopUp && !stopDown) {
         if (postCoords[2] + SALIENCY_DENSITY * 2 > IMAGE_COLS) {
            typeOfPost = pBLUE_RIGHT;
         } else if (postCoords[0] - SALIENCY_DENSITY * 2 <= 0) {
            typeOfPost = pBLUE_LEFT;
         } else {
            typeOfPost = pBLUE_EITHER;
         }
      } else if (!stopDown) {
         if (postCoords[0] - SALIENCY_DENSITY * 2 <= 0) {
            typeOfPost = pBLUE_EITHER;
         } else {
            typeOfPost = pBLUE_RIGHT;
         }
      } else if (!stopUp) {
         if (postCoords[2] + SALIENCY_DENSITY * 2 > IMAGE_COLS) {
            typeOfPost = pBLUE_EITHER;
         } else {
            typeOfPost = pBLUE_LEFT;
         }
      } else if (left - min > 2 * width && max - right < 2 * width &&
         (left - min) - (max - right) >= HIST_GOAL_SEP_THRESH) {
         typeOfPost = pBLUE_RIGHT;
      } else if (left - min < 2 * width && max - right > 2 * width &&
         (max - right) - (left - min) >= HIST_GOAL_SEP_THRESH) {
         typeOfPost = pBLUE_LEFT;
      } else {
         typeOfPost = pBLUE_EITHER;
      }
   } else if (numPosts == 1) {
      typeOfPost = pBLUE_EITHER;
   } else if (numPosts == 2) {
      typeOfPost = pBLUE_BOTH;
   } else {
      typeOfPost = pNONE;
   }
}

