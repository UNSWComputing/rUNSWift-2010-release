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

#include "perception/vision/BallDetection.hpp"
#include <algorithm>
#include "perception/vision/VisionClassifiedColour.hpp"
#include "utils/log.hpp"
#include "utils/basic_maths.hpp"

using namespace std;

#define BALL_EDGE_THRESHOLD 14

BallDetection::BallDetection() {
   possibleXCentre.reserve(NUM_CENTRE_REPEATS);
   possibleYCentre.reserve(NUM_CENTRE_REPEATS);
   possibleRadius.reserve(NUM_CENTRE_REPEATS);
}

void BallDetection::findBalls(ImageRegion **ballRegions,
      uint16_t numBallRegions, CameraToRR *convRR, Vision *vision,
      unsigned int *seed, RobotDetection *robotDetection,
      uint32_t *startOfScan, const std::pair<int, int> &horizon) {
   radius = 0;
   ImageRegion *bestBall = 0;
   bestBallRegion = NULL;
   uint16_t bestPixels = 0;
   for (uint16_t p = 0; p < numBallRegions; p++) {
      if (!isInsideRobot(robotDetection, ballRegions[p]) &&
            !isBallAboveMissedEdge(ballRegions[p], vision, horizon) &&
            ballRegions[p]->numPixels > bestPixels) {
         bestPixels = ballRegions[p]->numPixels;
         bestBall = ballRegions[p];
      }
   }
#if 1
   if (numBallRegions == 0 || bestBall == 0) {
      // Did not find a suitable ball
      numBalls = 0;
      return;
   } else if (bestBall->numPixels <= 1) {
      numBalls = 0;
      return;
   } else {
      numBalls = 0;
      radius = 0;
      bestBallRegion = bestBall;
      if (bestBall->numPixels < FINE_SCAN_THRESHOLD) {
         findBallEdgeSmall(vision, bestBall);
      } else if (bestBall->numPixels < LARGE_SCAN_THRESHOLD) {
         findBallEdge(vision, bestBall, 1);
      } else {
         findBallEdge(vision, bestBall, 2);
      }
      llog(VERBOSE) << "Finished finding the edges of the ball" << endl;
      if (numBallEdgePoints < MINIMUM_NUM_EDGE_POINTS) {
         radius = 0;
         numBalls = 0;
         return;
      } else if (bottomMost - topMost < MINIMUM_BALL_WIDTH ||
            rightMost - leftMost < MINIMUM_BALL_WIDTH) {
         radius = 0;
         numBalls = 0;
      }
      calculateBallProperties(seed);
      int32_t fieldEdge = startOfScan[ballCentre.first/SALIENCY_DENSITY] *
         SALIENCY_DENSITY;
      if (radius > MAXIMUM_BALL_RADIUS) {
         numBalls = 0;
         radius = 0;
      } else if (radius < MINIMUM_BALL_RADIUS) {
         numBalls = 0;
         radius = 0;
      } else if (fieldEdge != 0 && (fieldEdge + 4 > ballCentre.second ||
               bestBall->bottomMost.second * SALIENCY_DENSITY
               <= fieldEdge + 8)) {
         numBalls = 0;
         radius = 0;
      } else if (isCentreInsideRobot(robotDetection, ballCentre.first,
               ballCentre.second)) {
         numBalls = 0;
         radius = 0;
      } else if (isBallInMissedRobot(vision)) {
         numBalls = 0;
         radius = 0;
      } else if (!isBallInsideBallRegion(bestBall)) {
         numBalls = 0;
         radius = 0;
      } else {
         ballLoc[0] = convRR->convertToRR(
               ballCentre.first, ballCentre.second, true);
         float distance = convRR->ballDistanceByRadius(radius);
         if (distance + 2000 < ballLoc[0].distance) {
            llog(VERBOSE) << "Remove ball at " << ballCentre.first << " " <<
               ballCentre.second << " " << distance
               << " " << ballLoc[0].distance << endl;
            // Get rid of the ball
            numBalls = 0;
            radius = 0;
         } else if (distance < 0) {
            ballLoc[0].var[0] = convRR->calculateDistanceVariance(
                  ballLoc[0].distance, 25, 100);
         } else if (0 &&
               distance > ballLoc[0].distance / BALL_DISTANCE_TOLERANCE &&
               distance < ballLoc[0].distance * BALL_DISTANCE_TOLERANCE) {
            ballLoc[0].distance = distance;
            ballLoc[0].var[0] = convRR->calculateDistanceVariance(
                  distance, 25, 10);
         } else {
            ballLoc[0].var[0] = convRR->calculateDistanceVariance(
                  ballLoc[0].distance, 20, 200);
         }
         ballLoc[0].var[1] = convRR->calculateHeadingVariance(5);
      }
   }
#else
   if (numBallRegions == 0) {
      numBalls = 0;
      return;
   }
   uint16_t width = bestBall->rightMost - bestBall->leftMost;
   uint16_t height = bestBall->bottomMost.second - bestBall->topMost.second;
   if (height > width) {
      radius = (height * SALIENCY_DENSITY) >> 1;
   } else {
      radius = (width * SALIENCY_DENSITY) >> 1;
   }
   // Make sure that if the ball is only one pixel in the saliency image that
   // it is still drawn in off-nao
   if (radius == 0) {
      radius = 1;
   }
   // Get the ball information for off-nao
   ballCentre.first = ((bestBall->rightMost + bestBall->leftMost) >> 1) *
      SALIENCY_DENSITY;
   ballCentre.second = ((bestBall->bottomMost.second +
            bestBall->topMost.second) >> 1) * SALIENCY_DENSITY;
   ballLoc[0] = convRR->convertToRR(
         ballCentre.first,
         ballCentre.second,
         true);
   ballLoc[0].var[1] = convRR->calculateHeadingVariance(5);
   ballLoc[0].var[0] = convRR->calculateDistanceVariance(
         ballLoc[0].distance, 25, 100);
   numBalls = 1;
#endif
}

bool BallDetection::isBallAboveMissedEdge(ImageRegion *ballRegion,
      Vision *vision, const std::pair<int, int> &horizon) {
   uint16_t rows = IMAGE_ROWS/SALIENCY_DENSITY;
   uint16_t cols = IMAGE_COLS/SALIENCY_DENSITY;
   if (ballRegion->numPixels > 100) {
      return false;
   }
   int16_t middle = (ballRegion->rightMost + ballRegion->leftMost) >> 1;
   float gradient = SALIENCY_DENSITY*
      (horizon.second - horizon.first)/IMAGE_COLS;
   int16_t intercept = horizon.first / SALIENCY_DENSITY;
   const uint16_t hpoint = MAX(0, gradient*middle+intercept);
   for (int16_t i = (int16_t)ballRegion->topMost.second; i >= hpoint; --i) {
      if (vision->saliency[middle][i] == cFIELD_GREEN) {
         return false;
      }
   }
   bool found = false;
   uint16_t foundCount = 0;
   for (int16_t i = (int16_t)ballRegion->bottomMost.second; i < rows &&
       i < (int16_t)ballRegion->bottomMost.second + EXCLUDE_BALL_PIXELS
       && !found; ++i) {
      if (vision->saliency[middle][i] == cFIELD_GREEN) {
         foundCount++;
         found = true;
      }
   }
   middle = (ballRegion->topMost.second + ballRegion->bottomMost.second) >> 1;
   found = false;
   for (int16_t i = (int16_t)ballRegion->leftMost; i >= 0 &&
         i > (int16_t)ballRegion->leftMost - EXCLUDE_BALL_PIXELS
         && !found; --i) {
      if (vision->saliency[i][middle] == cFIELD_GREEN) {
         foundCount++;
         found = true;
      }
   }
   found = false;
   for (int16_t i = (int16_t)ballRegion->rightMost; i < cols &&
         i < (int16_t)ballRegion->rightMost + EXCLUDE_BALL_PIXELS
         && !found; ++i) {
      if (vision->saliency[i][middle] == cFIELD_GREEN) {
         foundCount++;
         found = true;
      }
   }
   if (foundCount > 2) {
      return false;
   } else {
      return true;
   }
}

bool BallDetection::isBallInMissedRobot(Vision *vision) {
   int rows = IMAGE_ROWS/SALIENCY_DENSITY;
   int cols = IMAGE_COLS/SALIENCY_DENSITY;
   int salRad = radius/SALIENCY_DENSITY;
   int ballX = ballCentre.first/SALIENCY_DENSITY;
   int ballY = ballCentre.second/SALIENCY_DENSITY;
   int numOrange = 0;
   int numRed = 0;
   int numWhite = 0;
   if (salRad > 3) {
      for (int i = ballX - salRad; i < ballX + salRad; ++i) {
         if (i >= 0 && i < cols) {
            for (int j = ballY - salRad; j < ballY + salRad; ++j) {
               if (j >= 0 && j < rows) {
                  if (vision->saliency[i][j] == cBALL) {
                     numOrange++;
                  } else if (vision->saliency[i][j] == cROBOT_RED) {
                     numRed++;
                  } else if (vision->saliency[i][j] == cWHITE) {
                     numWhite++;
                  }
               }
            }
         }
      }
   }
   if (numRed > numOrange) {
      return true;
   } else if (numOrange == 0) {
      if (ballCentre.second < 12 || ballCentre.second > IMAGE_ROWS - 12) {
         return true;
      } else {
         return false;
      }
   } else if ((ballCentre.second < 20 || ballCentre.second > IMAGE_ROWS - 20)
         && numOrange < 25 && numRed/(float)numOrange > 0.6) {
      return true;
   } else {
      return false;
   }
}

bool BallDetection::isBallInsideBallRegion(ImageRegion *ballRegion) {
   int leftRegion = ballRegion->leftMost * SALIENCY_DENSITY;
   int rightRegion = ballRegion->rightMost * SALIENCY_DENSITY;
   int topRegion = ballRegion->topMost.second * SALIENCY_DENSITY;
   int bottomRegion = ballRegion->bottomMost.second * SALIENCY_DENSITY;
   int leftBall = ballCentre.first - radius;
   int rightBall = ballCentre.first + radius;
   int topBall = ballCentre.second - radius;
   int bottomBall = ballCentre.second + radius;
   if (rightRegion < leftBall) {
      return false;
   }
   if (leftRegion > rightBall) {
      return false;
   }
   if (topRegion > bottomBall) {
      return false;
   }
   if (bottomRegion < topBall) {
      return false;
   }
   return true;
}

bool BallDetection::isCentreInsideRobot(RobotDetection *robotDetection,
      uint16_t xCoord, uint16_t yCoord) {
   uint16_t x = xCoord / SALIENCY_DENSITY;
   uint16_t y = yCoord / SALIENCY_DENSITY;
   for (int i = 0; i < robotDetection->numRobotRegions; i++) {
      if (robotDetection->robotRegions[i]->type == RED_ROBOT &&
            robotDetection->robotRegions[i]->leftMost <= x + 1 &&
            robotDetection->robotRegions[i]->rightMost + 1 >= x) {
        if (robotDetection->robotRegions[i]->bottomMost >= y + 10) {
           return true;
        } else if (robotDetection->robotRegions[i]->bottomMost >= y &&
              robotDetection->robotRegions[i]->bottomMost + SALIENCY_DENSITY
              < IMAGE_ROWS/SALIENCY_DENSITY) {
           return true;
        }
      }
   }
   return false;
}

bool BallDetection::isInsideRobot(RobotDetection *robotDetection,
      ImageRegion *ballRegion) {
   for (int i = 0; i < robotDetection->numRobotRegions; i++) {
      if (ballRegion->leftMost + 3 >=
            robotDetection->robotRegions[i]->leftMost
            && ballRegion->rightMost <=
            robotDetection->robotRegions[i]->rightMost + 3) {
         if (ballRegion->bottomMost.second + 10 <
               robotDetection->robotRegions[i]->bottomMost) {
            if (robotDetection->robotRegions[i]->type == RED_ROBOT) {
               return true;
            } else if (robotDetection->robotRegions[i]->type == BLUE_ROBOT &&
                  ballRegion->numPixels < 30) {
               return true;
            } else if (ballRegion->numPixels < 25) {
               return true;
            }
         } else if (robotDetection->robotRegions[i]->bottomMost + 3 >
               IMAGE_ROWS/SALIENCY_DENSITY &&
               ballRegion->bottomMost.second <=
               robotDetection->robotRegions[i]->bottomMost +
               SALIENCY_DENSITY && ballRegion->numPixels < 25) {
            if (robotDetection->robotRegions[i]->type == RED_ROBOT) {
               return true;
            } else if (robotDetection->robotRegions[i]->type == BLUE_ROBOT &&
                  ballRegion->numPixels < 10) {
               return true;
            } else if (ballRegion->numPixels > 3) {
               return true;
            }
         }
      }
   }
   return false;
}

void BallDetection::findBallEdgeSmall(Vision *vision,
      ImageRegion *ballRegion) {
   llog(VERBOSE) << "Finding small ball in image" << endl;
   bottomMost = 0;
   topMost = IMAGE_ROWS;
   leftMost = IMAGE_COLS;
   rightMost = 0;
   numBallEdgePoints = 0;
   int16_t yCentre = ((ballRegion->topMost.second +
         ballRegion->bottomMost.second) >> 1) * SALIENCY_DENSITY;
   int16_t xStart = ballRegion->leftMost * SALIENCY_DENSITY;
   int16_t xEnd = ballRegion->rightMost * SALIENCY_DENSITY;
   if (xStart <= 0 || xEnd >= IMAGE_COLS) {
      llog(ERROR) << "ERROR is small ball detection " << xStart << " " <<
         xEnd << endl;
   }
   for (int16_t i = xStart; i <= xEnd &&
         numBallEdgePoints < MAX_BALL_EDGE_POINTS; i++) {
      int16_t j = yCentre;
      if (j >= IMAGE_ROWS - 1) {
         j--;
      }
      for (; j >= 1; j--) {
         if (isEdge(i, j - 1, i, j + 1, vision) ||
               vision->getColour(j, i) == cFIELD_GREEN ||
               vision->getColour(j, i) == cWHITE) {
            ballEdgePoints[numBallEdgePoints].first = i;
            ballEdgePoints[numBallEdgePoints++].second = j;
            if (j < topMost) {
               topMost = j;
            }
            break;
         }
      }
      if (j == 1) {
         topMost = j;
      }
      // Loop stops at IMAGE_ROWS - 2 due to dodgy final row of camera
      for (j = yCentre + SALIENCY_DENSITY; j < IMAGE_ROWS - 2 &&
            numBallEdgePoints < MAX_BALL_EDGE_POINTS; j++) {
         if (isEdge(i, j - 1, i, j + 1, vision) ||
               vision->getColour(j, i) == cFIELD_GREEN ||
               vision->getColour(j, i) ==cWHITE) {
            ballEdgePoints[numBallEdgePoints].first = i;
            ballEdgePoints[numBallEdgePoints++].second = j;
            if (j > bottomMost) {
               bottomMost = j;
            }
            break;
         }
      }
      if (j == IMAGE_ROWS - 2) {
         bottomMost = j;
      }
   }
   int16_t xCentre = ((ballRegion->leftMost + ballRegion->rightMost) >> 1) *
      SALIENCY_DENSITY;
   int16_t yStart = topMost;
   int16_t yEnd = bottomMost;
   for (int16_t j = yStart; j < yEnd; j++) {
      int16_t i;
      for (i = xCentre - 2; i >= 1 && numBallEdgePoints
            < MAX_BALL_EDGE_POINTS; i--) {
         if (isEdge(i - 1, j, i + 1, j, vision) ||
               vision->getColour(j, i) == cFIELD_GREEN ||
               vision->getColour(j, i) == cWHITE) {
            ballEdgePoints[numBallEdgePoints].first = i;
            ballEdgePoints[numBallEdgePoints++].second = j;
            if (i < leftMost) {
               leftMost = i;
            }
            break;
         }
      }
      if (i == 1) {
         leftMost = i;
      }
      for (i = xCentre + 4; i < IMAGE_COLS - 1 && numBallEdgePoints <
            MAX_BALL_EDGE_POINTS; i++) {
         if (isEdge(i - 1, j, i + 1, j, vision) ||
               vision->getColour(j, i) == cFIELD_GREEN ||
               vision->getColour(j, i) == cWHITE) {
            ballEdgePoints[numBallEdgePoints].first = i;
            ballEdgePoints[numBallEdgePoints++].second = j;
            if (i > rightMost) {
               rightMost = i;
            }
            break;
         }
      }
      if (i == IMAGE_COLS - 1) {
         rightMost = i;
      }
   }
}

/**
 * Idea for how to write the function:
 * - Just scan in the bounds at the moment
**/ 
void BallDetection::findBallEdge(Vision *vision, ImageRegion *ballRegion,
      int numSkip) {
   llog(VERBOSE) << "Finding the ball edges" << endl;
   bottomMost = 0;
   topMost = IMAGE_ROWS;
   leftMost = IMAGE_COLS;
   rightMost = 0;
   numBallEdgePoints = 0;
   uint16_t lowestTop = IMAGE_ROWS;
   uint16_t highestBottom = 0;
   for (uint16_t i = 0; i < ballRegion->numScanPoints &&
         numBallEdgePoints + 1 < MAX_BALL_EDGE_POINTS; i++) {
      if (ballRegion->startScans[i].second < 0) {
         llog(ERROR) << "ERROR in startScans: " <<
            ballRegion->startScans[i].second << endl;
      }
      if (ballRegion->endScans[i].second * SALIENCY_DENSITY > IMAGE_ROWS) {
         llog(ERROR) << "ERROR in end Scans value: " << ballRegion->
            endScans[i].second * SALIENCY_DENSITY << endl;
      }
      if (ballRegion->leftMost + 1 != ballRegion->startScans[i].first &&
            ballRegion->rightMost - 1 != ballRegion->startScans[i].first &&
            i % numSkip != 0) {
         continue;
      }
      uint16_t startIndex = (ballRegion->startScans[i].second)
         * SALIENCY_DENSITY;
      uint16_t endIndex = (ballRegion->endScans[i].second) * SALIENCY_DENSITY;
      if (startIndex + 8 < endIndex - SALIENCY_DENSITY) {
         startIndex += 8;
      } else if (startIndex < endIndex) {
         startIndex = endIndex - SALIENCY_DENSITY;
      }
      uint16_t xIndex = ballRegion->startScans[i].first * SALIENCY_DENSITY;
      uint16_t j;
      for (j = startIndex; j >= 1; j--) {
         if (j - 1 < 0 || j + 1 >= IMAGE_ROWS || xIndex < 0 || xIndex >=
               IMAGE_COLS) {
            llog(ERROR) << "isEdge out of range " << xIndex << " " << j <<
               " line 269" << endl;
         }
         if (isEdge(xIndex, j - 1, xIndex, j + 1, vision) ||
              vision->getColour(j, xIndex) == cFIELD_GREEN ||
              vision->getColour(j, xIndex) == cWHITE) {
            ballEdgePoints[numBallEdgePoints].first = xIndex;
            ballEdgePoints[numBallEdgePoints++].second = j;
            if ((ballRegion->leftMost + 1 == ballRegion->startScans[i].first ||
                 ballRegion->rightMost - 1 == ballRegion->startScans[i].first)
                 && j < lowestTop) {
               lowestTop = j;
            }
            if (j < topMost) {
               topMost = j;
            }

            break;
         } else if (j == 1 &&
              (ballRegion->leftMost + 1 == ballRegion->startScans[i].first ||
               ballRegion->rightMost - 1 == ballRegion->startScans[i].first) &&
              j < lowestTop) {
            lowestTop = j;
         }
      }
      if (j == 0) {
         topMost = 0;
      }
      endIndex = (ballRegion->startScans[i].second) * SALIENCY_DENSITY;
      startIndex = (ballRegion->endScans[i].second) * SALIENCY_DENSITY;
      if (startIndex - 8 > endIndex + SALIENCY_DENSITY) {
         startIndex -= 8;
      } else {
         startIndex = endIndex + SALIENCY_DENSITY;
      }
      xIndex = ballRegion->startScans[i].first * SALIENCY_DENSITY;
      if (startIndex < 1) {
         startIndex = 1;
      }
      // Note that the loop is stopped at IMAGE_ROWS - 2 because the
      // row of pixels at the bottom of the image is full of random noise
      for (j = startIndex; j < IMAGE_ROWS - 2; j++) {
          if (j - 1 < 0 || j + 1 >= IMAGE_ROWS || xIndex < 0 || xIndex >=
               IMAGE_COLS) {
            llog(ERROR) << "isEdge out of range " << xIndex << " " << j <<
               " line 322" << endl;
         }
         if (isEdge(xIndex, j - 1, xIndex, j + 1, vision) ||
               vision->getColour(j, xIndex) == cFIELD_GREEN ||
               vision->getColour(j, xIndex) == cWHITE) {
            ballEdgePoints[numBallEdgePoints].first = xIndex;
            ballEdgePoints[numBallEdgePoints++].second = j;
            if ((ballRegion->leftMost + 1 == ballRegion->startScans[i].first ||
                  ballRegion->rightMost - 1 == ballRegion->startScans[i].first)
                  && j > highestBottom) {
               highestBottom = j;
            }
            if (j > bottomMost) {
               bottomMost = j;
            }
            break;
         } else if (j == IMAGE_ROWS - 3 && (ballRegion->leftMost + 1 ==
               ballRegion->startScans[i].first || ballRegion->rightMost - 1 ==
               ballRegion->startScans[i].first) && j > highestBottom) {
            // Ensure that the highestBottom variable gets set correctly
            // when the ball is over the bottom of the image
            highestBottom = j;
         }
      }
      if (j == IMAGE_ROWS - 2) {
         bottomMost = j;
      }
   }
   uint16_t leftStart = (ballRegion->leftMost + 1) * SALIENCY_DENSITY;
   uint16_t rightStart = (ballRegion->rightMost - 1) * SALIENCY_DENSITY;
   if (rightStart == 0) {
      rightStart = 1;
   }
   for (int16_t j = lowestTop; j < highestBottom && numBallEdgePoints + 1 <
         MAX_BALL_EDGE_POINTS; j += SALIENCY_DENSITY * numSkip) {
      int16_t i;
      for (i = leftStart; i >= 1; i--) {
         if (i - 1 < 0 || i + 1 >= IMAGE_COLS || j < 0 || j >=
               IMAGE_ROWS) {
            llog(ERROR) << "isEdge out of range " << i << " " << j <<
               " line 378" << endl;
         }
         if (isEdge(i - 1, j, i + 1, j, vision) ||
            vision->getColour(j, i) == cFIELD_GREEN ||
            vision->getColour(j, i) == cWHITE) {
            ballEdgePoints[numBallEdgePoints].first = i;
            ballEdgePoints[numBallEdgePoints++].second = j;
            if (i < leftMost) {
               leftMost = i;
            }
            break;
         }
      }
      if (i == 0) {
         leftMost = 0;
      }
      for (i = rightStart; i < IMAGE_COLS - 1; i++) {
         if (i - 1 < 0 || i + 1 >= IMAGE_COLS || j < 0 || j >=
               IMAGE_ROWS) {
            llog(ERROR) << "isEdge out of range " << i << " " << j <<
               " line 397" << endl;
         }
         if (isEdge(i - 1, j, i + 1, j, vision) ||
               vision->getColour(j, i) == cFIELD_GREEN ||
               vision->getColour(j, i) == cWHITE) {
            ballEdgePoints[numBallEdgePoints].first = i;
            ballEdgePoints[numBallEdgePoints++].second = j;
            if (i > rightMost) {
               rightMost = i;
            }
            break;
         }
      }
      if (i == IMAGE_COLS - 1) {
         rightMost = i;
      }
   }
}

inline bool BallDetection::isEdge(uint16_t i1, uint16_t j1,
      uint16_t i2, uint16_t j2, Vision *vision) {
   PixelValues p1 = vision->getPixelValues(j1, i1);
   PixelValues p2 = vision->getPixelValues(j2, i2);
   int calc = ABS(p1.v - p2.v);
   if (calc > BALL_EDGE_THRESHOLD) {
      return true;
   } else {
      return false;
   }
}

// Try isnan(), which is a function in #include <cmath>
// can also try INFINITY or NAN or -INFINITY, or
// isfinite (nonzero if x is finite)
void BallDetection::calculateBallProperties(unsigned int *seed) {
   std::pair<uint16_t, uint16_t> points[3];
   possibleXCentre.clear();
   possibleYCentre.clear();
   possibleRadius.clear();
   for (uint16_t counter = 0; counter <  NUM_CENTRE_REPEATS; counter++) {
      if (getThreeUniquePoints(seed, points)) {
         TwoParamLine l1 = findEquationOfLine(points[0], points[1]);
         TwoParamLine l2 = findEquationOfLine(points[1], points[2]);
         float xVal = 0;
         float yVal = 0;
         if (l1.m - l2.m == 0) {
            // This shouldn't be possible, but just in case, skip to the
            // next point
            continue;
         } else if (l1.m == 1/0.0f) {
            xVal = l1.b;
            yVal = l2.m * xVal + l2.b;
         } else if (l2.m == 1/0.0f) {
            xVal = l2.b;
            yVal = l1.m * xVal + l1.b;
         } else {
            xVal = (l2.b - l1.b) / (l1.m - l2.m);
            yVal = l1.m * xVal + l1.b;
         }
            possibleXCentre.push_back((uint16_t)xVal);
            possibleYCentre.push_back((uint16_t)yVal);
            possibleRadius.push_back((uint16_t)(DISTANCE(xVal, yVal,
                     points[0].first, points[0].second)));
      }
   }
   // For testing only
   if (possibleXCentre.size() > 0) {
      numBalls = 1;
      ballCentre.first = findCentreCluster(possibleXCentre);
      ballCentre.second = findCentreCluster(possibleYCentre);
      radius = findCentreCluster(possibleRadius);
   }
}

inline uint16_t BallDetection::findCentreCluster(
      std::vector<uint16_t> points) {
   sort(points.begin(), points.end());
   uint16_t medianIndex;
   uint16_t first = 0;
   uint16_t last = points.size() - 1;
   for (int p = 0; p < 15; p++) {
      medianIndex = (first + last) >> 1;
      if (points[medianIndex] - points[first] > points[last] -
            points[medianIndex]) {
         first++;
      } else {
         last--;
      }
   }
   medianIndex = (first + last) >> 1;
   return points[medianIndex];
}

inline bool BallDetection::getThreeUniquePoints(unsigned int *seed,
      std::pair<uint16_t, uint16_t> *points) {
   points[0] = ballEdgePoints[rand_r(seed) % numBallEdgePoints];
   int numAttempts = 0;
   points[1].first = points[0].first;
   points[1].second = points[0].second;
   points[2].first = points[0].first;
   points[2].second = points[0].second;
   while (points[0].first == points[1].first &&
         points[0].second == points[1].second &&
         numAttempts < 100) {
      points[1] = ballEdgePoints[rand_r(seed) % numBallEdgePoints];
      numAttempts++;
   }
   while (((points[2].first == points[0].first &&
            points[2].second == points[0].second) ||
         (points[2].first == points[1].first &&
          points[2].second == points[1].second)) &&
         numAttempts < 100) {
      points[2] = ballEdgePoints[rand_r(seed) % numBallEdgePoints];
      numAttempts++;
   }
   if (numAttempts == 100) {
      return false;
   } else {
      return true;
   }
}

inline TwoParamLine BallDetection::findEquationOfLine(
      std::pair<uint16_t, uint16_t> p1, std::pair<uint16_t, uint16_t> p2) {
   TwoParamLine line;
   float midX = (p1.first + p2.first) / 2.0f;
   float midY = (p1.second + p2.second) / 2.0f;
   if (p2.second - p1.second == 0) {
      // Special case where the line to be returned is vertical.
      // In this case, write NAN to the gradient, and the x intercpet
      // to line.b
      line.m = 1/0.0f;
      line.b = midX;
   } else {
      // formula for line perpendicular bisector of the line is
      // grad = -1/line_gradient
      line.m = (p1.first - p2.first) / (float)(p2.second - p1.second);
      line.b = midY - line.m * midX;
   }
   return line;
}
/**
 * to complete:
 * - on the large scans, start scanning a little further back from the edge if can so 
 *   hopefully not miss edges if relfections from the ball cause areas very close to it
 *   to be classified as orange
 * - Implement a better algorithm for finding the centre and radius of the ball - i think
 *   first sort, then find the median. Remove the point furthest away, get new median, and
 *   repeat a few times. The final median should be free from the influnece of outliers.
 * - Can also have variances be dependent on how distributed the centre points are
 **/

/**
 * For the new ball detection code:
 * maybe want to pass in the robot information so can clip the possible robots
 * against the balls (should this be done here, or in region analyser?
 * Also want better sanity checks, as the current take the maximum sized
 * region and nothing else is pretty basic
 **/
/**
 * Old ball detection code:
 * Scan through saliency image. When see an orange pixels, scan horizontally
 * until see no ball, both directions. Take the midpoint of this line, then
 * scan vertically. Take the midpoint of this line. The greater of the two
 * dimensions is the diamter of the ball, with the centre of the ball being
 * at the two midpoints. Sanity checks are:
 * - >= minimum ball radius (full res image). Min radius was 4
 * - > minimum contain ratio - where a square of sides 0.70711 * radius was
 * centered on the circle, and all pixels inside this square were examined.
 * Minimum contain ratio was 0.5
 * - maxY - minY < 4 * (maxX - minX) - mainly to stop a line of ball colours
 * up the side of the goals from being detected.
 * The first ball to satify these criteria was written to the blackboard
 **/
