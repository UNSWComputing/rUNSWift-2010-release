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


#include "perception/vision/RobotDetection.hpp"
#include "utils/RRCoord.hpp"
#include "utils/basic_maths.hpp"
#include "utils/log.hpp"

using namespace std;

void RobotDetection::findRobots(RobotRegion *robotRegions,
      uint16_t numRobotRegions, uint32_t *startOfScan, Colour saliency
      [IMAGE_COLS/SALIENCY_DENSITY][IMAGE_ROWS/SALIENCY_DENSITY]) {
   this->numRobotRegions = 0;
   numRobots = 0;
   numRemovedRobots = 0;
   for (uint16_t i = 0; i < numRobotRegions; i++) {
      if (robotRegions[i].deleted) {
      } else if (startOfScan[robotRegions[i].leftMost] + 1 <
            robotRegions[i].topMost &&
            startOfScan[robotRegions[i].rightMost] + 1 <
            robotRegions[i].topMost) {
      } else if (robotRegions[i].numTop * 3 < robotRegions[i].rightMost -
            robotRegions[i].leftMost) {
         addRobotToRemovedList(&(robotRegions[i]));
      } else if (robotRegions[i].rightMost - robotRegions[i].leftMost <
            MIN_ROBOT_REGION_WIDTH) {
         addRobotToRemovedList(&(robotRegions[i]));
      } else if (robotRegions[i].numWhite < MIN_NUM_WHITE_IN_ROBOT) {
      } else if (robotRegions[i].topMost <= 1 &&
            robotRegions[i].bottomMost >= IMAGE_ROWS/SALIENCY_DENSITY - 1 &&
            robotRegions[i].rightMost - robotRegions[i].leftMost <
            (IMAGE_COLS/SALIENCY_DENSITY)/3 &&
            robotRegions[i].leftMost > 1 && robotRegions[i].rightMost <
            IMAGE_COLS/SALIENCY_DENSITY - 2) {
      } else {
         findRobotType(&robotRegions[i], saliency, startOfScan);
         addRobotToList(&(robotRegions[i]));
      }
   }
   int16_t j = 0;
   for (int16_t i = 0; i < numRemovedRobots &&
         j < this->numRobotRegions; i++) {
      if (removedRobots[i]->leftMost > this->robotRegions[j]->rightMost + 1) {
         j++;
         i--;
      } else {
         if (removedRobots[i]->rightMost + 1 >=
               this->robotRegions[j]->leftMost &&
               removedRobots[i]->leftMost < this->robotRegions[j]->rightMost) {
            combineRobots(this->robotRegions[j], removedRobots[i]);
         }
         if (removedRobots[i]->leftMost <=
               this->robotRegions[j]->rightMost + 1 &&
               removedRobots[i]->rightMost > this->robotRegions[j]->leftMost) {
            combineRobots(this->robotRegions[j], removedRobots[i]);
         }
      }
   }
}

void RobotDetection::addRobotToList(RobotRegion *newRobot) {
   bool combined = false;
   if (numRobotRegions > 0) {
      uint16_t index = numRobotRegions - 1;
      if (robotRegions[index]->rightMost + 3 >= newRobot->leftMost &&
            robotRegions[index]->type == newRobot->type &&
            newRobot->type != UNKNOWN_ROBOT) {
         combined = true;
         combineRobots(robotRegions[index], newRobot);
      }
   }
   if (!combined && newRobot->type != UNKNOWN_ROBOT) {
      robotRegions[numRobotRegions] = newRobot;
      numRobotRegions++;
   }
}

void RobotDetection::combineRobots(RobotRegion *current,
      RobotRegion *newRobot) {
   if (newRobot->rightMost > current->rightMost) {
      current->rightMost = newRobot->rightMost;
   }
   if (current->topMost > newRobot->topMost) {
      current->topMost = newRobot->topMost;
   }
   if (current->bottomMost < newRobot->bottomMost) {
      current->bottomMost = newRobot->bottomMost;
   }
   if (current->leftMost > newRobot->leftMost) {
      current->leftMost = newRobot->leftMost;
   }
   current->numTop += newRobot->numTop;
   for (uint16_t x = 0; x < newRobot->numRegions &&
         current->numRegions + x < MAX_NUM_ROBOT_REGIONS; x++) {
      current->regions[current->numRegions + x] =
         newRobot->regions[x];
   }
   current->numRegions = newRobot->numRegions;
   current->numRobotRed += newRobot->numRobotRed;
   current->numRobotBlue += newRobot->numRobotBlue;
   current->numWhite += newRobot->numWhite;
   if (current->rightMostRobot < newRobot->rightMostRobot) {
      current->rightMostRobot = newRobot->rightMostRobot;
   }
   if (current->leftMostRobot > newRobot->leftMostRobot) {
      current->leftMostRobot = newRobot->leftMostRobot;
   }
   if (current->bottomMostRobot <
         newRobot->bottomMostRobot) {
      current->bottomMostRobot = newRobot->bottomMostRobot;
   }
   if (current->topMostRobot > newRobot->topMostRobot) {
      current->topMostRobot = newRobot->topMostRobot;
   }
}

void RobotDetection::addRobotToRemovedList(RobotRegion *robot) {
   if (numRemovedRobots < MAX_NUM_ROBOTS) {
      removedRobots[numRemovedRobots] = robot;
      numRemovedRobots++;
   }
}

void RobotDetection::sanityCheckRobots(uint16_t numGoalPosts,
      uint16_t *postCoords, CameraToRR *convRR, ImageRegion *bestBallRegion,
      uint16_t radius, std::pair<uint16_t, uint16_t> ballCentre) {
   for (int i = 0; i < numRobotRegions &&
         numRobots < MAX_NUM_ROBOTS; i++) {
      if (robotRegions[i]->type == BLUE_ROBOT &&
            robotRegions[i]->numRobotBlue > 0 && isColourInGoals(
               numGoalPosts, postCoords, robotRegions[i])) {
         robotRegions[i]->type = UNKNOWN_ROBOT;
      } else if (robotRegions[i]->leftMostRobot < IMAGE_COLS &&
            (robotRegions[i]->rightMostRobot -
               robotRegions[i]->leftMost) * 5 <
            robotRegions[i]->rightMost - robotRegions[i]->leftMost) {
         robotRegions[i]->type = UNKNOWN_ROBOT;
      } else if (robotRegions[i]->leftMostRobot < IMAGE_COLS &&
            (robotRegions[i]->rightMost -
               robotRegions[i]->leftMostRobot) * 5 <
            robotRegions[i]->rightMost - robotRegions[i]->leftMost) {
         robotRegions[i]->type = UNKNOWN_ROBOT;
      } else if (isColourInBall(robotRegions[i], bestBallRegion, radius,
               ballCentre)) {
         robotRegions[i]->type = UNKNOWN_ROBOT;
      } else if (robotRegions[i]->type != UNKNOWN_ROBOT) {
         uint16_t x = ((robotRegions[i]->rightMost +
                  robotRegions[i]->leftMost) / 2) * SALIENCY_DENSITY;
         uint16_t y = robotRegions[i]->bottomMost * SALIENCY_DENSITY;
         robotLocations[numRobots] = convRR->convertToRR(x, y, false);
         if (robotRegions[i]->leftMost <= 1 || robotRegions[i]->rightMost
               >= IMAGE_COLS/SALIENCY_DENSITY - 2) {
            robotLocations[numRobots].var[1] =
               convRR->calculateHeadingVariance(10);
         } else {
            robotLocations[numRobots].var[1] =
               convRR->calculateHeadingVariance(5);
         }
         if (robotRegions[i]->bottomMost >=
               IMAGE_ROWS/SALIENCY_DENSITY - 2) {
            robotLocations[numRobots].var[0] =
               convRR->calculateDistanceVariance(
                     robotLocations[numRobots].distance, 20, 400);
         } else {
            robotLocations[numRobots].var[0] =
               convRR->calculateDistanceVariance(
                     robotLocations[numRobots].distance, 1, 10000);
         }
         if (robotRegions[i]->type == RED_ROBOT) {
            robotTypes[numRobots] = RED_ROBOT;
         } else {
            robotTypes[numRobots] = BLUE_ROBOT;
         }
         if (robotRegions[i]->bottomMost >= IMAGE_ROWS/SALIENCY_DENSITY - 1) {
            canSeeBottomRobot[numRobots] = false;
         } else {
            canSeeBottomRobot[numRobots] = true;
         }
         robotImageCoords[numRobots * 4] = robotRegions[i]->topMost;
         robotImageCoords[numRobots * 4 + 1] = robotRegions[i]->rightMost;
         robotImageCoords[numRobots * 4 + 2] = robotRegions[i]->bottomMost;
         robotImageCoords[numRobots * 4 + 3] = robotRegions[i]->leftMost;
         ++numRobots;
      }
   }
}

bool RobotDetection::isColourInBall(RobotRegion *robotRegion,
      ImageRegion *bestBallRegion, uint16_t radius,
      std::pair<uint16_t, uint16_t> ballCentre) {
   int leftRobot = robotRegion->leftMostRobot * SALIENCY_DENSITY;
   int rightRobot = robotRegion->rightMostRobot * SALIENCY_DENSITY;
   int topRobot = robotRegion->topMostRobot * SALIENCY_DENSITY;
   int bottomRobot = robotRegion->bottomMostRobot * SALIENCY_DENSITY;
   if (bestBallRegion != NULL && radius > 25 &&
         bestBallRegion->numPixels > robotRegion->numRobotRed) {
      int leftBall = ballCentre.first - radius;
      int rightBall = ballCentre.first + radius;
      int topBall = ballCentre.second - radius;
      int bottomBall = ballCentre.second + radius;
      if (leftRobot > leftBall - 10 && rightRobot < rightBall + 10 &&
            topRobot > topBall - 10 && bottomRobot < bottomBall + 10) {
         return true;
      } else {
         return false;
      }
   } else {
      return false;
   }
}

inline bool RobotDetection::isColourInGoals(uint16_t numGoalPosts,
      uint16_t *postCoords, RobotRegion *robotRegion) {
   for (int i = 0; i < numGoalPosts; i++) {
      if (robotRegion->bottomMostRobot * SALIENCY_DENSITY <=
            postCoords[i * 4 + 3] + 16 && robotRegion->leftMostRobot *
            SALIENCY_DENSITY + 16 >= postCoords[i * 4] &&
            robotRegion->rightMostRobot * SALIENCY_DENSITY <=
            postCoords[i * 4 + 2] + 16) {
         return true;
      } else if (postCoords[i * 4 + 3] - postCoords[i * 4 + 1] > IMAGE_ROWS/2
            && robotRegion->bottomMostRobot * SALIENCY_DENSITY <=
            postCoords[i * 4 + 3] + 30 && robotRegion->leftMostRobot *
            SALIENCY_DENSITY + 30 >= postCoords[i * 4] &&
            robotRegion->rightMostRobot * SALIENCY_DENSITY <=
            postCoords[i * 4 + 2] + 30) {
         return true;
      }
   }
   if (robotRegion->bottomMostRobot - robotRegion->topMostRobot >
        (IMAGE_ROWS/SALIENCY_DENSITY)/2 && robotRegion->numRobotBlue > 2000) {
      return true;
   }
   return false;
}

void RobotDetection::findRobotType(RobotRegion *robotRegion, Colour saliency
      [IMAGE_COLS/SALIENCY_DENSITY][IMAGE_ROWS/SALIENCY_DENSITY],
      uint32_t *startOfScan) {
   if (robotRegion->numRobotRed > NUM_ROBOT_COLOURS_REQUIRED_BELOW) {
      robotRegion->type = RED_ROBOT;
   } else if (robotRegion->numRobotBlue > NUM_ROBOT_COLOURS_REQUIRED_BELOW) {
      robotRegion->type = BLUE_ROBOT;
   } else if (robotRegion->numWhite < MIN_NUM_WHITE_IN_ROBOT) {
      robotRegion->type = UNKNOWN_ROBOT;
   } else {
      uint16_t numRobotRed;
      uint16_t numRobotBlue;
      if (robotRegion->topMostRobot <= robotRegion->topMost + 2) {
         numRobotRed = MIN(NUM_ROBOT_COLOURS_REQUIRED_ABOVE - 2,
               (int)robotRegion->numRobotRed);
         numRobotBlue = MIN(NUM_ROBOT_COLOURS_REQUIRED_ABOVE - 2,
               (int)robotRegion->numRobotBlue);
      } else {
         numRobotRed = 0;
         numRobotBlue = 0;
      }
      uint32_t startVal = startOfScan[robotRegion->leftMost];
      if (startOfScan[robotRegion->rightMost] > startVal) {
         startVal = startOfScan[robotRegion->rightMost];
      }
      int endVal = MAX(0, (int)startVal - NUM_ROWS_ABOVE);
      bool found = false;
      for (int j = (int)startVal - 1; j >= endVal; --j) {
         if (j < 0 || j >= IMAGE_ROWS/SALIENCY_DENSITY) {
            llog(ERROR) << "Error in j " << j << endl;
         }
         for (int i = robotRegion->leftMost;
               i <= robotRegion->rightMost; i++) {
            if (i < 0 || i >= IMAGE_COLS/SALIENCY_DENSITY) {
               llog(ERROR) << "Error in i " << i << endl;
            }
            if (saliency[i][j] == cROBOT_RED) {
               if (!found) {
                  found = true;
                  endVal = MAX(j - 2, 0);
               }
               numRobotRed++;
               robotRegion->topMostRobot = j;
            } else if (saliency[i][j] == cROBOT_BLUE) {
               if (!found) {
                  found = true;
                  endVal = MAX(j - 2, 0);
               }
               numRobotBlue++;
               robotRegion->topMostRobot = j;
            }
            if (endVal < 0) {
               endVal = 0;
            }
         }
         if (numRobotRed > NUM_ROBOT_COLOURS_REQUIRED_ABOVE) {
            robotRegion->type = RED_ROBOT;
            return;
         } else if (numRobotBlue > NUM_ROBOT_COLOURS_REQUIRED_ABOVE) {
            robotRegion->type = BLUE_ROBOT;
            return;
         }
      }
      robotRegion->type = UNKNOWN_ROBOT;
   }
}

