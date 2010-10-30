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


#include "perception/vision/RegionBuilder.hpp"
#include "utils/log.hpp"

using namespace std;

RegionBuilder::RegionBuilder() {
}

RegionBuilder::~RegionBuilder() {
}

void RegionBuilder::buildRegions(Colour saliency
      [IMAGE_COLS/SALIENCY_DENSITY][IMAGE_ROWS/SALIENCY_DENSITY],
      uint32_t *startOfScan, int *endOfScan) {
   lastColRegions = holder1;
   curColRegions = holder2;

   uint16_t rows = IMAGE_ROWS/SALIENCY_DENSITY;
   uint16_t cols = IMAGE_COLS/SALIENCY_DENSITY;
   ImageRegion **temp;

   /**
    * If its red or blue or white, one region
    * If its orange, another region
    * Keep track of number of red, number of white
    * Build in the one big region list, then filter out
    * First identify the likely candidates for robots, and filter
    * out the balls completely inside the robots, then can go further
    * and look at the red regions that cannot be robots if no good ball
    * if found - could be quite flexible, and still allow for the ball 
    * to be found when touching a red robot. Possible question of when
    * the ball is behind the foot of a robot - probably better to look
    * at later.
    **/
   /**
    * Big question is whether I should scan the saliency image or scan the
    * full res image skipping lines
    **/
   numLastColRegions = 0;
   numRegions = 0;
   for (int i = 0; i < cols; i++) {
      numCurColRegions = 0;
      lastColCounter = 0;
      uint16_t lastSeen = 0;
      bool seenOrange = false;
      bool seenRegion = false;
      uint16_t startScan = 0;
      uint16_t numRRed = 0;
      uint16_t numRBlue = 0;
      uint16_t numWhite = 0;
      uint16_t numOrange = 0;
      uint16_t numGreen = 0;
      uint16_t bottomMostRobot = 0;
      uint16_t topMostRobot = IMAGE_ROWS;

      bool seenGreen = false;
      bool first = true;
      for (int j = startOfScan[i]; j < endOfScan[i] &&
            numRegions < MAX_NUM_REGIONS; j++) {
         if (saliency[i][j] == cGOAL_YELLOW) {
            first = false;
         }
         if (saliency[i][j] == cFIELD_GREEN) {
            if (first && !seenGreen && j - startOfScan[i] > 10) {
               first = false;
               addScanToRegions(startOfScan[i], j - 1, 0, 0, 0,
                     0, i, startOfScan[i], bottomMostRobot, topMostRobot);
            }
            seenGreen = true;
         }
         if (saliency[i][j] == cBALL) {
            if (seenOrange) {
               numOrange++;
            } else {
               if (seenRegion) {
                  seenRegion = false;
                  addScanToRegions(startScan, lastSeen, 0, numRRed,
                        numRBlue, numWhite, i, startOfScan[i], bottomMostRobot,
                        topMostRobot);
               }
               seenOrange = true;
               startScan = j;
               numOrange = 1;
               first = false;
            }
            lastSeen = j;
         } else if (saliency[i][j] == cWHITE ||
               saliency[i][j] == cROBOT_BLUE ||
               saliency[i][j] == cROBOT_RED) {
            if (seenRegion) {
            } else {
               if (seenOrange) {
                  seenOrange = false;
                  addScanToRegions(startScan, lastSeen, numOrange, 0, 0, 0, i,
                        startOfScan[i], bottomMostRobot, topMostRobot);
               }
               seenRegion = true;
               startScan = j;
               numWhite = 0;
               numRRed = 0;
               numRBlue = 0;
               numOrange = 0;
               numGreen = 0;
               if (first && !seenGreen) {
                  startScan = startOfScan[i];
               }
               first = false;
            }
            lastSeen = j;
            if (saliency[i][j] == cWHITE) {
               numWhite++;
            } else if (saliency[i][j] == cROBOT_BLUE) {
               if (topMostRobot == IMAGE_ROWS) {
                  topMostRobot = j;
               }
               bottomMostRobot = j;
               numRBlue++;
            } else {
               if (topMostRobot == IMAGE_ROWS) {
                  topMostRobot = j;
               }
               bottomMostRobot = j;
               numRRed++;
            }
         } else if (saliency[i][j] != cBALL && seenOrange &&
               j - lastSeen > BALL_CLASS_THRESH) {
            seenOrange = false;
            addScanToRegions(startScan, lastSeen, numOrange, 0, 0, 0, i,
                  startOfScan[i], bottomMostRobot, topMostRobot);
         } else if (saliency[i][j] == cFIELD_GREEN && seenRegion &&
               numGreen >= REGION_CLASS_THRESH) {
            seenRegion = false;
            if (numWhite == 0 && numRRed == 0 && numRBlue > 0 && startScan
                  == startOfScan[i]) {
            } else {
               addScanToRegions(startScan, lastSeen, 0, numRRed, numRBlue,
                     numWhite, i, startOfScan[i], bottomMostRobot,
                     topMostRobot);
            }
         } else if (saliency[i][j] == cFIELD_GREEN && seenRegion) {
            numGreen++;
         }

         if (j == rows - 1) {
            if (seenOrange) {
               addScanToRegions(startScan, lastSeen, numOrange, 0, 0, 0, i,
                     startOfScan[i], bottomMostRobot, topMostRobot);
            } else if (seenRegion) {
               addScanToRegions(startScan, lastSeen, 0, numRRed, numRBlue,
                   numWhite, i, startOfScan[i], bottomMostRobot, topMostRobot);
            }
         }
      }
      numLastColRegions = numCurColRegions;
      temp = lastColRegions;
      lastColRegions = curColRegions;
      curColRegions = temp;
   }
}

inline void RegionBuilder::addScanToRegions(uint16_t startScan,
      uint16_t endScan, uint16_t numOrange, uint16_t numRRed,
      uint16_t numRBlue, uint16_t numWhite, uint16_t i, uint16_t fieldEdge,
      uint16_t bottomMostRobot, uint16_t topMostRobot) {
   if (endScan - startScan < MINIMUM_SCAN_LENGTH) {
      return;
   }
   bool found = false;
   ImageRegion *region = 0;
   ImageRegion *temp;
   while (lastColCounter < numLastColRegions &&
         lastColRegions[lastColCounter]->prevStart <= endScan) {
      if (lastColRegions[lastColCounter]->prevEnd >= startScan &&
            ((lastColRegions[lastColCounter]->numOrange == 0 && numOrange == 0)
         || (lastColRegions[lastColCounter]->numOrange != 0 && numOrange != 0))
            ) {
         if (numRRed > 0 && lastColRegions[lastColCounter]->numRobotRed == 0 &&
               i - lastColRegions[lastColCounter]->leftMost > REGION_JOIN_GAP
               && startScan > fieldEdge) {
         } else if (lastColRegions[lastColCounter]->numRobotRed > 0 &&
               numRRed == 0 && startScan > fieldEdge &&
               i - lastColRegions[lastColCounter]->lastNotAllWhite >
               REGION_JOIN_GAP) {
         } else if (numRRed > 0 && lastColRegions[lastColCounter]->numRobotRed
               == 0 && i - lastColRegions[lastColCounter]->leftMost >
               REGION_JOIN_GAP_ON_FIELD_EDGE && startScan <= fieldEdge) {
         } else if (lastColRegions[lastColCounter]->numRobotRed > 0 &&
               numRRed == 0 &&
               i - lastColRegions[lastColCounter]->lastNotAllWhite >
               REGION_JOIN_GAP_ON_FIELD_EDGE && startScan <= fieldEdge) {
         } else if (numOrange == 0 && (endScan - startScan + 1) >
               (lastColRegions[lastColCounter]->averageScanLength /
               (i -lastColRegions[lastColCounter]->leftMost)) *
               AVERAGE_SCAN_LENGTH_THRESH &&
               i - lastColRegions[lastColCounter]->leftMost > 2) {
         } else if (numOrange == 0 &&
               (endScan - startScan + 1) * AVERAGE_SCAN_LENGTH_THRESH <
               (lastColRegions[lastColCounter]->averageScanLength /
               (i - lastColRegions[lastColCounter]->leftMost)) &&
               i - lastColRegions[lastColCounter]->leftMost > 2) {
         } else if (!found) {
            found = true;
            region = lastColRegions[lastColCounter];
         } else {
            if (region->leftMost < lastColRegions[lastColCounter]->leftMost) {
               temp = lastColRegions[lastColCounter];
               lastColRegions[lastColCounter] = region;
            } else {
               temp = region;
               region = lastColRegions[lastColCounter];
            }
            region->numPixels += temp->numPixels;
            region->numRobotRed += temp->numRobotRed;
            region->numRobotBlue += temp->numRobotBlue;
            region->numWhite += temp->numWhite;
            region->numOrange += temp->numOrange;
            region->numTop += temp->numTop;
            uint16_t index = region->numScanPoints;
            for (uint16_t p = 0; p < temp->numScanPoints && p + index <
                  MAX_NUM_SCAN_POINTS; p++) {
               region->startScans[p + index] = temp->startScans[p];
               region->endScans[p + index] = temp->endScans[p];
               region->numScanPoints++;
            }
            if (temp->leftMost < region->leftMost) {
               region->leftMost = temp->leftMost;
            }
            if (temp->rightMost > region->rightMost) {
               region->rightMost = temp->rightMost;
            }
            if (temp->topMost.second < region->topMost.second) {
               region->topMost = temp->topMost;
            }
            if (temp->bottomMost.second > region->bottomMost.second) {
               region->bottomMost = temp->bottomMost;
            }
            if (temp->bottomMostRobot > region->bottomMostRobot) {
               region->bottomMostRobot = temp->bottomMostRobot;
            }
            if (temp->topMostRobot < region->topMostRobot) {
               region->topMostRobot = temp->topMostRobot;
            }
            if (temp->leftMostRobot < region->leftMostRobot) {
               region->leftMostRobot = temp->leftMostRobot;
            }
            if (temp->rightMostRobot > region->rightMostRobot) {
               region->rightMostRobot = temp->rightMostRobot;
            }
            if (temp->lastNotAllWhite > region->lastNotAllWhite) {
               region->lastNotAllWhite = temp->lastNotAllWhite;
            }
            region->averageScanLength += temp->averageScanLength;
            temp->deleted = true;
         }
      }
      lastColCounter++;
   }
   if (lastColCounter > 0) {
      lastColCounter--;
   }
   if (!found) {
      regions[numRegions].clear();
      regions[numRegions].leftMost = i;
      region = &(regions[numRegions]);
      numRegions++;
   }
   if (numRRed > 0 || numRBlue > 0) {
      region->rightMostRobot = i;
      if (region->numRobotRed == 0 && region->numRobotBlue == 0) {
         region->leftMostRobot = i;
      }
      if (bottomMostRobot > region->bottomMostRobot) {
         region->bottomMostRobot = bottomMostRobot;
      }
      if (topMostRobot < region->topMostRobot) {
         region->topMostRobot = topMostRobot;
      }
   }
   region->numPixels += endScan - startScan + 1;
   region->numRobotRed += numRRed;
   region->numRobotBlue += numRBlue;
   region->numWhite += numWhite;
   region->numOrange += numOrange;
   if (fieldEdge >= startScan) {
      region->numTop++;
   }
   region->rightMost = i;
   uint16_t points = region->numScanPoints;
   if (points < MAX_NUM_SCAN_POINTS - 1) {
      region->startScans[points].first = i;
      region->endScans[points].first = i;
      region->startScans[points].second = startScan;
      region->endScans[points].second = endScan;
      region->numScanPoints++;
   }
   if (region->topMost.second > startScan) {
      region->topMost.second = startScan;
      region->topMost.first = i;
   }
   if (region->bottomMost.second < endScan) {
      region->bottomMost.second = endScan;
      region->bottomMost.first = i;
   }
   if (endScan - startScan + 1 != numWhite) {
      region->lastNotAllWhite = i;
   }
   region->averageScanLength += endScan - startScan + 1;
   region->prevStart = startScan;
   region->prevEnd = endScan;

   // Add the region to the curColRegions array
   curColRegions[numCurColRegions] = region;
   numCurColRegions++;
}

void RegionBuilder::analyseRegions(uint32_t *startOfScan) {
   numPossibleBallRegions = 0;
   numLineRegions = 0;
   numRobotRegions = 0;
   currentRobot = NULL;
   // This section assumes that the regions array is sorted in order of
   // ascending left most values. The region building code does this
   // automatically
   for (uint32_t p = 0; p < numRegions; ++p) {
      if (regions[p].deleted) {
      } else if (regions[p].numOrange > 0) {
         ballRegions[numPossibleBallRegions] = &(regions[p]);
         numPossibleBallRegions++;
         regions[p].classification = rBALL;
      } else {
         if (currentRobot != NULL && currentRobot->rightMost + 1
               < regions[p].leftMost) {
            currentRobot = NULL;
         }
         if (currentRobot == NULL) {
            if (regions[p].numRobotRed > 0) {
               newRobotRegion(RED_ROBOT, &(regions[p]));
               regions[p].classification = rROBOT;
            } else if (regions[p].numRobotBlue > 0) {
               newRobotRegion(BLUE_ROBOT, &(regions[p]));
               regions[p].classification = rROBOT;
            } else if (regions[p].numTop > 0) {
               if (regions[p].bottomMost.second - regions[p].topMost.second >
                   3) {
                  if (regions[p].numTop * 3 <
                        regions[p].rightMost - regions[p].leftMost) {
                     regions[p].classification = rFIELD_LINE;
                  } else if ((regions[p].averageScanLength /
                        regions[p].numScanPoints) * 3 <
                        regions[p].bottomMost.second -
                        regions[p].topMost.second) {
                     regions[p].classification = rFIELD_LINE;
                  } else {
                     newRobotRegion(UNKNOWN_ROBOT, &(regions[p]));
                     regions[p].classification = rMAYBE_ROBOT;
                  }
               } else {
                  regions[p].classification = rDELETED;
               }
            } else {
               if ((regions[p].rightMost >= IMAGE_COLS/SALIENCY_DENSITY - 1 ||
                       regions[p].leftMost == 0) &&
                  regions[p].rightMost - regions[p].leftMost <
                  regions[p].bottomMost.second - regions[p].topMost.second &&
                  (regions[p].averageScanLength / regions[p].numScanPoints) * 3
                  > regions[p].bottomMost.second - regions[p].topMost.second) {
                  regions[p].classification = rDELETED;
               } else {
                  regions[p].classification = rFIELD_LINE;
               }
            }
         } else {
            if (regions[p].numRobotRed > 0 && regions[p].numWhite < 2 &&
                  currentRobot->bottomMost < regions[p].topMost.second) {
               // Stop any robot red in the ball from being joined to a region
               regions[p].classification = rDELETED;
            } else if (regions[p].numRobotRed > 0 && currentRobot->type ==
                  BLUE_ROBOT) {
               regions[p].classification = rROBOT;
               newRobotRegion(RED_ROBOT, &(regions[p]));
            } else if (regions[p].numRobotBlue > 0 && currentRobot->type ==
                  RED_ROBOT) {
               regions[p].classification = rROBOT;
               newRobotRegion(BLUE_ROBOT, &(regions[p]));
            } else if (regions[p].numRobotRed > 0 && regions[p].topMost.second
                  > currentRobot->bottomMost + 8) {
               regions[p].classification = rDELETED;
            } else if (regions[p].numRobotRed > 0) {
               regions[p].classification = rCONNECTED_ROBOT;
               addToRobotRegion(&(regions[p]));
            } else if (regions[p].numRobotBlue > 0) {
               regions[p].classification = rCONNECTED_ROBOT;
               addToRobotRegion(&(regions[p]));
            } else if (regions[p].numTop > 0 && (regions[p].rightMost -
                  regions[p].leftMost < regions[p].bottomMost.second -
                  regions[p].topMost.second || regions[p].numTop * 1.5 >
                  regions[p].rightMost - regions[p].leftMost) &&
                  regions[p].bottomMost.second >= currentRobot->topMost) {
               addToRobotRegion(&(regions[p]));
               regions[p].classification = rCONNECTED_ROBOT;
            } else if (regions[p].topMost.second < currentRobot->bottomMost &&
                  regions[p].bottomMost.second >= currentRobot->topMost) {
               if (regions[p].rightMost < currentRobot->rightMost) {
                  addToRobotRegion(&(regions[p]));
                  regions[p].classification = rCONNECTED_ROBOT;
               } else if (regions[p].rightMost - regions[p].leftMost <
                    regions[p].bottomMost.second - regions[p].topMost.second) {
                  if (regions[p].bottomMost.second > currentRobot->bottomMost
                        + 10 && (regions[p].averageScanLength /
                        regions[p].numScanPoints) * 1.5 <
                        regions[p].bottomMost.second -
                        regions[p].topMost.second) {
                     regions[p].classification = rFIELD_LINE;
                  } else {
                     addToRobotRegion(&(regions[p]));
                     regions[p].classification = rCONNECTED_ROBOT;
                  }
               } else {
                  regions[p].classification = rFIELD_LINE;
               }
            } else if (regions[p].numTop > 0 && regions[p].bottomMost.second -
                  regions[p].topMost.second < 10) {
               regions[p].classification = rDELETED;
            } else {
               regions[p].classification = rFIELD_LINE;
            }
         }
      }
   }
   int count = numRobotRegions - 1;
   // Go backwards through the list of regions to join regions previously
   // classified as lines to robots
   for (int p = numRegions - 1; p >= 0; --p) {
      if (count < 0) {
         if (regions[p].classification == rFIELD_LINE) {
            lineRegions[numLineRegions] = &(regions[p]);
            numLineRegions++;
         }
      } else if (regions[p].classification == rFIELD_LINE &&
            regions[p].topMost.second < robotRegions[count].bottomMost &&
            (regions[p].rightMost <= robotRegions[count].rightMost ||
             (regions[p].rightMost > robotRegions[count].rightMost &&
              regions[p].leftMost <= robotRegions[count].leftMost))) {
         if (count > 0 && regions[p].leftMost <
               robotRegions[count - 1].rightMost) {
            count--;
            p++;
         } else if (regions[p].rightMost + 1 >= robotRegions[count].leftMost &&
               regions[p].rightMost - regions[p].leftMost <
               regions[p].bottomMost.second - regions[p].topMost.second &&
               regions[p].bottomMost.second >= robotRegions[count].topMost) {
            if (regions[p].numTop > 0) {
               currentRobot = &(robotRegions[count]);
               addToRobotRegion(&(regions[p]));
               regions[p].classification = rCONNECTED_ROBOT;
            } else if (regions[p].bottomMost.second >
                        robotRegions[count].bottomMost
                        + 10 && (regions[p].averageScanLength /
                        regions[p].numScanPoints) * 1.5 <
                        regions[p].bottomMost.second -
                        regions[p].topMost.second) {
               lineRegions[numLineRegions] = &(regions[p]);
               numLineRegions++;
            } else {
               currentRobot = &(robotRegions[count]);
               addToRobotRegion(&(regions[p]));
               regions[p].classification = rCONNECTED_ROBOT;
            }
         } else {
            lineRegions[numLineRegions] = &(regions[p]);
            numLineRegions++;
         }
      } else if (regions[p].classification == rFIELD_LINE) {
         lineRegions[numLineRegions] = &(regions[p]);
         numLineRegions++;
      }
   }
   // Go through the robot regions detected so far to see if any of
   // them look like lines that have been miss-identified as robots
   for (int p = 0; p < numRobotRegions; p++) {
      if (robotRegions[p].numRobotBlue > 0 ||
            robotRegions[p].numRobotRed > 0) {
      } else if (p > 0 && robotRegions[p - 1].rightMost + 1
            >= robotRegions[p].leftMost) {
      } else if (p + 1 < numRobotRegions && robotRegions[p].rightMost + 1 >=
            robotRegions[p + 1].leftMost) {
      } else if (robotRegions[p].bottomMost >= IMAGE_ROWS/SALIENCY_DENSITY - 1
            && robotRegions[p].rightMost - robotRegions[p].leftMost < 50 &&
            robotRegions[p].leftMost > 0 && robotRegions[p].rightMost <
            IMAGE_COLS/SALIENCY_DENSITY - 2) {
         deleteRobotRegion(p);
      }
   }
}

inline void RegionBuilder::newRobotRegion(RobotType type,
      ImageRegion *region) {
   robotRegions[numRobotRegions].type = type;
   robotRegions[numRobotRegions].leftMost = region->leftMost;
   robotRegions[numRobotRegions].rightMost = region->rightMost;
   robotRegions[numRobotRegions].bottomMost = region->bottomMost.second;
   robotRegions[numRobotRegions].topMost = region->topMost.second;
   robotRegions[numRobotRegions].regions[0] = region;
   robotRegions[numRobotRegions].numRegions = 1;
   robotRegions[numRobotRegions].deleted = false;
   robotRegions[numRobotRegions].numTop = region->numTop;
   robotRegions[numRobotRegions].numRobotBlue = region->numRobotBlue;
   robotRegions[numRobotRegions].numRobotRed = region->numRobotRed;
   robotRegions[numRobotRegions].numWhite = region->numWhite;
   robotRegions[numRobotRegions].bottomMostRobot = region->bottomMostRobot;
   robotRegions[numRobotRegions].topMostRobot = region->topMostRobot;
   robotRegions[numRobotRegions].rightMostRobot = region->rightMostRobot;
   robotRegions[numRobotRegions].leftMostRobot = region->leftMostRobot;
   currentRobot = &(robotRegions[numRobotRegions]);
   numRobotRegions++;
}

inline void RegionBuilder::addToRobotRegion(ImageRegion *region) {
   if (region->leftMost < currentRobot->leftMost) {
      currentRobot->leftMost = region->leftMost;
   }
   if (region->rightMost > currentRobot->rightMost) {
      currentRobot->rightMost = region->rightMost;
   }
   if (region->bottomMost.second > currentRobot->bottomMost) {
      currentRobot->bottomMost = region->bottomMost.second;
   }
   if (region->topMost.second < currentRobot->topMost) {
      currentRobot->topMost = region->topMost.second;
   }
   if (currentRobot->type == UNKNOWN_ROBOT) {
      if (region->numRobotRed > 0) {
         currentRobot->type = RED_ROBOT;
      } else {
         currentRobot->type = BLUE_ROBOT;
      }
   }
   if (currentRobot->numRegions < MAX_NUM_ROBOT_REGIONS) {
      currentRobot->regions[currentRobot->numRegions] = region;
      currentRobot->numRegions++;
   }
   if (currentRobot->bottomMostRobot < region->bottomMostRobot) {
      currentRobot->bottomMostRobot = region->bottomMostRobot;
   }
   if (currentRobot->topMostRobot > region->topMostRobot) {
      currentRobot->topMostRobot = region->topMostRobot;
   }
   if (currentRobot->leftMostRobot > region->leftMostRobot) {
      currentRobot->leftMostRobot = region->leftMostRobot;
   }
   if (currentRobot->rightMostRobot < region->rightMostRobot) {
      currentRobot->rightMostRobot = region->rightMostRobot;
   }
   currentRobot->numTop += region->numTop;
   currentRobot->numRobotRed += region->numRobotRed;
   currentRobot->numRobotBlue += region->numRobotBlue;
   currentRobot->numWhite += region->numWhite;
}

inline void RegionBuilder::deleteRobotRegion(int index) {
   robotRegions[index].deleted = true;
   for (int i = 0; i < robotRegions[index].numRegions; i++) {
      robotRegions[index].regions[i]->classification = rFIELD_LINE;
   }
}


