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

#include "perception/vision/FieldLineDetection.hpp"
#include "utils/log.hpp"

using namespace std;

void FieldLineDetection::processFieldLines(ImageRegion **lineRegions,
      uint16_t numLineRegions, CameraToRR *convRR) {
   numFieldLinePoints = 0;
   llog(VERBOSE) << "number of line regions is  " << numLineRegions << endl;
   int numPoints = 0;
   for (uint16_t i = 0; i < numLineRegions; i++) {
      numPoints += lineRegions[i]->numScanPoints;
   }
   // cout << "numPoints is " << numPoints << endl;
   int addVal = numPoints/(MAX_FIELD_LINE_POINTS / 2.0);
   if (addVal < 1) {
      addVal = 1;
   }
   // cout << "addVal is " << addVal << endl;
   int counter = 0;
   for (uint16_t i = 0; i < numLineRegions && numFieldLinePoints <
         MAX_FIELD_LINE_POINTS - 1; i++) {
      for (uint16_t j = 0; j < lineRegions[i]->numScanPoints &&
            numFieldLinePoints < MAX_FIELD_LINE_POINTS - 1; j++, counter++) {
         /*if (lineRegions[i]->startScans[j].second !=
               lineRegions[i]->endScans[j].second) {*/
         if (counter % addVal == 0) {
            fieldLinePoints[numFieldLinePoints] =
               convRR->convertToRR(
                  lineRegions[i]->startScans[j].first * SALIENCY_DENSITY,
                  lineRegions[i]->startScans[j].second * SALIENCY_DENSITY,
                  false);
            numFieldLinePoints++;
            fieldLinePoints[numFieldLinePoints] =
               convRR->convertToRR(
                  lineRegions[i]->endScans[j].first * SALIENCY_DENSITY,
                  lineRegions[i]->endScans[j].second * SALIENCY_DENSITY,
                  false);
            numFieldLinePoints++;
         }
      }
   }
}
