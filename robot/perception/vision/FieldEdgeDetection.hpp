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
#include <utility>

#include "perception/vision/VisionConstants.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/ImageToRR.hpp"
#include "utils/Line.hpp"

class FieldEdgeDetection {
   public:
      /**
       * Number of lines found by RANSAC
       **/
      uint8_t numEdgeLines;
      /**
       * Parametised forms of up to 2 field edge lines
       **/
      Line edgeLines[MAX_FIELD_EDGES];
      /**
       * Parametised forms of up to 2 field edge lines that have been
       * transformed to robot relative coordinates
       **/
      Line RRedgeLines[MAX_FIELD_EDGES];
      /**
       * Y-Values of field edge points, index is X-Value
       * If there is no field, value is -1
      **/
      std::vector<std::pair<uint16_t, uint16_t> > edgePoints;
      /**
       * The coordinates of the top of the field in the image
       * The coordinates are given relative to the saliency scan,
       * not the full image
       **/
      uint32_t startScanCoords[IMAGE_COLS/SALIENCY_DENSITY];

      /**
       * Find coordinates of points that may be at the edge
       * of the field by using the saliency scan
       * @param saliency a subsampled, colour-classified image
       * @param horizon:(y-value of the horizon line at x = 0,
       *                 y-value of the horizon line at x = IMAGE_COLS-1)
       **/
      void fieldEdgePoints(Colour saliency
            [IMAGE_COLS/SALIENCY_DENSITY][IMAGE_ROWS/SALIENCY_DENSITY],
            const std::pair<int, int> &horizon, int *endOfScan);
      /**
       * Find up to two lines formed by field edge points
       * using the RANSAC algorithm
       **/
      void fieldEdgeLines(unsigned int seed, CameraToRR *convRR);

      /**
       * Fills the startScanCoords array to find the coordinates
       * in the saliency scan where the field starts
       **/
      void findStartScanCoords(Colour saliency
            [IMAGE_COLS/SALIENCY_DENSITY][IMAGE_ROWS/SALIENCY_DENSITY]);

      explicit FieldEdgeDetection();

   private:
      /**
      * Implementation of the RANSAC algorithm for finding a straight line
      * amongst a noisy set of points.
      *
      * @return Whether a line has been found
      * @param points The dataset to look in, vector of (x,y) pairs
      * @param result The chosen line
      * @param con Pointer to the concensus set array
      * @param k Maximum number of iterations of RANSAC
      * @param e Maximum distance a point can be from a line to be in its
      *          concensus set
      * @param n The minimum number of points needed to form a concensus set
      **/
      bool ransacLine(
            const std::vector<std::pair<uint16_t, uint16_t> >& points,
            Line& result, std::pair<uint16_t, uint16_t>& resultp1,
            std::pair<uint16_t, uint16_t>& resultp2, bool **con,
            uint16_t k, float e, uint16_t n, unsigned int seed);

      std::pair<uint16_t, uint16_t> edgeLinePoints[2][2];
};

