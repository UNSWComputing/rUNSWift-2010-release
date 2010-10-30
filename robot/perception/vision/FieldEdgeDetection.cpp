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

#include "perception/vision/FieldEdgeDetection.hpp"

#include <limits>

#include "utils/log.hpp"
#include "utils/basic_maths.hpp"
#include "utils/angles.hpp"

using namespace std;

FieldEdgeDetection::FieldEdgeDetection() {
   // reserve space for the maximum number of field edge points we may have
   // note, this is a lot faster than letting the vector resize itself
   edgePoints.reserve(IMAGE_COLS/SALIENCY_DENSITY);
}

void FieldEdgeDetection::fieldEdgePoints(Colour saliency
      [IMAGE_COLS/SALIENCY_DENSITY][IMAGE_ROWS/SALIENCY_DENSITY],
      const std::pair<int, int> &horizon, int *endOfScan) {
   float gradient = SALIENCY_DENSITY*
      (horizon.second - horizon.first)/IMAGE_COLS;
   int16_t intercept = horizon.first / SALIENCY_DENSITY;
   edgePoints.clear();
   for (uint16_t i = 0; i < IMAGE_COLS/SALIENCY_DENSITY; ++i) {
      const uint16_t hpoint = MAX(0, gradient*i+intercept);
      for (uint16_t j = hpoint; j < endOfScan[i]; ++j) {
         if (saliency[i][j] == cFIELD_GREEN) {
            if (j != 0) {
               edgePoints.push_back(pair<uint16_t, uint16_t>(
                        i*SALIENCY_DENSITY, j*SALIENCY_DENSITY));
            }
            break;
         }
      }
   }
}

void FieldEdgeDetection::fieldEdgeLines(unsigned int seed,
      CameraToRR *convRR) {
   // some magic constants (see RANSAC documentation)
   uint16_t k = 40;
   float e = 8.0;
   uint16_t n = 35;
   numEdgeLines = 0;
   bool *con;
   if (edgePoints.size() > 1 &&
       ransacLine(edgePoints, edgeLines[0], edgeLinePoints[0][0],
         edgeLinePoints[0][1], &con, k, e, n, seed)) {
       llog(DEBUG1) << "line 1 found with params (" <<
         edgeLines[0].t1 << "," << edgeLines[0].t2 << ","
         << edgeLines[0].t3 << ")" << endl;
      // we found a line, let's try and get one more
      ++numEdgeLines;
      vector<pair<uint16_t, uint16_t> > edgePoints2;
      for (uint16_t i = 0; i < edgePoints.size(); ++i) {
         // make sure it isn't part of the first line
         if (con[i] == false) {
            edgePoints2.push_back(edgePoints[i]);
         }
      }
      if (edgePoints2.size() > 1 &&
          ransacLine(edgePoints2, edgeLines[1], edgeLinePoints[1][0],
            edgeLinePoints[1][1], &con, k, e, n, seed)) {
         llog(DEBUG1) << "line 2 found with params (" <<
            edgeLines[0].t1 << "," << edgeLines[0].t2 << ","
            << edgeLines[0].t3 << ")" << endl;
         ++numEdgeLines;
      }
   }
   if (numEdgeLines == 2) {
      // begin sanity checks, sort lines by gradient
      Line *l1;
      Line *l2;
      if ((edgeLines[0].t1/edgeLines[0].t2) <
          (edgeLines[1].t1/edgeLines[1].t2)) {
         l1 = &edgeLines[0];
         l2 = &edgeLines[1];
      } else {
         l1 = &edgeLines[1];
         l2 = &edgeLines[0];
      }

      // sanity check 1, avoid lines that intersect outside the image
      llog(DEBUG2) << "sanity check 1: intersect inside frame" << endl;
      float x_intercept = (l1->t3*l2->t2 - l2->t3*l1->t2)/
         (float)(l1->t2*l2->t1 - l1->t1*l2->t2);
      float y_intercept = (l1->t3*l2->t1 - l2->t3*l1->t1)/
         (float)(l1->t1*l2->t2 - l2->t1*l1->t2);
      if (x_intercept < 0 || x_intercept > IMAGE_COLS ||
            y_intercept < 0 || y_intercept > IMAGE_ROWS) {
         --numEdgeLines;
      }

      if (numEdgeLines == 2) {
         // sanity check 2, avoid lines that are close to being parallel
         llog(DEBUG2) << "sanity check 2: angle between lines" << endl;
         float theta = atan2f(
               (-1*(l1->t1)/(float)(l1->t2) + (l2->t1)/(float)(l2->t2)),
               (1 + (l1->t1)/(float)(l1->t2) * (l2->t1)/(float)(l2->t2)));
         if (-MIN_ANGLE_BETWEEN_EDGE_LINES < theta
               && theta < MIN_ANGLE_BETWEEN_EDGE_LINES) {
            --numEdgeLines;
         }
      }
   }
   llog(DEBUG2) << "robot-relative transformation start" << endl;
   for (k = 0; k < numEdgeLines; k++) {
      // Find two points on the line
      RRCoord p1 = convRR->convertToRR(edgeLinePoints[k][0].first,
            edgeLinePoints[k][0].second, false);
      RRCoord p2 = convRR->convertToRR(edgeLinePoints[k][1].first,
            edgeLinePoints[k][1].second, false);
      int32_t x1 = p1.distance * cos(p1.heading);
      int32_t y1 = p1.distance * sin(p1.heading);
      int32_t x2 = p2.distance * cos(p2.heading);
      int32_t y2 = p2.distance * sin(p2.heading);
      // Find perpendicular distance of this line from the origin
      float dist = abs((x2-x1)*y1 - (y2-y1)*x1)/
         sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
      RRedgeLines[k] = Line(x1, y1, x2, y2);
      RRedgeLines[k].var = SQUARE(1000) + dist + edgeLines[k].var * 1000;
   }
   llog(DEBUG2) << "robot-relative transformation done" << endl;
}

bool FieldEdgeDetection::ransacLine(
      const vector<pair<uint16_t, uint16_t> >& points, Line& result,
      pair<uint16_t, uint16_t>& resultp1, pair<uint16_t, uint16_t>& resultp2,
      bool **con, uint16_t k, float e, uint16_t n, unsigned int seed) {
   if (points.size() < n) {
      return false;
   }
   // error of best line found so far
   float minerr = numeric_limits<float>::max();
   // arrays for storing concensus sets
   bool *best_concensus;
   bool *this_concensus;
   static bool c1[IMAGE_COLS/SALIENCY_DENSITY];
   static bool c2[IMAGE_COLS/SALIENCY_DENSITY];
   best_concensus = c1;

   for (uint16_t i = 0; i < k; ++i) {
      // randomly select 2 distinct points
      uint32_t p1 = rand_r(&seed) % points.size();
      uint32_t p2 = p1;
      while (p1 == p2) {
         p2 = rand_r(&seed) % points.size();
      }
      // generate the line between those points
      Line l(points[p1].first, points[p1].second,
             points[p2].first, points[p2].second);
      l.var = 0;
      // figure out the variance (sum of distances of points from the line)
      // could use dist() here, but since the denominator is consistent, we
      // save time and implement it again here.
      float denom = sqrt(l.t1*l.t1 + l.t2*l.t2);
      float newe = e*denom;
      uint16_t concensus = 0;
      uint16_t q;
      vector<pair<uint16_t, uint16_t> >::const_iterator j;
      // store the concensus set in a bit-array
      // don't overwrite the current best one
      if (c1 == best_concensus) {
         this_concensus = c2;
      } else {
         this_concensus = c1;
      }
      for (j = points.begin(), q = 0; j != points.end(); ++j, ++q) {
         float dist = (l.t1*((*j).first) + l.t2*((*j).second) + l.t3);
         if (dist < 0) {
            dist *= -1;
         }
         if (dist < newe) {
            l.var += dist;
            ++concensus;
            this_concensus[q] = true;
         } else {
            this_concensus[q] = false;
         }
      }
      l.var /= denom;
      static float k = 0.2;
      l.var = k*l.var - concensus;
      if (l.var < minerr && concensus >= n) {
         minerr = l.var;
         l.var = l.var/(points.size()*e);
         result = l;
         best_concensus = this_concensus;
         resultp1 = points[p1];
         resultp2 = points[p2];
      }
   }

   if (minerr < numeric_limits<float>::max()) {
      *con = best_concensus;
      return true;
   } else {
      return false;
   }
}

void FieldEdgeDetection::findStartScanCoords(Colour saliency
      [IMAGE_COLS/SALIENCY_DENSITY][IMAGE_ROWS/SALIENCY_DENSITY]) {
   uint16_t i;
   int32_t intersec;
   /* If there are no lines, work out if the entire image consists of
    * the field or the entire image is of the background.
    */
   if (numEdgeLines == 0) {
      uint16_t numGreen = 0;
      for (uint16_t x = 0; x < IMAGE_COLS/SALIENCY_DENSITY; x++) {
         if (saliency[x][0] == cFIELD_GREEN) {
            numGreen++;
         }
         startScanCoords[x] = 0;
      }
      if (numGreen < MIN_GREEN_THRESHOLD) {
         for (uint16_t x = 0; x < IMAGE_COLS/SALIENCY_DENSITY; x++) {
            // There is no field seen, so give all the array values that
            // far exceed the number of rows in the saliency scan so that
            // no scan lines are used
            startScanCoords[x] = IMAGE_ROWS;
         }
      }
   } else {
      for (uint16_t x = 0; x < IMAGE_COLS/SALIENCY_DENSITY; x++) {
         int32_t maxVal = -1;
         for (i = 0; i < numEdgeLines; i++) {
            intersec = (-edgeLines[i].t3 -
                  ((x * SALIENCY_DENSITY) * edgeLines[i].t1))/edgeLines[i].t2;
            if (intersec > maxVal) {
               maxVal = intersec;
            }
         }
         if (maxVal < 0) {
            maxVal = 0;
         }
         startScanCoords[x] = maxVal/SALIENCY_DENSITY;
      }
   }
}
