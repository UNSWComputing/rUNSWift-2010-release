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
#include <vector>
#include "perception/vision/VisionConstants.hpp"
#include "perception/vision/RegionBuilder.hpp"
#include "perception/vision/ImageToRR.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/RobotDetection.hpp"

class Vision;

class BallDetection {
   public:
      /**
       * The radius of the ball detected. Will be 0 if no ball was found
       * in the frame
       **/
      uint16_t radius;
      /**
       * The image coordinates of the centre of the ball
       **/
      std::pair<uint16_t, uint16_t> ballCentre;
      /**
       * The coordinates that have been detected to be on
       * the edge of the ball
       **/
      uint16_t numBallEdgePoints;
      std::pair<uint16_t, uint16_t> ballEdgePoints[MAX_BALL_EDGE_POINTS];
      /**
       * The choosen region to represent a ball
       **/
      ImageRegion *bestBallRegion;

      /**
       * Information about the ball that is to be written to the blackboard
       **/
      int numBalls;
      RRCoord ballLoc[MAX_BALLS];

      explicit BallDetection();

      void findBalls(ImageRegion **ballRegions, uint16_t numBallRegions,
            CameraToRR *convRR, Vision *vision, unsigned int *seed,
            RobotDetection *robotDetection, uint32_t *startOfScan,
            const std::pair<int, int> &horizon);
   private:

      /**
       * Possible properties of the ball given by the intersection of 
       * perpendicular bisectors algorithm
       **/
      std::vector<uint16_t> possibleXCentre;
      std::vector<uint16_t> possibleYCentre;
      std::vector<uint16_t> possibleRadius;

      /**
       * The extreme edge points found by the edge finding algorithm
       **/
      int16_t topMost;
      int16_t bottomMost;
      int16_t leftMost;
      int16_t rightMost;

      /**
       * A dirty hack to delete cases where some of the background is below
       * the field edge, mainly due to looking side on to the goal posts
       * causing the field edge detectino to be wrong, and balls get seen
       * in this area
       **/
      bool isBallAboveMissedEdge(ImageRegion *ballRegion, Vision *vision,
            const std::pair<int, int> &horizon);
      /**
       * Another dirty hack to remove balls from robots that are
       * occassionally missed
       **/
      bool isBallInMissedRobot(Vision *vision);

      /**
       * Tests to see if the detected ball contains the original
       * ballRegion
       **/
      bool isBallInsideBallRegion(ImageRegion *ballRegion);

      /**
       * Finds our if the centre of the ball is contained inside a robot
       * region.
       **/
      bool isCentreInsideRobot(RobotDetection *robotDetection,
            uint16_t xCoord, uint16_t y);

      /**
       * Finds out if the ball is contained inside a robot region. If
       * so, it means that the ball region is mostly a result of bad
       * classification of the pink band
       **/
      bool isInsideRobot(RobotDetection *robotDetection,
            ImageRegion *ballRegion);

      /**
       * Finds edge points of a ball when only a few orange points are seen
       * in the saliency scan. It scans every row and column
       **/
      void findBallEdgeSmall(Vision *vision, ImageRegion *ballRegion);

      /**
       * Finds edge points of a ball. If numSkip is 1, it will scan every
       * SALIENCY_DENSITY row or column. If numSkip is 2, it will scan
       * every 2 * SALIENCY_DENSITY rows and columns
       **/
      void findBallEdge(Vision *vision, ImageRegion *ballRegion, int numSkip);

      /**
       * Tests whether there is enough difference in values between two pixels
       * for an edge to exist between the pixels
       **/
      inline bool isEdge(uint16_t i1, uint16_t j1, uint16_t i2,
            uint16_t j2, Vision *vision);

      /**
       * Given a series of edge points, this calculates the likely centre
       * and radius of the ball
       **/
      void calculateBallProperties(unsigned int *seed);

      /**
       * Finds the centre of a cluster of possible points. It removes outliers
       * from the list and then finds the median. It is used to find the
       * best centre and radius of the ball
       **/
      inline uint16_t findCentreCluster(std::vector<uint16_t> points);

      /**
       * Finds three different coordinates from a list of coordinates
       **/
      inline bool getThreeUniquePoints(unsigned int *seed,
            std::pair<uint16_t, uint16_t> *points);

      /**
       * Finds the equation of a perpendicular bisector between two points
       **/
      inline TwoParamLine findEquationOfLine(std::pair<uint16_t, uint16_t> p1,
            std::pair<uint16_t, uint16_t> p2);
};
