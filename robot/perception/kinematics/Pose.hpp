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
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <tvmet/Matrix.h>
#include <tvmet/Vector.h>
#include <tvmet/util/Incrementor.h>
#include <utility>
#include "../../utils/RRCoord.hpp"
#include "utils/JointValues.hpp"

/**
 * The Pose class contains precomputed kinematic data that is useful
 * to other modules. (Currently only vision).
 *
 * The idea is that the kinematics module computes the full kinematic chain
 * and then stores the resulting matrix in this Pose class.
 *
 * Vision then queries the Pose class and the cached results are used each
 * time.
 */
class Pose {
   public:
      explicit Pose(
         boost::numeric::ublas::matrix<float> cameraToWorldTransform,
         std::pair<int, int> horizon);

      Pose(const Pose &pose);
      Pose() {}

      /**
       *  Returns a pair for the horizon.
       *  pair.first is the y position of the horizon at x = 0
       *  pair.second is the y position of the horizon at x = 640
       */
      std::pair<int, int> getHorizon() const;

      /**
       * Returns the robot relative coord for a given pixel at a particular
       * height.
       *
       * @param x pixel
       * @param y pixel
       * @param height of intersection plane.
       */
      RRCoord imageToRobotRelative(int x, int y, int h) const;

      /* Returns a pointer to the exclusion array used by vision */
      const int16_t *getExclusionArray() const;
      int16_t *getExclusionArray();

      const boost::numeric::ublas::matrix<float> getC2wTransform() const;

      boost::numeric::ublas::matrix<float> cameraToWorldTransform;
      tvmet::Matrix<float, 4, 4> c2w;
      static const uint16_t EXCLUSION_RESOLUTION = 100;
   private:
      tvmet::Vector<float, 4> origin, zunit, corigin;
      std::pair<int, int> horizon;
      int16_t exclusionArray[EXCLUSION_RESOLUTION];
};
