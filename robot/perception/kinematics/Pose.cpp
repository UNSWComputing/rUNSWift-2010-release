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

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <tvmet/Matrix.h>
#include <tvmet/Vector.h>
#include <tvmet/util/Incrementor.h>
#include <cmath>
#include "Pose.hpp"
#include "perception/vision/ImageToRR.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "utils/log.hpp"
#include "utils/Timer.hpp"

using namespace boost::numeric::ublas;

Pose::Pose(const Pose &pose) : c2w(pose.c2w) {
   origin = pose.origin;
   zunit = pose.zunit;
   horizon = pose.horizon;
   // cameraToWorldTransform = matrix<float>(pose.cameraToWorldTransform);
   // FIXME(brockw)
   for (int i = 0; i < EXCLUSION_RESOLUTION; i++)
      exclusionArray[i] = pose.exclusionArray[i];
}

Pose::Pose(matrix<float> cameraToWorldTransform, std::pair<int, int> horizon) {
   this->cameraToWorldTransform = cameraToWorldTransform;
   this->horizon = horizon;
   matrix<float> c2wf = cameraToWorldTransform;
   c2w = c2wf(0, 0), c2wf(0, 1), c2wf(0, 2), c2wf(0, 3),
         c2wf(1, 0), c2wf(1, 1), c2wf(1, 2), c2wf(1, 3),
         c2wf(2, 0), c2wf(2, 1), c2wf(2, 2), c2wf(2, 3),
         c2wf(3, 0), c2wf(3, 1), c2wf(3, 2), c2wf(3, 3);
   origin = 0, 0, 0, 1;
   zunit = 0, 0, 0, 1;
   corigin = c2w * origin;
   llog(DEBUG3) << "Camera Origin: " << corigin << std::endl;
}

RRCoord Pose::imageToRobotRelative(int x, int y, int h) const {
   Timer timer;
   // calculate vector to pixel in camera space
   tvmet::Vector<float, 4> lOrigin, lOrigin2;
   lOrigin2 = (((IMAGE_COLS)/2.0 - x) * PIXEL_SIZE),
             (((IMAGE_ROWS)/2.0 - y) * PIXEL_SIZE),
             0,
             1;

   lOrigin = c2w * lOrigin2;

   tvmet::Vector<float, 4> toPixel, toPixel2;
   toPixel2 = 0, 0, FOCAL_LENGTH, 1;
   toPixel = c2w * toPixel2;

   tvmet::Vector<float, 4> cdir(toPixel - lOrigin);

   float lambda = (h - lOrigin(2))/(1.0 * cdir(2));
   tvmet::Vector<float, 4> intercept;
   intercept = lOrigin(0) + lambda * cdir(0),
               lOrigin(1) + lambda * cdir(1),
               h,
               1;
   RRCoord r;
   r.distance = sqrt(pow(intercept(0), 2) + pow(intercept(1), 2));
   r.heading = atan2(intercept(1), intercept(0));
   return r;
}

std::pair<int, int> Pose::getHorizon() const {
   return horizon;
}
const int16_t *Pose::getExclusionArray() const {
   return exclusionArray;
}

int16_t *Pose::getExclusionArray() {
   return exclusionArray;
}

const boost::numeric::ublas::matrix<float> Pose::getC2wTransform() const {
   matrix<float> c(4, 4);
   c(0, 0) = c2w(0, 0);
   c(1, 0) = c2w(1, 0);
   c(2, 0) = c2w(2, 0);
   c(3, 0) = c2w(3, 0);
   c(0, 1) = c2w(0, 1);
   c(1, 1) = c2w(1, 1);
   c(2, 1) = c2w(2, 1);
   c(3, 1) = c2w(3, 1);
   c(0, 2) = c2w(0, 2);
   c(1, 2) = c2w(1, 2);
   c(2, 2) = c2w(2, 2);
   c(3, 2) = c2w(3, 2);
   c(0, 3) = c2w(0, 3);
   c(1, 3) = c2w(1, 3);
   c(2, 3) = c2w(2, 3);
   c(3, 3) = c2w(3, 3);
   return c;
}
