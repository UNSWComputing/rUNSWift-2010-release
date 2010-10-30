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
#include <boost/numeric/ublas/lu.hpp>
#include <utility>
#include <vector>
#include "utils/matrix_helpers.hpp"
#include "../../utils/RRCoord.hpp"
#include "utils/JointValues.hpp"
#include "utils/SensorValues.hpp"
#include "Pose.hpp"
#include "perception/vision/RobotCamera.hpp"

class Kinematics {
   public:
      friend class KinematicsAdapter;
      Kinematics();

      enum Link {
                  FOOT = 0,
                  BODY = 8,
                  CAMERA = 12,
                  NECK = 9
                };

      enum Chain {
                  LEFT_CHAIN = 0,
                  RIGHT_CHAIN = 1
                 };

      /* Creates a Pose with the current evaluated DH Chain */
      Pose getPose();

      void updateDHChain();

      boost::numeric::ublas::matrix<float> evaluateDHChain(Link from, Link to,
                                                           Chain foot);

      boost::numeric::ublas::matrix<float>
                             createCameraToFootTransform(Chain foot);

      boost::numeric::ublas::matrix<float>
                             createFootToWorldTransform(Chain foot);

      boost::numeric::ublas::matrix<float>
                             createCameraToWorldTransform(Chain foot);

      boost::numeric::ublas::matrix<float>
                             createWorldToFOVTransform(
                             const boost::numeric::ublas::matrix<float> &m);

      boost::numeric::ublas::matrix<float>
                             fovToImageSpaceTransform(
                     const boost::numeric::ublas::matrix<float> &transform,
                     const boost::numeric::ublas::matrix<float> &point);

      void determineBodyExclusionArray(
                        const boost::numeric::ublas::matrix<float> &m,
                        int16_t *points);

      Chain determineSupportChain();

      void setSensorValues(SensorValues sensorValues);

      std::pair<int, int> calculateHorizon(
                        const boost::numeric::ublas::matrix<float> &m);
   private:
      SensorValues sensorValues;
      Chain supportChain;
      WhichCamera whichCamera;

      boost::numeric::ublas::matrix<float> transformL[12];
      boost::numeric::ublas::matrix<float> transformR[12];
      float cameraOffsetXBottom;
      float cameraOffsetYBottom;
      float cameraOffsetXTop;
      float cameraOffsetYTop;
      float bodyPitchOffset;
      std::vector<std::vector<boost::numeric::ublas::matrix<float> > >
                                                               bodyParts;
};
