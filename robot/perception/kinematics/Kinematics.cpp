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
#include <cmath>
#include <vector>
#include "Kinematics.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "utils/angles.hpp"
#include "utils/Timer.hpp"
#include "utils/log.hpp"
#include "utils/body.hpp"


using namespace boost::numeric::ublas;

// used with filterZeros function
static const float VERY_SMALL = 0.0001;
// eventually will get these static const from constants section
static const float foot = 45.11;
static const float tibia = 102.74;
static const float thigh = 100;
static const float hip = 49.79;
static const float hip_offsetZ = 84.79;
static const float neck_offsetZ = 126.50;
static const float trunk_length = 84.79 + 126.50;

static const float camera_out_bottom = 48.80;
static const float camera_up_bottom = 23.81;

static const float camera_out_top = 53.90;
static const float camera_up_top = 67.90;

static const float camera_bottom_angle = 40.0;
static const float camera_top_angle = 0;


Kinematics::Kinematics() {
   std::vector<matrix<float> > chest;

   // Left side of body
   chest.push_back(vec4(-90, 155, Limbs::NeckOffsetZ + -30, 1));
   chest.push_back(vec4(-70, 155, Limbs::NeckOffsetZ + 0, 1));
   chest.push_back(vec4(-50, 155, Limbs::NeckOffsetZ + 30, 1));
   chest.push_back(vec4(-30, 155, Limbs::NeckOffsetZ + 40, 1));
   chest.push_back(vec4(-20, 155, Limbs::NeckOffsetZ + 44, 1));
   chest.push_back(vec4(-10, 155, Limbs::NeckOffsetZ + 47, 1));
   chest.push_back(vec4(0, 155, Limbs::NeckOffsetZ + 50, 1));
   chest.push_back(vec4(10, 155, Limbs::NeckOffsetZ + 47, 1));
   chest.push_back(vec4(20, 155, Limbs::NeckOffsetZ + 44, 1));
   chest.push_back(vec4(30, 155, Limbs::NeckOffsetZ + 40, 1));
   chest.push_back(vec4(50, 155, Limbs::NeckOffsetZ + 30, 1));
   chest.push_back(vec4(70, 155, Limbs::NeckOffsetZ + 0, 1));
   chest.push_back(vec4(90, 155, Limbs::NeckOffsetZ + -30, 1));

   chest.push_back(vec4(40, 70, Limbs::NeckOffsetZ - 40, 1));
   // bodyParts.push_back(chest);
   // chest.clear();

   chest.push_back(vec4(60, 60, Limbs::NeckOffsetZ - 70, 1));
   chest.push_back(vec4(62, 50, Limbs::NeckOffsetZ - 70, 1));
   chest.push_back(vec4(63, 40, Limbs::NeckOffsetZ - 70, 1));
   chest.push_back(vec4(65, 20, Limbs::NeckOffsetZ - 70, 1));
   chest.push_back(vec4(65, 10, Limbs::NeckOffsetZ - 70, 1));
   chest.push_back(vec4(65, 0, Limbs::NeckOffsetZ - 70, 1));
   bodyParts.push_back(chest);
   chest.clear();

   // reflection
   chest.push_back(vec4(65, 0, Limbs::NeckOffsetZ - 70, 1));
   chest.push_back(vec4(65, -10, Limbs::NeckOffsetZ - 70, 1));
   chest.push_back(vec4(65, -20, Limbs::NeckOffsetZ - 70, 1));
   chest.push_back(vec4(63, -40, Limbs::NeckOffsetZ - 70, 1));
   chest.push_back(vec4(62, -50, Limbs::NeckOffsetZ - 70, 1));
   chest.push_back(vec4(60, -60, Limbs::NeckOffsetZ - 70, 1));
   // bodyParts.push_back(chest);
   // chest.clear();

   // right side of body
   chest.push_back(vec4(40, -70, Limbs::NeckOffsetZ - 40, 1));

   chest.push_back(vec4(90, -155, Limbs::NeckOffsetZ + -30, 1));
   chest.push_back(vec4(70, -155, Limbs::NeckOffsetZ + 0, 1));
   chest.push_back(vec4(50, -155, Limbs::NeckOffsetZ + 30, 1));
   chest.push_back(vec4(30, -155, Limbs::NeckOffsetZ + 40, 1));
   chest.push_back(vec4(20, -155, Limbs::NeckOffsetZ + 44, 1));
   chest.push_back(vec4(10, -155, Limbs::NeckOffsetZ + 47, 1));
   chest.push_back(vec4(0, -155, Limbs::NeckOffsetZ + 50, 1));
   chest.push_back(vec4(-10, -155, Limbs::NeckOffsetZ + 47, 1));
   chest.push_back(vec4(-20, -155, Limbs::NeckOffsetZ + 44, 1));
   chest.push_back(vec4(-30, -155, Limbs::NeckOffsetZ + 40, 1));
   chest.push_back(vec4(-50, -155, Limbs::NeckOffsetZ + 30, 1));
   chest.push_back(vec4(-70, -155, Limbs::NeckOffsetZ + 0, 1));
   chest.push_back(vec4(-90, -155, Limbs::NeckOffsetZ + -30, 1));
   bodyParts.push_back(chest);
   chest.clear();
}

Kinematics::Chain Kinematics::determineSupportChain() {
   float lsum = sensorValues.sensors[Sensors::LFoot_FSR_FrontLeft]  +
                sensorValues.sensors[Sensors::LFoot_FSR_FrontRight] +
                sensorValues.sensors[Sensors::LFoot_FSR_RearLeft]   +
                sensorValues.sensors[Sensors::LFoot_FSR_RearRight];
   float rsum = sensorValues.sensors[Sensors::RFoot_FSR_FrontLeft]  +
                sensorValues.sensors[Sensors::RFoot_FSR_FrontRight] +
                sensorValues.sensors[Sensors::RFoot_FSR_RearLeft]   +
                sensorValues.sensors[Sensors::RFoot_FSR_RearRight];
   if (lsum > rsum) return LEFT_CHAIN;
   return RIGHT_CHAIN;
}

void Kinematics::updateDHChain() {
   // decide if i am using the top or bottom camera
   float camera_out, camera_up, camera_angle, coffsetY, coffsetX;
   if (whichCamera == TOP_CAMERA) {
      camera_out = camera_out_top;
      camera_up = camera_up_top;
      camera_angle = camera_top_angle;
      coffsetY = cameraOffsetYTop;
      coffsetX = cameraOffsetXTop;
   } else {
      camera_out = camera_out_bottom;
      camera_up = camera_up_bottom;
      camera_angle = camera_bottom_angle;
      coffsetY = cameraOffsetYBottom;
      coffsetX = cameraOffsetXBottom;
   }
   llog(DEBUG3) << "Camera out: " << camera_out << std::endl;
   llog(DEBUG3) << "Camera up: " << camera_up << std::endl;
   llog(DEBUG3) << "Camera angle: " << camera_angle << std::endl;

   JointValues jointValues = sensorValues.joints;

   float cameraOffset = camera_angle + coffsetY;
   float offsetX = DEG2RAD(coffsetX);
   float d1 = sqrt(pow(camera_out, 2) + pow(camera_up, 2));
   float d2 = trunk_length -  hip;

   float d3 = hip * sqrt(2);
   float a1 = atan(camera_up/camera_out) + DEG2RAD(cameraOffset);
   // float a2 = atan(camera_up/camera_out) + M_PI/2;
   float l10 = d1 * sin(a1);
   float d11 = d1 * cos(a1);
   float a3 = DEG2RAD(cameraOffset) -M_PI;

   float Cp = jointValues.angles[Joints::HeadPitch];
   float Cy = jointValues.angles[Joints::HeadYaw];
   float Hyp = -jointValues.angles[Joints::LHipYawPitch];
   float Hp = -jointValues.angles[Joints::LHipPitch];
   float Hr = -jointValues.angles[Joints::LHipRoll];
   float Kp = -jointValues.angles[Joints::LKneePitch];
   float Ap = -jointValues.angles[Joints::LAnklePitch];
   float Ar = -jointValues.angles[Joints::LAnkleRoll];

   Hp += DEG2RAD(bodyPitchOffset);

   // paramters
   transformL[0] = createDHMatrix(0, 0, foot, M_PI/2);
   transformL[1] = createDHMatrix(0, M_PI/2, 0, M_PI/2 + Ar);
   transformL[2] = createDHMatrix(0, M_PI/2, 0, Ap);
   transformL[3] = createDHMatrix(tibia, 0, 0, Kp);
   transformL[4] = createDHMatrix(thigh, 0, 0, Hp);
   transformL[5] = createDHMatrix(0, -M_PI/2, 0, -M_PI/4 + Hr);
   transformL[6] = createDHMatrix(0, M_PI/2, -d3, M_PI/2 + Hyp);
   transformL[7] = createDHMatrix(0, 3 * M_PI/4, 0, 0);
   transformL[8] = createDHMatrix(0, 0, d2, Cy + offsetX);
   transformL[9] = createDHMatrix(0, -M_PI/2, 0, a3 + Cp);
   transformL[10] = createDHMatrix(0, -M_PI/2, l10, M_PI/2);
   transformL[11] = createDHMatrix(0, -M_PI/2, d11, 0);

   Hp = -jointValues.angles[Joints::RHipPitch];
   Hr = -jointValues.angles[Joints::RHipRoll];
   Kp = -jointValues.angles[Joints::RKneePitch];
   Ap = -jointValues.angles[Joints::RAnklePitch];
   Ar = -jointValues.angles[Joints::RAnkleRoll];

   Hp += DEG2RAD(bodyPitchOffset);
   // paramters
   transformR[0] = createDHMatrix(0, 0, foot, M_PI/2);
   transformR[1] = createDHMatrix(0, M_PI/2, 0, M_PI/2 + Ar);
   transformR[2] = createDHMatrix(0, M_PI/2, 0, Ap);
   transformR[3] = createDHMatrix(tibia, 0, 0, Kp);
   transformR[4] = createDHMatrix(thigh, 0, 0, Hp);
   transformR[5] = createDHMatrix(0, -M_PI/2, 0, M_PI/4 + Hr);
   transformR[6] = createDHMatrix(0, M_PI/2, d3, M_PI/2 + Hyp);
   transformR[7] = createDHMatrix(0, M_PI/4, 0, 0);
   transformR[8] = createDHMatrix(0, 0, d2, Cy + offsetX);
   transformR[9] = createDHMatrix(0, -M_PI/2, 0, a3 + Cp);
   transformR[10] = createDHMatrix(0, -M_PI/2, l10, M_PI/2);
   transformR[11] = createDHMatrix(0, -M_PI/2, d11, 0);
}

Pose Kinematics::getPose() {
   Chain foot = determineSupportChain();
   matrix<float> c2w = prod(createFootToWorldTransform(foot),
                            createCameraToFootTransform(foot));
   std::pair<int, int> horizon = calculateHorizon(c2w);
   Pose pose(c2w, horizon);

   matrix<float> b2c = evaluateDHChain(BODY, CAMERA, foot);
   determineBodyExclusionArray(b2c, pose.getExclusionArray());
   return pose;
}
matrix<float> Kinematics::createCameraToWorldTransform(Chain foot) {
   matrix<float> c2f = createCameraToFootTransform(foot);
   matrix<float> f2w = createFootToWorldTransform(foot);
   return prod(f2w, c2f);
}

matrix<float> Kinematics::evaluateDHChain(Link from, Link to,
                                          Chain foot) {
   matrix<float> finalTransform = identity_matrix<float>(4);
   if (foot == RIGHT_CHAIN) {
      for (int i = from; i < to; i++) {
         finalTransform = prod(finalTransform, transformR[i]);
      }
   } else {
      for (int i = from; i < to; i++) {
         finalTransform = prod(finalTransform, transformL[i]);
      }
   }
   return finalTransform;
}
matrix<float> Kinematics::createCameraToFootTransform(Chain foot) {
   return evaluateDHChain(FOOT, CAMERA, foot);
}

// World is defined as the centre of the two feet on the ground plane,
// with a heading equal to the average of the two feet directions
matrix<float> Kinematics::createFootToWorldTransform(Chain foot) {
   matrix<float> b2lf = evaluateDHChain(FOOT, BODY, foot);
   matrix<float> b2rf = evaluateDHChain(FOOT, BODY, (Chain)!foot);

   matrix<float> rf2b(4, 4);
   Timer timer;
   invertMatrix(b2rf, rf2b);


   matrix<float> rf2lf = prod(rf2b, b2lf);

   matrix<float> z(4, 1);
   z(0, 0) = 0;
   z(1, 0) = 0;
   z(2, 0) = 0;
   z(3, 0) = 1;
   matrix<float> forward(4, 1);
   forward(0, 0) = 1;
   forward(1, 0) = 0;
   forward(2, 0) = 0;
   forward(3, 0) = 1;

   // first find position of centre of two feet on the ground.
   z = prod(rf2lf, z);

   // find direction of second foot in first foot coords
   forward = prod(rf2lf, forward) - z;

   matrix<float> position = -z/2;
   position(3, 0) = 1;
   position(2, 0) = 0;  // on the ground

   matrix<float> result = identity_matrix<float>(4);
   result = prod(translateMatrix(position(0, 0), position(1, 0), 0),
                          result);
   result = prod(rotateZMatrix(-atan2(forward(1, 0), forward(0, 0))/2.0),
                               result);
   return result;
}

void Kinematics::setSensorValues(SensorValues sensorValues) {
   this->sensorValues = sensorValues;
}

matrix<float>
Kinematics::createWorldToFOVTransform(
               const boost::numeric::ublas::matrix<float> &c2w) {
   matrix<float> w2c = c2w;

   invertMatrix(c2w, w2c);
   matrix<float> ctest = vec4(0, 0, 0, 1);

   float ex = 0;
   float ey = 0;
   float ez = 1.0/tan(DEG2RAD(46.4/2));

   matrix<float> projection = projectionMatrix(ex, ey, ez);
   matrix<float> transform = prod(projection, w2c);

   return transform;
}

void Kinematics::determineBodyExclusionArray(
                                 const boost::numeric::ublas::matrix<float> &m,
                                 int16_t *points) {
   for (int i = 0; i < Pose::EXCLUSION_RESOLUTION; i++) {
      points[i] = 500;
   }
   // pixel off screen really low
   matrix<float> transform = createWorldToFOVTransform(m);

   for (unsigned int part = 0; part < bodyParts.size(); part++) {
      matrix<float> last  = fovToImageSpaceTransform(transform,
                                               bodyParts[part][0]);
      for (unsigned int i = 0; i < bodyParts[part].size(); i++) {
         matrix<float> m  = fovToImageSpaceTransform(transform,
                                                  bodyParts[part][i]);
         if (m(2, 0) <= 0) {
            last = m;
            continue;
         }
         int lIndex = (int)(last(0, 0)/640.0 * Pose::EXCLUSION_RESOLUTION);
         int cIndex = (int)(m(0, 0)/640.0 * Pose::EXCLUSION_RESOLUTION);
         int lPixel = last(1, 0);
         int cPixel = m(1, 0);
         float gradient = 0;
         if (cIndex - lIndex != 0) {
            float denom = cIndex - lIndex;
            gradient = (cPixel - lPixel)/(denom);
         }
         cIndex = MIN(MAX(cIndex, 0), (int) Pose::EXCLUSION_RESOLUTION);
         lIndex = MIN(MAX(lIndex, 0), (int) Pose::EXCLUSION_RESOLUTION);
         int index = lIndex;
         while (index != cIndex && last(2, 0) > 0) {
            if (index >= 0 && index < Pose::EXCLUSION_RESOLUTION &&
                index != lIndex && index != cIndex) {
                int nPixel = last(1, 0)  + gradient * (index - lIndex);
                if (nPixel < points[index]) points[index] = nPixel;
            }
            index += (cIndex - lIndex) > 0 ? 1 : -1;
         }

         // get Index of last.
         // keep adding one and linearly interpolate
         index = (int)(m(0, 0)/640.0 * Pose::EXCLUSION_RESOLUTION);
         if (index >= 0 && index < Pose::EXCLUSION_RESOLUTION) {
            if (m(1, 0) < points[index]) {
               points[index] = m(1, 0);
            }
         }
         last = m;
      }
   }
}


std::pair<int, int> Kinematics::calculateHorizon(const matrix<float> &c2w) {
   matrix<float> transform = createWorldToFOVTransform(c2w);

   // set up horizon points in world coordinates.
   // the idea is to now convert these points into camera space and then
   // project them onto pixels to find out where the horizon is
   // Note: the -1000 and 1000 are completely arbitrary...
   matrix<float> pixel1 = vec4(-1000, INT_MAX, 0, 1);
   matrix<float> pixel2 = vec4(1000, INT_MAX, 0, 1);

   pixel1 = fovToImageSpaceTransform(transform, pixel1);
   pixel2 = fovToImageSpaceTransform(transform, pixel2);

   // find gradient of the horizon
   matrix<float> dir = pixel1 - pixel2;

   // we now convert the horizon to a nice format that vision can use.
   // it is just the two y intercepts at x = 0 and x = IMAGE_COLS
   float lambda1 = -pixel1(0, 0)/dir(0, 0);
   float lambda2 = (IMAGE_COLS - pixel1(0, 0))/dir(0, 0);

   float y1 = lambda1 * dir(1, 0) + pixel1(1, 0);
   float y2 = lambda2 * dir(1, 0) + pixel1(1, 0);

   return std::pair<int, int>(y1, y2);
}

matrix<float>
Kinematics::fovToImageSpaceTransform(const matrix<float> &transform,
                                     const matrix<float> &point) {
   // use image space transform to find the perspective scaling factor
   matrix<float> pixel = prod(transform, point);

   // divide x and y by the perspective scaling factor
   pixel(0, 0) /= pixel(3, 0);
   pixel(1, 0) /= pixel(3, 0);
   pixel(2, 0) = pixel(3, 0);
   pixel(3, 0) = 1;

   // now we have the pixel in a space that spans from (-1, 1) in the x
   // direction and (-1, 1) in the y.

   // therefore we need to scale this up to our image size which is what
   // the code below does
   float xscale = IMAGE_COLS/2;
   float yscale = IMAGE_ROWS/2;
   pixel(0, 0) = (pixel(0, 0)) * xscale + xscale;
   pixel(1, 0) = (pixel(1, 0)) * xscale + yscale;
   return pixel;
}


