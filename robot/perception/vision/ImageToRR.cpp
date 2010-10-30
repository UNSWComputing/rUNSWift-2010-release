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

#include "ImageToRR.hpp"
#include "utils/SPLDefs.hpp"
#include "utils/body.hpp"
#include "utils/log.hpp"
#include "utils/basic_maths.hpp"

using namespace std;

CameraToRR::CameraToRR() {
   camera = 0;

   currentIndex = 0;
   for (int i = 0; i < IMAGE_COLS/SALIENCY_DENSITY; i++) {
      endScanCoords[i] = IMAGE_ROWS/SALIENCY_DENSITY;
   }
}

CameraToRR::~CameraToRR() {
}

void CameraToRR::setCamera(RobotCamera *cam) {
   camera = cam;
}

void CameraToRR::updateAngles(SensorValues val) {
   values = val;

   previousReadings[currentIndex] = values.sensors[1];
   currentIndex = (++currentIndex)%10;
   robotMoving = false;
   for (int i = 0; i < 10 && !robotMoving; i++) {
      if (ABS(previousReadings[i] - values.sensors[1]) > 3 ||
            ABS(previousReadings[i]) > 10) {
         robotMoving = true;
      }
   }
}

RRCoord CameraToRR::convertToRR(int16_t i, int16_t j, bool isBall) {
   extern bool offNao;

   RRCoord myloc;
   if (offNao) {
      // Return an empty struct
      myloc.distance = 0;
      myloc.heading = 0;
   } else {
      myloc = pose.imageToRobotRelative(i, j, isBall ? 35 : 0);
   }
   myloc.var[1] = 0;
   myloc.var[0] = 0;
   return myloc;
}

float CameraToRR::pixelSeparationToDistance(int pixelSeparation,
      int realSeparation) {
   return (FOCAL_LENGTH * realSeparation) /
      (PIXEL_SIZE * pixelSeparation);
}

float CameraToRR::ballDistanceByRadius(int radius) {
   extern bool offNao;
   radius += 2;
   bool isTopCamera = true;
   float height_to_neck = 440;
   float neck_pitch = 0;
   if (!offNao) {
      if (camera->getCamera() == BOTTOM_CAMERA) {
         isTopCamera = false;
      }
      const float *angles = values.joints.angles;
      neck_pitch = angles[Joints::HeadPitch];
   }
   float height;
   if (isTopCamera) {
      height = height_to_neck + NECK_TO_TOP_CAMERA *
            -sin(neck_pitch + ANGLE_OFFSET_TOP_CAMERA);
   } else {
      height = height_to_neck + NECK_TO_BOTTOM_CAMERA *
         -sin(neck_pitch + ANGLE_OFFSET_BOTTOM_CAMERA);
   }
   height -= BALL_RADIUS;
   float distance = (FOCAL_LENGTH * BALL_RADIUS * 1.3) /
      (PIXEL_SIZE * radius);

   if (distance < height) {
      return -1;
   }
   distance = sqrt(SQUARE(distance) - SQUARE(height));
   return distance;
}

bool CameraToRR::isRobotMoving() {
   if (robotMoving) {
      llog(VERBOSE) << "The robot is moving" << endl;
   } else {
      llog(VERBOSE) << "The robot is not moving" << endl;
   }
   return true;
}

float CameraToRR::calculateDistanceVariance(float distance, float divider,
      float constant) {
   return SQUARE((constant + distance / divider)/2);
}

float CameraToRR::calculateHeadingVariance(float size) {
   return SQUARE(DEG2RAD(size));
}

void CameraToRR::findEndScanValues() {
   int exclusionRes = Pose::EXCLUSION_RESOLUTION;
   int imageRes = IMAGE_COLS/SALIENCY_DENSITY;
   const int16_t *points = pose.getExclusionArray();
   for (int i = 0; i < imageRes; i++) {
      if (points[(i * exclusionRes)/imageRes] < IMAGE_ROWS) {
         endScanCoords[i] = points[(i * exclusionRes)/imageRes] /
            SALIENCY_DENSITY;
      } else {
         endScanCoords[i] = IMAGE_ROWS/SALIENCY_DENSITY;
      }
   }
}
