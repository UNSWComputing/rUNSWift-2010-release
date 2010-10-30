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

#include <stdint.h>
#include <math.h>
#include "utils/RRCoord.hpp"
#include "perception/vision/RobotCamera.hpp"
#include "utils/SensorValues.hpp"
#include "perception/kinematics/Pose.hpp"
#include "perception/vision/VisionDefs.hpp"
/**
 * All #define measurements are in mm or radians
 **/

/**
 * Arbitary focal length of the camera
 **/
#define FOCAL_LENGTH 1
/**
 * Pixel size calculated from FOV of camera and the arbitary focal length
 **/

// aldebaran's:
#define PIXEL_SIZE 0.00126536

/**
 * Straight line distance from the neck to the camera
 **/
#define NECK_TO_TOP_CAMERA 86.6927
#define NECK_TO_BOTTOM_CAMERA 54.2988
/**
 * Angle of the camera from the neck
 **/
#define ANGLE_OFFSET_TOP_CAMERA -0.89984
#define ANGLE_OFFSET_BOTTOM_CAMERA -0.45393

#define ANGLE_BOTTOM_CAMERA 0.69813

class CameraToRR {
   public:
      CameraToRR();
      ~CameraToRR();

      void setCamera(RobotCamera *camera);
      void updateAngles(SensorValues values);
      RRCoord convertToRR(int16_t i, int16_t j, bool isBall);
      Pose pose;
      float pixelSeparationToDistance(int pixelSeparation, int realSeparation);
      float ballDistanceByRadius(int radius);
      bool isRobotMoving();

      /**
       * Calculates the variance for the distance field of RRCoord.
       * The formula is var = constant + distance / divider
       **/
      float calculateDistanceVariance(float distance, float divider,
            float constant);
      /**
       * Calculates the variance for the heading field of RRCoord.
       * size is the angle of uncertainty in degrees
       **/
      float calculateHeadingVariance(float size);

      /**
       * Finds the saliency scan coordinates where vertical scans
       * should be stopped to avoid considering the robots own body
       * The coordinates in the array returned are SALIENCY SCAN VALUES
       **/
      void findEndScanValues();
      int endScanCoords[IMAGE_COLS/SALIENCY_DENSITY];

   private:
      RobotCamera *camera;
      SensorValues values;

      float previousReadings[10];
      int currentIndex;
      bool robotMoving;
};



