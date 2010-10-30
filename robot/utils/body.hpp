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

/**
 * Body.hpp
 * Description: Contains Joint Codes, Limits, Motor Reductions and Limb
 *              lengths and weights
 * Modified: 2009-11-06 (for Naoqi 1.3.17)
 */

#pragma once

#include <iostream>
#include <cmath>
#include <string>
#include "utils/angles.hpp"

namespace Joints {
   // Codes for each joint as used by NaoQi
   typedef enum JointCodesEnum {
      HeadYaw = 0,
      HeadPitch,
      LShoulderPitch,
      LShoulderRoll,
      LElbowYaw,
      LElbowRoll,
      LHipYawPitch,
      LHipRoll,
      LHipPitch,
      LKneePitch,
      LAnklePitch,
      LAnkleRoll,
      RHipRoll,
      RHipPitch,
      RKneePitch,
      RAnklePitch,
      RAnkleRoll,
      RShoulderPitch,
      RShoulderRoll,
      RElbowYaw,
      RElbowRoll,
      NUMBER_OF_JOINTS
   } JointCode;

   const JointCode jointCodes[] = {
      HeadYaw,
      HeadPitch,
      LShoulderPitch,
      LShoulderRoll,
      LElbowYaw,
      LElbowRoll,
      LHipYawPitch,
      LHipRoll,
      LHipPitch,
      LKneePitch,
      LAnklePitch,
      LAnkleRoll,
      RHipRoll,
      RHipPitch,
      RKneePitch,
      RAnklePitch,
      RAnkleRoll,
      RShoulderPitch,
      RShoulderRoll,
      RElbowYaw,
      RElbowRoll
   };

   // Note: Functions are defined at the end of this file...
   const std::string jointNames[] = {
      "HeadYaw",
      "HeadPitch",
      "LShoulderPitch",
      "LShoulderRoll",
      "LElbowYaw",
      "LElbowRoll",
      "LHipYawPitch",
      "LHipRoll",
      "LHipPitch",
      "LKneePitch",
      "LAnklePitch",
      "LAnkleRoll",
      "RHipRoll",
      "RHipPitch",
      "RKneePitch",
      "RAnklePitch",
      "RAnkleRoll",
      "RShoulderPitch",
      "RShoulderRoll",
      "RElbowYaw",
      "RElbowRoll"
   };

   const std::string fliteJointNames[] = {
      "Head Yaw",
      "Head Pitch",
      "L Shoulder Pitch",
      "L Shoulder Roll",
      "L Elbow Yaw",
      "L Elbow Roll",
      "L Hip Yaw Pitch",
      "L Hip Roll",
      "L Hip Pitch",
      "L Knee Pitch",
      "L Ankle Pitch",
      "L Ankle Roll",
      "R Hip Roll",
      "R Hip Pitch",
      "R Knee Pitch",
      "R Ankle Pitch",
      "R Ankle Roll",
      "R Shoulder Pitch",
      "R Shoulder Roll",
      "R Elbow Yaw",
      "R Elbow Roll"
   };

   const std::string simJointNames[] = {
      "headPan",
      "headTilt",
      "armLeft0",
      "armLeft1",
      "armLeft2",
      "armLeft3",
      "legLeft0",
      "legLeft1",
      "legLeft2",
      "legLeft3",
      "legLeft4",
      "legLeft5",
      "legRight1",
      "legRight2",
      "legRight3",
      "legRight4",
      "legRight5",
      "armRight0",
      "armRight1",
      "armRight2",
      "armRight3"
   };

   // Limits for Joints in DEGREES
   namespace Degrees {
      const float HeadYaw_Min          = -120;
      const float HeadYaw_Max          = 120;
      const float HeadPitch_Min        = -39;
      const float HeadPitch_Max        = 30;
      const float LShoulderPitch_Min   = -120;
      const float LShoulderPitch_Max   = 120;
      const float LShoulderRoll_Max    = 95;
      const float LShoulderRoll_Min    = 0;
      const float LElbowRoll_Min       = -90;
      const float LElbowRoll_Max       = 0;
      const float LElbowYaw_Min        = -120;
      const float LElbowYaw_Max        = 120;
      const float LHipYawPitch_Min     = -66;
      const float LHipYawPitch_Max     = 42;
      const float LHipPitch_Min        = -104.5;
      const float LHipPitch_Max        = 28.5;
      const float LHipRoll_Min         = -25;
      const float LHipRoll_Max         = 45;
      const float LKneePitch_Min       = -5;
      const float LKneePitch_Max       = 125;
      const float LAnklePitch_Min      = -70.5;
      const float LAnklePitch_Max      = 54;
      const float LAnkleRoll_Min       = -45;
      const float LAnkleRoll_Max       = 25;
      const float RHipPitch_Min        = -104.5;
      const float RHipPitch_Max        = 28.5;
      const float RHipRoll_Min         = -45;
      const float RHipRoll_Max         = 25;
      const float RKneePitch_Min       = -5;
      const float RKneePitch_Max       = 125;
      const float RAnklePitch_Min      = -70.5;
      const float RAnklePitch_Max      = 54;
      const float RAnkleRoll_Min       = -25;
      const float RAnkleRoll_Max       = 45;
      const float RShoulderPitch_Min   = -120;
      const float RShoulderPitch_Max   = 120;
      const float RShoulderRoll_Min    = -95;
      const float RShoulderRoll_Max    = 0;
      const float RElbowRoll_Min       = 0;
      const float RElbowRoll_Max       = 90;
      const float RElbowYaw_Min        = -120;
      const float RElbowYaw_Max        = 120;

      // Array to hold the maximum and minimum boundaries
      const float MaxAngle[] = {
         HeadYaw_Max,
         HeadPitch_Max,
         LShoulderPitch_Max,
         LShoulderRoll_Max,
         LElbowYaw_Max,
         LElbowRoll_Max,
         LHipYawPitch_Max,
         LHipRoll_Max,
         LHipPitch_Max,
         LKneePitch_Max,
         LAnklePitch_Max,
         LAnkleRoll_Max,
         RHipRoll_Max,
         RHipPitch_Max,
         RKneePitch_Max,
         RAnklePitch_Max,
         RAnkleRoll_Max,
         RShoulderPitch_Max,
         RShoulderRoll_Max,
         RElbowYaw_Max,
         RElbowRoll_Max
      };

      const float MinAngle[] = {
         HeadYaw_Min,
         HeadPitch_Min,
         LShoulderPitch_Min,
         LShoulderRoll_Min,
         LElbowYaw_Min,
         LElbowRoll_Min,
         LHipYawPitch_Min,
         LHipRoll_Min,
         LHipPitch_Min,
         LKneePitch_Min,
         LAnklePitch_Min,
         LAnkleRoll_Min,
         RHipRoll_Min,
         RHipPitch_Min,
         RKneePitch_Min,
         RAnklePitch_Min,
         RAnkleRoll_Min,
         RShoulderPitch_Min,
         RShoulderRoll_Min,
         RElbowYaw_Min,
         RElbowRoll_Min
      };
   };

   // Limits for Joints in RADIANS
   namespace Radians {
      const float HeadYaw_Min          = DEG2RAD(-120);
      const float HeadYaw_Max          = DEG2RAD(120);
      const float HeadPitch_Min        = DEG2RAD(-39);
      const float HeadPitch_Max        = DEG2RAD(30);
      const float LShoulderPitch_Min   = DEG2RAD(-120);
      const float LShoulderPitch_Max   = DEG2RAD(120);
      const float LShoulderRoll_Max    = DEG2RAD(95);
      const float LShoulderRoll_Min    = DEG2RAD(0);
      const float LElbowRoll_Min       = DEG2RAD(-90);
      const float LElbowRoll_Max       = DEG2RAD(0);
      const float LElbowYaw_Min        = DEG2RAD(-120);
      const float LElbowYaw_Max        = DEG2RAD(120);
      const float LHipYawPitch_Min     = DEG2RAD(-66);
      const float LHipYawPitch_Max     = DEG2RAD(42);
      const float LHipPitch_Min        = DEG2RAD(-104.5);
      const float LHipPitch_Max        = DEG2RAD(28.5);
      const float LHipRoll_Min         = DEG2RAD(-25);
      const float LHipRoll_Max         = DEG2RAD(45);
      const float LKneePitch_Min       = DEG2RAD(-5);
      const float LKneePitch_Max       = DEG2RAD(125);
      const float LAnklePitch_Min      = DEG2RAD(-70.5);
      const float LAnklePitch_Max      = DEG2RAD(54);
      const float LAnkleRoll_Min       = DEG2RAD(-45);
      const float LAnkleRoll_Max       = DEG2RAD(25);
      const float RHipPitch_Min        = DEG2RAD(-104.5);
      const float RHipPitch_Max        = DEG2RAD(28.5);
      const float RHipRoll_Min         = DEG2RAD(-45);
      const float RHipRoll_Max         = DEG2RAD(25);
      const float RKneePitch_Min       = DEG2RAD(-5);
      const float RKneePitch_Max       = DEG2RAD(125);
      const float RAnklePitch_Min      = DEG2RAD(-70.5);
      const float RAnklePitch_Max      = DEG2RAD(54);
      const float RAnkleRoll_Min       = DEG2RAD(-25);
      const float RAnkleRoll_Max       = DEG2RAD(45);
      const float RShoulderPitch_Min   = DEG2RAD(-120);
      const float RShoulderPitch_Max   = DEG2RAD(120);
      const float RShoulderRoll_Min    = DEG2RAD(-95);
      const float RShoulderRoll_Max    = DEG2RAD(0);
      const float RElbowRoll_Min       = DEG2RAD(0);
      const float RElbowRoll_Max       = DEG2RAD(90);
      const float RElbowYaw_Min        = DEG2RAD(-120);
      const float RElbowYaw_Max        = DEG2RAD(120);

      // Array to hold the maximum and minimum boundaries
      const float MaxAngle[] = {
         HeadYaw_Max,
         HeadPitch_Max,
         LShoulderPitch_Max,
         LShoulderRoll_Max,
         LElbowYaw_Max,
         LElbowRoll_Max,
         LHipYawPitch_Max,
         LHipRoll_Max,
         LHipPitch_Max,
         LKneePitch_Max,
         LAnklePitch_Max,
         LAnkleRoll_Max,
         RHipRoll_Max,
         RHipPitch_Max,
         RKneePitch_Max,
         RAnklePitch_Max,
         RAnkleRoll_Max,
         RShoulderPitch_Max,
         RShoulderRoll_Max,
         RElbowYaw_Max,
         RElbowRoll_Max
      };

      const float MinAngle[] = {
         HeadYaw_Min,
         HeadPitch_Min,
         LShoulderPitch_Min,
         LShoulderRoll_Min,
         LElbowYaw_Min,
         LElbowRoll_Min,
         LHipYawPitch_Min,
         LHipRoll_Min,
         LHipPitch_Min,
         LKneePitch_Min,
         LAnklePitch_Min,
         LAnkleRoll_Min,
         RHipRoll_Min,
         RHipPitch_Min,
         RKneePitch_Min,
         RAnklePitch_Min,
         RAnkleRoll_Min,
         RShoulderPitch_Min,
         RShoulderRoll_Min,
         RElbowYaw_Min,
         RElbowRoll_Min
      };
   };


   // Motor speed reductions. These determine the maximum rotational
   // speed of the motors controlling the joint. It is 'highly' advised
   // that when controlling the motors these limits are taken into
   // consideration.  They have been placed in the same namespace as
   // their joint angle counter-parts as it makes sense given the
   // classification.

   // Motor limits in Degrees
   namespace Degree {
      // Maximum rotation speed (Degrees) per cycle (10ms)
      const float MOTOR1_REDUCTION1_DEG = 1.89;
      const float MOTOR1_REDUCTION2_DEG = 2.90;
      const float MOTOR2_REDUCTION1_DEG = 3.53;
      const float MOTOR2_REDUCTION2_DEG = 3.05;

      const float HeadPitchSpeed       = MOTOR2_REDUCTION2_DEG;
      const float HeadYawSpeed         = MOTOR2_REDUCTION1_DEG;
      const float ShoulderPitchSpeed   = MOTOR2_REDUCTION1_DEG;
      const float ShoulderRollSpeed    = MOTOR2_REDUCTION2_DEG;
      const float ElbowYawSpeed        = MOTOR2_REDUCTION1_DEG;
      const float ElbowRollSpeed       = MOTOR2_REDUCTION2_DEG;
      const float HipYawPitchSpeed     = MOTOR1_REDUCTION1_DEG;
      const float HipRollSpeed         = MOTOR1_REDUCTION1_DEG;
      const float HipPitchSpeed        = MOTOR1_REDUCTION2_DEG;
      const float KneePitchSpeed       = MOTOR1_REDUCTION2_DEG;
      const float AnklePitchSpeed      = MOTOR1_REDUCTION2_DEG;
      const float AnkleRollSpeed       = MOTOR1_REDUCTION1_DEG;

      // Joint ordering is the same as angles
      const float MaxSpeed[] = {
         HeadYawSpeed,
         HeadPitchSpeed,
         ShoulderPitchSpeed,  // Left arm
         ShoulderRollSpeed,
         ElbowYawSpeed,
         ElbowRollSpeed,
         HipYawPitchSpeed,    // Left leg
         HipRollSpeed,
         HipPitchSpeed,
         KneePitchSpeed,
         AnklePitchSpeed,
         AnkleRollSpeed,
         HipYawPitchSpeed,    // Right leg
         HipRollSpeed,
         HipPitchSpeed,
         KneePitchSpeed,
         AnklePitchSpeed,
         AnkleRollSpeed,
         ShoulderPitchSpeed,  // Right arm
         ShoulderRollSpeed,
         ElbowYawSpeed,
         ElbowRollSpeed
      };
   };

   // Motor limits in Radians
   namespace Radians {
      // Maximum rotation speed (Radians) per cycle (10ms)
      const float MOTOR1_REDUCTION1_RAD = DEG2RAD(1.89);
      const float MOTOR1_REDUCTION2_RAD = DEG2RAD(2.90);
      const float MOTOR2_REDUCTION1_RAD = DEG2RAD(3.53);
      const float MOTOR2_REDUCTION2_RAD = DEG2RAD(3.05);

      const float HeadPitchSpeed       = MOTOR2_REDUCTION2_RAD;
      const float HeadYawSpeed         = MOTOR2_REDUCTION1_RAD;
      const float ShoulderPitchSpeed   = MOTOR2_REDUCTION1_RAD;
      const float ShoulderRollSpeed    = MOTOR2_REDUCTION2_RAD;
      const float ElbowYawSpeed        = MOTOR2_REDUCTION1_RAD;
      const float ElbowRollSpeed       = MOTOR2_REDUCTION2_RAD;
      const float HipYawPitchSpeed     = MOTOR1_REDUCTION1_RAD;
      const float HipRollSpeed         = MOTOR1_REDUCTION1_RAD;
      const float HipPitchSpeed        = MOTOR1_REDUCTION2_RAD;
      const float KneePitchSpeed       = MOTOR1_REDUCTION2_RAD;
      const float AnklePitchSpeed      = MOTOR1_REDUCTION2_RAD;
      const float AnkleRollSpeed       = MOTOR1_REDUCTION1_RAD;

      const float MaxSpeed[] = {
         HeadYawSpeed,
         HeadPitchSpeed,
         ShoulderPitchSpeed,  // Left arm
         ShoulderRollSpeed,
         ElbowYawSpeed,
         ElbowRollSpeed,
         HipYawPitchSpeed,    // Left leg
         HipRollSpeed,
         HipPitchSpeed,
         KneePitchSpeed,
         AnklePitchSpeed,
         AnkleRollSpeed,
         HipYawPitchSpeed,    // Right leg
         HipRollSpeed,
         HipPitchSpeed,
         KneePitchSpeed,
         AnklePitchSpeed,
         AnkleRollSpeed,
         ShoulderPitchSpeed,  // Right arm
         ShoulderRollSpeed,
         ElbowYawSpeed,
         ElbowRollSpeed
      };
   };




   /**
    * Given the specified joint, caps it at its limits if the angle exceeds the boundary
    * @param joint Joint to check the angle against
    * @param angle Angle to check (in DEGREES)
    */
   static inline float limitJointDegrees(JointCode joint, const float &angle) {
      if (std::isnan(angle)) return 0.0;
      if (angle > Degrees::MaxAngle[joint]) return Degrees::MaxAngle[joint];
      if (angle < Degrees::MinAngle[joint]) return Degrees::MinAngle[joint];
      return angle;
   }

   /**
    * Given the specified joint, caps it at its limits if the angle exceeds the boundary
    * @param joint Joint to check the angle against
    * @param angle Angle to check (in RADIANS)
    */
   static inline float limitJointRadians(JointCode joint, const float &angle) {
      if (std::isnan(angle)) return 0.0;
      if (angle > Radians::MaxAngle[joint]) return Radians::MaxAngle[joint];
      if (angle < Radians::MinAngle[joint]) return Radians::MinAngle[joint];
      return angle;
   }
};

/**
 * Limb Masses and Lengths
 * Taken from the Naoqi documentation. Limb names as Naoqi uses.
 */

namespace Limbs {
   const float NeckOffsetZ = 126.5;
   const float ShoulderOffsetY = 98.0;
   const float UpperArmLength = 90.0;
   const float LowerArmLength = 55.55;
   const float ShoulderOffsetZ = 100.0;
   const float HipOffsetZ = 84.79;
   const float HipOffsetY = 49.79;
   const float ThighLength = 100.0;
   const float TibiaLength = 102.74;
   const float FootHeight = 45.11;

   const float Length[] = {
      NeckOffsetZ,
      ShoulderOffsetY,
      UpperArmLength,
      LowerArmLength,
      ShoulderOffsetZ,
      HipOffsetZ,
      HipOffsetY,
      ThighLength,
      TibiaLength,
      FootHeight
   };

   // TODO(stuartr) these aren't correct for the V3+
   const float ChestMass = 1026.3;
   const float HeadMass = 476.7;
   const float RightBicepMass = 121.7;
   const float RightPelvisMass = 72.4;
   const float RightThighMass = 398.;
   const float RightTibiaMass = 297.1;
   const float RightFootMass = 163.;
   const float LeftBicepMass = 120.7;
   const float LeftPelvisMass = 71.1;
   const float LeftThighMass = 397.8;
   const float LeftTibiaMass = 297.;
   const float LeftFootMass = 163.2;

   const float Mass[] = {
      ChestMass,
      HeadMass,
      RightBicepMass,
      RightPelvisMass,
      RightThighMass,
      RightTibiaMass,
      RightFootMass,
      LeftBicepMass,
      LeftPelvisMass,
      LeftThighMass,
      LeftTibiaMass,
      LeftFootMass,
   };
};

namespace LEDs {
   enum LEDCode {
      LeftEar1 = 0,
      LeftEar2,
      LeftEar3,
      LeftEar4,
      LeftEar5,
      LeftEar6,
      LeftEar7,
      LeftEar8,
      LeftEar9,
      LeftEar10,
      RightEar1,
      RightEar2,
      RightEar3,
      RightEar4,
      RightEar5,
      RightEar6,
      RightEar7,
      RightEar8,
      RightEar9,
      RightEar10,
      LeftEyeRed1,
      LeftEyeRed2,
      LeftEyeRed3,
      LeftEyeRed4,
      LeftEyeRed5,
      LeftEyeRed6,
      LeftEyeRed7,
      LeftEyeRed8,
      LeftEyeGreen1,
      LeftEyeGreen2,
      LeftEyeGreen3,
      LeftEyeGreen4,
      LeftEyeGreen5,
      LeftEyeGreen6,
      LeftEyeGreen7,
      LeftEyeGreen8,
      LeftEyeBlue1,
      LeftEyeBlue2,
      LeftEyeBlue3,
      LeftEyeBlue4,
      LeftEyeBlue5,
      LeftEyeBlue6,
      LeftEyeBlue7,
      LeftEyeBlue8,
      RightEyeRed1,
      RightEyeRed2,
      RightEyeRed3,
      RightEyeRed4,
      RightEyeRed5,
      RightEyeRed6,
      RightEyeRed7,
      RightEyeRed8,
      RightEyeGreen1,
      RightEyeGreen2,
      RightEyeGreen3,
      RightEyeGreen4,
      RightEyeGreen5,
      RightEyeGreen6,
      RightEyeGreen7,
      RightEyeGreen8,
      RightEyeBlue1,
      RightEyeBlue2,
      RightEyeBlue3,
      RightEyeBlue4,
      RightEyeBlue5,
      RightEyeBlue6,
      RightEyeBlue7,
      RightEyeBlue8,
      ChestRed,
      ChestGreen,
      ChestBlue,
      LeftFootRed,
      LeftFootGreen,
      LeftFootBlue,
      RightFootRed,
      RightFootGreen,
      RightFootBlue,
      NUMBER_OF_LEDS
   };

   const std::string ledNames[NUMBER_OF_LEDS] = {
      "Ears/Led/Left/0Deg/Actuator/Value",
      "Ears/Led/Left/36Deg/Actuator/Value",
      "Ears/Led/Left/72Deg/Actuator/Value",
      "Ears/Led/Left/108Deg/Actuator/Value",
      "Ears/Led/Left/144Deg/Actuator/Value",
      "Ears/Led/Left/180Deg/Actuator/Value",
      "Ears/Led/Left/216Deg/Actuator/Value",
      "Ears/Led/Left/252Deg/Actuator/Value",
      "Ears/Led/Left/288Deg/Actuator/Value",
      "Ears/Led/Left/324Deg/Actuator/Value",
      "Ears/Led/Right/0Deg/Actuator/Value",
      "Ears/Led/Right/36Deg/Actuator/Value",
      "Ears/Led/Right/72Deg/Actuator/Value",
      "Ears/Led/Right/108Deg/Actuator/Value",
      "Ears/Led/Right/144Deg/Actuator/Value",
      "Ears/Led/Right/180Deg/Actuator/Value",
      "Ears/Led/Right/216Deg/Actuator/Value",
      "Ears/Led/Right/252Deg/Actuator/Value",
      "Ears/Led/Right/288Deg/Actuator/Value",
      "Ears/Led/Right/324Deg/Actuator/Value",
      "Face/Led/Red/Left/0Deg/Actuator/Value",
      "Face/Led/Red/Left/45Deg/Actuator/Value",
      "Face/Led/Red/Left/90Deg/Actuator/Value",
      "Face/Led/Red/Left/135Deg/Actuator/Value",
      "Face/Led/Red/Left/180Deg/Actuator/Value",
      "Face/Led/Red/Left/225Deg/Actuator/Value",
      "Face/Led/Red/Left/270Deg/Actuator/Value",
      "Face/Led/Red/Left/315Deg/Actuator/Value",
      "Face/Led/Green/Left/0Deg/Actuator/Value",
      "Face/Led/Green/Left/45Deg/Actuator/Value",
      "Face/Led/Green/Left/90Deg/Actuator/Value",
      "Face/Led/Green/Left/135Deg/Actuator/Value",
      "Face/Led/Green/Left/180Deg/Actuator/Value",
      "Face/Led/Green/Left/225Deg/Actuator/Value",
      "Face/Led/Green/Left/270Deg/Actuator/Value",
      "Face/Led/Green/Left/315Deg/Actuator/Value",
      "Face/Led/Blue/Left/0Deg/Actuator/Value",
      "Face/Led/Blue/Left/45Deg/Actuator/Value",
      "Face/Led/Blue/Left/90Deg/Actuator/Value",
      "Face/Led/Blue/Left/135Deg/Actuator/Value",
      "Face/Led/Blue/Left/180Deg/Actuator/Value",
      "Face/Led/Blue/Left/225Deg/Actuator/Value",
      "Face/Led/Blue/Left/270Deg/Actuator/Value",
      "Face/Led/Blue/Left/315Deg/Actuator/Value",
      "Face/Led/Red/Right/0Deg/Actuator/Value",
      "Face/Led/Red/Right/45Deg/Actuator/Value",
      "Face/Led/Red/Right/90Deg/Actuator/Value",
      "Face/Led/Red/Right/135Deg/Actuator/Value",
      "Face/Led/Red/Right/180Deg/Actuator/Value",
      "Face/Led/Red/Right/225Deg/Actuator/Value",
      "Face/Led/Red/Right/270Deg/Actuator/Value",
      "Face/Led/Red/Right/315Deg/Actuator/Value",
      "Face/Led/Green/Right/0Deg/Actuator/Value",
      "Face/Led/Green/Right/45Deg/Actuator/Value",
      "Face/Led/Green/Right/90Deg/Actuator/Value",
      "Face/Led/Green/Right/135Deg/Actuator/Value",
      "Face/Led/Green/Right/180Deg/Actuator/Value",
      "Face/Led/Green/Right/225Deg/Actuator/Value",
      "Face/Led/Green/Right/270Deg/Actuator/Value",
      "Face/Led/Green/Right/315Deg/Actuator/Value",
      "Face/Led/Blue/Right/0Deg/Actuator/Value",
      "Face/Led/Blue/Right/45Deg/Actuator/Value",
      "Face/Led/Blue/Right/90Deg/Actuator/Value",
      "Face/Led/Blue/Right/135Deg/Actuator/Value",
      "Face/Led/Blue/Right/180Deg/Actuator/Value",
      "Face/Led/Blue/Right/225Deg/Actuator/Value",
      "Face/Led/Blue/Right/270Deg/Actuator/Value",
      "Face/Led/Blue/Right/315Deg/Actuator/Value",
      "ChestBoard/Led/Red/Actuator/Value",
      "ChestBoard/Led/Green/Actuator/Value",
      "ChestBoard/Led/Blue/Actuator/Value",
      "LFoot/Led/Red/Actuator/Value",
      "LFoot/Led/Green/Actuator/Value",
      "LFoot/Led/Blue/Actuator/Value",
      "RFoot/Led/Red/Actuator/Value",
      "RFoot/Led/Green/Actuator/Value",
      "RFoot/Led/Blue/Actuator/Value",
   };
};

namespace Sensors {
   typedef enum SensorCodesEnum {
      InertialSensor_AccX = 0,
      InertialSensor_AccY,
      InertialSensor_AccZ,
      InertialSensor_GyrX,
      InertialSensor_GyrY,
      InertialSensor_GyrRef,
      InertialSensor_AngleX,
      InertialSensor_AngleY,

      LFoot_FSR_FrontLeft,
      LFoot_FSR_FrontRight,
      LFoot_FSR_RearLeft,
      LFoot_FSR_RearRight,
      LFoot_FSR_CenterOfPressure_X,
      LFoot_FSR_CenterOfPressure_Y,

      RFoot_FSR_FrontLeft,
      RFoot_FSR_FrontRight,
      RFoot_FSR_RearLeft,
      RFoot_FSR_RearRight,
      RFoot_FSR_CenterOfPressure_X,
      RFoot_FSR_CenterOfPressure_Y,

      LFoot_Bumper_Left,
      LFoot_Bumper_Right,
      RFoot_Bumper_Left,
      RFoot_Bumper_Right,

      ChestBoard_Button,
      Battery_Charge,
      Battery_Current,
      US,

      NUMBER_OF_SENSORS
   } SensorCode;

   const SensorCode sensorCodes[] = {
      InertialSensor_AccX,
      InertialSensor_AccY,
      InertialSensor_AccZ,
      InertialSensor_GyrX,
      InertialSensor_GyrY,
      InertialSensor_GyrRef,
      InertialSensor_AngleX,
      InertialSensor_AngleY,

      LFoot_FSR_FrontLeft,
      LFoot_FSR_FrontRight,
      LFoot_FSR_RearLeft,
      LFoot_FSR_RearRight,
      LFoot_FSR_CenterOfPressure_X,
      LFoot_FSR_CenterOfPressure_Y,

      RFoot_FSR_FrontLeft,
      RFoot_FSR_FrontRight,
      RFoot_FSR_RearLeft,
      RFoot_FSR_RearRight,
      RFoot_FSR_CenterOfPressure_X,
      RFoot_FSR_CenterOfPressure_Y,

      LFoot_Bumper_Left,
      LFoot_Bumper_Right,
      RFoot_Bumper_Left,
      RFoot_Bumper_Right,

      ChestBoard_Button,
      Battery_Charge,
      Battery_Current,
      US
   };

   const std::string sensorNames[] = {
      "InertialSensor/AccX",
      "InertialSensor/AccY",
      "InertialSensor/AccZ",
      "InertialSensor/GyrX",
      "InertialSensor/GyrY",
      "InertialSensor/GyrRef",
      "InertialSensor/AngleX",
      "InertialSensor/AngleY",

      "LFoot/FSR/FrontLeft",
      "LFoot/FSR/FrontRight",
      "LFoot/FSR/RearLeft",
      "LFoot/FSR/RearRight",
      "LFoot/FSR/CenterOfPressure/X",
      "LFoot/FSR/CenterOfPressure/Y",

      "RFoot/FSR/FrontLeft",
      "RFoot/FSR/FrontRight",
      "RFoot/FSR/RearLeft",
      "RFoot/FSR/RearRight",
      "RFoot/FSR/CenterOfPressure/X",
      "RFoot/FSR/CenterOfPressure/Y",

      "LFoot/Bumper/Left",
      "LFoot/Bumper/Right",
      "RFoot/Bumper/Left",
      "RFoot/Bumper/Right",

      "ChestBoard/Button",
      "Battery/Charge",
      "Battery/Current",
      "US"
   };
}

namespace Sonar {
   // From Naoqi docs. Typical use might be something like ON + CONTINUOUS
   namespace Mode {
      const float OFF        = 0.0f;
      const float ON         = 4.0f;
      const float BOTH       = 8.0f;
      const float CONTINUOUS = 64.0f;
   }

   const float MIN = 0.29f;
   const float INVALID = 2.55f;

   const std::string actuatorName = "US/Actuator/Value";

   typedef enum ReadingCodesEnum {
      Left0,
      Left1,
      Left2,
      Left3,
      Left4,
      Left5,
      Left6,
      Left7,
      Left8,
      Left9,
      Right0,
      Right1,
      Right2,
      Right3,
      Right4,
      Right5,
      Right6,
      Right7,
      Right8,
      Right9,
      NUMBER_OF_READINGS
   } ReadingCode;

   const ReadingCode readingCodes[] = {
      Left0,
      Left1,
      Left2,
      Left3,
      Left4,
      Left5,
      Left6,
      Left7,
      Left8,
      Left9,
      Right0,
      Right1,
      Right2,
      Right3,
      Right4,
      Right5,
      Right6,
      Right7,
      Right8,
      Right9
   };

   const std::string readingNames[] = {
      "US/Left/Sensor/Value",
      "US/Left/Sensor/Value1",
      "US/Left/Sensor/Value2",
      "US/Left/Sensor/Value3",
      "US/Left/Sensor/Value4",
      "US/Left/Sensor/Value5",
      "US/Left/Sensor/Value6",
      "US/Left/Sensor/Value7",
      "US/Left/Sensor/Value8",
      "US/Left/Sensor/Value9",
      "US/Right/Sensor/Value",
      "US/Right/Sensor/Value1",
      "US/Right/Sensor/Value2",
      "US/Right/Sensor/Value3",
      "US/Right/Sensor/Value4",
      "US/Right/Sensor/Value5",
      "US/Right/Sensor/Value6",
      "US/Right/Sensor/Value7",
      "US/Right/Sensor/Value8",
      "US/Right/Sensor/Value9"
   };
}
