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
 * FastWalkGenerator.hpp
 * BH 5th May 2010
 */

#pragma once

#include <cmath>
#include "motion/generator/Generator.hpp"
#include "utils/XYZ_Coord.hpp"


class FastWalkGenerator : Generator {
   public:
   explicit FastWalkGenerator();
   ~FastWalkGenerator();
   JointValues makeJoints(ActionCommand::All* request,
                          Odometry* odometry,
                          const SensorValues &sensors);
   bool isActive();
   ActionCommand::Body active;

   void readOptions(boost::program_options::variables_map& config);
   void reset();
   void stop();
   private:
   // config option for how high the walk should be
   float bend;

   bool stopping;
   bool stopped;
   bool legsTogether;
   float z;  // zero
   // Time-step
   float dt;
   // Timers
   float t;

   // walk limits
   float limForwardLPatter;
   float limForwardRPatter;
   float limTurnPatter;
   float limLeftPatter;

   // arms
   float shoulderPitchL;
   float shoulderPitchR;
   float shoulderRollL;
   float shoulderRollR;
   float elbowYawR;
   float elbowYawL;
   float elbowRollL;
   float elbowRollR;

   // coronal angles
   float rock;
   float maxLeft;
   float leftR;
   float lastLeftR;
   float leftL;
   float lastLeftL;

   // sagittal angles
   float legBendConst;
   float maxLegBend;
   float legBend;
   float firstLegBend;
   float lastLegBend;
   float stepHeight;

   float maxForwardL;
   float lastForwardL;
   float forwardL;
   float maxForwardR;
   float forwardR;
   float lastForwardR;

   float liftL;
   float lastLiftL;
   float liftR;
   float lastLiftR;

   float maxTurn;
   float turnLR;
   float lastTurnLR;

   float maxPower;

   void initialise();

   float T;
   float lastSinu;
   float diff;
   float filZMPL;
   float lastZMPL;
   float filteredTotalPressure;
   float liftTimer;
   bool leftPhase;
   float filHighZMPF;
   float filLowZMPF;
   float filAcc;

   struct Hpr {
      float Hp;
      float Hr;
      // Hpr(): Hp(0.0f), Hr(0.0f) { }
   };

   /**
    *  calculates the sinusoidal position along a path from start to finish 
    *  given the total time available T and the current time
    *  returns a value between start and finish inclusive
    */
   float moveSin(float start, float finish, float fraction);

   /**
    *  Walk Cycle Helpers
    */
   bool targetWalkPatter(float forward, float left, float turn);
   // bool targetWalkStomp(float forward, float left, float turn);
   bool still(float forwardL, float forwardR, float left, float turn);

   // Foot to Body coord transform
   XYZ_Coord mf2b(float Hyp, float Hp, float Hr, float Kp, float Ap,
                  float Ar, float xf, float yf, float zf);
   Hpr hipAngles(float Hyp, float Hp, float Hr, float Kp, float Ap,
                 float Ar, float xf, float yf, float zf, XYZ_Coord e);
};

