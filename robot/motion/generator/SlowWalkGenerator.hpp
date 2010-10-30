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
 * SlowWalkGenerator.hpp
 * BH 16th Jan 2010
 */

#pragma once

#include <cmath>
#include "motion/generator/Generator.hpp"
#include "utils/XYZ_Coord.hpp"


class SlowWalkGenerator : Generator {
   public:
   // Walking phase
   enum WalkPhase {
      Start        = 0,
      RockLR       = 1,
      RockRL       = 2,
      LiftFootR    = 3,
      FootForwardR = 4,
      DownFootR    = 5,
      LiftFootL    = 6,
      FootForwardL = 7,
      DownFootL    = 8,
      // BendKnees    = 9,
      FootLeftL    = 9,
      FootRightR   = 10,
      LeftKick     = 11,
      BackKickL    = 12,
      None         = 13,
      RightKick    = 14,
      ForwardKickL = 15,
      ForwardKickR = 16,
      BackKickR    = 17,
      ForwardRight = 18,
      ForwardLeft  = 19,
      ForwardPassingL = 20,
      ForwardPassingR = 21,
      NUMBER_OF_WALK_PHASES
   };
   explicit SlowWalkGenerator();
   ~SlowWalkGenerator();
   JointValues makeJoints(ActionCommand::All* request,
                          Odometry* odometry,
                          const SensorValues &sensors);
   bool isActive();
   WalkPhase walkPhase;
   WalkPhase kickPhase;
   ActionCommand::Body active;

   void readOptions(boost::program_options::variables_map& config);
   void reset();
   void stop();
   private:
   bool stopping;
   bool stopped;
   bool legsTogether;
   // Time-step
   float dt;
   // Timers
   float t;
   float globalTime;
   float rockTime;
   float footForwardTime;

   float z;  // zero

   // coronal angles
   float rock;
   float coronalLeanL;
   float lastCoronalLeanL;
   float coronalLeanR;
   float lastCoronalLeanR;
   float maxLeft;
   float leftR;
   float lastLeftR;
   float leftL;
   float lastLeftL;

   // sagittal angles
   float legBendConst;
   float liftBend;
   float maxLegBend;
   float legBend;
   float firstLegBend;
   float lastLegBend;
   float legLiftHeight;

   float maxForwardL;
   float maxForwardR;
   float forwardL;
   float lastForwardL;
   float forwardR;
   float lastForwardR;

   float liftL;
   float lastLiftL;
   float liftR;
   float lastLiftR;

   float maxTurn;
   float turnLR;
   float lastTurnLR;

   // Kicks
   int kicksLeft;
   bool kicking;
   float backKickAnklePitchL;
   float backKickAnklePitchR;
   float shoulderPitchL;
   float shoulderPitchR;
   float shoulderRollL;
   float shoulderRollR;
   float kickPeriod;
   float forwardKickPitch;

   void initialise();

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
   float moveSin(float start, float finish, float period);

   /**
    *  Walk Cycle Options
    */
   void oWalkCycle(float period, WalkPhase nextPhase);
   void oStart(float period, WalkPhase nextPhase);
   // void oBendKnees(float period, WalkPhase nextPhase);
   void oLiftFootR(float period, WalkPhase nextPhase);
   void oDownFootR(float period, WalkPhase nextPhase);
   void oLiftFootL(float period, WalkPhase nextPhase);
   void oDownFootL(float period, WalkPhase nextPhase);
   void oRockRL(float period, WalkPhase nextPhase);
   void oRockLR(float period, WalkPhase nextPhase);
   void oFootForwardL(float period, WalkPhase nextPhase);
   void oFootForwardR(float period, WalkPhase nextPhase);
   void oFootLeftL(float period, WalkPhase nextPhase);
   void oFootRightR(float period, WalkPhase nextPhase);

   // Kicks
   void oKickL(float forwardK, float leftK, float turnK,
               WalkPhase nextPhase);
   void oKickR(float forwardK, float leftK, float turnK,
               WalkPhase nextPhase);

   // Foot to Body coord transform
   XYZ_Coord mf2b(float Hyp, float Hp, float Hr, float Kp, float Ap,
                  float Ar, float xf, float yf, float zf);
   Hpr hipAngles(float Hyp, float Hp, float Hr, float Kp, float Ap,
                 float Ar, float xf, float yf, float zf, XYZ_Coord e);
};

