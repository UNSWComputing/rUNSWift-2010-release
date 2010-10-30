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
 * SlowWalkGenerator.cpp
 * BH 31 Mar 2010
 */

#include "motion/generator/SlowWalkGenerator.hpp"
#include <cmath>
#include "utils/angles.hpp"
#include "utils/body.hpp"
#include "utils/log.hpp"
#include "utils/basic_maths.hpp"

using boost::program_options::variables_map;
using namespace std;
using namespace Joints;
using namespace Sensors;

SlowWalkGenerator::SlowWalkGenerator()
   :t(0.0f), z(0.0f) {
   initialise();
   llog(INFO) << "SlowWalkGenerator constructed" << std::endl;
}

SlowWalkGenerator::~SlowWalkGenerator() {
   llog(INFO) << "SlowWalkGenerator destroyed" << std::endl;
}

void SlowWalkGenerator::initialise() {
   llog(INFO) << "slowwalk initializing" << endl;
   legBendConst = legBend = lastLegBend = maxLegBend = DEG2RAD(30.0f);
   dt = 0.01f;
   t = globalTime = 0.0f;
   stopping = false;
   stopped = true;
   legsTogether = false;
   rock = DEG2RAD(22);
   coronalLeanL = lastCoronalLeanL = coronalLeanR = lastCoronalLeanR = 0.0f;
   maxLeft  = leftL = lastLeftL =  leftR =  lastLeftR  = 0.0f;   // max 50 mm
   legLiftHeight = DEG2RAD(5.0f);
   liftR = lastLiftR = liftL = lastLiftL = 0.0f;
   maxForwardL = maxForwardR = 0.0f;  // max 60.0 mm
   forwardL  = lastForwardL = forwardR = lastForwardR = 0.0f;
   maxTurn = turnLR = lastTurnLR = 0.0f;  // max 40 deg per step
   walkPhase = RockLR;
   kicksLeft = 1;
   kicking = false;
   kickPhase = None;
   backKickAnklePitchL = backKickAnklePitchR = 0.0f;
   shoulderPitchL = shoulderPitchR = shoulderRollL = shoulderRollR = 0.0f;
   kickPeriod = 20.0f;
   forwardKickPitch = 0.0f;
}

JointValues SlowWalkGenerator::makeJoints(ActionCommand::All* request,
                                          Odometry* odometry,
                                          const SensorValues &sensors) {
   float forward       = request->body.forward;
   float left          = request->body.left;
   float turn          = request->body.turn;
   float power         = request->body.power;
   float forwardK      = forward;
   float leftK         = left;
   float turnK         = turn;
   if (power > 0.0f) forward = left = turn = 0.0f;

   if (t == 0.0f) {  // Accept new walk body commands
      active = request->body;

      if (stopping) {
         maxForwardL = maxForwardR = 0.0f;
         maxTurn = 0.0f;
         maxLeft = 0.0f;
         if (legsTogether) {
            rock = 0.0f;
            legLiftHeight = 0.0f;
         }
      } else {
         stopped = false;  // (re)activate slowwalk
         // Scale back ambitious omni-directional values: forward, left, turn
         float scaling = 1.0f;  // assume all OK
         if (ABS(forward)/60.0f > scaling)  scaling = ABS(forward)/60.0f;
         if (ABS(left)/50.0f > scaling)     scaling = ABS(left)/50.0f;
         float turnLimit = DEG2RAD(40.0f);
         if (ABS(turn)/turnLimit > scaling) scaling = ABS(turn)/turnLimit;
         forward /= scaling;
         left /=  scaling;
         turn /= scaling;
         float sum = ABS(forward) + ABS(left);   // keep proportional
         if (sum > 50.0f) {
            forward *= ABS(forward)/sum;
            left    *= ABS(left)/sum;
         }
         // llog(INFO) << "max1.* " << forward << " " << left <<
         //    " " << turn << std::endl;

         // slowwalk interface & adjustments
         // rock = DEG2RAD(23.0);
         maxForwardL = maxForwardR = forward;
         maxLeft = left/200.0;  // approx
         maxTurn = turn;
         legLiftHeight = DEG2RAD(5.0f)*(1.0f-ABS(turn)/DEG2RAD(60.0f));
         legLiftHeight *= (1.0f-ABS(left)/60.0f);  // adjust for left
      }
      rockTime = 1.2f;
      footForwardTime = 0.5f;
      if (ABS(maxTurn) > DEG2RAD(20.0f))
         footForwardTime += 0.5f+0.2f*(ABS(maxTurn)-DEG2RAD(20.0f));
   } else {
      request->body = active;
   }

   float xEst = maxForwardL+maxForwardR;
   *odometry = *odometry + Odometry(xEst*dt/4.2f*1.01,
                                    maxLeft*200.0f*dt/4.2f*.98,
                                    1.2f*maxTurn/4.2f*dt);

   // Compensate for legBends under 30 degrees
   liftBend = 0.0f;
   if (legBendConst < DEG2RAD(30.0f)) {
      liftBend = (DEG2RAD(30.0f)-legBendConst)/2.0f;
      if (power > 0.8f) power = 0.8f;
   }

   // Update timers
   t += dt;
   globalTime += dt;
   //  llog(INFO) << "phase: " << walkPhase << "  t: " << t << std::endl;

   if (!kicking && ABS(power) > 0.01f && kicksLeft > 0) {
      kicking = true;
      kickPhase = None;
      if (power <= 0.0 ) power = 0.01;
      if (power >= 1.0 ) power = 1.0;
      if (ABS(forwardK-135.0f) < 0.01f) {
         if (ABS(leftK-60.0f) < 0.01f && ABS(turnK) < 0.01) {
            kickPhase = ForwardKickL;
            walkPhase = RockLR;
            kickPeriod = 0.9f - power * 0.5f;
         }
         if (ABS(leftK) < 0.01f && ABS(turnK+M_PI/4.0f) < 0.01) {
            kickPhase = ForwardRight;
            walkPhase = RockLR;
            kickPeriod = 1.5f - power * 0.4f;
         }
         if (ABS(leftK+60.0f) < 0.01f && ABS(turnK) < 0.01) {
            kickPhase = ForwardKickR;
            walkPhase = RockRL;
            kickPeriod = 0.9f - power * 0.5f;
         }
         if (ABS(leftK) < 0.01f && ABS(turnK-M_PI/4.0f) < 0.01) {
            kickPhase = ForwardLeft;
            walkPhase = RockRL;
            kickPeriod = 1.5f - power * 0.4f;
         }
         if (power > 0.0f &&
             ABS(leftK-60.0f) < 0.01f && ABS(turnK+M_PI/2.0f) < 0.01) {
            kickPhase = RightKick;
            walkPhase = RockLR;
            kickPeriod = 1.5 - power * 0.8f;
         }
         if (ABS(leftK+60.0f) < 0.01f && ABS(turnK-M_PI/2.0f) < 0.01) {
            kickPhase = LeftKick;
            walkPhase = RockRL;
            kickPeriod = 1.5 - power * 0.8f;
         }
         if (ABS(leftK-60.0f) < 0.01f && ABS(turnK-M_PI) < 0.01) {
            kickPhase = BackKickL;
            walkPhase = RockRL;
         }
         if (ABS(leftK+60.0f) < 0.01f && ABS(turnK-M_PI) < 0.01) {
            kickPhase = BackKickR;
            walkPhase = RockLR;
         }
      }
      if (ABS(forwardK-100.0f) < 0.01f) {
         if (ABS(leftK-60.0f) < 0.01f && ABS(turnK) < 0.01) {
            kickPhase = ForwardPassingL;
            walkPhase = RockRL;
            kickPeriod = 1.2+sqrt(0.15*0.15f/power);
         }
         if (ABS(leftK+60.0f) < 0.01f && ABS(turnK) < 0.01) {
            kickPhase = ForwardPassingR;
            walkPhase = RockLR;
            kickPeriod = 1.2+sqrt(0.15*0.15f/power);
         }
      }
      if (kickPhase == None) kicking = false;
   }
   // llog(INFO) << "kickPhase: " << kickPhase << std::endl;


   if (!kicking) {
      // Normal Walk
      if (walkPhase == RockLR) oRockLR(rockTime, LiftFootL);
      if (walkPhase == LiftFootL) oLiftFootL(0.2f, FootForwardL);
      if (walkPhase == FootForwardL) oFootForwardL(footForwardTime, DownFootL);
      if (walkPhase == DownFootL) oDownFootL(0.2f, RockRL);
      if (walkPhase == RockRL) oRockRL(rockTime, LiftFootR);
      if (walkPhase == LiftFootR) oLiftFootR(0.2f, FootForwardR);
      if (walkPhase == FootForwardR) oFootForwardR(footForwardTime, DownFootR);
      if (walkPhase == DownFootR) oDownFootR(0.2f, RockLR);
   } else {
      // Kick backwards
      if (kickPhase == BackKickL || kickPhase == ForwardPassingL) {  // two step
         if (walkPhase == RockLR) oRockLR(rockTime, LiftFootL);
         if (walkPhase == LiftFootL) oLiftFootL(0.5f, FootForwardL);
         if (walkPhase == FootForwardL) oKickL(forwardK, leftK, turnK,
                                                DownFootL);
         if (walkPhase == DownFootL) oDownFootL(0.6f, RockRL);
         // start here, weight on left foot - assume ball is at left foot
         maxForwardR = 65.0f;  // step forward
         if (walkPhase == RockRL) oRockRL(rockTime, LiftFootR);
         if (walkPhase == LiftFootR) oLiftFootR(0.5f, FootForwardR);
         if (walkPhase == FootForwardR)
                oFootForwardR(footForwardTime, DownFootR);
         if (walkPhase == DownFootR) oDownFootR(0.6f, RockLR);
      } else {
         if (kickPhase == BackKickR || kickPhase == ForwardPassingR) {  // 2 st.
            // start here, weight on right foot - assume ball is at right foot
            maxForwardL = 65.0f;  // step forward
            if (walkPhase == RockLR) oRockLR(rockTime, LiftFootL);
            if (walkPhase == LiftFootL) oLiftFootL(0.5f, FootForwardL);
            if (walkPhase == FootForwardL)
                  oFootForwardL(footForwardTime, DownFootL);
            if (walkPhase == DownFootL) oDownFootL(0.6f, RockRL);
            if (walkPhase == RockRL) oRockRL(rockTime, LiftFootR);
            if (walkPhase == LiftFootR) oLiftFootR(0.5f, FootForwardR);
            if (walkPhase == FootForwardR) oKickR(forwardK, leftK, turnK,
                                                DownFootR);
            if (walkPhase == DownFootR) oDownFootR(0.6f, RockLR);
         } else {
            // other kicks
            if (walkPhase == RockLR) oRockLR(0.8f, LiftFootL);
            if (walkPhase == LiftFootL) oLiftFootL(0.5f, FootForwardL);
            if (walkPhase == FootForwardL) oKickL(forwardK, leftK, turnK,
                                                  DownFootL);
            if (walkPhase == DownFootL) oDownFootL(0.6f, RockRL);
            if (walkPhase == RockRL) oRockRL(0.8f, LiftFootR);
            if (walkPhase == LiftFootR) oLiftFootR(0.5f, FootForwardR);
            if (walkPhase == FootForwardR) oKickR(forwardK, leftK, turnK,
                                                  DownFootR);
            if (walkPhase == DownFootR) oDownFootR(0.6f, RockLR);
         }
      }
   }

   // calculate sagittal forward walk values (closed-form inverse kinematics)
   float dirL    = 1.0f;
   if (forwardL < 0.0f) dirL = -1.0f;
   float HpL0    = liftL + legBend;
   HpL0 += (ABS(leftL) + ABS(leftR))/1.5f;
   float hL      = 100.0f*cos(HpL0);
   hL           += sqrt(102.74f*102.74f-10000.0f*sin(HpL0)*sin(HpL0));
   hL           /= cos(coronalLeanL + leftL);
   float dL      = sqrt(forwardL*forwardL + hL*hL);
   float beta1L  = acos((102.74f*102.74f+dL*dL-10000.0f)/(2.0f*102.74f*dL));
   float beta2L  = acos((10000.0f+dL*dL-102.74f*102.74f)/(2.0f*100.0f*dL));
   float tempL   = hL/dL;
   if (tempL > 1.0f) tempL = 1.0f;
   float deltaL  = asin(tempL);
   float HpL     = beta1L + dirL*(M_PI/2.0f - deltaL);
   float ApL     = beta2L + dirL*(deltaL - M_PI/2.0f);
   float KpL     = HpL + ApL;

   float dirR    = 1.0f;
   if (forwardR < 0.0f) dirR = -1.0f;
   float HpR0    = liftR + legBend;
   HpR0 += (ABS(leftL) + ABS(leftR))/1.5f;
   float hR      = 100.0f*cos(HpR0);
   hR           += sqrt(102.74f*102.74f-10000.0f*sin(HpR0)*sin(HpR0));
   hR           /= cos(coronalLeanR + leftR);
   float dR      = sqrt(forwardR*forwardR + hR*hR);
   float beta1R  = acos((102.74f*102.74f+dR*dR-10000.0f)/(2.0f*102.74f*dR));
   float beta2R  = acos((10000.0f+dR*dR-102.74f*102.74f)/(2.0f*100.0f*dR));
   float tempR   = hR/dR;
   if (tempR > 1.0f) tempR = 1.0f;
   float deltaR  = asin(tempR);
   float HpR     = beta1R + dirR*(M_PI/2.0f - deltaR);
   float ApR     = beta2R + dirR*(deltaR - M_PI/2.0f);
   float KpR     = HpR + ApR;
   //
   float HrL = coronalLeanL + leftL;
   float HrR = coronalLeanR + leftR;
   float ArL = -HrL;
   float ArR = -HrR;

   // Adjust HpL, HrL, ApL, ArL LEFT based on Hyp turn to keep ankle in situ
   XYZ_Coord tL = mf2b(z, -HpL, HrL, KpL, -ApL, ArL, z, z, z);
   XYZ_Coord sL;
   float Hyp = -turnLR;
   for (int i = 0; i < 3; i++) {
      sL = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, z, z, z);
      XYZ_Coord e((tL.x-sL.x), (tL.y-sL.y), (tL.z-sL.z));
      Hpr hpr = hipAngles(Hyp, -HpL, HrL, KpL, -ApL, ArL, z, z, z, e);
      HpL -= hpr.Hp;
      HrL += hpr.Hr;
   }
   // ApL and ArL to make sure LEFT foot is parallel to ground
   XYZ_Coord up = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, 1.0f, 0.0f, 0.0f);
   XYZ_Coord ur = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, 0.0f, 1.0f, 0.0f);
   ApL = ApL + asin(sL.z-up.z);
   ArL = ArL + asin(sL.z-ur.z);

   // Adjust HpR, HrR, ApR, ArR (RIGHT) based on Hyp turn to keep ankle in situ
   // Map to LEFT - we reuse the left foot IK because of symmetry right foot
   float Hr = -HrR;
   float Ar = -ArR;
   // Target foot origin in body coords
   XYZ_Coord t = mf2b(z, -HpR, Hr, KpR, -ApR, Ar, z, z, z);
   XYZ_Coord s;
   Hyp = -turnLR;
   for (int i = 0; i < 3; i++) {
      s = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, z, z, z);
      XYZ_Coord e((t.x-s.x), (t.y-s.y), (t.z-s.z));
      Hpr hpr = hipAngles(Hyp, -HpR, Hr, KpR, -ApR, Ar, z, z, z, e);
      HpR -= hpr.Hp;
      Hr += hpr.Hr;
   }
   // Ap and Ar to make sure foot is parallel to ground
   XYZ_Coord u1 = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, 1.0f, 0.0f, 0.0f);
   XYZ_Coord u2 = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, 0.0f, 1.0f, 0.0f);
   ApR = ApR + asin(s.z-u1.z);
   Ar = Ar + asin(s.z-u2.z);
   // map back from left foot to right foot
   HrR = -Hr;
   ArR = -Ar;


   JointValues j = sensors.joints;
   for (uint8_t i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
      j.stiffnesses[i] = 1.0f;  //  was 0.9f

   // Arms
   j.angles[LShoulderPitch] = DEG2RAD(90)+shoulderPitchL;
   j.angles[LShoulderRoll] = DEG2RAD(15)+shoulderRollL;
   j.angles[LElbowYaw] = DEG2RAD(0);
   j.angles[LElbowRoll] = DEG2RAD(0);

   j.angles[RShoulderPitch] = DEG2RAD(90)+shoulderPitchR;
   j.angles[RShoulderRoll] = DEG2RAD(-15)-shoulderRollR;
   j.angles[RElbowYaw] = DEG2RAD(0);
   j.angles[RElbowRoll] = DEG2RAD(0);

   // Turn
   j.angles[Joints::LHipYawPitch] = -turnLR;

   // Sagittal Joints
   j.angles[Joints::LHipPitch]    = -HpL+forwardKickPitch;
   j.angles[Joints::RHipPitch]    = -HpR+forwardKickPitch;
   j.angles[Joints::LKneePitch]   =  KpL;
   j.angles[Joints::RKneePitch]   =  KpR;
   j.angles[Joints::LAnklePitch]  = -ApL+backKickAnklePitchL;
   j.angles[Joints::RAnklePitch]  = -ApR+backKickAnklePitchR;

   // Coronal Joints
   j.angles[Joints::LHipRoll]     =  HrL;
   j.angles[Joints::RHipRoll]     =  HrR;
   j.angles[Joints::LAnkleRoll]   =  ArL;
   j.angles[Joints::RAnkleRoll]   =  ArR;

   return j;
}

bool SlowWalkGenerator::isActive() {
   return !stopped;
}

void SlowWalkGenerator::readOptions(variables_map& config) {}

void SlowWalkGenerator:: reset() {
   initialise();
   llog(INFO) << "slowwalk reset" << endl;
}

void SlowWalkGenerator:: stop() {
   stopping = true;
   llog(INFO) << "slowwalk stop" << endl;
}

float SlowWalkGenerator:: moveSin(float start, float finish,
                                         float period) {
   if ( period <= 0.0f || t < 0 ) return start;
   if ( t > period )              return finish;
   float difference = finish - start;
   float angle = M_PI * t / period;
   return start + difference * (1 - cos(angle))/2.0f;
}

void SlowWalkGenerator::oKickL(float forwardK, float leftK, float turnK,
                               WalkPhase nextPhase) {
   if (kickPhase == ForwardKickL) {  // kick forward with left foot
      // rock = DEG2RAD(22.0);
      float period = kickPeriod - 0.28f;
      if (t >= 0 && t < 0.14f) {
         float angle = t*M_PI/0.14f;
         forwardL = -80.0f * (1 - cos(angle))/2.0f;
         shoulderPitchL = -DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         shoulderPitchR = DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         liftL = lastLiftL
                 + (liftBend+DEG2RAD(10)) * (1.0f - cos(angle))/2.0f;
         legBend = legBendConst + DEG2RAD(5) * (1.0f - cos(angle))/2.0f;
      }
      if (t >= 0.14f && t < 0.14f+period) {  // kick
         float angle = (t-0.14f)*M_PI/period;
         forwardL = -80.0f + 160.0f * (1 - cos(angle))/2.0f;
         shoulderPitchL = -DEG2RAD(30.0f)
             + DEG2RAD(60.0f) * (1 - cos(angle))/2.0f;
         shoulderPitchR = DEG2RAD(30.0f)
             - DEG2RAD(60.0f) * (1 - cos(angle))/2.0f;
         legBend = legBendConst + DEG2RAD(5) * (1.0f + cos(angle))/2.0f;
      }
      if (t >= 0.14f+period+0.1f && t < kickPeriod+0.1f) {
         float angle = (t-0.14f-period)*M_PI/0.14f;
         forwardL = 80 - 80.0f * (1 - cos(angle))/2.0f;
         shoulderPitchL = DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
         shoulderPitchR = -DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
      }
      if (t >= kickPeriod+0.1f) {
         t = 0.0f;
         walkPhase = nextPhase;
         kicksLeft = 0;
         stopping = true;
         lastLiftL = lastLiftL + DEG2RAD(10) + liftBend;
      }
      return;
   }
   if (kickPhase == ForwardRight) {  // kick forward-right with left foot
      // rock = DEG2RAD(22.0);
      float period = kickPeriod - 0.8f;
      if (t >= 0 && t < 0.4f) {
         float angle = t*M_PI/0.4f;
         forwardL = -60.0f * (1 - cos(angle))/2.0f;
         shoulderPitchL = -DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         shoulderPitchR = DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         liftL = lastLiftL
                + (DEG2RAD(15)+liftBend) * (1.0f - cos(angle))/2.0f;
         legBend = legBendConst + DEG2RAD(5) * (1.0f - cos(angle))/2.0f;
         turnLR = DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         leftL = 0.05f * (1.0f - cos(angle))/2.0f;
         shoulderRollL = DEG2RAD(20.0f) * (1 - cos(angle))/2.0f;
         shoulderRollR = DEG2RAD(60.0f) * (1 - cos(angle))/2.0f;
      }
      if (t >= 0.4f && t < 0.4f+period) {  // kick
         float angle = (t-0.4f)*M_PI/period;
         forwardL = -60.0f + 140.0f * (1 - cos(angle))/2.0f;
         shoulderPitchL = -DEG2RAD(30.0f)
             + DEG2RAD(60.0f) * (1 - cos(angle))/2.0f;
         shoulderPitchR = DEG2RAD(30.0f)
             - DEG2RAD(60.0f) * (1 - cos(angle))/2.0f;
         legBend = legBendConst + DEG2RAD(5) * (1.0f + cos(angle))/2.0f;
         turnLR = DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
         leftL = 0.05f * (1.0f + cos(angle))/2.0f;
         shoulderRollL = DEG2RAD(20.0f) * (1 + cos(angle))/2.0f;
         shoulderRollR = DEG2RAD(60.0f) * (1 + cos(angle))/2.0f;
      }
      if (t >= 0.4f+period+0.1f && t < kickPeriod+0.1f) {
         float angle = (t-0.4f-period)*M_PI/0.4f;
         forwardL = 60.0f * (1 - cos(angle))/2.0f;
         shoulderPitchL = DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
         shoulderPitchR = -DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
      }
      if (t >= kickPeriod+0.1f) {
         t = 0.0f;
         walkPhase = nextPhase;
         kicksLeft = 0;
         stopping = true;
         lastLiftL = lastLiftL + DEG2RAD(15) + liftBend;
      }
      return;
   }
   if (kickPhase == RightKick) {  // ie with left leg
      // rock = DEG2RAD(22.0);
      float period = kickPeriod-0.6f;
      if (t >= 0 && t < 0.2f) {
         float angle = t*M_PI/0.2f;
         liftL = lastLiftL
                + (DEG2RAD(2)+liftBend) * (1.0f - cos(angle))/2.0f;
         leftL = .26 * (1 - cos(angle))/2.0f;
         shoulderRollR = DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
      }
      if (t >= 0.2f && t < 0.4f) {
         float angle = (t-0.2f)*M_PI/0.2f;
         forwardL = 70.0f * (1 - cos(angle))/2.0f;
         shoulderPitchL = DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         shoulderPitchR = -DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         leftL    = .26f;
      }
      // the kick
      if (t >= 0.4f && t < period+0.4f) {
         float angle = (t-0.4f)*M_PI/period;
         leftL = 0.26f - 0.35f * (1 - cos(angle))/2.0f;
         forwardL = 70.0f;
         liftL = lastLiftL
                + (DEG2RAD(2)+liftBend) * (1.0f + cos(angle))/2.0f;
         shoulderRollR = DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
         shoulderRollL = DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
      }
      if (t >= period+0.4+0.1f && t < kickPeriod+0.1f) {
         float angle = (t-period-0.4)*M_PI/0.2f;
         forwardL = 70.0f * (1 + cos(angle))/2.0f;
         leftL = -0.09f * (1 + cos(angle))/2.0f;
         shoulderPitchL = DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
         shoulderPitchR = -DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
         shoulderRollL = DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
      }
      if (t >= kickPeriod+0.1f) {
         t = 0.0f;
         walkPhase = nextPhase;
         kicksLeft = 0;
      }
      return;
   }
   if (kickPhase == BackKickL) {  // kick backwards
      float period = 2.0f;
      if (t >= 0.0f && t < 0.3f) {
         float angle = (t-0.0f)*M_PI/0.3;
         liftL = lastLiftL + DEG2RAD(27) * (1.0f - cos(angle))/2.0f;
      }
      if (t >= 0.3f && t < 0.6f) {
         float angle = (t-0.3f)*M_PI/0.3;
         forwardL = -60 + 150.0f * (1.0f - cos(angle))/2.0f;
         liftL = lastLiftL + DEG2RAD(27);
      }
      if (t >= 0.6f && t < 1.4f) {
         float angle = (t-0.6f)*M_PI/0.8;
         forwardL = 90.0f;
         backKickAnklePitchL = DEG2RAD(80.0f)* (1.0f - cos(angle))/2.0f;
      }
      if (t >= 1.4f && t < 1.6f) {
         float angle = (t-1.4f)*M_PI/0.2;
         forwardL = 90 - 150 * (1.0f - cos(angle))/2.0f;
         backKickAnklePitchL = DEG2RAD(80.0f) * (1.0f + cos(angle))/2.0f;
      }
      if (t >= 1.6f && t < period) {
         float angle = (t-1.6f)*M_PI/0.4;
         liftL = lastLiftL + DEG2RAD(27) * (1.0f + cos(angle))/2.0f;
         forwardL = -60.0f * (1.0f + cos(angle))/2.0f;
      }
      if (t >= period) {
         t = 0.0f;
         walkPhase = nextPhase;
         kicksLeft = 0;
         kickPhase = None;
         lastForwardL = 0.0;
         backKickAnklePitchL = 0.0f;
      }
      return;
   }
   if (kickPhase == ForwardPassingL) {
      // rock = DEG2RAD(22.0f);
      float period = kickPeriod - 1.2f;
      if (t >= 0 && t < 0.6f) {
         float angle = t*M_PI/0.6f;
         shoulderPitchL = -DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         shoulderPitchR = DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         liftL = lastLiftL
                + (DEG2RAD(10)+liftBend) * (1.0f - cos(angle))/2.0f;
      }
      if (t >= 0.6f && t < 0.6f+period) {  // kick
         float angle = (t-0.6f)*M_PI/period;
         forwardL = -65.0f + 145.0f * (1 - cos(angle))/2.0f;
         shoulderPitchL = -DEG2RAD(30.0f)
             + DEG2RAD(60.0f) * (1 - cos(angle))/2.0f;
         shoulderPitchR = DEG2RAD(30.0f)
             - DEG2RAD(60.0f) * (1 - cos(angle))/2.0f;
      }
      if (t >= 0.6f+period && t < kickPeriod) {
         float angle = (t-0.6f-period)*M_PI/0.6f;
         forwardL = 80.0f * (1 + cos(angle))/2.0f;
         shoulderPitchL = DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
         shoulderPitchR = -DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
      }
      if (t >= kickPeriod) {
         t = 0.0f;
         walkPhase = nextPhase;
         kicksLeft = 0;
         stopping = true;
         lastLiftL = lastLiftL + DEG2RAD(10) + liftBend;
         lastForwardL = 0.0;
      }
      return;
   }
}

void SlowWalkGenerator::oKickR(float forwardK, float leftK, float turnK,
                               WalkPhase nextPhase) {
   if (kickPhase == ForwardKickR) {  // kick forward with right foot
      // rock = DEG2RAD(22.0);
      float period = kickPeriod - 0.28f;
      if (t >= 0 && t < 0.14f) {
         float angle = t*M_PI/0.14f;
         forwardR = -60.0f * (1 - cos(angle))/2.0f;
         shoulderPitchR = -DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         shoulderPitchL = DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         liftR = lastLiftR
                + (DEG2RAD(10)+liftBend) * (1.0f - cos(angle))/2.0f;
         legBend = legBendConst + DEG2RAD(5) * (1.0f - cos(angle))/2.0f;
      }
      if (t >= 0.14f && t < 0.14f+period) {  // kick
         float angle = (t-0.14f)*M_PI/period;
         forwardR = -60.0f + 140.0f * (1 - cos(angle))/2.0f;
         shoulderPitchR = -DEG2RAD(30.0f)
             + DEG2RAD(60.0f) * (1 - cos(angle))/2.0f;
         shoulderPitchL = DEG2RAD(30.0f)
             - DEG2RAD(60.0f) * (1 - cos(angle))/2.0f;
         legBend = legBendConst + DEG2RAD(5) * (1.0f + cos(angle))/2.0f;
      }
      if (t >= 0.14f+period+0.1f && t < kickPeriod+0.1f) {
         float angle = (t-0.14f-period)*M_PI/0.14f;
         forwardR = 80 - 80.0f * (1 - cos(angle))/2.0f;
         shoulderPitchR = DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
         shoulderPitchL = -DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
      }
      if (t >= kickPeriod+0.1f) {
         t = 0.0f;
         walkPhase = nextPhase;
         kicksLeft = 0;
         stopping = true;
         lastLiftR = lastLiftR + DEG2RAD(10) + liftBend;
      }
      return;
   }
   if (kickPhase == ForwardLeft) {  // kick forward-left with right foot
      // rock = DEG2RAD(22.0);
      float period = kickPeriod - 0.8f;
      if (t >= 0 && t < 0.4f) {
         float angle = t*M_PI/0.4f;
         forwardR = -60.0f * (1 - cos(angle))/2.0f;
         shoulderPitchR = -DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         shoulderPitchL = DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         liftR = lastLiftR
                + (DEG2RAD(15)+liftBend) * (1.0f - cos(angle))/2.0f;
         legBend = legBendConst + DEG2RAD(5) * (1.0f - cos(angle))/2.0f;
         turnLR = DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         leftR = 0.05f * (1.0f - cos(angle))/2.0f;
         shoulderRollR = DEG2RAD(20.0f) * (1 - cos(angle))/2.0f;
         shoulderRollL = DEG2RAD(60.0f) * (1 - cos(angle))/2.0f;
      }
      if (t >= 0.4f && t < 0.4f+period) {  // kick
         float angle = (t-0.4f)*M_PI/period;
         forwardR = -60.0f + 140.0f * (1 - cos(angle))/2.0f;
         shoulderPitchR = -DEG2RAD(30.0f)
             + DEG2RAD(60.0f) * (1 - cos(angle))/2.0f;
         shoulderPitchL = DEG2RAD(30.0f)
             - DEG2RAD(60.0f) * (1 - cos(angle))/2.0f;
         legBend = legBendConst + DEG2RAD(5) * (1.0f + cos(angle))/2.0f;
         turnLR = DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
         leftR = 0.05f * (1.0f + cos(angle))/2.0f;
         shoulderRollR = DEG2RAD(20.0f) * (1 + cos(angle))/2.0f;
         shoulderRollL = DEG2RAD(60.0f) * (1 + cos(angle))/2.0f;
      }
      if (t >= 0.4f+period+0.1f && t < kickPeriod+0.1f) {
         float angle = (t-0.4f-period)*M_PI/0.4f;
         forwardR = 60.0f * (1 - cos(angle))/2.0f;
         shoulderPitchR = DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
         shoulderPitchL = -DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
      }
      if (t >= kickPeriod+0.1f) {
         t = 0.0f;
         walkPhase = nextPhase;
         kicksLeft = 0;
         stopping = true;
         lastLiftR = lastLiftR + DEG2RAD(15) + liftBend;
      }
      return;
   }
   if (kickPhase == LeftKick) {  // ie with right leg
      // rock = DEG2RAD(22.0);
      float period = kickPeriod-0.6f;
      if (t >= 0 && t < 0.2f) {
         float angle = t*M_PI/0.2f;
         liftR = lastLiftR
                + (DEG2RAD(2)+liftBend) * (1.0f - cos(angle))/2.0f;
         leftR = -.26 * (1 - cos(angle))/2.0f;
         shoulderRollL = DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
      }
      if (t >= 0.2f && t < 0.4f) {
         float angle = (t-0.2f)*M_PI/0.2f;
         forwardR = 70.0f * (1 - cos(angle))/2.0f;
         shoulderPitchR = DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         shoulderPitchL = -DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         leftR    = -.26f;
      }
      // the kick
      if (t >= 0.4f && t < period+0.4f) {
         float angle = (t-0.4f)*M_PI/period;
         leftR = -0.26f + 0.35f * (1 - cos(angle))/2.0f;
         forwardR = 70.0f;
         liftR = lastLiftR
                + (DEG2RAD(2)+liftBend) * (1.0f + cos(angle))/2.0f;
         shoulderRollL = DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
         shoulderRollR = DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
      }
      if (t >= period+0.4+0.1f && t < kickPeriod+0.1f) {
         float angle = (t-period-0.4)*M_PI/0.2f;
         forwardR = 70.0f * (1 + cos(angle))/2.0f;
         leftR = 0.09f * (1 + cos(angle))/2.0f;
         shoulderPitchR = DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
         shoulderPitchL = -DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
         shoulderRollR = DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
      }
      if (t >= kickPeriod+0.1f) {
         t = 0.0f;
         walkPhase = nextPhase;
         kicksLeft = 0;
      }
      return;
   }
   if (kickPhase == BackKickR) {  // kick backwards
      float period = 2.0f;
      if (t >= 0 && t < 0.25f*period) {  // lift 25deg
         float angle = t*4.0f*M_PI/period;
         liftR = lastLiftR + DEG2RAD(27) * (1.0f - cos(angle))/2.0f;
      }
      if (t >= 0.25f*period && t < 0.5f*period) {  // forward to 65
         float angle = t*4.0f*M_PI/period - M_PI;
         forwardR = -60 + 150.0f * (1.0f - cos(angle))/2.0f;
         liftR = lastLiftR + DEG2RAD(27);
      }
      if (t >= 0.5f*period && t < 0.75f*period) {  // kick ankle down
         forwardR = 90.0f;
         backKickAnklePitchR = DEG2RAD(80.0f);
         liftR = lastLiftR + DEG2RAD(27);
      }
      if (t >= 0.75f*period && t < 1.0f*period) {   // back 120 and right ankle
         float angle = t*4.0f*M_PI/period - 3.0f*M_PI;
         liftR = lastLiftR + DEG2RAD(27) * (1.0f + cos(angle))/2.0f;
         forwardR = 90.0f * (1.0f + cos(angle))/2.0f;
         backKickAnklePitchR = 0.0;
      }
      if (t >= period) {
         t = 0.0f;
         walkPhase = nextPhase;
         kicksLeft = 0;
         kickPhase = None;
         lastForwardR = 0.0;
         backKickAnklePitchR = 0.0f;
      }
      return;
   }
   if (kickPhase == ForwardPassingR) {
      // rock = DEG2RAD(22.0);
      float period = kickPeriod - 1.2f;
      if (t >= 0 && t < 0.6f) {
         float angle = t*M_PI/0.6f;
         shoulderPitchR = -DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         shoulderPitchL = DEG2RAD(30.0f) * (1 - cos(angle))/2.0f;
         liftR = lastLiftR
          + (DEG2RAD(10)+liftBend) * (1.0f - cos(angle))/2.0f;
      }
      if (t >= 0.6f && t < 0.6f+period) {  // kick
         float angle = (t-0.6f)*M_PI/period;
         forwardR = -65.0f + 145.0f * (1 - cos(angle))/2.0f;
         shoulderPitchR = -DEG2RAD(30.0f)
             + DEG2RAD(60.0f) * (1 - cos(angle))/2.0f;
         shoulderPitchL = DEG2RAD(30.0f)
             - DEG2RAD(60.0f) * (1 - cos(angle))/2.0f;
      }
      if (t >= 0.6f+period && t < kickPeriod) {
         float angle = (t-0.6f-period)*M_PI/0.6f;
         forwardR = 80.0f * (1 + cos(angle))/2.0f;
         shoulderPitchR = DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
         shoulderPitchL = -DEG2RAD(30.0f) * (1 + cos(angle))/2.0f;
      }
      if (t >= kickPeriod) {
         t = 0.0f;
         walkPhase = nextPhase;
         kicksLeft = 0;
         stopping = true;
         lastLiftR = lastLiftR + DEG2RAD(10) + liftBend;
         lastForwardR = 0.0;
      }
      return;
   }
}


void SlowWalkGenerator::oRockRL(float period, WalkPhase nextPhase) {
   coronalLeanR = moveSin(lastCoronalLeanR, -rock, period);
   coronalLeanL = moveSin(lastCoronalLeanL, -rock, period);
   forwardL     = moveSin(lastForwardL, 0.0f, period);
   forwardR     = moveSin(lastForwardR, -lastForwardL, period);
   leftL        = moveSin(lastLeftL, 0.0f, period);
   leftR        = moveSin(lastLeftR, -lastLeftL, period);
   if (t >= period) {
      lastCoronalLeanR = coronalLeanR;
      lastCoronalLeanL = coronalLeanL;
      lastForwardL     = forwardL;
      lastForwardR     = forwardR;
      lastLeftL        = leftL;
      lastLeftR        = leftR;
      t                = 0.0f;
      walkPhase        = nextPhase;
      if (stopping && ABS(coronalLeanR) < 0.01f && ABS(coronalLeanL) < 0.01f
            && ABS(rock) < 0.01f && legsTogether && !kicking) initialise();
   }
   return;
}

void SlowWalkGenerator::oRockLR(float period, WalkPhase nextPhase) {
   coronalLeanL = moveSin(lastCoronalLeanL, rock, period);
   coronalLeanR = moveSin(lastCoronalLeanR, rock, period);
   forwardR     = moveSin(lastForwardR, 0.0f, period);
   forwardL     = moveSin(lastForwardL, -lastForwardR, period);
   leftR        = moveSin(lastLeftR, 0.0f, period);
   leftL        = moveSin(lastLeftL, -lastLeftR, period);
   if (t >= period) {
      lastCoronalLeanL = coronalLeanL;
      lastCoronalLeanR = coronalLeanR;
      lastForwardR     = forwardR;
      lastForwardL     = forwardL;
      lastLeftR        = leftR;
      lastLeftL        = leftL;
      t                = 0.0f;
      walkPhase        = nextPhase;
      if (stopping && ABS(coronalLeanR) < 0.01f && ABS(coronalLeanL) < 0.01f
            && ABS(rock) < 0.01f && legsTogether && !kicking) initialise();
   }
   return;
}

void SlowWalkGenerator::oLiftFootR(float period, WalkPhase nextPhase) {
   liftR = moveSin(lastLiftR, legLiftHeight, period);
   if (t >= period) {
      lastLiftR     = liftR;
      t = 0.0f;
      walkPhase = nextPhase;
   }
   return;
}

void SlowWalkGenerator::oDownFootR(float period, WalkPhase nextPhase) {
   liftR = moveSin(lastLiftR, 0.0f, period);
   if (t >= period) {
      lastLiftR     = liftR;
      t             = 0.0f;
      walkPhase     = nextPhase;
      if (kicksLeft == 0) kicking = false;
      if (stopping && ABS(lastTurnLR) < 0.01f &&
          ABS(lastForwardL) < 0.01f && ABS(lastForwardR) < 0.01f &&
          ABS(lastLeftL) < 0.01f && ABS(lastLeftR) < 0.01f )
          legsTogether= true;
   }
   return;
}

void SlowWalkGenerator::oLiftFootL(float period, WalkPhase nextPhase) {
   liftL = moveSin(lastLiftL, legLiftHeight, period);
   if (t >= period) {
      lastLiftL     = liftL;
      t             = 0.0f;
      walkPhase     = nextPhase;
   }
   return;
}

void SlowWalkGenerator::oFootForwardL(float period, WalkPhase nextPhase) {
   forwardL = moveSin(lastForwardL, maxForwardL,
                             period);
   // if (t < 0.2f*period) liftL = moveSin(0.0f, legLiftHeight, 0.2f*period);
   // if (t >= 0.8f*period) liftL = moveSin(legLiftHeight, 0.0f, 0.2f*period);
   if (maxLeft <= 0.0f ) leftL = moveSin(lastLeftL, 0.0f, period);
   if (maxLeft > 0.0f )  leftL = moveSin(lastLeftL, maxLeft, period);
   if (maxTurn >= 0.0f ) turnLR  = moveSin(lastTurnLR, maxTurn, period);
   if (maxTurn < 0.0f )  turnLR  = moveSin(lastTurnLR, 0.2f*maxTurn, period);
   if (t >= period) {
      lastForwardL  = forwardL;
      lastLeftL     = leftL;
      lastTurnLR    = turnLR;
      t             = 0.0f;
      walkPhase     = nextPhase;
   }
   return;
}

void SlowWalkGenerator::oDownFootL(float period, WalkPhase nextPhase) {
   liftL = moveSin(lastLiftL, 0.0f, period);
   if (t >= period) {
      lastLiftL     = liftL;
      t             = 0.0f;
      walkPhase     = nextPhase;
      if (kicksLeft == 0) kicking = false;
      if (stopping && ABS(lastTurnLR) < 0.01f &&
          ABS(lastForwardL) < 0.01f && ABS(lastForwardR) < 0.01f &&
          ABS(lastLeftL) < 0.01f && ABS(lastLeftR) < 0.01f )
          legsTogether= true;
   }
   return;
}

void SlowWalkGenerator::oFootForwardR(float period, WalkPhase nextPhase) {
   forwardR = moveSin(lastForwardR, maxForwardR, period);
   if (maxLeft >= 0.0f) leftR = moveSin(lastLeftR, 0.0f, period);
   if (maxLeft < 0.0f)  leftR = moveSin(lastLeftR, maxLeft, period);
   if (maxTurn <= 0.0f) turnLR  = moveSin(lastTurnLR, -maxTurn, period);
   if (maxTurn > 0.0f)  turnLR  = moveSin(lastTurnLR, -0.2f*maxTurn, period);
   if (t >= period) {
      lastForwardR     = forwardR;
      lastLeftR        = leftR;
      lastTurnLR       = turnLR;
      t                = 0.0f;
      walkPhase        = nextPhase;
   }
   return;
}

XYZ_Coord SlowWalkGenerator::mf2b(float Hyp, float Hp, float Hr,
                                  float Kp,  float Ap, float Ar,
                                  float xf, float yf, float zf) {
// MFOOT2BODY Transform coords from foot to body
   XYZ_Coord result;
   float pi = M_PI;
   float tibia        = 100.0f;  // 102.74f;
   float thigh        = 100.0f;
   float k  = sqrt(2.0f);
   float c1 = cos(Ap);
   float c2 = cos(Hr + pi/4.0f);
   float c3 = cos(Hyp - pi/2.0f);
   float c4 = cos(Hp);
   float c5 = cos(Kp);
   float c6 = cos(Ar - pi/2.0f);
   float s1 = sin(Kp);
   float s2 = sin(Hp);
   float s3 = sin(Hyp - 1.0f/2.0f*pi);
   float s4 = sin(Hr + 1.0f/4.0f*pi);
   float s5 = sin(Ap);
   float s6 = sin(Ar - 1.0f/2.0f*pi);
   result.x = thigh*(s2*s3 - c2*c3*c4) + tibia*(s1*(c4*s3 + c2*c3*s2) +
       c5*(s2*s3 - c2*c3*c4)) - yf*(c6*(c1*(s1*(c4*s3 + c2*c3*s2) +
       c5*(s2*s3 - c2*c3*c4)) - s5*(s1*(s2*s3 - c2*c3*c4) - c5*(c4*s3 +
       c2*c3*s2))) + c3*s4*s6) + zf*(s6*(c1*(s1*(c4*s3 + c2*c3*s2) +
       c5*(s2*s3 - c2*c3*c4)) - s5*(s1*(s2*s3 - c2*c3*c4) - c5*(c4*s3 +
       c2*c3*s2))) - c3*c6*s4) + xf*(c1*(s1*(s2*s3 - c2*c3*c4) -
       c5*(c4*s3 + c2*c3*s2)) + s5*(s1*(c4*s3 + c2*c3*s2) +
       c5*(s2*s3 - c2*c3*c4)));
   result.y = xf*(c1*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f) + s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f)) + s5*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f))) + tibia*(c5*(c4*((k*s4)/2.0f +
       (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f +
       (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f)) + thigh*(c4*((k*s4)/2.0f +
       (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) - yf*(s6*((c2*k)/2.0f -
       (k*s3*s4)/2.0f) + c6*(c1*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f)) - s5*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f) + s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f)))) - zf*(c6*((c2*k)/2.0f - (k*s3*s4)/2.0f) -
       s6*(c1*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f)) -
       s5*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f))));
   result.z = yf*(s6*((c2*k)/2.0f + (k*s3*s4)/2.0f) +
       c6*(c1*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)) -
       s5*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f)))) -
       tibia*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)) -
       thigh*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       xf*(c1*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f))) +
       zf*(c6*((c2*k)/2.0f + (k*s3*s4)/2.0f) - s6*(c1*(c5*(c4*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)) - s5*(c5*(s2*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) + s1*(c4*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f))));
   return result;
}

SlowWalkGenerator::Hpr SlowWalkGenerator::hipAngles(float Hyp, float Hp,
                                  float Hr, float Kp,  float Ap, float Ar,
                                  float xf, float yf, float zf, XYZ_Coord e) {
   Hpr result;
   float pi = M_PI;
   float tibia        = 100.0f;  // 102.74f;
   float thigh        = 100.0f;
   float k  = sqrt(2.0f);
   float c1 = cos(Ap);
   float c2 = cos(Hr + pi/4.0f);
   float c3 = cos(Hyp - pi/2.0f);
   float c4 = cos(Hp);
   float c5 = cos(Kp);
   float c6 = cos(Ar - pi/2.0f);
   float s1 = sin(Kp);
   float s2 = sin(Hp);
   float s3 = sin(Hyp - 1.0f/2.0f*pi);
   float s4 = sin(Hr + 1.0f/4.0f*pi);
   float s5 = sin(Ap);
   float s6 = sin(Ar - 1.0f/2.0f*pi);
   float j11 = thigh*(c4*s3 + c2*c3*s2) - tibia*(s1*(s2*s3 -
       c2*c3*c4) - c5*(c4*s3 + c2*c3*s2)) + xf*(c1*(s1*(c4*s3 +
       c2*c3*s2) + c5*(s2*s3 - c2*c3*c4)) - s5*(s1*(s2*s3 -
       c2*c3*c4) - c5*(c4*s3 + c2*c3*s2))) + c6*yf*(c1*(s1*(s2*s3 -
       c2*c3*c4) - c5*(c4*s3 + c2*c3*s2)) + s5*(s1*(c4*s3 +
       c2*c3*s2) + c5*(s2*s3 - c2*c3*c4))) - s6*zf*(c1*(s1*(s2*s3 -
       c2*c3*c4) - c5*(c4*s3 + c2*c3*s2)) + s5*(s1*(c4*s3 +
       c2*c3*s2) + c5*(s2*s3 - c2*c3*c4)));
   float j12 = yf*(c6*(c1*(c3*s1*s2*s4 - c3*c4*c5*s4) +
       s5*(c3*c4*s1*s4 + c3*c5*s2*s4)) - c2*c3*s6) -
       tibia*(c3*s1*s2*s4 - c3*c4*c5*s4) - zf*(s6*(c1*(c3*s1*s2*s4 -
       c3*c4*c5*s4) + s5*(c3*c4*s1*s4 + c3*c5*s2*s4)) + c2*c3*c6) +
       xf*(c1*(c3*c4*s1*s4 + c3*c5*s2*s4) - s5*(c3*s1*s2*s4 -
       c3*c4*c5*s4)) + c3*c4*s4*thigh;
   float j21 = xf*(c1*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f)) -
       s5*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f))) -
       tibia*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f)) -
       thigh*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       c6*yf*(c1*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f))) -
       s6*zf*(c1*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f)));
   float j22 = tibia*(c4*c5*((c2*k)/2.0f - (k*s3*s4)/2.0f) -
       s1*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)) + xf*(c1*(c4*s1*((c2*k)/2.0f -
       (k*s3*s4)/2.0f) + c5*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)) +
       s5*(c4*c5*((c2*k)/2.0f - (k*s3*s4)/2.0f) - s1*s2*((c2*k)/2.0f -
       (k*s3*s4)/2.0f))) + yf*(s6*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       c6*(c1*(c4*c5*((c2*k)/2.0f - (k*s3*s4)/2.0f) - s1*s2*((c2*k)/2.0f -
       (k*s3*s4)/2.0f)) - s5*(c4*s1*((c2*k)/2.0f - (k*s3*s4)/2.0f) +
       c5*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)))) + zf*(c6*((k*s4)/2.0f +
       (c2*k*s3)/2.0f) + s6*(c1*(c4*c5*((c2*k)/2.0f - (k*s3*s4)/2.0f) -
       s1*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)) - s5*(c4*s1*((c2*k)/2.0f -
       (k*s3*s4)/2.0f) + c5*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)))) +
       c4*thigh*((c2*k)/2.0f - (k*s3*s4)/2.0f);
   float j31 = tibia*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) +
       (c3*c4*k)/2.0f) + s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) -
       (c3*k*s2)/2.0f)) -
       xf*(c1*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)) -
       s5*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f))) +
       thigh*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) -
       c6*yf*(c1*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f))) +
       s6*zf*(c1*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)));
   float j32 = -tibia*(c4*c5*((c2*k)/2.0f + (k*s3*s4)/2.0f) -
       s1*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)) - xf*(c1*(c4*s1*((c2*k)/2.0f +
       (k*s3*s4)/2.0f) + c5*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)) +
       s5*(c4*c5*((c2*k)/2.0f + (k*s3*s4)/2.0f) - s1*s2*((c2*k)/2.0f +
       (k*s3*s4)/2.0f))) - yf*(s6*((k*s4)/2.0f - (c2*k*s3)/2.0f) -
       c6*(c1*(c4*c5*((c2*k)/2.0f + (k*s3*s4)/2.0f) - s1*s2*((c2*k)/2.0f +
       (k*s3*s4)/2.0f)) - s5*(c4*s1*((c2*k)/2.0f + (k*s3*s4)/2.0f) +
       c5*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)))) - zf*(c6*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) + s6*(c1*(c4*c5*((c2*k)/2.0f + (k*s3*s4)/2.0f) -
       s1*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)) - s5*(c4*s1*((c2*k)/2.0f +
       (k*s3*s4)/2.0f) + c5*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)))) -
       c4*thigh*((c2*k)/2.0f + (k*s3*s4)/2.0f);
   float xbe = e.x;
   float ybe = e.y;
   float zbe = e.z;
   float lambda = 0.4f;
   float la2 = lambda*lambda;
   float la4 = la2*la2;
   float j322 = j32*j32;
   float j222 = j22*j22;
   float j122 = j12*j12;
   float j212 = j21*j21;
   float j112 = j11*j11;
   float j312 = j31*j31;
   float sigma = 1.0f/(la4 + j112*j222 + j122*j212 + j112*j322 + j122*j312 +
   j212*j322 + j222*j312 + j112*la2 + j122*la2 + j212*la2 + j222*la2 +
   j312*la2 + j322*la2 - 2.0f*j11*j12*j21*j22 - 2.0f*j11*j12*j31*j32 -
   2.0f*j21*j22*j31*j32);
   result.Hp = sigma*xbe*(j11*j222 + j11*j322 + j11*la2 - j12*j21*j22 -
   j12*j31*j32) + sigma*ybe*(j122*j21 + j21*j322 + j21*la2 - j11*j12*j22 -
   j22*j31*j32) + sigma*zbe*(j122*j31 + j222*j31 + j31*la2 - j11*j12*j32 -
   j21*j22*j32);
   result.Hr =  sigma*xbe*(j12*j212 + j12*j312 + j12*la2 - j11*j21*j22 -
   j11*j31*j32) + sigma*ybe*(j112*j22 + j22*j312 + j22*la2 - j11*j12*j21 -
   j21*j31*j32) + sigma*zbe*(j112*j32 + j212*j32 + j32*la2 - j11*j12*j31 -
   j21*j22*j31);
   return result;
}
