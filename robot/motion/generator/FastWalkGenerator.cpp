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
 * FastWalkGenerator.cpp
 * BH 11th June 2010
 */

#include "motion/generator/FastWalkGenerator.hpp"
#include <cmath>
#include "utils/angles.hpp"
#include "utils/body.hpp"
#include "utils/log.hpp"
#include "utils/basic_maths.hpp"

using boost::program_options::variables_map;
using namespace std;
using namespace Joints;
using namespace Sensors;

FastWalkGenerator::FastWalkGenerator()
   :bend(30.0), z(0.0f) {
   initialise();
   llog(INFO) << "FastWalkGenerator constructed" << std::endl;
}

FastWalkGenerator::~FastWalkGenerator() {
   llog(INFO) << "FastWalkGenerator destroyed" << std::endl;
}

void FastWalkGenerator::initialise() {
   llog(INFO) << "Fastwalk initializing" << endl;
   legBendConst = legBend = lastLegBend = maxLegBend = DEG2RAD(bend);
   dt = 0.01f;
   t = 0.0f;
   stopping = legsTogether = false;
   stopped = true;
   maxForwardL = maxForwardR = 0.0f;
   forwardL  = lastForwardL = forwardR = lastForwardR = 0.0f;
   maxLeft  = leftL = lastLeftL =  leftR =  lastLeftR  = 0.0f;
   liftR = lastLiftR = liftL = lastLiftL = stepHeight = 0.0f;
   maxTurn = turnLR = lastTurnLR = maxPower = lastSinu = 0.0f;
   T = 0.38f;
   shoulderPitchL = shoulderPitchR = shoulderRollL = shoulderRollR = 0.0f;
   lastZMPL = filteredTotalPressure = 10.0f;
   liftTimer = 0.0f;
   filZMPL = filAcc = 0.0f;
   filHighZMPF = filLowZMPF = 0.0f;
   limForwardLPatter = limForwardRPatter = 60.0f;
   limLeftPatter = 30;
   limTurnPatter = 0.35f;
}

JointValues FastWalkGenerator::makeJoints(ActionCommand::All* request,
                                          Odometry* odometry,
                                          const SensorValues &sensors) {
   float forward       = request->body.forward;
   float left          = request->body.left;
   float turn          = request->body.turn;
   float power         = request->body.power;

   // set walking parameters to zero if stop is requested
   if (stopping) {
      forward = 0.0f;
      turn = 0.0f;
      left = 0.0f;
   } else {
      stopped = false;  // (re)activate fastwalk
   }

   // increase max walk parameters for uber-users
   limForwardLPatter = limForwardRPatter = 60.0f;
   if (power >= 50.0f && power <= 100.0f)
      limForwardLPatter = limForwardRPatter = power;

   // adjust walk parameters to give precedence over turn
   if (power >= 50.0f && power <= 100.0f) {
      float safeForwardT = power/3.0f*(3.8f-8.0f*ABS(turn));
      float safeForwardA = power/3.0f*(3.8f-8.0f*ABS(maxTurn));
      if (ABS(turn) > 0.1f && ABS(maxForwardL) > safeForwardT) {
         int sign = (maxForwardL ? 1 : -1);
         forward = sign*(safeForwardT);  // reduce speed
         if (ABS(maxForwardL) > safeForwardA) {
            sign = (turn ? 1 : -1);
            turn = sign*0.1f;
         }
      }
   }

   // track requested walk params
   targetWalkPatter(forward, left, turn);

   // Various Center of Pressure calcs to help stabilise walk (ZMP)
   float pressureL =
      +sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
      +sensors.sensors[Sensors::LFoot_FSR_FrontRight]
      +sensors.sensors[Sensors::LFoot_FSR_RearLeft]
      +sensors.sensors[Sensors::LFoot_FSR_RearRight];
   float pressureR =
      +sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
      +sensors.sensors[Sensors::RFoot_FSR_FrontRight]
      +sensors.sensors[Sensors::RFoot_FSR_RearLeft]
      +sensors.sensors[Sensors::RFoot_FSR_RearRight];
   float totalPressure = pressureL + pressureR;
   filteredTotalPressure = 0.8f*filteredTotalPressure + 0.2f*totalPressure;
   float ZMPL = 0.0f;
   if (ABS(totalPressure) > 0.000001f) {
      ZMPL =
      (80.0f*sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
      +30.0f*sensors.sensors[Sensors::LFoot_FSR_FrontRight]
      +80.0f*sensors.sensors[Sensors::LFoot_FSR_RearLeft]
      +30.0f*sensors.sensors[Sensors::LFoot_FSR_RearRight]
      -30.0f*sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
      -80.0f*sensors.sensors[Sensors::RFoot_FSR_FrontRight]
      -30.0f*sensors.sensors[Sensors::RFoot_FSR_RearLeft]
      -80.0f*sensors.sensors[Sensors::RFoot_FSR_RearRight])/totalPressure;
   }
   float ZMPF = 0.0f;
   if (ABS(totalPressure) > 0.000001f) {
      ZMPF =
      ((forwardL+50.0f)*sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
      +(forwardL-50.0f)*sensors.sensors[Sensors::LFoot_FSR_RearLeft]
      +(forwardL+50.0f)*sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
      +(forwardL-50.0f)*sensors.sensors[Sensors::RFoot_FSR_RearLeft]
      +(forwardR+50.0f)*sensors.sensors[Sensors::LFoot_FSR_FrontRight]
      +(forwardR-50.0f)*sensors.sensors[Sensors::LFoot_FSR_RearRight]
      +(forwardR+50.0f)*sensors.sensors[Sensors::RFoot_FSR_FrontRight]
      +(forwardR-50.0f)*sensors.sensors[Sensors::RFoot_FSR_RearRight])
      /totalPressure;
   }

   // Walk stabilisation feedback signals
   filZMPL = 0.5f*filZMPL + 0.5*ZMPL;
   // filHighZMPF = 0.8f*filHighZMPF + 0.2f*DEG2RAD(ZMPF*0.05f);
   float headPitchAdj = sensors.joints.angles[Joints::HeadPitch]/30.0f;

   if (power >= 50.0f && power <= 100.0f) {
      float acc = sensors.sensors[Sensors::InertialSensor_AccX];
      filAcc    = 0.5*filAcc + 0.5*(DEG2RAD(acc*0.2f));

      if (filHighZMPF == 0.0f) filHighZMPF = DEG2RAD(ZMPF*0.03f);
      else
         filHighZMPF = 0.8f*filHighZMPF + 0.2f*DEG2RAD(ZMPF*0.03f);  // was .02

      if (filLowZMPF == 0.0f) filLowZMPF = DEG2RAD(ZMPF*0.01f);
      else
         filLowZMPF = 0.99f*filLowZMPF + 0.01f*DEG2RAD(ZMPF*0.01f);
      // filLowZMPF = 0.0f;

      // headPitchAdj = 0.0f;
   } else {
      if (filHighZMPF == 0.0f) filHighZMPF = DEG2RAD(ZMPF*0.03f);
      else
         filHighZMPF = 0.8f*filHighZMPF + 0.2f*DEG2RAD(ZMPF*0.03f);  // was .02
      if (filLowZMPF == 0.0f) filLowZMPF = DEG2RAD(ZMPF*0.01f);
      else
         filLowZMPF = 0.99f*filLowZMPF + 0.01f*DEG2RAD(ZMPF*0.01f);
   }

   // Update timers
   t += dt;

   // Scale back ambitious omni-directional values: forward, left, turn
   float useForwardL = maxForwardL;
   float useForwardR = maxForwardR;
   float useLeft     = maxLeft;
   float useTurn     = maxTurn;
   float sum         = 0.0f;
   if (power >= 50.0f && power <= 100.0f) {
      sum = (ABS(maxForwardL)+ABS(maxForwardR))/2.0f/limForwardLPatter
          + ABS(maxLeft)/limLeftPatter*200.0f;
      if (sum > 1.0f) {
         useForwardL *= ABS(maxForwardL)/limForwardLPatter/sum;
         useForwardR *= ABS(maxForwardR)/limForwardRPatter/sum;
         useLeft     *= ABS(maxLeft)/limLeftPatter*200.0f/sum;
      }
   } else {
      sum = (ABS(maxForwardL)+ABS(maxForwardR))/2.0f/limForwardLPatter
          + ABS(maxLeft)/limLeftPatter*200.0f
          + ABS(maxTurn)/limTurnPatter;
      if (sum > 1.3f) {  // allow parameters to add to some degree. was 1.5f
         useForwardL *= ABS(maxForwardL)/limForwardLPatter/sum;
         useForwardR *= ABS(maxForwardR)/limForwardRPatter/sum;
         useLeft     *= ABS(maxLeft)/limLeftPatter*200.0f/sum;
         useTurn     *= ABS(maxTurn)/limTurnPatter/sum;
      }
   }

   // adjust forward L & R when turing
      useForwardL = useForwardL - 100.0f*tan(useTurn/2.0f);
      useForwardR = useForwardR + 100.0f*tan(useTurn/2.0f);

   // Stabilise/Sychronise walks coronally
   if (lastZMPL > 0.0f && filZMPL < 0.0f) {
      t = 0;
      leftPhase = true;
   }
   if (leftPhase && t > T) t = 0.0f;
   if (lastZMPL < 0.0f && filZMPL > 0.0f) {
      t = T/2.0f;
      leftPhase = false;
   }
   if (!leftPhase &&  t > 1.5f*T) t = T/2.0f;

   // reset omni-directional foot movement integrator
   if (ABS(t) < dt/2.0f || ABS(T/2.0f-t) < dt/2.0f) {
      lastSinu = 0.0f;
   }

   // update calibrated odometry
   *odometry = *odometry
               + Odometry((useForwardL+useForwardR)/2.0f/T*dt*0.9f,
                        useLeft*200.0f/T*dt*0.9f, useTurn/T*dt*0.9f);

   // stopped conditions
   if (stopping && still(useForwardL, useForwardR, useLeft, useTurn)) {
      initialise();
   }

   // Walk Cycle
   // reset / sync legs
   if (ABS(t) < 0.0001f) {
      forwardL = -useForwardL/4.0f;
      forwardR = +useForwardR/4.0f;
      if (useLeft >= 0.0f) leftL = leftR  = 0.0f;
      if (useTurn >= 0.0f) turnLR = useTurn/8.0f;
   }
   if (ABS(T/2.0f-t) < 0.0001f) {
      forwardL = +useForwardL/4.0f;
      forwardR = -useForwardR/4.0f;
      if (useLeft <= 0.0f) leftL = leftR  = 0.0f;
      if (useTurn <= 0.0f) turnLR = useTurn/8.0f;
   }

   // float ankleURoll = 0.0f;
   float ankleUPitchL = 0.0f;  // to be used for pushing off and sliding down
   float ankleUPitchR = 0.0f;
   if (power >= 50.0f && power <= 100.0f) {
   // Generate Ueber-Walk Cylce
      // rock = DEG2RAD(0.0f)*sin(t/T*2.0f*M_PI);
      // T = .43;
      stepHeight = 3.0f+ABS(useForwardL+useForwardR)/2.0f*4.0/100.0f;
      if (stopped || still(useForwardL, useForwardR, useLeft, useTurn))
         stepHeight = 0.0f;
      // legBend = DEG2RAD(30.0f);
      liftL = liftR = 0.0f;
      forwardL -= useForwardL/T*dt;
      forwardR -= useForwardR/T*dt;
      leftL    -= useLeft/T*dt;
      leftR    -= useLeft/T*dt;
      // legLift
      float liftFrac = 0.50f;  // of walk cycle
      float liftT = liftFrac*T;
      float startT = ((1.0f-2.0f*liftFrac)/4.0f-.0001)*T;
      if (t >= startT && t < startT+liftT && leftPhase) {
         float iT = t - startT;
         float legLift = ((1.0f-cos(iT*2.0f*M_PI/liftT))/2.0f);
         // float legLift = 0.0;
         // if (iT < 0.25f*liftT) legLift = (1.0f-cos(t*4.0f*M_PI/liftT))/2.0f;
         // if (iT >= 0.25f*liftT && iT < 0.75f*liftT) legLift = 1.0f;
         // if (iT >= 0.75*liftT)
         //    legLift = (1.0f+cos((t-0.75f)*4.0f*M_PI/liftT))/2.0f;
         liftL = DEG2RAD(stepHeight*(legLift));
         int sign = (useForwardL ? 1 : -1);
         ankleUPitchL = sign*DEG2RAD(stepHeight*sqrt(legLift));
      }
      startT += T/2.0f;
      if (t >= startT && t < startT+liftT && !leftPhase) {
         float iT = t - startT;
         float legLift = ((1.0f-cos(iT*2.0f*M_PI/liftT))/2.0f);
         // float legLift = 0.0;
         // if (iT < 0.25f*liftT) legLift = (1.0f-cos(t*4.0f*M_PI/liftT))/2.0f;
         // if (iT >= 0.25f*liftT && iT < 0.75f*liftT) legLift = 1.0f;
         // if (iT >= 0.75*liftT)
         //    legLift = (1.0f+cos((t-0.75f)*4.0f*M_PI/liftT))/2.0f;
         liftR = DEG2RAD(stepHeight*(legLift));  // was sqrt
         int sign = (useForwardR ? 1 : -1);
         ankleUPitchR = sign*DEG2RAD(stepHeight*sqrt(legLift));
      }  //
      // omni-dir movement
      float movFrac = 0.4f;
      float movT = movFrac*T;
      startT = ((1.0f-2.0f*movFrac)/4.0f)*T;
      if (t >= startT && t < startT+movT+dt && leftPhase) {  // left leg
         float iT = t - startT;
         float a = 4.0f/SQUARE(movT);
         diff = iT*a*dt;
         if (iT >= movT/2.0f) diff = (movT-iT)*a*dt;
         // float sinu = sqrt((1.0f - cos(iT*M_PI/movT))/2.0f);  // changed
         // if (t > startT+movT) sinu = 1.0f;
         // diff = sinu-lastSinu;
         // lastSinu   = sinu;
         forwardL  += useForwardL*diff;
         leftL     += useLeft*diff;
         turnLR    += useTurn*diff;
      }
      startT += T/2.0f;
      if (t >= startT && t < startT+movT+dt && !leftPhase) {  // right leg
         float iT = t - startT;
         // float sinu = sqrt((1.0f - cos(iT*M_PI/movT))/2.0f);  // changed
         // if (t > startT+movT) sinu = 1.0f;
         // diff = sinu-lastSinu;
         // lastSinu   = sinu;
         float a = 4.0f/SQUARE(movT);
         diff = iT*a*dt;
         if (iT >= movT/2.0f) diff = (movT-iT)*a*dt;
         forwardR  += useForwardR*diff;
         leftR     += useLeft*diff;
         turnLR    -= useTurn*diff;
      }
      shoulderPitchR = forwardR/200.0f;
      shoulderPitchL = forwardL/200.0f;
      elbowYawR = elbowYawL = DEG2RAD(90.0f);
      elbowRollL = elbowRollR = DEG2RAD(45.0f)*maxForwardR/100.0f;
   } else {
   // Generate Walk Cylce
      T = .38;
      stepHeight = 3.0f+ABS(useForwardL+useForwardR)/2.0f*4.0/100.0f;
      if (stopped || still(useForwardL, useForwardR, useLeft, useTurn))
         stepHeight = 0.0f;
      // legBend = DEG2RAD(30.0f);
      liftL = liftR = 0.0f;
      forwardL -= useForwardL/T*dt;
      forwardR -= useForwardR/T*dt;
      leftL    -= useLeft/T*dt;
      leftR    -= useLeft/T*dt;
      // legLift
      float liftFrac = 0.50f;  // of walk cycle
      float liftT = liftFrac*T;
      float startT = ((1.0f-2.0f*liftFrac)/4.0f-.0001)*T;
      if (t >= startT && t < startT+liftT && leftPhase) {
         float iT = t - startT;
         float legLift = ((1.0f-cos(iT*2.0f*M_PI/liftT))/2.0f);
         liftL += DEG2RAD(stepHeight*(sqrt(legLift)));
      }
      startT += T/2.0f;
      if (t >= startT && t < startT+liftT && !leftPhase) {
         float iT = t - startT;
         float legLift = ((1.0f-cos(iT*2.0f*M_PI/liftT))/2.0f);
         liftR += DEG2RAD(stepHeight*(sqrt(legLift)));
      }  //
      // omni-dir movement
      float movFrac = 0.4f;
      float movT = movFrac*T;
      startT = ((1.0f-2.0f*movFrac)/4.0f)*T;
      if (t >= startT && t < startT+movT+dt && leftPhase) {  // left leg
         float iT = t - startT;
         float sinu = sqrt((1.0f - cos(iT*M_PI/movT))/2.0f);  // changed
         if (t > startT+movT) sinu = 1.0f;
         diff = sinu-lastSinu;
         lastSinu   = sinu;
         forwardL  += useForwardL*diff;
         leftL     += useLeft*diff;
         turnLR    += useTurn*diff;
      }
      startT += T/2.0f;
      if (t >= startT && t < startT+movT+dt && !leftPhase) {  // right leg
         float iT = t - startT;
         float sinu = sqrt((1.0f - cos(iT*M_PI/movT))/2.0f);  // changed
         if (t > startT+movT) sinu = 1.0f;
         diff = sinu-lastSinu;
         lastSinu   = sinu;
         forwardR  += useForwardR*diff;
         leftR     += useLeft*diff;
         turnLR    -= useTurn*diff;
      }
      shoulderPitchR = forwardR/200.0f;
      shoulderPitchL = forwardL/200.0f;
   }
   lastZMPL = filZMPL;

   // calculate sagittal forward walk values (closed-form inverse kinematics)
   float dirL    = 1.0f;
   if (forwardL < 0.0f) dirL = -1.0f;
   float HpL0    = liftL + legBend + (ABS(leftL) + ABS(leftR))/1.5f;
   float hL      = 100.0f*cos(HpL0);
   hL           += sqrt(102.74f*102.74f-10000.0f*sin(HpL0)*sin(HpL0));
   hL           /= cos(leftL);
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
   float HpR0    = liftR + legBend + (ABS(leftL) + ABS(leftR))/1.5f;
   float hR      = 100.0f*cos(HpR0);
   hR           += sqrt(102.74f*102.74f-10000.0f*sin(HpR0)*sin(HpR0));
   hR           /= cos(leftR);
   float dR      = sqrt(forwardR*forwardR + hR*hR);
   float beta1R  = acos((102.74f*102.74f+dR*dR-10000.0f)/(2.0f*102.74f*dR));
   float beta2R  = acos((10000.0f+dR*dR-102.74f*102.74f)/(2.0f*100.0f*dR));
   float tempR   = hR/dR;
   if (tempR > 1.0f) tempR = 1.0f;
   float deltaR  = asin(tempR);
   float HpR     = beta1R + dirR*(M_PI/2.0f - deltaR);
   float ApR     = beta2R + dirR*(deltaR - M_PI/2.0f);
   float KpR     = HpR + ApR;

   float HrL     = leftL+rock;
   float HrR     = leftR+rock;
   float ArL     = -HrL;
   float ArR     = -HrR;

   // turning
   if (false) {
      // use first order approximation to IK for small turn anlges
      HpL -= turnLR;
      HpR -= turnLR;
   }
   if (true) {
      // Use iterative inverse kinematics for turning
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

      // Adjust HpR, HrR, ApR, ArR RIGHT based on Hyp turn to keep ankle in situ
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
   }

   JointValues j = sensors.joints;
   float stiffness = 0.65f;
   if (power >= 50.0f && power <= 100.0f) stiffness = 0.8f;
   if (filteredTotalPressure < 0.25f
      || (liftTimer > 0.0f && liftTimer < 1.0f)) {
      // llog(INFO) << " Fastwalk: filteredTotalPressure = " <<
      // filteredTotalPressure << std::endl;
      stiffness = 0.1f;
      HpL = HpR = ApL = ApR = DEG2RAD(30.0f);
      KpL = KpR = DEG2RAD(60.0f);
      HrL = HrR = ArL = ArR = 0.0f;
      liftTimer += dt;
   } else {
      liftTimer = 0.0f;
   }
   for (uint8_t i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
      j.stiffnesses[i] = stiffness;

   // Arms
   j.angles[LShoulderPitch] = DEG2RAD(90)+shoulderPitchL;
   j.angles[LShoulderRoll] = DEG2RAD(15)+shoulderRollL;
   j.angles[LElbowYaw] = DEG2RAD(0)-elbowYawL;
   j.angles[LElbowRoll] = DEG2RAD(0)-elbowRollL;

   j.angles[RShoulderPitch] = DEG2RAD(90)+shoulderPitchR;
   j.angles[RShoulderRoll] = DEG2RAD(-15)-shoulderRollR;
   j.angles[RElbowYaw] = DEG2RAD(0)+elbowYawR;
   j.angles[RElbowRoll] = DEG2RAD(0)+elbowRollR;

   // Turn
   j.angles[Joints::LHipYawPitch] = -turnLR;

   // Sagittal Joints
   j.angles[Joints::LHipPitch]    = -HpL-filHighZMPF+headPitchAdj
                                    +filLowZMPF-filAcc;
   j.angles[Joints::RHipPitch]    = -HpR-filHighZMPF+headPitchAdj
                                    +filLowZMPF-filAcc;
   j.angles[Joints::LKneePitch]   =  KpL;
   j.angles[Joints::RKneePitch]   =  KpR;
   j.angles[Joints::LAnklePitch]  = -ApL-ankleUPitchL;
   j.angles[Joints::RAnklePitch]  = -ApR-ankleUPitchR;

   // Coronal Joints
   j.angles[Joints::LHipRoll]     =  HrL;
   j.angles[Joints::RHipRoll]     =  HrR;
   j.angles[Joints::LAnkleRoll]   =  ArL;
   j.angles[Joints::RAnkleRoll]   =  ArR;

   return j;
}

bool FastWalkGenerator::isActive() {
   return !stopped;
}

void FastWalkGenerator::readOptions(variables_map& config) {
   bend = config["walk.b"].as<float>();
}

void FastWalkGenerator:: reset() {
   initialise();
   llog(INFO) << "Fastwalk reset" << endl;
}

void FastWalkGenerator:: stop() {
   stopping = true;
   llog(INFO) << "Fastwalk stop" << endl;
}

bool FastWalkGenerator:: still(float forwardL, float forwardR,
                               float left, float turn) {
   return ABS(forwardL) < 1.1f && ABS(forwardR) < 1.1f &&
   ABS(left) < 0.0015f && ABS(turn) < 0.005f;
}

bool FastWalkGenerator:: targetWalkPatter(float forward,
                                                float left,
                                                float turn) {
   if ((forward-maxForwardL > 0 && maxForwardL < limForwardLPatter)
      || maxForwardL < -limForwardLPatter-0.5f) maxForwardL += 0.5f;
   if ((forward-maxForwardL < 0 && maxForwardL > -limForwardLPatter)
      || maxForwardL > limForwardLPatter+0.5f) maxForwardL -= 0.5f;
   if ((forward-maxForwardR > 0 && maxForwardR < limForwardRPatter)
      || maxForwardR < -limForwardRPatter-0.5f) maxForwardR += 0.5f;
   if ((forward-maxForwardR < 0 && maxForwardR > -limForwardRPatter)
      || maxForwardR > limForwardRPatter+0.5f) maxForwardR -= 0.5f;
   if ((left/200.0f-maxLeft > 0 && maxLeft < limLeftPatter/200.0f)
      || maxLeft < -limLeftPatter/200.0f-0.0015f) maxLeft += 0.0015f;
   if ((left/200.0f-maxLeft < 0 && maxLeft > -limLeftPatter/200.0f)
      || maxLeft > limLeftPatter/200.0f+0.0015f) maxLeft -= 0.0015f;
   if ((turn-maxTurn > 0 && maxTurn < limTurnPatter)
      || maxTurn < -limTurnPatter-.0035f) maxTurn += .0035f;
   if ((turn-maxTurn < 0 && maxTurn > -limTurnPatter)
      || maxTurn > limTurnPatter+.0035f) maxTurn -= .0035;
   return false;
}

float FastWalkGenerator:: moveSin(float start, float finish, float fraction) {
   if ( fraction <= 0.0f) return start;
   if ( fraction > 1.0f) return finish;
   float difference = finish - start;
   float angle = M_PI * fraction;
   return start + difference * (1 - cos(angle))/2.0f;
}

XYZ_Coord FastWalkGenerator::mf2b(float Hyp, float Hp, float Hr,
                                  float Kp,  float Ap, float Ar,
                                  float xf, float yf, float zf) {
// MFOOT2BODY Transform coords from foot to body
   XYZ_Coord result;
   float pi = M_PI;
   float tibia        = 102.74f;
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

FastWalkGenerator::Hpr FastWalkGenerator::hipAngles(float Hyp, float Hp,
                                  float Hr, float Kp,  float Ap, float Ar,
                                  float xf, float yf, float zf, XYZ_Coord e) {
   Hpr result;
   float pi = M_PI;
   float tibia        = 102.74f;
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

