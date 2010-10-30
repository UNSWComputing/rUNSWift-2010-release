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

#include "motion/generator/WaveWalkGenerator.hpp"
#include <cmath>
#include "utils/angles.hpp"
#include "utils/basic_maths.hpp"
#include "utils/body.hpp"
#include "utils/clip.hpp"
#include "utils/log.hpp"

using boost::program_options::variables_map;
using std::fmod;

// Generate a "heartbeat" function for forward steps
// This is the left foot one, the right foot one is T/2 out of phase
inline float WaveWalkGenerator::rho(float t) {
   if (t < c_l)
      return -(T + t - c_r - lift_T) / (T/2 - lift_T);
   else if (t >= c_l && t < c_l + lift_T)
      return -1 + 2 * (t - c_l) / lift_T;
   else if (t >= c_l + lift_T && t < c_r)
      return 1 - (t - c_l - lift_T) / (T/2 - lift_T);
   else if (t >= c_r && t < c_r + lift_T)
      return 0.0;
   else
      return -(t - c_r - lift_T) / (T/2 - lift_T);
}

// Generate a "shark" function for sidestepping
// This is the left foot leading one. The right one is T/2 out of phase.
// The non-leadingi right one is sigma(T-t). The left one is T/2 out of phase.
inline float WaveWalkGenerator::sigma(float t) {
   if (t < c_l)
      return 0.0;
   else if (t >= c_l && t < c_l + lift_T)
      return (t - c_l) / lift_T;
   else if (t >= c_l + lift_T && t < c_r)
      return 1 - (t - c_l - lift_T) / (T/2 - lift_T);
   else
      return 0.0;
}

// Generate a "speed-hump" function for turning
// This is the one for turning positively, negative is T/2 out of phase
inline float WaveWalkGenerator::tau(float t) {
   if (t < c_l)
      return 1.0;
   else if (t >= c_l && t < c_l + lift_T)
      return 1 - (t - c_l) / lift_T;
   else if (t >= c_l + lift_T && t < c_r)
      return 0.0;
   else if (t >= c_r && t < c_r + lift_T)
      return (t - c_r) / lift_T;
   else
      return 1.0;
}

// Generate a "early-riser" function for leg lifts
// This is a Lagrangian polynomial through (0,0), (1/3T, 1), (2/3T, 0.5), (T, 0)
// This is the left foot lift, the right foot is T/2 out of phase
inline float WaveWalkGenerator::psi(float t) {
   if (t >= c_l && t <= c_l + lift_T)
      return 6.75 / pow(lift_T, 3) * (t - c_l) * pow(t - c_l - lift_T, 2);
   else
      return 0.0;
}

WaveWalkGenerator::WaveWalkGenerator(bool s)
   : stand(s),
     stiffness(0.66f),
     rock(0.0f),
     spread(0.0f),
     lift(0.0f),
     bend(0.0f),
     f_size(0.0f),
     l_size(0.0f),
     t_size(0.0f),
     mul(4.0f),
     frequency(1.0f),
     t(0.0f),
     T(1.0f),
     lift_T(1.0 / 4.0) {
   c_l = (T / 4.0) - (lift_T / 2);
   c_r = (3 * T / 4.0) - (lift_T / 2);
   llog(INFO) << "WaveWalkGenerator constructed" << std::endl;
}

WaveWalkGenerator::~WaveWalkGenerator() {
   llog(INFO) << "WaveWalkGenerator destroyed" << std::endl;
}

JointValues WaveWalkGenerator::makeJoints(ActionCommand::All* request,
                                          Odometry* odometry,
                                          const SensorValues &sensors) {
   JointValues j = sensors.joints;
   for (uint8_t i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
      j.stiffnesses[i] = stiffness;

   if (!isActive()) {
      active = request->body;
   } else {
      request->body = active;
   }

   // The mm -> rad calculations for forward and left are kinda dodgy hacks
   if (t >= 0 && t < c_l + 0.01f) {
      float new_forward = CLIP((float)DEG2RAD(active.forward*15/100.f),
                               -f_size, f_size);
      forward = MAX(1.0f - t/c_l, 0.0f) * last_forward +
                MIN(t/c_l, 1.0f) * new_forward;
      float new_left = CLIP((float)DEG2RAD(active.left*15/40.f),
                            -l_size, l_size);
      left = MAX(1.0f - t/c_l, 0.0f) * last_left +
                MIN(t/c_l, 1.0f) * new_left;
      float new_turn = CLIP(-active.turn, -t_size, t_size);
      turn = MAX(1.0f - t/c_l, 0.0f) * last_turn +
                MIN(t/c_l, 1.0f) * new_turn;
   } else if (t >= T/2 && t < c_r + 0.01f) {
      float new_forward = CLIP((float)DEG2RAD(active.forward*15/100.f),
                               -f_size, f_size);
      forward = MAX(1.0f - (t - T/2)/c_l, 0.0f) * last_forward +
                MIN((t - T/2)/c_l, 1.0f) * new_forward;
      float new_left = CLIP((float)DEG2RAD(active.left*15/40.f),
                            -l_size, l_size);
      left = MAX(1.0f - (t - T/2)/c_l, 0.0f) * last_left +
                MIN((t - T/2)/c_l, 1.0f) * new_left;
      float new_turn = CLIP(-active.turn, -t_size, t_size);
      turn = MAX(1.0f - (t - T/2)/c_l, 0.0f) * last_turn +
                MIN((t - T/2)/c_l, 1.0f) * new_turn;
   } else if (forward != last_forward || left != last_left ||
              turn != last_turn) {
      last_forward = forward;
      last_left = left;
      last_turn = turn;
   }

   // calculate rock angle
   float theta = rock * sin(2 * M_PI * frequency * t);
   // calculate lift angles
   float phi_l = lift * psi(t);
   float phi_r = lift * psi(fmod(t + T/2, T));
   // calculate forward step adjustment angles for left and right legs
   float alpha_l = forward * rho(t);
   float alpha_r = forward * rho(fmod(t + T/2, T));
   // calculate side step adjustment angles for left and right legs
   float beta_l = ABS(left) * ((left > 0) ? sigma(t)
                                          : sigma(fmod(T - (t + T/2), T)));
   float beta_r = ABS(left) * ((left > 0) ? sigma(T - t)
                                          : sigma(fmod(t + T/2, T)));
   // calculate leg opening angle
   float gamma = ABS(turn) * tau((turn > 0) ? t : fmod(t + T/2, T));

   // Pause if t == 0 || T/2 and new command is STAND
   if (!(t == 0 || (t >= T/2 && t <= T/2 + 0.01f)) ||
       request->body.actionType != ActionCommand::Body::STAND)
      t += 0.01f;
   // Prevent time overflow
   if (t >= T) t = 0.0f;

   // Joint for turning
   j.angles[Joints::LHipYawPitch] = -gamma;

   // Joints for foot lifting / stepping forward
   j.angles[Joints::LHipPitch]   = -(phi_l + bend) - alpha_l + gamma;
   j.angles[Joints::RHipPitch]   = -(phi_r + bend) - alpha_r + gamma;
   j.angles[Joints::LKneePitch]  = 2 * (phi_l + bend);
   j.angles[Joints::RKneePitch]  = 2 * (phi_r + bend);
   j.angles[Joints::LAnklePitch] = -(phi_l + bend) + alpha_l;
   j.angles[Joints::RAnklePitch] = -(phi_r + bend) + alpha_r;

   // Joints for rocking / strafing
   j.angles[Joints::LHipRoll]   = theta + beta_l + spread/2;
   j.angles[Joints::RHipRoll]   = theta - beta_r - spread/2;
   j.angles[Joints::LAnkleRoll] = -theta - beta_l - spread/2;
   j.angles[Joints::RAnkleRoll] = -theta + beta_r + spread/2;

   return j;
}

bool WaveWalkGenerator::isActive() {
   return t != 0.0f && !(t >= T/2 && t < T/2 + 0.01f);
}

void WaveWalkGenerator::reset() {
   t = 0.0f;
   last_forward = 0;
   last_left = 0;
   last_turn = 0;
}

void WaveWalkGenerator::readOptions(variables_map& config) {
   float f = config["walk.f"].as<float>();
   float st = config["walk.st"].as<float>();
   float r = DEG2RAD(config["walk.r"].as<float>());
   float s = DEG2RAD(config["walk.s"].as<float>());
   float l = DEG2RAD(config["walk.l"].as<float>());
   float fs = DEG2RAD(config["walk.fs"].as<float>());
   float ls = DEG2RAD(config["walk.ls"].as<float>());
   float ts = DEG2RAD(config["walk.ts"].as<float>());
   float b = DEG2RAD(config["walk.b"].as<float>());
   float m = config["walk.m"].as<float>();

   // check value constraints
   if (!(f >  0 &&
         (st == -1.0f || (st >= 0.0f && st <= 1.0f)) &&
         r >= 0 &&
         s >= 0 &&
         -(r + s/2) >= Joints::Radians::LHipRoll_Min &&
         (r + s/2) <= Joints::Radians::RHipRoll_Max &&
         l >= 0 &&
         b >= 0 && b + l <= Joints::Radians::LKneePitch_Max/2 &&
         fs >= 0 &&
         ls >= 0 &&
         ts >= 0 && -(b + l + ts) >= Joints::Radians::LHipPitch_Min &&
         m >  0 && fmod(m, 2.0f) == 0)) {
      llog(ERROR) << "Abandoning walk options: invalid input" << std::endl;
      return;
   }

   // set new parameters & re-calculate derived values
   bend = b;
   if (!stand) {
      frequency = f;
      stiffness = st;
      rock = r;
      spread = s;
      lift = l;
      f_size = fs;
      l_size = ls;
      t_size = ts;
      mul = m;

      T = 1.0 / f;
      lift_T = T / m;
      c_l = (T / 4.0) - (T / 2.0 / m);
      c_r = (3 * T / 4.0) - (T / 2.0 / m);
   }

   // reset time to 0
   // n.b: calling readOptions() while in the middle of a walk is undefined
   t = 0.0f;
   llog(INFO) << "Successfully changed walk options" << std::endl;
}
