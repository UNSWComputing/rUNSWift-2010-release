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

#include "motion/generator/GoalieSitGenerator.hpp"
#include "utils/log.hpp"

using namespace Joints;

inline bool MOVE(JointValues& j, JointCode joint,
                 float target, float speed=1.0) {
   float max_dist = Radians::MaxSpeed[joint] * ABS(speed);
   if (ABS(target - j.angles[joint]) < max_dist) {
      j.angles[joint] = target;
      return true;
   } else if (target < j.angles[joint]) {
      j.angles[joint] -= max_dist;
   } else {
      j.angles[joint] += max_dist;
   }
   return false;
}

GoalieSitGenerator::GoalieSitGenerator()
   : stopping(false),
     stopped(true) {
   llog(INFO) << "GoalieSitGenerator created" << std::endl;
}

GoalieSitGenerator::~GoalieSitGenerator() {
   llog(INFO) << "GoalieSitGenerator destroyed" << std::endl;
}

JointValues GoalieSitGenerator::makeJoints(ActionCommand::All* request,
                                           Odometry* odometry,
                                           const SensorValues &sensors) {
   JointValues joints = sensors.joints;
   float phi = DEG2RAD(30.0f);
   uint8_t i = Joints::HeadYaw;
   joints.stiffnesses[HeadYaw] = 0.0f;  // HeadYaw
   joints.stiffnesses[HeadPitch] = 0.0f;  // HeadPitch
   joints.stiffnesses[LShoulderPitch] = 0.0f;  // LShoulderPitch
   joints.stiffnesses[LShoulderRoll] = 0.0f;  // LShoulderRoll
   joints.stiffnesses[LElbowYaw] = 0.0f;  // LElbowYaw
   joints.stiffnesses[LElbowRoll] = 0.0f;  // LElbowRoll
   joints.stiffnesses[LHipYawPitch] = 0.66f;  // LHipYawPitch
   joints.stiffnesses[LHipRoll] = 0.66f;  // LHipRoll
   joints.stiffnesses[LHipPitch] = 0.66f;  // LHipPitch
   joints.stiffnesses[LKneePitch] = 0.66f;  // LKneePitch
   joints.stiffnesses[LAnklePitch] = 0.66f;  // LAnklePitch
   joints.stiffnesses[LAnkleRoll] = 0.66f;  // LAnkleRoll
   joints.stiffnesses[RHipRoll] = 0.66f;  // RHipRoll
   joints.stiffnesses[RHipPitch] = 0.66f;  // RHipPitch
   joints.stiffnesses[RKneePitch] = 0.66f;  // RKneePitch
   joints.stiffnesses[RAnklePitch] = 0.66f;  // RAnklePitch
   joints.stiffnesses[RAnkleRoll] = 0.66f;  // RAnkleRoll
   joints.stiffnesses[RShoulderPitch] = 0.0f;  // RShoulderPitch
   joints.stiffnesses[RShoulderRoll] = 0.0f;  // RShoulderRoll
   joints.stiffnesses[RElbowYaw] = 0.0f;  // RElbowYaw
   joints.stiffnesses[RElbowRoll] = 0.0f;  // RElbowRoll
   if (stopping) {
      bool stop = true;
      stop &= MOVE(joints, LHipYawPitch, 0.0f, 0.3);
      stop &= MOVE(joints, LHipRoll, 0.0f, 0.3);
      stop &= MOVE(joints, LHipPitch, -phi, 0.3);
      stop &= MOVE(joints, LKneePitch, 2*phi, 0.3);
      stop &= MOVE(joints, LAnklePitch, -phi, 0.3);
      stop &= MOVE(joints, LAnkleRoll, 0.0f, 0.3);
      stop &= MOVE(joints, RHipRoll, 0.0f, 0.3);
      stop &= MOVE(joints, RHipPitch, -phi, 0.3);
      stop &= MOVE(joints, RKneePitch, 2*phi, 0.3);
      stop &= MOVE(joints, RAnklePitch, -phi, 0.3);
      stop &= MOVE(joints, RAnkleRoll, 0.0f, 0.3);
      stopping = !stop;
      stoppped = stop;
   } else {
      stop &= MOVE(joints, LHipYawPitch, DEG2RAD(-7.5), 0.3);
      stop &= MOVE(joints, LHipRoll, DEG2RAD(-5), 0.3);
      stop &= MOVE(joints, LHipPitch, DEG2RAD(-45), 0.3);
      stop &= MOVE(joints, LKneePitch, DEG2RAD(125), 0.3);
      stop &= MOVE(joints, LAnklePitch, DEG2RAD(-70), 0.3);
      stop &= MOVE(joints, LAnkleRoll, DEG2RAD(5), 0.3);
      stop &= MOVE(joints, RHipRoll, DEG2RAD(5), 0.3);
      stop &= MOVE(joints, RHipPitch, DEG2RAD(-45), 0.3);
      stop &= MOVE(joints, RKneePitch, DEG2RAD(125), 0.3);
      stop &= MOVE(joints, RAnklePitch, DEG2RAD(-70), 0.3);
      stop &= MOVE(joints, RAnkleRoll, DEG2RAD(-5), 0.3);
   }
   return j;
}

bool GoalieSitGenerator::isActive() { return !stopped; }

void GoalieSitGenerator::reset() {
   stopped = true;
   stopping = false;
}

void GoalieSitGenerator::stop() { stopping = true; }
