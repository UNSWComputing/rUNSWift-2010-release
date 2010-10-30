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

#include <fstream>
#include <limits>
#include "motion/generator/ActionGenerator.hpp"
#include "utils/log.hpp"
#include "utils/angles.hpp"

using namespace std;
using boost::program_options::variables_map;

ActionGenerator::ActionGenerator(std::string filename) : file_name(filename) {
   max_iter = 0;
   current_time = NOT_RUNNING;
};

ActionGenerator::~ActionGenerator() {
   llog(INFO) << "ActionGenerator destroyed" << std::endl;
};

bool ActionGenerator::isActive() {
   return current_time != NOT_RUNNING;
};

void ActionGenerator::reset() {
   current_time = NOT_RUNNING;
}

JointValues ActionGenerator::makeJoints(ActionCommand::All* request,
                                        Odometry* odometry,
                                        const SensorValues &sensors) {
   if (current_time == NOT_RUNNING) {
      interpolate(sensors.joints);
      current_time = 0;
      active = request->body;
   } else {
      request->body = active;
   }
   JointValues j = joints[current_time++];
   if (current_time == (signed int)joints.size())  // if we just did last action
      current_time = NOT_RUNNING;
   return j;
};

void ActionGenerator::interpolate(JointValues newJoint, int duration) {
   if (joints.empty()) {
      max_iter = duration / 10;

      // Reserve space for the interpolation when the generator
      // first called
      for (int i = 0; i < max_iter; i++) {
         joints.push_back(newJoint);
      }
      joints.push_back(newJoint);
   } else {
      int inTime = 0;
      float offset[Joints::NUMBER_OF_JOINTS];

      if (duration != 0) {
         inTime = duration / 10;
         JointValues currentJoint = joints.back();

         // Calculate the difference between new joint and the previous joint
         for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
            offset[i] = (newJoint.angles[i] - currentJoint.angles[i]) / inTime;
         }

         for (int i = 0; i < inTime; i++) {
            JointValues inJoint;
            for (int j = 0; j < Joints::NUMBER_OF_JOINTS; j++) {
               inJoint.angles[j] = joints.back().angles[j] + offset[j];
               inJoint.stiffnesses[j] = MAX_STIFF;
            }
            joints.push_back(inJoint);
         }
      } else {
         JointValues firstJoint = joints.at(max_iter);
         // Calculate the difference between the joint at MAX_ITER position
         // with the new joint
         for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
            offset[i] = (firstJoint.angles[i] - newJoint.angles[i]) / max_iter;
         }

         joints[0] = newJoint;
         for (int i = 1; i < max_iter; i++) {
            for (int j = 0; j < Joints::NUMBER_OF_JOINTS; j++) {
               joints[i].angles[j] = joints[i - 1].angles[j] + offset[j];
               joints[i].stiffnesses[j] = MAX_STIFF;
            }
         }
      }
   }
}

void ActionGenerator::constructPose(std::string path) {
   // Load from .pos file - code from previous years, adapted
   // ifstream in(string("/home/nao/data/pos/" + filename + ".pos").c_str());
   ifstream in(string(path + "/" + file_name + ".pos").c_str());
   llog(INFO) << "ActionGenerator("<< file_name << ") creating" << endl;

   if (!in.is_open()) {
      llog(FATAL) << "ActionGenerator can not open " << file_name << endl;
   } else {
      int duration = 0;
      float angles = 0.0;
      while (!in.eof()) {
         if (in.peek() == '\n') {
            in.ignore();
            continue;
         }
         // Ignore comments
         if (in.peek() == '#') {
            in.ignore(std::numeric_limits<int>::max(), '\n');
            continue;
         }
         JointValues newJoint;

         // Read the angles in the file to create a new JointValue
         for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
            in >> angles;
            // Convert degree to radian because the values in the file
            // are in degree
            newJoint.angles[i] = DEG2RAD(angles);
            newJoint.stiffnesses[i] = MAX_STIFF;
         }
         // Prevent reading the last input twice
         if (in) {
            in >> duration;
            interpolate(newJoint, duration);
         }
      }
      in.close();
   }
   llog(INFO) << "ActionGenerator("<< file_name << ") created" << endl;
}

void ActionGenerator::readOptions(variables_map& config) {
   std::string path = config["motion.path"].as<std::string>();

   constructPose(path);
}
