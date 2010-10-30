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

#include "libagent/libagent.hpp"
#include <sys/mman.h>        /* For shared memory */
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */
#include <unistd.h>          /* For sleep */
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <limits>
#include <string>
#include <vector>
#include "utils/basic_maths.hpp"
#include "utils/body.hpp"
#include "utils/speech.hpp"
#include "utils/options.hpp"

using namespace AL;
using std::string;
namespace po = boost::program_options;

int ret;  // variable to grab return values from system calls

const std::string Agent::name("libagent");

void preCallback_(Agent* a) { a->preCallback(); }
void postCallback_(Agent* a) { a->postCallback(); }

/* Borrowed from B-Human code release '09 */
inline unsigned int timeNow() {
   static unsigned int base = 0;
   struct timeval tv;
   gettimeofday(&tv, 0);
   unsigned int time = (unsigned int) (tv.tv_sec * 1000 + tv.tv_usec / 1000);
   if (!base)
      base = time - 10000;
   return time - base;
}

void Agent::doNetworking() {
   SAY("fixing networking", true);
   string command = "/bin/su -c '/usr/bin/killall udhcpc'";
   log->info(name, command);
   ret = system(command.c_str());
   command = "/bin/su -c '/sbin/iwconfig wlan0 " + wirelessIwconfigArgs + "'";
   log->info(name, command);
   ret = system(command.c_str());
   if (wirelessStatic == true) {
      command = "/bin/su -c '/sbin/ifconfig wlan0 " + wirelessIfconfigArgs +
                "'";
      log->info(name, command);
      ret = system(command.c_str());
   } else {
      command = "/bin/su -c '/sbin/udhcpc -i wlan0' &";
      log->info(name, command);
      ret = system(command.c_str());
   }
   if (wiredStatic == true) {
      command = "/bin/su -c '/sbin/ifconfig eth0 " + wiredIfconfigArgs + "'";
      log->info(name, command);
      ret = system(command.c_str());
   } else {
      command = "/bin/su -c '/sbin/udhcpc -i eth0' &";
      log->info(name, command);
      ret = system(command.c_str());
   }
}

void Agent::doAvahi() {
   SAY("fixing avahi");
   ret = system("/bin/su -c '/etc/init.d/avahi-daemon restart'");
}

void Agent::doBattery(float charge, float current, int status) {
   // Start complaining if battery < 30%
   // measured in 33 discrete levels (ie, to 3% accuracy)
   int battery = charge * 33;
   // if battery decreasing & <= 30%
   if (old_battery > battery && battery <= 10) {
      SAY("battery " + boost::lexical_cast<string>(battery * 3) +
          " percent");
   }
   old_battery = battery;
   std::bitset<16> b = status;
   if (!old_battery_status[5] && b[5]) {
      log->info(name, "Discharging");
   } else if (!old_battery_status[6] && b[6]) {
      log->info(name, "Fully Charged");
   } else if (!old_battery_status[7] && b[7]) {
      log->info(name, "Charging");
   }
   old_battery_status = b;
}

void Agent::doTemps() {
   static int t = 0;
   if (t % 100 == 0 &&
       *temperature_pointers[t/100] > 70 &&
       !limp && !shared_data->standing &&
       shared_data->joints[shared_data->actuators_read].stiffnesses[t/100] > 0)
      SAY("OVERHEATING: " + Joints::fliteJointNames[t/100]);
   t = (t + 1) % (Joints::NUMBER_OF_JOINTS * 100);
}

void Agent::doButtons(bool chest, bool left, bool right) {
   // deal with button presses
   if (chest_up > MAX_CLICK_INTERVAL && chest_presses) {
      buttons.push(chest_presses);
      chest_presses = 0;
      if (buttons.pop(2)) {
         if (left || right) {
            head_limp = !head_limp;
            SAY(std::string("head ") +
                (head_limp ? "limp" : "stiff"));
         } else {
            if (limp) {
               limp = false;
               shared_data->standing = true;
            } else {
               limp = true;
               sit_step = -1.0f;
               shared_data->standing = false;
               motion->killAll();
            }
            SAY(std::string("body ") +
                (limp ? "limp" : "stiff"));
         }
      } else if (buttons.pop(3)) {
         if (left || right) {
            SAY("Restarting now key");  // yay transliteration
            // Runlevel a is set up to run nao restart on demand.
            // This avoids the problem where killall naoqi kills us and
            // therefore our children (including /etc/init.d/naoqi).
            // See man pages for inittab, init for details
            ret = system("/bin/su -c '/usr/bin/killall runswift'");
            ret = system("/bin/su -c '/sbin/init a'");
            // should not get here
         } else {
            if (skipped_frames > MAX_SKIPS) {
               SAY("loaded runswift");
               if (!fork()) {
                  struct sched_param s;
                  s.sched_priority = 0;
                  int ret = sched_setscheduler(0, SCHED_OTHER, &s);
                  ret = system("/bin/su -c '/usr/bin/killall runswift'");
                  ret = execlp("/home/nao/bin/runswift", "runswift",
                                                         (char*)NULL);
                  ::exit(ret);
               }
            } else {
               SAY("killed runswift");
               ret = system("/bin/su -c '/usr/bin/killall runswift'");
               while (waitpid(-1, NULL, WNOHANG) > 0);
            }
         }
      } else if (buttons.pop(4)) {
         if ((left || right)) {
            SAY("flashing chest board");
            ret = system("/bin/su -c '/sbin/init c'");
         } else {
            doNetworking();
            doAvahi();
         }
      }
   }

   // special shutdown handler
   // we set chest_down to int_min so only one shutdown will happen
   if (chest_down > 300) {  // 3 seconds
      if (left || right) {
         SAY("Flashing and Restarting");
         ret = system("/bin/su -c '/usr/bin/killall runswift'");
         ret = system("/bin/su -c '/sbin/init b'");
         limp = true;
         chest_down = std::numeric_limits<int>::min();
      } else {
         SAY("Shutting down");
         ret = system("/bin/su -c '/usr/bin/killall runswift'");
         ret = system("/bin/su -c /sbin/halt");
         limp = true;
         chest_down = std::numeric_limits<int>::min();
      }
   }

   // update counters
   if (chest) {
      if (chest_down >= 0) chest_down++;
      chest_up = 0;
   } else {
      chest_up++;
      if (chest_down > 0) {
         chest_presses++;
         chest_down = 0;
      }
   }
}

void Agent::doLEDs(ActionCommand::LED& leds) {
   uint8_t i;

   // Battery (Left Ear)
   uint16_t tmp = shared_data->sensors[shared_data->sensors_latest].
                     sensors[Sensors::Battery_Charge] * 10;
   for (i = LEDs::LeftEar1; i <= LEDs::LeftEar10; ++i) {
      led_command[5][i][0] = (tmp >= i) ? 1.0f : 0.0f;
   }

   // Battery (Right Ear)
   for (; i <= LEDs::RightEar10; ++i) {
      led_command[5][i][0] = (tmp >= i - 10) ? 1.0f : 0.0f;
   }

/** DO NOT DELETE - This is here so I don't forget how to do ears **
   // Right Ear
   for (tmp = leds.rightEar; i <= LEDs::RightEar10; ++i) {
      led_command[5][i][0] = (float) (tmp % 2);
      tmp /= 2;
   }
*/

   // Eyes
   for (; i <= LEDs::LeftEyeRed8; ++i)
      led_command[5][i][0] = (float) leds.leftEye.red;
   for (; i <= LEDs::LeftEyeGreen8; ++i)
      led_command[5][i][0] = (float) leds.leftEye.green;
   for (; i <= LEDs::LeftEyeBlue8; ++i)
      led_command[5][i][0] = (float) leds.leftEye.blue;
   for (; i <= LEDs::RightEyeRed8; ++i)
      led_command[5][i][0] = (float) leds.rightEye.red;
   for (; i <= LEDs::RightEyeGreen8; ++i)
      led_command[5][i][0] = (float) leds.rightEye.green;
   for (; i <= LEDs::RightEyeBlue8; ++i)
      led_command[5][i][0] = (float) leds.rightEye.blue;

   // Chest
   led_command[5][LEDs::ChestRed][0] = (float) leds.chestButton.red;
   led_command[5][LEDs::ChestGreen][0] = (float) leds.chestButton.green;
   led_command[5][LEDs::ChestBlue][0] = (float) leds.chestButton.blue;

   // Feet
   led_command[5][LEDs::LeftFootRed][0] = (float) leds.leftFoot.red;
   led_command[5][LEDs::LeftFootGreen][0] = (float) leds.leftFoot.green;
   led_command[5][LEDs::LeftFootBlue][0] = (float) leds.leftFoot.blue;
   led_command[5][LEDs::RightFootRed][0] = (float) leds.rightFoot.red;
   led_command[5][LEDs::RightFootGreen][0] = (float) leds.rightFoot.green;
   led_command[5][LEDs::RightFootBlue][0] = (float) leds.rightFoot.blue;
}

void Agent::preCallback() {
   // Avoid race condition on shutdown
   if (shuttingDown) {
      return;
   }

   const unsigned int now = timeNow();
   shared_data->actuators_read = shared_data->actuators_latest;
   JointValues& joints = shared_data->joints[shared_data->actuators_read];
   SensorValues& sensors = shared_data->sensors[shared_data->sensors_latest];

   doLEDs(shared_data->leds[shared_data->actuators_read]);
   if (limp || shared_data->standing) {
      uint8_t i;
      for (i = Joints::HeadYaw; i <= Joints::HeadPitch; ++i) {
         head_angle_command[5][i][0] = sit_angles[i];
         head_stiffness_command[5][i][0] = 0.0f;
      }
      for (i = Joints::LShoulderPitch; i < Joints::NUMBER_OF_JOINTS; ++i) {
         angle_command[5][i-2][0] = sit_angles[i];
         stiffness_command[5][i-2][0] = 0.0f;
      }

      // Make chest button blink green if limp but still in contact
      // or blink red if not in contact or purple if ready to stand
      float blink = now / 200 & 1;
      if (shared_data->standing) {
         led_command[5][LEDs::ChestRed][0] = blink;
         led_command[5][LEDs::ChestGreen][0] = blink;
         led_command[5][LEDs::ChestBlue][0] = 0.0;
      } else if (skipped_frames <= MAX_SKIPS) {
         led_command[5][LEDs::ChestRed][0] = 0.0f;
         led_command[5][LEDs::ChestGreen][0] = blink;
         led_command[5][LEDs::ChestBlue][0] = 0.0f;
      } else {
         led_command[5][LEDs::ChestRed][0] = blink;
         led_command[5][LEDs::ChestGreen][0] = 0.0f;
         led_command[5][LEDs::ChestBlue][0] = 0.0f;
      }
   } else if (skipped_frames <= MAX_SKIPS && sit_step == -1.0f) {
      uint8_t i;
      for (i = Joints::HeadYaw; i <= Joints::HeadPitch; ++i) {
         head_angle_command[5][i][0] = joints.angles[i];
         head_stiffness_command[5][i][0] = joints.stiffnesses[i];
      }
      if (joints.AL_command == AL_ON) {
         static int stand_t = 0;
         motion->setStiffnesses("Body", 0.66);
         ALValue config;
         switch (alwalk_state) {
         case INACTIVE:
            config.arraySetSize(13);
            for (i = 0; i < 13; ++i)
               config[i].arraySetSize(2);
            config[0][0]  = "WALK_MAX_TRAPEZOID";
            config[0][1]  = 4.5;    // 4.5
            config[1][0]  = "WALK_MIN_TRAPEZOID";
            config[1][1]  = 3.5;    // 3.5
            config[2][0]  = "WALK_STEP_MAX_PERIOD";
            config[2][1]  = 30;     // 30
            config[3][0]  = "WALK_STEP_MIN_PERIOD";
            config[3][1]  = 21;     // 21
            config[4][0]  = "WALK_MAX_STEP_X";
            config[4][1]  = 0.04;   // 0.04
            config[5][0]  = "WALK_MAX_STEP_Y";
            config[5][1]  = 0.04;   // 0.04
            config[6][0]  = "WALK_MAX_STEP_THETA";
            config[6][1]  = 20;     // 20
            config[7][0]  = "WALK_STEP_HEIGHT";
            config[7][1]  = 0.015;  // 0.015
            config[8][0]  = "WALK_FOOT_SEPARATION";
            config[8][1]  = 0.095;  // 0.095
            config[9][0]  = "WALK_FOOT_ORIENTATION";
            config[9][1]  = 0;      // 0
            config[10][0] = "WALK_TORSO_HEIGHT";
            config[10][1] = joints.AL_height;   // 0.31
            config[11][0] = "WALK_TORSO_ORIENTATION_X";
            config[11][1] = 0;      // 0
            config[12][0] = "WALK_TORSO_ORIENTATION_Y";
            config[12][1] = 0;      // 0
            motion->setMotionConfig(config);
            stand_angles.clear();
            stand_angles.push_back(0.f);
            stand_angles.push_back(0.f);
            stand_angles.push_back(-joints.AL_bend);
            stand_angles.push_back(2*joints.AL_bend);
            stand_angles.push_back(-joints.AL_bend);
            stand_angles.push_back(0.f);
            stand_angles.push_back(0.f);
            stand_angles.push_back(-joints.AL_bend);
            stand_angles.push_back(2*joints.AL_bend);
            stand_angles.push_back(-joints.AL_bend);
            stand_angles.push_back(0.f);
            alwalk_state = WALKING;
            break;
         case WALKING:
            if (joints.AL_stop) {
               // motion->walkTo(0.001, 0.001, 0.001);
               motion->post.angleInterpolation(leg_names, stand_angles,
                                               0.25, true);
               alwalk_state = STANDING;
            } else {
               motion->setWalkTargetVelocity(joints.AL_x, joints.AL_y,
                                             joints.AL_theta,
                                             joints.AL_frequency);
            }
            break;
         case STOPPING:
            if (!motion->walkIsActive()) {
               motion->post.angleInterpolation(leg_names, stand_angles,
                                               0.25, true);
               alwalk_state = STANDING;
            }
            break;
         case STANDING:
            if (stand_t++ > 25) {
               stand_t = 0;
               motion->killWalk();
               alwalk_state = STOPPED;
            }
            break;
         case STOPPED:
            alwalk_state = INACTIVE;
            break;
         }
      } else {
         for (i = Joints::LShoulderPitch; i < Joints::NUMBER_OF_JOINTS; ++i) {
            angle_command[5][i-2][0] = joints.angles[i];
            stiffness_command[5][i-2][0] = joints.stiffnesses[i];
         }
      }
   } else {
      if (sit_step == -1.0f) {
         sit_step = 0.0f;
         for (uint8_t i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
            sit_joints.angles[i] = sensors.joints.angles[i];
         SAY("lost contact");
         motion->killAll();
      }
      if (sit_step < 1.0f) {
         // interpolate legs over 2s, arms over 0.2s
         sit_step += 0.005f;
         uint8_t i;
         for (i = Joints::HeadYaw; i <= Joints::HeadPitch; ++i) {
            head_angle_command[5][i][0] = (1 - sit_step) * sit_joints.angles[i]
                                        + sit_step * sit_angles[i];
            head_stiffness_command[5][i][0] = 1.0f;
         }
         for (i = Joints::LShoulderPitch; i < Joints::NUMBER_OF_JOINTS; ++i) {
            float k = (i >= Joints::LShoulderPitch &&
                       i <= Joints::LElbowRoll) ||
                      (i >= Joints::RShoulderPitch &&
                       i <= Joints::RElbowRoll) ? 10 : 1;
            k = MIN(1.0f, k * sit_step);
            angle_command[5][i-2][0] = (1 - k) * sit_joints.angles[i] +
                                     k * sit_angles[i];
            stiffness_command[5][i-2][0] = 1.0f;
         }
      } else {
         limp = true;
         sit_step = -1.0f;
         uint8_t i;
         for (i = Joints::HeadYaw; i <= Joints::HeadPitch; ++i) {
            head_angle_command[5][i][0] = sit_angles[i];
            head_stiffness_command[5][i][0] = 0.0f;
         }
         for (i = Joints::LShoulderPitch; i < Joints::NUMBER_OF_JOINTS; ++i) {
            angle_command[5][i-2][0] = sit_angles[i];
            stiffness_command[5][i-2][0] = 0.0f;
         }
      }

      // Make chest button solid red/green if sitting down
      // (depending on whether or not we are back in contact)
      if (skipped_frames > MAX_SKIPS) {
         led_command[5][LEDs::ChestRed][0] = 1.0f;
         led_command[5][LEDs::ChestGreen][0] = 0.0f;
      } else {
         led_command[5][LEDs::ChestRed][0] = 0.0f;
         led_command[5][LEDs::ChestGreen][0] = 1.0f;
      }
      led_command[5][LEDs::ChestBlue][0] = 0.0f;
   }
   // sonar_command[5][0][0] = shared_data->sonar[shared_data->actuators_read];
   sonar_command[5][0][0] = Sonar::Mode::CONTINUOUS + Sonar::Mode::ON;

   if (head_limp) {
      float blink = now / 200 & 1;
      led_command[5][LEDs::LeftEyeGreen8][0] = blink;
      led_command[5][LEDs::RightEyeGreen8][0] = blink;
      head_stiffness_command[5][Joints::HeadYaw][0] = 0.0f;
      head_stiffness_command[5][Joints::HeadPitch][0] = 0.0f;
   }

   head_stiffness_command[4][0] = (int)now + time_offset;
   head_angle_command[4][0] = (int)now + time_offset;
   stiffness_command[4][0] = (int)now + time_offset;
   angle_command[4][0] = (int)now + time_offset;
   led_command[4][0] = (int)now + time_offset;
   sonar_command[4][0] = (int)now + time_offset;
   dcm->setAlias(head_stiffness_command);
   dcm->setAlias(head_angle_command);
   if (limp || shared_data->standing ||
       !(skipped_frames <= MAX_SKIPS && sit_step == -1.0f &&
         joints.AL_command == AL_ON)) {
      dcm->setAlias(stiffness_command);
      dcm->setAlias(angle_command);
   }
   dcm->setAlias(led_command);
   // dcm->setAlias(sonar_command);
}

void Agent::postCallback() {
   // Avoid race condition on shutdown
   if (shuttingDown) {
      return;
   }

   // Read sensors
   SensorValues s;
   int i;
   for (i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
      s.sensors[i] = *sensor_pointers[i];
   for (i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
      s.joints.angles[i] = *joint_pointers[i];
   for (i = 0; i < Sonar::NUMBER_OF_READINGS; ++i)
      s.sonar[i] = *sonar_pointers[i];

   if (shared_data->joints[shared_data->actuators_latest].AL_command == AL_ON)
      s.AL_isActive = !(alwalk_state == STOPPED || alwalk_state == INACTIVE);
   // Find the right index to write to
   for (i = 0; i != shared_data->sensors_latest &&
               i != shared_data->sensors_read; ++i);
   shared_data->sensors[i] = s;

   float* v = s.sensors;
   doBattery(v[Sensors::Battery_Charge],
             v[Sensors::Battery_Current],
             *battery_status_pointer);
   doButtons(v[Sensors::ChestBoard_Button],
             v[Sensors::LFoot_Bumper_Left] || v[Sensors::LFoot_Bumper_Right],
             v[Sensors::RFoot_Bumper_Left] || v[Sensors::RFoot_Bumper_Right]);
   doTemps();

   shared_data->buttons[i] = buttons;
   buttons.clear();
   shared_data->sensors_latest = i;

   // Get the value of the semaphore, and hence the lag between us and runswift
   // We only V() the semaphore if it is 0.
   sem_getvalue(semaphore, &i);
   if (i <= 0) {
      sem_post(semaphore);
      if (skipped_frames > MAX_SKIPS) {
         log->info(name, "Back after " +
                         boost::lexical_cast<string>(skipped_frames));
      }
      skipped_frames = 0;
   } else {
      skipped_frames++;
      if (skipped_frames <= 2)
         log->error(name, "runswift missed one of our cycles");
   }
}

Agent::Agent(AL::ALPtr<AL::ALBroker> pBroker, const std::string& pName)
   : ALModule(pBroker, pName),
     alwalk_state(INACTIVE),
     shuttingDown(false),
     skipped_frames(0),
     sit_step(-1.0f),
     limp(true),
     head_limp(false),
     chest_down(0),
     chest_up(0),
     chest_presses(0),
     old_battery(0),
     old_battery_status(0) {
   uint8_t i;

   // need to initialise first, before using log
   log = new ALLoggerProxy(pBroker);
   if (log == NULL)
      throw ALERROR(name, "constructor", "log == NULL");

   log->info(name, "Constructing");

   try {
      po::options_description cmdline_options =
            store_and_notify(std::vector<string>(0), vm, NULL);
      wirelessIwconfigArgs = vm["network.wireless.iwconfig_flags"].as<string>();
      wirelessIfconfigArgs =
                      vm["network.wireless.static.ifconfig_flags"].as<string>();
      wiredIfconfigArgs =
                         vm["network.wired.static.ifconfig_flags"].as<string>();

      wirelessStatic = vm["network.wireless.static"].as<bool>();
      wiredStatic = vm["network.wired.static"].as<bool>();

      playerNum = vm["player.number"].as<int>();
      teamNum = vm["player.team"].as<int>();

      doNetworking();
   } catch(po::error& e) {
      log->error(name, "failed parsing command line arguments");
      log->error(name, e.what());
   } catch(std::exception& e) {
      log->error(name, "failed parsing command line arguments");
      log->error(name, e.what());
   }

   dcm = new DCMProxy(pBroker);
   if (dcm == NULL)
      throw ALERROR(name, "constructor", "dcm == NULL");

   memory = new ALMemoryProxy(pBroker);
   if (memory == NULL)
      throw ALERROR(name, "constructor", "memory == NULL");

   motion = new ALMotionProxy(pBroker);
   motion->setWalkArmsEnable(false, false);
   ALValue config;
   config.arraySetSize(13);
   for (i = 0; i < 13; ++i)
      config[i].arraySetSize(2);
   config[0][0]  = "WALK_MAX_TRAPEZOID",       config[0][1]  = 4.5;    // 4.5
   config[1][0]  = "WALK_MIN_TRAPEZOID",       config[1][1]  = 3.5;    // 3.5
   config[2][0]  = "WALK_STEP_MAX_PERIOD",     config[2][1]  = 30;     // 30
   config[3][0]  = "WALK_STEP_MIN_PERIOD",     config[3][1]  = 21;     // 21
   config[4][0]  = "WALK_MAX_STEP_X",          config[4][1]  = 0.04;   // 0.04
   config[5][0]  = "WALK_MAX_STEP_Y",          config[5][1]  = 0.04;   // 0.04
   config[6][0]  = "WALK_MAX_STEP_THETA",      config[6][1]  = 20;     // 20
   config[7][0]  = "WALK_STEP_HEIGHT",         config[7][1]  = 0.015;  // 0.015
   config[8][0]  = "WALK_FOOT_SEPARATION",     config[8][1]  = 0.095;  // 0.095
   config[9][0]  = "WALK_FOOT_ORIENTATION",    config[9][1]  = 0;      // 0
   config[10][0] = "WALK_TORSO_HEIGHT",        config[10][1] = 0.31;   // 0.31
   config[11][0] = "WALK_TORSO_ORIENTATION_X", config[11][1] = 0;      // 0
   config[12][0] = "WALK_TORSO_ORIENTATION_Y", config[12][1] = 0;      // 0
   motion->setMotionConfig(config);

   leg_names.push_back("LHipYawPitch"), stand_angles.push_back(0.f);
   leg_names.push_back("LHipRoll"), stand_angles.push_back(0.f);
   leg_names.push_back("LHipPitch"), stand_angles.push_back(DEG2RAD(-30.f));
   leg_names.push_back("LKneePitch"), stand_angles.push_back(DEG2RAD(60.f));
   leg_names.push_back("LAnklePitch"), stand_angles.push_back(DEG2RAD(-30.f));
   leg_names.push_back("LAnkleRoll"), stand_angles.push_back(0.f);
   leg_names.push_back("RHipRoll"), stand_angles.push_back(0.f);
   leg_names.push_back("RHipPitch"), stand_angles.push_back(DEG2RAD(-30.f));
   leg_names.push_back("RKneePitch"), stand_angles.push_back(DEG2RAD(60.f));
   leg_names.push_back("RAnklePitch"), stand_angles.push_back(DEG2RAD(-30.f));
   leg_names.push_back("RAnkleRoll"), stand_angles.push_back(0.f);

   // open shared memory as RW, create if !exist, permissions 600
   shared_fd = shm_open(AGENT_MEMORY, O_RDWR | O_CREAT, 0600);
   if (shared_fd < 0)
      throw ALERROR(name, "constructor", "shm_open() failed");
   // make shared memory file correct size
   if (ftruncate(shared_fd, sizeof(AgentData)) == -1)
      throw ALERROR(name, "constructor", "ftruncate() failed");
   // map shared memory to process memory
   shared_data = (AgentData*) mmap(NULL, sizeof(AgentData),
                                   PROT_READ | PROT_WRITE,
                                   MAP_SHARED, shared_fd, 0);
   if (shared_data == MAP_FAILED)
      throw ALERROR(name, "constructor", "mmap() failed");

   // Initialise shared memory
   shared_data->sensors_read = 0;
   shared_data->sensors_latest = 0;
   shared_data->actuators_read = 0;
   shared_data->actuators_latest = 0;

   SensorValues null_sensors;
   JointValues null_joints;
   ActionCommand::LED null_leds;
   for (i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
      null_joints.angles[i] = 0.0f;
      null_joints.stiffnesses[i] = 0.0f;
      null_joints.temperatures[i] = 0.0f;
   }
   null_sensors.joints = null_joints;
   for (i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
      null_sensors.sensors[i] = 0.0f;
   for (i = 0; i < 3; ++i) {
      shared_data->sensors[i] = null_sensors;
      shared_data->joints[i] = null_joints;
      shared_data->leds[i] = null_leds;
      shared_data->sonar[i] = 0.0f;
   }
   shared_data->standing = false;
   pthread_mutexattr_t attr;
   pthread_mutexattr_init(&attr);
   pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
   pthread_mutex_init(&shared_data->lock, &attr);

   // create semaphore with permissions 600, value 0
   semaphore = sem_open(AGENT_SEMAPHORE, O_RDWR | O_CREAT, 0600, 0);
   if (semaphore < 0)
      throw ALERROR(name, "constructor", "sem_open() failed");

   // Initialise ALMemory pointers
   std::vector<std::string> key_names;
   for (i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
      sensor_pointers.push_back((float*) memory->getDataPtr(
                                std::string("Device/SubDeviceList/") +
                                Sensors::sensorNames[i] + "/Sensor/Value"));
   for (i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
      joint_pointers.push_back((float*) memory->getDataPtr(
                               std::string("Device/SubDeviceList/") +
                               Joints::jointNames[i] +
                               "/Position/Sensor/Value"));
      temperature_pointers.push_back((float*) memory->getDataPtr(
                                    std::string("Device/SubDeviceList/") +
                                    Joints::jointNames[i] +
                                    "/Temperature/Sensor/Value"));
   }
   for (i = 0; i < Sonar::NUMBER_OF_READINGS; ++i)
      sonar_pointers.push_back((float*) memory->getDataPtr(
                               std::string("Device/SubDeviceList/") +
                               Sonar::readingNames[i]));
   battery_status_pointer = (int*) memory->
      getDataPtr("Device/SubDeviceList/Battery/Charge/Sensor/Status");

   // DCM Command Aliases
   ALValue alias;
   alias.arraySetSize(2);
   alias[0] = string("JointAngles");
   alias[1].arraySetSize(Joints::NUMBER_OF_JOINTS - 2);
   for (i = Joints::LShoulderPitch; i < Joints::NUMBER_OF_JOINTS; ++i) {
      alias[1][i-2] = string(Joints::jointNames[i]).
                      append("/Position/Actuator/Value");
   }
   dcm->createAlias(alias);
   alias[0] = string("HeadAngles");
   alias[1].arraySetSize(2);
   for (i = Joints::HeadYaw; i <= Joints::HeadPitch; ++i) {
      alias[1][i] = string(Joints::jointNames[i]).
                    append("/Position/Actuator/Value");
   }
   dcm->createAlias(alias);
   alias[0] = string("LEDs");
   alias[1].arraySetSize(LEDs::NUMBER_OF_LEDS);
   for (i = 0; i < LEDs::NUMBER_OF_LEDS; ++i) {
      alias[1][i] = string(LEDs::ledNames[i]);
   }
   dcm->createAlias(alias);
   alias[0] = string("JointStiffnesses");
   alias[1].arraySetSize(Joints::NUMBER_OF_JOINTS - 2);
   for (i = Joints::LShoulderPitch; i < Joints::NUMBER_OF_JOINTS; ++i) {
      alias[1][i-2] = string(Joints::jointNames[i]).
                      append("/Hardness/Actuator/Value");
   }
   dcm->createAlias(alias);
   alias[0] = string("HeadStiffnesses");
   alias[1].arraySetSize(2);
   for (i = Joints::HeadYaw; i <= Joints::HeadPitch; ++i) {
      alias[1][i] = string(Joints::jointNames[i]).
                    append("/Hardness/Actuator/Value");
   }
   dcm->createAlias(alias);
   alias[0] = string("Sonar");
   alias[1].arraySetSize(1);
   alias[1][0] = Sonar::actuatorName;
   dcm->createAlias(alias);

   // Set up Commands ALValue
   angle_command.arraySetSize(6);
   angle_command[0] = string("JointAngles");
   angle_command[1] = string("ClearAfter");
   angle_command[2] = string("time-separate");
   angle_command[3] = 0;
   angle_command[4].arraySetSize(1);
   angle_command[5].arraySetSize(Joints::NUMBER_OF_JOINTS - 2);
   for (i = Joints::LShoulderPitch; i < Joints::NUMBER_OF_JOINTS; ++i) {
      angle_command[5][i-2].arraySetSize(1);
   }
   head_angle_command.arraySetSize(6);
   head_angle_command[0] = string("HeadAngles");
   head_angle_command[1] = string("ClearAfter");
   head_angle_command[2] = string("time-separate");
   head_angle_command[3] = 0;
   head_angle_command[4].arraySetSize(1);
   head_angle_command[5].arraySetSize(2);
   for (i = Joints::HeadYaw; i <= Joints::HeadPitch; ++i) {
      head_angle_command[5][i].arraySetSize(1);
   }
   led_command.arraySetSize(6);
   led_command[0] = string("LEDs");
   led_command[1] = string("ClearAfter");
   led_command[2] = string("time-separate");
   led_command[3] = 0;
   led_command[4].arraySetSize(1);
   led_command[5].arraySetSize(LEDs::NUMBER_OF_LEDS);
   for (i = 0; i < LEDs::NUMBER_OF_LEDS; ++i) {
      led_command[5][i].arraySetSize(1);
   }
   stiffness_command.arraySetSize(6);
   stiffness_command[0] = string("JointStiffnesses");
   stiffness_command[1] = string("ClearAfter");
   stiffness_command[2] = string("time-separate");
   stiffness_command[3] = 0;
   stiffness_command[4].arraySetSize(1);
   stiffness_command[5].arraySetSize(Joints::NUMBER_OF_JOINTS - 2);
   for (i = Joints::LShoulderPitch; i < Joints::NUMBER_OF_JOINTS; ++i) {
      stiffness_command[5][i-2].arraySetSize(1);
   }
   head_stiffness_command.arraySetSize(6);
   head_stiffness_command[0] = string("HeadStiffnesses");
   head_stiffness_command[1] = string("ClearAfter");
   head_stiffness_command[2] = string("time-separate");
   head_stiffness_command[3] = 0;
   head_stiffness_command[4].arraySetSize(1);
   head_stiffness_command[5].arraySetSize(2);
   for (i = Joints::HeadYaw; i <= Joints::HeadPitch; ++i) {
      head_stiffness_command[5][i].arraySetSize(1);
   }
   sonar_command.arraySetSize(6);
   sonar_command[0] = string("Sonar");
   sonar_command[1] = string("ClearAfter");
   sonar_command[2] = string("time-separate");
   sonar_command[3] = 0;
   sonar_command[4].arraySetSize(1);
   sonar_command[4][0] = dcm->getTime(20);
   sonar_command[5].arraySetSize(1);
   sonar_command[5][0].arraySetSize(1);
   sonar_command[5][0][0] = 68.0f;
   dcm->setAlias(sonar_command);

   // Get offset between our clock and the DCM's
   time_offset = (int)dcm->getTime(0) - timeNow();

   // Set up callbacks
   dcm->getModule()->atPostProcess(boost::bind(postCallback_, this));
   dcm->getModule()->atPreProcess(boost::bind(preCallback_, this));

   // Fix the clock if an ntp server is available
   ret = system("/bin/su -c '/etc/init.d/openntpd restart'");
   ret = system("/usr/bin/amixer -qs < /home/nao/data/volumeinfo.dat");


   // Play the 'oneg-nook' jingle
   ret =
      system("/usr/bin/aplay -q /opt/naoqi/share/naoqi/wav/start_jingle.wav");

   // Start rUNSWift
   // Uncomment this to have rUNSWift start when you turn the robot on
   /*
   SAY("loaded runswift");
   if (!fork()) {
      struct sched_param s;
      s.sched_priority = 0;
      int ret = sched_setscheduler(0, SCHED_OTHER, &s);
      ret = execlp("/home/nao/bin/runswift", "runswift",
            (char*)NULL);
      ::exit(ret);
   }
   */

   log->info(name, "Constructed");
}

Agent::~Agent() {
   log->info(name, "Destroying");
   shuttingDown = true;
   ret = system("/bin/su -c '/usr/bin/killall runswift'");
   if (shared_data != MAP_FAILED) munmap(shared_data, sizeof(AgentData));
   if (shared_fd >= 0) close(shared_fd);
   if (semaphore != SEM_FAILED) sem_close(semaphore);
   delete dcm;
   delete memory;
   log->info(name, "Destroyed");
   delete log;
}

extern "C" int _createModule(AL::ALPtr<AL::ALBroker> pBroker) {
   AL::ALModule::createModule<Agent>(pBroker, "Agent");
   return 0;
}
