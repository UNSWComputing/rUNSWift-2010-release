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

#include <boost/program_options.hpp>
#include <sys/utsname.h>
#include <signal.h>
#include <errno.h>
#include <vector>
#include <map>
#include <iostream>
#include <string>

#include "utils/log.hpp"
#include "utils/options.hpp"
#include "soccer.hpp"
#include "thread/ThreadWatcher.hpp"
#include "perception/PerceptionThread.hpp"
#include "motion/MotionAdapter.hpp"
#include "transmitter/OffNao.hpp"
#include "transmitter/Nao.hpp"
#include "receiver/Nao.hpp"
#include "gamecontroller/GameController.hpp"

#define handle_error_en(en, msg) \
   do { \
      errno = en;\
      perror(msg);\
      exit(1);\
   } while (0)

namespace po = boost::program_options;
using namespace std;
using namespace boost;

extern string git_version;

/**
 * A timer function.  After arg seconds, runswift will shut down.
 *
 * @param arg coerced to int.  seconds to shutdown
 * @returns NULL
 */
static void *shutdownTimer(void * arg) {
   int time = (int)arg;
   if (time > 5) {
     sleep(time - 5);
     SAY("shutting down");
     sleep(5);
   } else {
     sleep(time);
   }
   attemptingShutdown = true;
   return NULL;
}


/** Entry point for runswift application */
int main(int argc, char **argv) {
   po::variables_map vm;
   try {
      po::options_description generic("Generic options");
      generic.add_options()
         ("help,h", "produce help message")
         ("version,v", "print version string");

      po::options_description cmdline_options =
            store_and_notify(argc, argv, vm, &generic);

      if (vm.count("help")) {
         cout << cmdline_options << endl;
         return 1;
      }

      if (vm.count("version")) {
         cout << "rUNSWift Nao soccer player " << git_version << endl;
         return 1;
      }

      cout << "rUNSWift V." << git_version << endl;

      options_print(vm);
   } catch(program_options::error& e) {
      cerr << "Error when parsing command line arguments: " << e.what() << endl;
      return 1;
   } catch(std::exception& e) {
      cerr << e.what() << endl;
      return 1;
   }

   bool alkernel;

   struct utsname name;
   if (uname(&name) == -1) {
      cout << "Cannot get system name, assuming you are not on a robot" << endl;
      alkernel = false;
   } else if (string(name.release).find("aldebaran") != string::npos) {
      cout << "Aldebaran kernel detected!" << endl;
      alkernel = true;
   } else {
      alkernel = false;
   }

   offNao = false;
   attemptingShutdown = false;
   initLogs(vm["debug.log"].as<string>(),
            vm["debug.logpath"].as<string>(),
            vm["debug.log.motion"].as<bool>());
   blackboard = new Blackboard(vm);
   llog(SILENT) << "Log level is " << logLevel << endl;
   llog(INFO) << "RUNSWIFT soccer library spinning up!" << endl;

   registerSignalHandlers();

   // Start all threads
   pthread_t perception;
   pthread_t motion;
   pthread_t gameController;
   pthread_t offnaotransmitter;
   pthread_t naotransmitter;
   pthread_t naoreceiver;
   if (vm["debug.perception"].as<bool>()) {
      pthread_create(&perception, NULL, &safelyRun<PerceptionThread>, NULL);
      llog(INFO) << "Perception is running" << endl;
   }
   if (vm["debug.motion"].as<bool>()) {
      // something special for motion when on Nao, we want real-time performance
      if (alkernel) {
         int s;
         struct sched_param param;
         int policy;
         pthread_attr_t attr;
         s = pthread_attr_init(&attr);
         if (s != 0) {
            handle_error_en(s, "pthread_attr_init");
         }
         s = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
         if (s != 0) {
            handle_error_en(s, "pthread_attr_setinheritsched");
         }
         s = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
         if (s != 0) {
            handle_error_en(s, "pthread_attr_setschedpolicy");
         }
         param.sched_priority = 65;
         s = pthread_attr_setschedparam(&attr, &param);
         if (s != 0) {
            handle_error_en(s, "pthread_attr_setschedparam");
         }
         s = pthread_create(&motion, &attr, &safelyRun<MotionAdapter>, NULL);
         if (s != 0) {
            handle_error_en(s, "pthread_create");
         }
         pthread_attr_destroy(&attr);
         llog(INFO) << "Motion is running" << endl;
         s = pthread_getschedparam(motion, &policy, &param);
         if (s != 0) {
            handle_error_en(s, "pthread_getschedparam");
         }
         llog(INFO) << "Motion granted priority: " <<
            param.sched_priority << endl;
      } else {
         pthread_create(&motion, NULL, &safelyRun<MotionAdapter>, NULL);
         llog(INFO) << "Motion is running" << endl;
      }
   }
   if (vm["debug.gamecontroller"].as<bool>()) {
      pthread_create(&gameController, NULL, &safelyRun<GameController>, NULL);
      llog(INFO) << "GameController is running" << endl;
   }
   if (vm["debug.offnaotransmitter"].as<bool>()) {
      pthread_create(&offnaotransmitter, NULL, &safelyRun<OffNaoTransmitter>,
                     NULL);
      llog(INFO) << "Off-Nao Transmitter is running" << endl;
   }
   if (vm["debug.naotransmitter"].as<bool>()) {
      pthread_create(&naotransmitter, NULL, &safelyRun<NaoTransmitter>,
                     NULL);
      llog(INFO) << "Nao Transmitter is running" << endl;
   }
   if (vm["debug.naoreceiver"].as<bool>()) {
      pthread_create(&naoreceiver, NULL, &safelyRun<NaoReceiver>,
                     NULL);
      llog(INFO) << "Nao Receiver is running" << endl;
   }

   if (vm["debug.shutdowntime"].as<int>()) {
      pthread_t timer;
      pthread_create(&timer, NULL, &shutdownTimer,
                     (void*)vm["debug.shutdowntime"].as<int>());
      llog(INFO) << "Timer is running" << endl;
   }

   // Wait for all threads to end
   if (vm["debug.perception"].as<bool>()) {
      pthread_join(perception, NULL);
      llog(INFO) << "Perception thread joined" << endl;
   }
   if (vm["debug.motion"].as<bool>()) {
      pthread_join(motion, NULL);
      llog(INFO) << "Motion thread joined" << endl;
   }
   if (vm["debug.gamecontroller"].as<bool>()) {
      pthread_join(gameController, NULL);
      llog(INFO) << "GameController thread joined" << endl;
   }
   if (vm["debug.offnaotransmitter"].as<bool>()) {
      pthread_join(offnaotransmitter, NULL);
      llog(INFO) << "Off-Nao Transmitter thread joined" << endl;
   }
   if (vm["debug.naotransmitter"].as<bool>()) {
      pthread_join(naotransmitter, NULL);
      llog(INFO) << "Nao Transmitter thread joined" << endl;
   }
   if (vm["debug.naoreceiver"].as<bool>()) {
      pthread_join(naoreceiver, NULL);
      llog(INFO) << "Nao Receiver thread joined" << endl;
   }

   delete blackboard;
   delLogs();

   return 0;
}

