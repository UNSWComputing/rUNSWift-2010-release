#ifndef THREADWATCHER_TCC
#define THREADWATCHER_TCC

#include "thread/ThreadWatcher.hpp"
#include <setjmp.h>
#include <alerror.h>
#include <iostream>

#include "utils/Timer.hpp"
#include "utils/log.hpp"
#include "utils/speech.hpp"
#include "soccer.hpp"
#include "blackboard/Blackboard.hpp"

/**
 * safeRun runs the class::tick() in a try block, catching any exceptions.
 * @return Always return NULL
 */

template <class T>
void overtimeAlert(int) {
   SAY(T::name + " thread has frozen");
   signal(SIGALRM, overtimeAlert<T>);
   struct itimerval itval5;
   itval5.it_value.tv_sec = 5;
   itval5.it_value.tv_usec = 0;
   itval5.it_interval.tv_sec = 0;
   itval5.it_interval.tv_usec = 0;
   setitimer(ITIMER_REAL, &itval5, NULL);
}

template <class T>
void* safelyRun(void* foo) {
   // register thread name
   pthread_t threadID = pthread_self();
   threadNames[threadID] = T::name;
   llog(INFO) << "Registering thread name '" << T::name
      << "' with ID " << threadID << std::endl;

   if (T::name == "Perception") {
      signal(SIGALRM, overtimeAlert<T>);
   }
   while (!attemptingShutdown) {
      try {
         // we don't care if this leaks
         // but it shouldn't reach this line again in this thread
         jumpPoints[threadID] =
            reinterpret_cast<jmp_buf*>(malloc(sizeof(jmp_buf)));
         if (!jumpPoints[threadID])
            llog(FATAL) << "malloc failed for" << threadNames[threadID] << "\n";

         llog(INFO) << "Module '" << T::name << "' started\n";

         // register jump point for where to resume if we crash
         if (!setjmp(*jumpPoints[threadID])) {
            T t;
            Timer timer;
            int32_t elapsed = 0.0;
            while (!attemptingShutdown) {
               timer.restart();

               int32_t cycleTime = readFrom(thread, cycleTimes[T::name]);
               if (cycleTime != -1) {
                  // set watchdog timer to alert us about stuck threads
                  if (T::name == "Perception") {
                     struct itimerval itval5;
                     itval5.it_value.tv_sec = 5;
                     itval5.it_value.tv_usec = 0;
                     itval5.it_interval.tv_sec = 0;
                     itval5.it_interval.tv_usec = 0;
                     setitimer(ITIMER_REAL, &itval5, NULL);
                  }

                  // Execute one cycle of the module
                  t.tick();

                  // unset watchdog timer to alert us about stuck threads
                  if (T::name == "Perception") {
                     struct itimerval itval0;
                     itval0.it_value.tv_sec = 0;
                     itval0.it_value.tv_usec = 0;
                     itval0.it_interval.tv_sec = 0;
                     itval0.it_interval.tv_usec = 0;
                     setitimer(ITIMER_REAL, &itval0, NULL);
                  }

                  elapsed = timer.elapsed_us();
                  llog(INFO) << "Module '" << T::name << "' took "
                     << elapsed << " us." << std::endl;
                  if (elapsed < cycleTime) {
                     usleep(cycleTime - elapsed);
                  } else if (T::name != "Motion") {
                     llog(ERROR) << "WARNING: Thread " + T::name +
                        " ran overtime" << ": " << elapsed/1000  << "ms!" <<
                        std::endl;
                     if (elapsed >= 1000000 && T::name == "Perception")
                        SAY("perception overtime");
                  }
               } else {
                  sleep(1);  // thread does not need a 'tick'
               }
            }
         }
         llog(INFO) << "Module '" << T::name
            << "' thread disabled." << std::endl;
      } catch(const std::exception& e) {
         SAY("exception caught");
         free(jumpPoints[threadID]);
         usleep(500000);
         llog(ERROR) << "exception derivative was caught with error msg: "
            << e.what() << std::endl;
         llog(ERROR) << "in " << T::name
            << " with id " << threadID << std::endl;
      } catch(...) {
         SAY("exception caught");
         free(jumpPoints[threadID]);
         usleep(500000);
         llog(ERROR) << "Something was thrown from "
            << T::name
            << " with id " << threadID << std::endl;
      }
   }
   return NULL;
}

#endif

