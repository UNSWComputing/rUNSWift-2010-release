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
 * @file The thread watchers. The purpose is to watch threads and restart
 * their corresponding modules on exceptions or runtime errors.
 * It also allows for the runtime starting/stopping of various modules.
 */
#pragma once

#include <pthread.h>
#include <signal.h>
#include <setjmp.h>
#include <string>
#include "utils/ConcurrentMap.hpp"

#define ALL_SIGNALS -1  // for indicating that we should register
                        // all singal handlers

/**
 * Registers handleSignals() to all the signals we want to handle.
 * @param signal The signal to register. Valid ones are SIGSEGV, SIGFPE, SIGSTKFLT. By default we re-register all the signals again.
 * @see handleSignals
 */
void registerSignalHandlers(int signal = ALL_SIGNALS);

/**
 * safelyRun runs class::tick() inside a block that is allowed to 
 * throw any exceptions or any signal (if this occurs the module 
 * will just be restart, too bad for the memory it was using).
 */
template <class T> void* safelyRun(void* foo);

extern ConcurrentMap<pthread_t, jmp_buf*>     jumpPoints;
extern ConcurrentMap<pthread_t, std::string> threadNames;
extern bool attemptingShutdown;

#include "thread/ThreadWatcher.tcc"
