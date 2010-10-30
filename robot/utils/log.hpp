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

#pragma once

#include <boost/thread/mutex.hpp>
#include <ostream>
#include <cstdio>
#include <string>
#include <map>

/**
 * Possible log levels
 * When something is to be logged an associated level is passed with it. Only
 * messages with levels greater than logLevel are actually logged at runtime.
 * Levels should be set to an appropriate point, so that selecting a logLevel
 * such as INFO gives a corresponding level of logging.
 * A message is FATAL if the robot cannot function in its current state.
 * An ERROR, while still allowing a robot to run, greatly impairs its ability
 * to function accurately or as intended.
 * INFO gives useful, concise summaries of the robot's functioning.
 * VERBOSE details at length every step that INFO summarises.
 * DEBUGx gives multiple levels of debugging, for the coder's pleasure.
 * @see logLevel
 */
enum LogLevel {
   SILENT  = -100,
   QUIET   = -67,
   FATAL   = -33,
   ERROR   = 0,
   WARNING = 10,
   INFO    = 20,
   VERBOSE = 40,
   /*DEBUG   = 60,*/
   DEBUG1  = 60,
   DEBUG2  = 80,
   DEBUG3  = 100
};
extern std::map<std::string, LogLevel> logLevels;

/**
 * The cutoff for messages to be logged
 * Every message has an associated log level. This global variable determines
 * which of those to log. If it is set to SILENT there should be no output.
 * Default level is ERROR. The level can be changed at runtime, and should
 * reflect the amount of information wanted. More information is wanted while
 * debugging, and though less is wanted on game-day, fatal and error messages
 * are still important.
 * @see LogLevel
 */
extern int logLevel;

/**
 * A map to map from log filename stream pointer. To avoid recalculating
 * directory name and repoening file.
 */
extern std::map<std::string, std::ostream *> *logStreams;

/**
 * Where the logs get stored
 */
extern std::string logpath;

/**
 * Initialises the logLevel, motionLogging flag, and logStreams
 */
void initLogs(std::string level, std::string path, bool motion);

/**
 * Destruct all log streams
 */
void delLogs();

/**
 * stream for log messages (log file name base on code file name)
 *
 * @param file     source code file name
 * @param logLevel the requested log level
 * @return an ostream that writes to the appropriate log file
 */
std::ostream &realLlog(const char *file, int logLevel);

/**
 * stream for log messages (log file name base on code file name)
 *
 * @param logLevel the requested log level
 * @return an ostream that writes to the appropriate log file
 */
#define llog(logLevel) realLlog(__FILE__, logLevel)

