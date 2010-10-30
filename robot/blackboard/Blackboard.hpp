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
#include <boost/serialization/split_member.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include <signal.h>
#include <string>
#include <map>
#include <vector>
#include <list>

#include "utils/ActionCommand.hpp"
#include "utils/body.hpp"
#include "utils/ButtonPresses.hpp"
#include "utils/Odometry.hpp"
#include "utils/SensorValues.hpp"
#include "utils/RRCoord.hpp"
#include "utils/AbsCoord.hpp"
#include "utils/Line.hpp"
#include "utils/BroadcastData.hpp"
#include "utils/RobotObstacle.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/RobotRegion.hpp"
#include "perception/vision/RobotCamera.hpp"
#include "perception/localisation/LocalisationDefs.hpp"
#include "perception/kinematics/Pose.hpp"
#include "gamecontroller/RoboCupGameControlData.hpp"
#include "utils/log.hpp"

/**
 * Macro to wrap reads to module's blackboard.
 * @param module which module's blackboard to read
 * @param component the component to read
 */
#define readFrom(module, component) \
   blackboard->read(&(blackboard->module.component))

/**
 * Macro to wrap array reads from module's blackboard.
 * Performs a memcpy on the provided arguments.
 * @param module which module's blackboard to read from
 * @param component while component to be written
 * @param dest where to write to
 */
#define readArray(module, component, dest) \
   memcpy(dest, blackboard->module.component, \
         sizeof(blackboard->module.component));

/**
 * Macro to wrap writes to module's blackboard.
 * @param module which module's blackboard to write
 * @param component while component to write
 * @param value the value to be written
 */
#define writeTo(module, component, value) \
   blackboard->write(&(blackboard->module.component), value);

/**
 * Macro to wrap array writes to module's blackboard.
 * Performs a memcpy on the provided arguments.
 * @param module which module's blackboard to write
 * @param component while component to write
 * @param value the value to be written
 */
#define writeArray(module, component, value) \
   memcpy(blackboard->module.component, value, \
         sizeof(blackboard->module.component));

/**
 * Macro to wrap acquiring a mutex on a the blackboard.
 * @param module which module's blackboard to acquire
 */
#define acquireLock(name) \
   (blackboard->locks.name)->lock();

/**
 * Macro to wrap releasing a mutex on the blackboard.
 * @param module which module's blackboard to release
 */
#define releaseLock(name) \
   (blackboard->locks.name)->unlock();

/**
 * Blackboard shared memory class, used for inter-module communication.
 * The Blackboard is friends with each of the module adapters
 * so that they can access Blackboard privates (and nobody else can)
 * The safeRun templated function has access to the blackboard
 * to look up thread timings and things like that
 */

class NetworkReader;
class OverviewTab;

class Blackboard {
   // Adapter friends
   friend class LocalisationAdapter;
   friend class VisionAdapter;
   friend class MotionAdapter;
   friend class BehaviourAdapter;
   friend class GameController;
   friend class OffNaoTransmitter;
   friend class NaoTransmitter;
   friend class NaoReceiver;
   friend class KinematicsAdapter;
   friend class PerceptionThread;
   friend class BallFilter;  // big hack fix this DAVIDC

   // Off-nao friends
   friend class DumpReader;
   friend class NetworkReader;
   friend class LocalisationReader;
   friend class OverviewTab;
   friend class CameraTab;
   friend class CalibrationTab;
   friend class VisionTab;
   friend class Connection;
   friend class VariableView;
   friend class FieldView;
   friend class SensorTab;
   friend class CameraPoseTab;
   friend class GraphTab;
   friend class ZMPTab;

   // Functions
   template <class T> friend void *safelyRun(void * foo);
   friend void handleSignals(int sigNumber, siginfo_t* info, void*);
   friend bool isIncapacitated(const int &playerNum);

   public:
   explicit Blackboard(boost::program_options::variables_map vm);
   ~Blackboard();

   /* Function to read a component from the Blackboard */
   template<class T> const T& read(const T *component);

   /* Write a component to the Blackboard */
   template<class T> void write(T *component, const T& value);

   /* We now have a private inner-class blackboard for each of the
    * modules. Appropriate constructors should be placed in Blackboard.cpp
    * since there's no guarantee as to which order threads will start in. */

   private:

   /* Stores command-line/config-file options
    * should be read by modules on start
    * functionality may be added later to allow change at runtime */
   boost::program_options::variables_map config;

   /* Options callback for changes at runtime */
   void readOptions();

   /* Data Kinematics module will be sharing with others */
   class KinematicsBlackboard {
      public:
      explicit KinematicsBlackboard();
      void readOptions(boost::program_options::variables_map& config);
      Pose pose;
      float cameraOffsetXBottom;
      float cameraOffsetYBottom;
      float cameraOffsetXTop;
      float cameraOffsetYTop;
      float bodyPitchOffset;
   } kinematics;

   /* Data Behaviour module will be sharing with others */
   class BehaviourBlackboard {
      public:
         explicit BehaviourBlackboard();
         void readOptions(boost::program_options::variables_map& config);
         ActionCommand::All request;
         WhichCamera whichCamera;
         int targetx, targety;
         bool usePF;
   } behaviour;

   /* Data Localisation module will be sharing */
   class LocalisationBlackboard {
      public:
         explicit LocalisationBlackboard();
         /** Number of positions the robot reported it might be at */
         int numRobotPos;
         /** the first one is where the robot really thinks it is */
         std::vector<AbsCoord>robotPos;
         uint32_t ballLostCount;
         /** filtered global position of ball according to own observations */
         AbsCoord egoBallPosAbs;
         /** filtered global position of ball according to entire team */
         AbsCoord teamBallPosAbs;
         /** filtered robot-relative position of the ball */
         RRCoord ballPosRr;
         /** kidnap factor -- measure of how "lost" we are */
         float kidnapFactor;
        /** true if PF is running */
         bool usingPF;
         /** How many frames since the sonars picked up a close object */
         uint32_t sonarLostCount;
         /** filtered positions of visual robots */
         std::list<RobotObstacle> robotObstacles;
   } localisation;

   /* Data Vision module will be sharing with others */
   class VisionBlackboard {
      public:
         explicit VisionBlackboard();
         /** Describes which post(s) are in view */
         WhichPosts posts;
         /** Robot-relative coordinate of the bottom-center of the post */
         RRCoord post[MAX_POSTS];
         /** Image coordinate of the post **/
         uint16_t postCoords[MAX_POSTS * 4];
         /** TEMP: distances to the goal posts determined by different
             methods **/
         float distanceProjection[MAX_POSTS];
         float distanceGoalPostWidth[MAX_POSTS];
         float distanceGoalSep;
         /** TEMP: if see the bottom of the goal posts **/
         bool canSeeBottom[MAX_POSTS];
         /** Number of ball hypotheses */
         int numBalls;
         /** Robot-relative coordinate of the center of the ball */
         RRCoord ball[MAX_BALLS];
         /** Camera coordinate of the center of the ball
          *  I added this to test ball tracking...
          */
         std::pair<uint16_t, uint16_t>  ballInCameraCoords;
         uint16_t ballRadius;
         /** Number of field edges detected in this frame */
         int numEdges;
         /** Lines representing the field edges */
         Line edges[MAX_FIELD_EDGES];
         Line frameEdges[MAX_FIELD_EDGES];

         /** Points on field lines */
         RRCoord fieldLinePoints[MAX_FIELD_LINE_POINTS];
         /** Number of points on field lines */
         int numFieldLinePoints;
         /** Robot information **/
         RobotType robotTypes[MAX_NUM_ROBOTS];
         RRCoord robotLocations[MAX_NUM_ROBOTS];
         uint16_t numRobots;
         bool canSeeBottomRobot[MAX_NUM_ROBOTS];
         uint16_t robotImageCoords[MAX_NUM_ROBOTS * 4];
         /** Saliency scan */
         Colour *saliency;
         /** Pointer to the current frame being processed by Vision */
         uint8_t const* currentFrame;
   } vision;

   class PerceptionBlackboard {
      public:
         explicit PerceptionBlackboard();
         uint32_t kinematics;
         uint32_t localisation;
         uint32_t vision;
         uint32_t behaviour;
         uint32_t total;
   } perception;

   /* Data GameController will be sharing */
   class GameControllerBlackboard {
      public:
         explicit GameControllerBlackboard();
         void readOptions(boost::program_options::variables_map& config);
         bool connect;
         bool connected;
         RoboCupGameControlData data;
         TeamInfo our_team;
         bool team_red;
         int player_number;
         GameType game_type;
   } gameController;

   /* Data Motion module will be sharing with others */
   class MotionBlackboard {
      public:
         explicit MotionBlackboard();
         SensorValues sensors;
         float uptime;
         ActionCommand::All active;
         Odometry odometry;
         ButtonPresses buttons;
   } motion;

   /* Data received from friendly robots */
   class ReceiverBlackboard {
      public:
         explicit ReceiverBlackboard();
         // one for each robot on the team
         BroadcastData data[3];
         int team;
         void readOptions(boost::program_options::variables_map& config);
         time_t lastReceived[3];
   } receiver;

   /* Data ThreadWatcher will be sharing with others */
   class ThreadBlackboard {
      public:
         explicit ThreadBlackboard();
         std::map<std::string, int32_t> cycleTimes;
         std::map<std::string, boost::function<void()> > configCallbacks;
   } thread;

   /* Locks used for inter-thread synchronisation */
   class SynchronisationBlackboard {
    public:
      explicit SynchronisationBlackboard();
      boost::shared_ptr<boost::mutex> buttons;
      boost::shared_ptr<boost::mutex> behaviourmotion;
      boost::shared_ptr<boost::mutex> vision;
   } locks;
};

/* Anyone who #include's the Blackboard needs an
 * extern to the global blackboard instance */
extern Blackboard *blackboard;

#include "Blackboard.tcc"

