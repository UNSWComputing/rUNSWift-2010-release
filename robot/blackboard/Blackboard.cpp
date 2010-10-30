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

#include "blackboard/Blackboard.hpp"
#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <limits>
#include "utils/angles.hpp"
#include "utils/log.hpp"

using namespace std;
using namespace boost;

/* The global Blackboard instance */
Blackboard *blackboard;

Blackboard::Blackboard(boost::program_options::variables_map vm)
   : config(vm) {
   readOptions();
   function<void()> ro = boost::bind(&Blackboard::readOptions, this);
   thread.configCallbacks["Blackboard"] = ro;
   llog(INFO) << "Initialising the blackboard" << endl;
}

Blackboard::~Blackboard() {
   thread.configCallbacks["Blackboard"] = function<void()>();
   llog(INFO) << "Blackboard destroyed" << endl;
}

void Blackboard::readOptions() {
   behaviour.readOptions(config);
   gameController.readOptions(config);
   receiver.readOptions(config);
   kinematics.readOptions(config);
}

Blackboard::BehaviourBlackboard::BehaviourBlackboard()
   : targetx(0), targety(0) {
   llog(INFO) << "Initialising blackboard: behaviour" << endl;
   whichCamera = BOTTOM_CAMERA;
   usePF = false;
}

void Blackboard::BehaviourBlackboard::
                 readOptions(program_options::variables_map& config) {
   string a = config["default.body"].as<string>();
   if (a == "NONE") request.body.actionType = ActionCommand::Body::NONE;
   if (a == "STAND") request.body.actionType = ActionCommand::Body::STAND;
   if (a == "WALK") request.body.actionType = ActionCommand::Body::WALK;
   if (a == "SLOW") request.body.actionType = ActionCommand::Body::SLOW;
   if (a == "FAST") request.body.actionType = ActionCommand::Body::FAST;
   if (a == "WAVE") request.body.actionType = ActionCommand::Body::WAVE;
   if (a == "AL") request.body.actionType = ActionCommand::Body::AL;
   if (a == "INITIAL") request.body.actionType = ActionCommand::Body::INITIAL;
   if (a == "KICK") request.body.actionType = ActionCommand::Body::KICK;
   if (a == "SQUAT") request.body.actionType = ActionCommand::Body::SQUAT;
   if (a == "GETUP_FRONT")
      request.body.actionType = ActionCommand::Body::GETUP_FRONT;
   if (a == "GETUP_BACK")
      request.body.actionType = ActionCommand::Body::GETUP_BACK;
   if (a == "DEAD") request.body.actionType = ActionCommand::Body::DEAD;
   if (a == "REF_PICKUP")
      request.body.actionType = ActionCommand::Body::REF_PICKUP;
   if (a == "THROW_IN")
      request.body.actionType = ActionCommand::Body::THROW_IN;
   if (a == "GOALIE_SIT")
      request.body.actionType = ActionCommand::Body::GOALIE_SIT;
   request.body.forward = config["default.forward"].as<int>();
   request.body.left = config["default.left"].as<int>();
   request.body.turn = DEG2RAD(config["default.turn"].as<float>());
   request.body.power = config["default.power"].as<float>();
   targetx = config["behaviour.targetx"].as<int>();
   targety = config["behaviour.targety"].as<int>();
}

Blackboard::LocalisationBlackboard::LocalisationBlackboard() {
   llog(INFO) << "Initialising blackboard: localisation" << endl;
   robotPos = vector<AbsCoord>();
   robotPos.push_back(AbsCoord());
   numRobotPos = 1;
   ballLostCount = numeric_limits<uint32_t>::max();
   kidnapFactor = 0;
   usingPF = false;
}

Blackboard::VisionBlackboard::VisionBlackboard() {
   llog(INFO) << "Initialising blackboard: vision" << endl;
   posts = pNONE;
   numBalls = 0;
   numEdges = 0;
   numFieldLinePoints = 0;
   numRobots = 0;
   saliency = NULL;
   currentFrame = NULL;
}

Blackboard::PerceptionBlackboard::PerceptionBlackboard() {
   kinematics = 0;
   localisation = 0;
   vision = 0;
   behaviour = 0;
   total = 33;
}

Blackboard::MotionBlackboard::MotionBlackboard() {
   llog(INFO) << "Initilalising blackboard: motion" << endl;
   uptime = 0;
}

Blackboard::KinematicsBlackboard::KinematicsBlackboard() {
   llog(INFO) << "Initilalising blackboard: kinematics" << endl;
}

void Blackboard::KinematicsBlackboard::
                 readOptions(program_options::variables_map& config) {
   cameraOffsetXBottom = config["kinematics.cameraoffsetXbottom"].as<float>();
   cameraOffsetYBottom = config["kinematics.cameraoffsetYbottom"].as<float>();
   cameraOffsetXTop = config["kinematics.cameraoffsetXtop"].as<float>();
   cameraOffsetYTop = config["kinematics.cameraoffsetYtop"].as<float>();
   bodyPitchOffset = config["kinematics.bodyPitchOffset"].as<float>();
}

Blackboard::GameControllerBlackboard::GameControllerBlackboard() {
   llog(INFO) << "Initialising blackboard: gameController" << endl;
   connected = false;
   game_type = MATCH;
   memset(&our_team, 0, sizeof our_team);
   memset(&data, 0, sizeof data);
}

void Blackboard::GameControllerBlackboard::
                 readOptions(program_options::variables_map& config) {
   connect = config["gamecontroller.connect"].as<bool>();
   player_number = config["player.number"].as<int>();
   string a = config["game.type"].as<string>();
   if (a == "MATCH") game_type = MATCH;
   if (a == "DRIBBLE") game_type = DRIBBLE;
   if (a == "OPEN") game_type = OPEN;
   if (a == "PASSING") game_type = PASSING;
   our_team.teamNumber = config["player.team"].as<int>();
   our_team.teamColour = (int) (config["gamecontroller.ourcolour"].
         as<string>() == "red");
   team_red = our_team.teamColour;
   our_team.score = config["gamecontroller.ourscore"].as<int>();
   for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
      our_team.players[i].penalty = PENALTY_NONE;
      our_team.players[i].secsTillUnpenalised = 0;
   }
   TeamInfo their_team;
   their_team.teamNumber = config["gamecontroller.opponentteam"].as<int>();
   their_team.teamColour = (our_team.teamColour + 1) % 2;
   their_team.score = config["gamecontroller.opponentscore"].as<int>();
   for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
      their_team.players[i].penalty = PENALTY_NONE;
      their_team.players[i].secsTillUnpenalised = 0;
   }
   map<string, int> gcStateMap;
   gcStateMap["INITIAL"] = STATE_INITIAL;
   gcStateMap["READY"] = STATE_READY;
   gcStateMap["SET"] = STATE_SET;
   gcStateMap["PLAYING"] = STATE_PLAYING;
   gcStateMap["FINISHED"] = STATE_FINISHED;
   if (gcStateMap.count(config["gamecontroller.state"].as<string>())) {
      data.state = gcStateMap[config["gamecontroller.state"].as<string>()];
   } else {
      data.state = STATE_INVALID;
   }
   data.firstHalf = config["gamecontroller.firsthalf"].as<bool>();
   data.kickOffTeam = (int) (config["gamecontroller.kickoffteam"].as<string>()
         == "red");
   map<string, int> gcSecStateMap;
   gcSecStateMap["NORMAL"] = STATE2_NORMAL;
   gcSecStateMap["PENALTYSHOOT"] = STATE2_PENALTYSHOOT;
   data.secondaryState = gcSecStateMap[
      config["gamecontroller.secondarystate"].as<string>()];
   data.secsRemaining = config["gamecontroller.secsremaining"].as<int>();
   data.teams[our_team.teamColour] = our_team;
   data.teams[their_team.teamColour] = their_team;
}

Blackboard::ReceiverBlackboard::ReceiverBlackboard() {
   lastReceived[0] = time(NULL);
   lastReceived[1] = time(NULL);
   lastReceived[2] = time(NULL);
}

void Blackboard::ReceiverBlackboard::
   readOptions(program_options::variables_map& config) {
   team = config["player.team"].as<int>();
}

Blackboard::ThreadBlackboard::ThreadBlackboard() {
   llog(INFO) << "Initialising blackboard: thread" << endl;
   cycleTimes["Perception"] = 0;  // as fast as possible, waits on camera read
   cycleTimes["Motion"] = 0;  // as fast as possible, waits on agent semaphore
   cycleTimes["GameController"] = 0;  // as fast as possible, waits on udp read
   cycleTimes["Off-Nao Transmitter"] = 50000;  // 20fps limit
   cycleTimes["Nao Transmitter"] = 200000;  // 5fps limit
   cycleTimes["Nao Receiver"] = -1;  // never run

   configCallbacks["Motion"];
   configCallbacks["Blackboard"];
   configCallbacks["GameController"];
   configCallbacks["Kinematics"];
   configCallbacks["Receiver"];
   configCallbacks["Vision"];
}

Blackboard::SynchronisationBlackboard::SynchronisationBlackboard() {
   buttons = boost::shared_ptr<boost::mutex>(new boost::mutex());
   behaviourmotion = boost::shared_ptr<boost::mutex>(new boost::mutex());
   vision = boost::shared_ptr<boost::mutex>(new boost::mutex());
}
