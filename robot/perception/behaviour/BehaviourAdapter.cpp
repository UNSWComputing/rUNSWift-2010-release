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

#include <limits>
#include <utility>
#include <vector>
#include "perception/behaviour/BehaviourAdapter.hpp"
#include "perception/behaviour/SkillParams.hpp"
#include "perception/behaviour/SafetySkill.hpp"
#include "perception/behaviour/python/PythonSkill.hpp"
#include "blackboard/Blackboard.hpp"
#include "perception/vision/WhichCamera.hpp"
#include "utils/log.hpp"
#include "utils/ActionCommand.hpp"
#include "utils/body.hpp"
#include "utils/SensorValues.hpp"

using namespace std;
const std::string BehaviourAdapter::name("Behaviour");

BehaviourAdapter::BehaviourAdapter() {
   llog(INFO) << "Constructing BehaviourAdapter" << endl;
   SkillParams params = getParams();
   WhichCamera camera = readFrom(behaviour, whichCamera);
   bool usePF = false;
   skill = (Skill *) new PythonSkill(
      (blackboard->config)["behaviour.path"].as<string>(),
      (blackboard->config)["behaviour.pythonclass"].as<string>(),
      &params,
      &camera,
      &usePF);
   writeTo(behaviour, whichCamera, camera);
   writeTo(behaviour, usePF, usePF);
   skill = (Skill*) new SafetySkill(skill);
}

BehaviourAdapter::~BehaviourAdapter() {
   delete skill;
}

void BehaviourAdapter::tick() {
   // Initialise head, body and led actions to NULL, skills will override
   ActionCommand::All actions = ActionCommand::All();
   // Initialise whichCamera, skills may override
   WhichCamera camera = readFrom(behaviour, whichCamera);
   // Grab stuff from blackboard
   SkillParams params = getParams();
   // Execute skill
   bool usePF = false;
   skill->execute(&params, &actions, &camera, &usePF);
   // Write ActionCommands to blackboard
   writeTo(behaviour, request, actions);
   writeTo(behaviour, usePF, usePF);
   writeTo(behaviour, whichCamera, camera);
}

SkillParams BehaviourAdapter::getParams() {
   SkillParams params;
   params.vNumBalls = readFrom(vision, numBalls);
   params.ballLocation = readFrom(vision, ballInCameraCoords);
   params.vRrBallLocation = readFrom(vision, ball[0]);
   params.ballLostCount = readFrom(localisation, ballLostCount);
   params.sonarLostCount = readFrom(localisation, sonarLostCount);
   params.lEgoBallPosAbs = readFrom(localisation, egoBallPosAbs);
   params.lTeamBallPosAbs = readFrom(localisation, teamBallPosAbs);
   params.lBallPosRr = readFrom(localisation, ballPosRr);
   params.posts.which = readFrom(vision, posts);
   readArray(vision, post, params.posts.pos);
   readArray(vision, canSeeBottom, params.posts.onField);
   params.targetx = readFrom(behaviour, targetx);
   params.targety = readFrom(behaviour, targety);
   params.usingPF = readFrom(localisation, usingPF);
   params.kidnapFactor = readFrom(localisation, kidnapFactor);
   const vector<AbsCoord> &robotPos = readFrom(localisation, robotPos);
   AbsCoord pos;
   if (robotPos.size()) {
      pos = robotPos[0];
   }
   params.robotPos = pos;
   acquireLock(behaviourmotion);
   params.sensors = readFrom(motion, sensors);
   params.uptime = readFrom(motion, uptime);
   releaseLock(behaviourmotion);
   params.gcConnected = readFrom(gameController, connected);
   params.gcData = readFrom(gameController, data);
   params.gcTeam = readFrom(gameController, our_team);
   params.player = (blackboard->config)["player.number"].as<int>();
   params.pythonSkill =
      (blackboard->config)["behaviour.pythonclass"].as<string>();
   params.acActive = readFrom(motion, active);
   readArray(receiver, data, params.teamData);
   readArray(vision, robotTypes, params.robotTypes);
   readArray(vision, robotLocations, params.robotLocations);
   params.numRobots = readFrom(vision, numRobots);
   params.whichCamera = readFrom(behaviour, whichCamera);
   params.robotObstacles = readFrom(localisation, robotObstacles);
   params.gameType = readFrom(gameController, game_type);
   return params;
}

