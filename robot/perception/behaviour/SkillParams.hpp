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

#include <stdint.h>
#include <utility>
#include <string>
#include <list>
#include "utils/RRCoord.hpp"
#include "utils/AbsCoord.hpp"
#include "utils/RobotObstacle.hpp"
#include "utils/SensorValues.hpp"
#include "utils/ActionCommand.hpp"
#include "utils/BroadcastData.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/WhichCamera.hpp"
#include "perception/vision/RobotRegion.hpp"
#include "gamecontroller/RoboCupGameControlData.hpp"

struct PostInfo {
   RRCoord pos[MAX_POSTS];
   bool onField[MAX_POSTS];
   WhichPosts which;
};

/**
 * List of items that skills may use in their
 * decision-making. Should correspond to items
 * on the Blackboard. Add things here that you
 * want to use in behaviour.
 */
struct SkillParams {
   int vNumBalls;
   int player;
   std::pair<uint16_t, uint16_t> ballLocation;
   RRCoord vRrBallLocation;
   uint32_t ballLostCount;
   uint32_t sonarLostCount;
   AbsCoord lEgoBallPosAbs;
   AbsCoord lTeamBallPosAbs;
   RRCoord lBallPosRr;
   PostInfo posts;
   SensorValues sensors;
   int targetx;
   int targety;
   AbsCoord robotPos;
   float uptime;
   bool gcConnected;
   RoboCupGameControlData gcData;
   TeamInfo gcTeam;
   std::string pythonSkill;
   ActionCommand::All acActive;
   BroadcastData teamData[3];
   bool usingPF;
   float kidnapFactor;
   RobotType robotTypes[MAX_NUM_ROBOTS];
   RRCoord robotLocations[MAX_NUM_ROBOTS];
   uint16_t numRobots;
   WhichCamera whichCamera;
   std::list<RobotObstacle> robotObstacles;
   GameType gameType;
};

