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

#include "utils/AbsCoord.hpp"
#include "utils/RRCoord.hpp"
#include "utils/ActionCommand.hpp"

class BroadcastData {
   public:
      BroadcastData()
      : playerNum(0), team(0), robotPos(), lostCount(0),
        egoBallPosAbs(), ballPosRr(), acB(ActionCommand::Body::DEAD),
        uptime(0.0) {
      }

      BroadcastData(const BroadcastData& bd)
      : playerNum(bd.playerNum),
        team(bd.team),
        robotPos(bd.robotPos),
        lostCount(bd.lostCount),
        egoBallPosAbs(bd.egoBallPosAbs),
        ballPosRr(bd.ballPosRr), acB(bd.acB), uptime(bd.uptime) {
      }

      BroadcastData(const int &playerNum, const int &team,
            const AbsCoord &robotPos, const uint32_t &lostCount,
            const AbsCoord &egoBallPosAbs, const RRCoord &ballPosRr,
            const ActionCommand::Body::ActionType &acB, const float &uptime)
      : playerNum(playerNum),
        team(team),
        robotPos(robotPos),
        lostCount(lostCount),
        egoBallPosAbs(egoBallPosAbs),
        ballPosRr(ballPosRr), acB(acB), uptime(uptime) {
      }
      int playerNum;
      int team;
      AbsCoord robotPos;
      uint32_t lostCount;
      AbsCoord egoBallPosAbs;
      RRCoord ballPosRr;
      ActionCommand::Body::ActionType acB;
      float uptime;
};
