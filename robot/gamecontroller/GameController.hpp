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
 * GameController.hpp
 * Modified: 2009-11-13
 * Description: A thread to recieve game state information from the Game
 * Controller sever and implements the Button Interface. Adapted from 2009 code.
 */

#pragma once

#include <string>
#include "gamecontroller/RoboCupGameControlData.hpp"
#include "utils/ButtonPresses.hpp"

class GameController {
   public:
      // Constructor
      GameController();
      // Destructor
      ~GameController();
      // Called on each cycle
      void tick();
      // Name, must be same as directory
      static const std::string name;
   private:
      RoboCupGameControlData data;
      TeamInfo our_team;
      bool team_red;
      bool connected;
      int sock;

      /**
       * Connect to the GameController
       */
      void initialiseConnection();

      /**
       * Update the state using the Button Interface
       */
      void buttonUpdate();

      /**
       * Update the state using the GameController Interface
       */
      void wirelessUpdate();

      /**
       * Parse data from the GameController
       * @param update Pointer to newly recieved GC data
       */
      void parseData(RoboCupGameControlData *update);

      // parseData helper functions

      bool isValidData(RoboCupGameControlData *gameData);

      bool checkHeader(char* header);

      bool isThisGame(RoboCupGameControlData *gameData);

      bool gameDataEqual(void* gameData, void* previous);

      void rawSwapTeams(RoboCupGameControlData *gameData);

      /**
       * Internal state for speech updates
       */
      uint8 lastState;
      uint16 myLastPenalty;

      /* Player & team number re-checked from config each cycle */
      int playerNumber;
      int teamNumber;

      /* Structure containing mask of buttons that have been pushed */
      ButtonPresses buttons;
};
