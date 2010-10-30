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

typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned int   uint32;

#define GAMECONTROLLER_PORT             3838

#define GAMECONTROLLER_STRUCT_VERSION   7

#define GAMECONTROLLER_STRUCT_HEADER    "RGme"

#define MAX_NUM_PLAYERS             11

#define TEAM_BLUE                   0
#define TEAM_CYAN                   0
#define TEAM_RED                    1
#define TEAM_MAGENTA                1

#define GOAL_BLUE                   0
#define GOAL_YELLOW                 1

#define STATE_INITIAL               0
#define STATE_READY                 1
#define STATE_SET                   2
#define STATE_PLAYING               3
#define STATE_FINISHED              4
/* RUNSWIFT ADDITIONAL */
#define STATE_INVALID               5
#define STATE_PENALISED             6

#define STATE2_NORMAL               0
#define STATE2_PENALTYSHOOT         1

#define PENALTY_NONE                        0
// SPL
#define PENALTY_SPL_BALL_HOLDING            1
#define PENALTY_SPL_PLAYER_PUSHING          2
#define PENALTY_SPL_OBSTRUCTION             3
#define PENALTY_SPL_INACTIVE_PLAYER         4
#define PENALTY_SPL_ILLEGAL_DEFENDER        5
#define PENALTY_SPL_LEAVING_THE_FIELD       6
#define PENALTY_SPL_PLAYING_WITH_HANDS      7
#define PENALTY_SPL_REQUEST_FOR_PICKUP      8
// HL Kid Size
#define PENALTY_HL_KID_BALL_MANIPULATION    1
#define PENALTY_HL_KID_PHYSICAL_CONTACT     2
#define PENALTY_HL_KID_ILLEGAL_ATTACK       3
#define PENALTY_HL_KID_ILLEGAL_DEFENSE      4
#define PENALTY_HL_KID_REQUEST_FOR_PICKUP   5
// HL Teen Size
#define PENALTY_HL_TEEN_BALL_MANIPULATION   1
#define PENALTY_HL_TEEN_PHYSICAL_CONTACT    2
#define PENALTY_HL_TEEN_ILLEGAL_ATTACK      3
#define PENALTY_HL_TEEN_ILLEGAL_DEFENSE     4
#define PENALTY_HL_TEEN_REQUEST_FOR_PICKUP  5

#define PENALTY_MANUAL                      15

struct RobotInfo {
    uint16 penalty;             // penalty state of the player
    uint16 secsTillUnpenalised; // estimate of time till unpenalised
};

struct TeamInfo {
    uint8 teamNumber;          // unique team number
    uint8 teamColour;          // colour of the team
    uint8 goalColour;          // colour of the goal
    uint8 score;               // team's score
    RobotInfo players[MAX_NUM_PLAYERS];       // the team's players
};

struct RoboCupGameControlData {
    char   header[4];           // header to identify the structure
    uint32 version;             // version of the data structure
    uint8 playersPerTeam;       // The number of players on a team
    uint8 state;                // state of the game (STATE_READY, STATE_PLAYING, etc)
    uint8 firstHalf;            // 1 = game in first half, 0 otherwise
    uint8 kickOffTeam;          // the next team to kick off
    uint8 secondaryState;       // Extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
    uint8 dropInTeam;           // team that caused last drop in
    uint16 dropInTime;          // number of seconds passed since the last drop in.  -1 before first dropin
    uint32 secsRemaining;       // estimate of number of seconds remaining in the half
    TeamInfo teams[2];
};

/* RUNSWIFT ADDITIONAL */
enum GameType {
   MATCH = 0,
   DRIBBLE, 
   OPEN,
   PASSING
};

// data structure header
#define GAMECONTROLLER_RETURN_STRUCT_HEADER    "RGrt"

#define GAMECONTROLLER_RETURN_STRUCT_VERSION   1

#define GAMECONTROLLER_RETURN_MSG_MAN_PENALISE 0
#define GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE 1

struct RoboCupGameControlReturnData {
    char    header[4];
    uint32  version;
    uint16  team;
    uint16  player;             // player number - 1 based
    uint32  message;
};

/* RUNSWIFT ADDITIONAL */
extern const char* gameControllerStateNames[];

extern const char *gameControllerPenaltyNames[];
