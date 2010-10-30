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
 * ActionCommand.hpp
 * Modified: 2009-11-08
 * Description: Commands which are accepted by the Locomotion Module
 * This is the new interface between the Behaviour and Locomotion
 * Body are for the walks and special actions which use the body
 * Head are for the head yaw and pitch
 * LED are for the ear, face, chest and foot LEDs
 * Tidied up from 2009 code
 */

#pragma once
#include <stdint.h>
#include <iostream>
#include "utils/body.hpp"

namespace ActionCommand {

/**
 * Command for controlling the body
 * Note: Some ActionType Commands WILL disable the head
 */
struct Body {
   // Predefined actions. These take precedence over walk parameters
   enum ActionType {
      NONE = 0,

      // Stand - common starting pose for all other actions
      STAND,

      // Walks
      WALK, SLOW, FAST, WAVE, AL,

      // Actions
      GETUP_FRONT, GETUP_BACK,
      KICK,
      INITIAL,
      DEAD, REF_PICKUP,
      SQUAT, SQUAT_FORWARD, OPEN_FEET,
      THROW_IN,
      GOALIE_SIT,

      NUM_ACTION_TYPES
   };
   ActionType actionType;

   // Walk/Kick Parameters
   int forward;  // How far forward (negative for backwards)  (mm)
   int left;     // How far to the left (negative for rightwards) (mm)
   float turn;   // How much anti-clockwise turn (negative for clockwise) (rad)
   float power;  // How much kick power (0.0-1.0)

   /**
    * Constructor for walks and kicks
    * @param at Action Type
    * @param f  How far forward (mm)
    * @param l  How far to the left (mm)
    * @param t  How much counter-clockwise turn (rad)
    * #param p  How
    */
   Body(ActionType at=NONE, int f=0, int l=0, float t=0.0, float p=1.0)
      : actionType(at),
        forward(f),
        left(l),
        turn(t),
        power(p) {}
};

const uint8_t priorities[Body::NUM_ACTION_TYPES] = {
   0,  // NONE
   0,  // STAND
   0,  // WALK
   0,  // SLOW
   0,  // FAST
   0,  // WAVE
   0,  // AL
   2,  // GETUP_FRONT
   2,  // GETUP_BACK
   0,  // KICK
   3,  // INITIAL
   1,  // DEAD
   0,  // REF_PICKUP
   0,  // SQUAT
   0,  // SQUAT_FORWARD
   0,  // OPEN_FEET
   0,  // THROW_IN
   0,  // GOALIE_SIT
};

/**
 * Command for controlling the head
 */
struct Head {
   float yaw;         // LEFT-RIGHT motion. Positive is LEFT
   float pitch;       // UP-DOWN angle. Positive is DOWN
   bool isRelative;   // TRUE to add to current head angles [DEFAULT]
   float yawSpeed;    // Speed of the yaw [0.0, 1.0]
   float pitchSpeed;  // Speed of the pitch [0.0, 1.0]

   /**
    * Constructor
    * @param y Yaw amount (Left is positive) (rad)
    * @param p Pitch amount (Down is positive) (rad)
    * @param r Enable relative adjustment (default). False for absolute
    * @param ys Yaw speed [0.0, 1.0]
    * @param ps Pitch speed [0.0, 1.0]
    * @param pid Use the PID-controller
    */
   Head(float y=0.0, float p=0.0, bool r=true,
        float ys=1.0, float ps=1.0) : yaw(y),
                                      pitch(p),
                                      isRelative(r),
                                      yawSpeed(ys),
                                      pitchSpeed(ps) {}
};

struct LED {
   struct rgb {
      bool red;
      bool green;
      bool blue;

      rgb(bool r=false, bool g=false, bool b= false) : red(r),
                                                       green(g),
                                                       blue(b) {}
   };

   uint16_t leftEar;   // Number of left ear segments lit [10-bit field]
   uint16_t rightEar;  // Number of right ear segments lit [10-bit field]
   rgb leftEye;        // Colour of left eye (default: white)
   rgb rightEye;       // Colour of right eye (default: white)
   rgb chestButton;    // Colour of chest button (default: white)
   rgb leftFoot;       // Colour of left foot (default: off)
   rgb rightFoot;      // Colour of right foot (default: off)

   LED(uint16_t lear=0x3FF, uint16_t rear=0x3FF, rgb leye=rgb(true, true, true),
       rgb reye= rgb(true, true, true), rgb cb= rgb(true, true, true),
       rgb lf= rgb(), rgb rf= rgb()) : leftEar(lear),
                                      rightEar(rear),
                                      leftEye(leye),
                                      rightEye(reye),
                                      chestButton(cb),
                                      leftFoot(lf),
                                      rightFoot(rf) {}
};

/**
 * Wrapper for the other action commands, makes it easier to pass them around
 */
struct All {
   Head head;
   Body body;
   LED leds;
   float sonar;

   All() : head(), body(Body::NONE), leds(), sonar(Sonar::Mode::OFF) {
   }

   All(Head h, Body b, LED l, float s) {
      head = h;
      body = b;
      leds = l;
      sonar = s;
   }
};

//  These classes support stream output for debugging
static inline bool operator==(const LED::rgb &a, const LED::rgb &b) {
   return (a.red == b.red) && (a.green == b.green) && (a.blue == b.blue);
}

static inline bool operator!=(const LED::rgb &a, const LED::rgb &b) {
   return !(a == b);
}

static inline std::ostream &operator<<(std::ostream &out , const LED::rgb &a) {
   out << '{' << a.red << ", " << a.green << ", " << a.blue << '}';
   return out;
}

static inline std::ostream &operator<<(std::ostream &out , const Head &a) {
   out << '[' << a.yaw << ", " << a.pitch << ", " << a.isRelative << ']';
   return out;
}

static inline std::ostream &operator<<(std::ostream &out , const Body &a) {
   out << '[' << a.actionType << ", " << a.forward << ", " << a.left
              << ", " << a.turn << "," << a.power << ']';
   return out;
}

static inline std::ostream &operator<<(std::ostream &out , const LED &a) {
   out << '[' << a.leftEar << ", " << a.rightEar << ", " << a.leftEye << ", "
              << a.rightEye << "," << a.chestButton << ","
              << a.leftFoot << "," << a.rightFoot << ']';
   return out;
}
};  // namespace ActionCommand
