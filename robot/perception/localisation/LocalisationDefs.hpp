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

#include <string>
#include <sstream>
#include <utility>
#include "utils/SPLDefs.hpp"
#include "utils/AbsCoord.hpp"
#include "utils/angles.hpp"
#include "utils/Line.hpp"
#include "gamecontroller/RoboCupGameControlData.hpp"

/* post positions in AbsCoord*/
#define GOAL_POST_ABS_X_DIST ((FIELD_LENGTH / 2) - FIELD_LINE_WIDTH)
#define GOAL_POST_ABS_Y_DIST (GOAL_WIDTH / 2)

/* top of field is on the opponent's goal side, going clock wise
 * -- using symmetrical left and right
 */
#define LINE_STEP 200
#define TOP_LEFT_FIELD_X (FULL_FIELD_LENGTH / 2)
#define TOP_LEFT_FIELD_Y (FULL_FIELD_WIDTH / 2)
#define TOP_RIGHT_FIELD_X (FULL_FIELD_LENGTH / 2)
#define TOP_RIGHT_FIELD_Y (-FULL_FIELD_WIDTH / 2)
#define BOTTOM_LEFT_FIELD_X (-FULL_FIELD_LENGTH / 2)
#define BOTTOM_LEFT_FIELD_Y (FULL_FIELD_WIDTH / 2)
#define BOTTOM_RIGHT_FIELD_X (-FULL_FIELD_LENGTH / 2)
#define BOTTOM_RIGHT_FIELD_Y (-FULL_FIELD_WIDTH / 2)

/* Number of steps per circle for seeing one goal post */
#define NUM_SEGMENTS 36  // 12 /* 30 degrees per step */
#define RADIANCE_SETP (M_PI / (NUM_SEGMENTS / 2))

#define LEFT 0
#define RIGHT 1

/* Useful, but not in math.h */
#define PI_8 (M_PI_4/2.0)

/* Base standard deviation for each dimension */
#define STD_DEV_XY_CONST 1000.0f
#define STD_DEV_T_CONST DEG2RAD(10.0f)

/* Don't do global updates based on a single post and
 * edge if you are this close to the side or goal line */
#define INSIDE_FIELD_SAFETY_MARGIN 300

/* Don't allow robot to exceed these coordiantes */
#define FIELD_X_CLIP ((FIELD_LENGTH+FIELD_LENGTH_OFFSET)/2)
#define FIELD_Y_CLIP ((FIELD_WIDTH+FIELD_WIDTH_OFFSET)/2)

/* Max number of particules for the PF to create */
#define MAX_ROBOT_POS 1000

/* Dimensions that are filtered by the KF */
typedef enum {
   X = 0,
   Y,
   THETA,
   STATE_VEC_DIM
} StateVector;

#define OFFNAO_TEAM TEAM_BLUE

struct Point {
   Point() : x(0), y(0) {}

   Point(float x, float y) {
      this->x = x;
      this->y = y;
   }

   float distTo(Point p) {
      return (sqrt(SQUARE(x - p.x) + SQUARE(y - p.y)));
   }

   const std::string toString() {
      std::stringstream ss;
      ss << "(" << x << ", " << y << ")";
      return ss.str();
   }

   float x;
   float y;
};

struct Edge {
   Edge() : start(), end() {}

   Edge(Point a, Point b) {
      start = a;
      end = b;
   }

   Point start;
   Point end;
};


struct EdgeInfo {
   float xDist;
   float yDist;
   float robotHeading;
};

/* list of edges in clock wise direction, two adjacent edges are joined
 * to each other
 */
#define MAX_OBS_EDGES 2

#define E_TOP 0
#define E_RIGHT 1
#define E_BOTTOM 2
#define E_LEFT 3
#define E_COUNT 4
const Edge edges_abs[] = {
   Edge(Point(TOP_LEFT_FIELD_X, TOP_LEFT_FIELD_Y),
         Point(TOP_RIGHT_FIELD_X, TOP_RIGHT_FIELD_Y)),
   Edge(Point(TOP_RIGHT_FIELD_X, TOP_RIGHT_FIELD_Y),
         Point(BOTTOM_RIGHT_FIELD_X, BOTTOM_RIGHT_FIELD_Y)),
   Edge(Point(BOTTOM_LEFT_FIELD_X, BOTTOM_LEFT_FIELD_Y),
         Point(BOTTOM_RIGHT_FIELD_X, BOTTOM_RIGHT_FIELD_Y)),
   Edge(Point(TOP_LEFT_FIELD_X, TOP_LEFT_FIELD_Y),
         Point(BOTTOM_LEFT_FIELD_X, BOTTOM_LEFT_FIELD_Y))
};

const Point posts_abs[2][2] = {
   { Point(GOAL_POST_ABS_X_DIST,  GOAL_POST_ABS_Y_DIST),
     Point(GOAL_POST_ABS_X_DIST, -GOAL_POST_ABS_Y_DIST)
   },
   { Point(-GOAL_POST_ABS_X_DIST, -GOAL_POST_ABS_Y_DIST),
     Point(-GOAL_POST_ABS_X_DIST,  GOAL_POST_ABS_Y_DIST)
   }
};

// FIELD_LENGTH away -- more likely to be on the field, more dynamic!!!
#define FIELD_DIAGONAL \
      sqrt(SQUARE(FULL_FIELD_WIDTH) + SQUARE(FULL_FIELD_LENGTH))

#define IS_VALID_DIST(dist) \
   (dist <= FIELD_DIAGONAL)

#define WEIGHT_X     0
#define WEIGHT_Y     1
#define WEIGHT_THETA 2
#define NUM_WEIGHTS  3
struct Particle {
   float weight[3];
   AbsCoord pos;

   /* default constructor, sets weights to 1, pos to fault Point() */
   Particle() : pos() {
      for (int i = 0; i < NUM_WEIGHTS; i++) {
         /* 0 (low confidence) <= weights <= 1 (high confidence) */
         weight[i] = 1;
      }
   }

   /* constructor which initialises the weights to 1 but pos to pt
    * passed in
    */
   explicit Particle(AbsCoord pos) {
      for (int i = 0; i < NUM_WEIGHTS; i++) {
         /* 0 (low confidence) <= weights <= 1 (high confidence) */
         weight[i] = 1;
      }
      this->pos = pos;
   }

   /* return the x, y position of the particle as a point */
   Point point() {
      return (Point(pos.x, pos.y));
   }

   float getWeight() const {
      return (0.35 * this->weight[WEIGHT_X]
            + 0.35 * this->weight[WEIGHT_Y]
            + 0.3 * this->weight[WEIGHT_THETA]);
   }

   float minWeight() const {
      return (this->weight[WEIGHT_X] < this->weight[WEIGHT_THETA] ?
            this->weight[WEIGHT_X] : this->weight[WEIGHT_THETA]);
   }

   AbsCoord getPos() const {
      return (this->pos);
   }

   const std::string toString() {
      std::stringstream ss;
      ss << "(" << pos.x << "[" << weight[0] << "], " << pos.y
         << "[" << weight[1] << "], " << pos.theta << "("
         << RAD2DEG(pos.theta) << ")""[" << weight[2] << "])";
      return ss.str();
   }
};

static inline bool operator < (const Particle& p1, const Particle& p2) {
   return (p1.getWeight() < p2.getWeight());
}

static inline bool compareParticles(const Particle& p1, const Particle& p2) {
   return (p1.getWeight() > p2.getWeight());
}

/* returns true if the given field edge has both non-zero x & y
 * coeffieicnets; false otherwise */
static inline bool isValidEdge(Line line) {
   /* if both the x, y coefficents are 0 then this is not a line! */
   return (line.t1 != 0 || line.t2 != 0);
}

/* Returns the minimum heading between 2 angles given in radians */
/* TODO(yanjinz) refactorme! */
static inline float minHeadingDiff(float thetaA, float thetaB) {
   float minTheta = MIN_ANGLE_360(RAD2DEG(thetaA),
         RAD2DEG(thetaB));
   if (minTheta < -180) minTheta += 360;
   if (minTheta > 180) minTheta -= 360;
   return minTheta;
}



