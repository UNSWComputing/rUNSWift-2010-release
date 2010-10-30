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

/*
 * =====================================================================
 *
 *       Filename:  ParticlFilter.cpp
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  10/03/10 10:08:27
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Yanjin Zhu (), yanjinz@cse.unsw.edu.au
 *        Company:
 *
 * =====================================================================
 */

#include <stdlib.h>
#include "perception/localisation/PFilter.hpp"
#include "LocalisationUtils.hpp"

using namespace std;

#define BOUNDING_DIST 600

#define WEIGHT_LIMIT(numRobots, weight) (weight / (1.0 * numRobots))

/* keep it if no weights == 0 */
#define KEEP_PARTICLE(particle, weightLimit) \
   ( \
     (particle).weight[WEIGHT_X] > weightLimit \
     && (particle).weight[WEIGHT_Y] > weightLimit \
     && (particle).weight[WEIGHT_THETA] > weightLimit \
   )

#define RESAMPLE_DISCARD(seed) (rand_r(&seed) % 100 > 80)

void updateResampleDiscardWeights(Particle *particle) {
   for (int i = 0; i < NUM_WEIGHTS; i++) {
      if (particle->weight[i] == 0) {
         particle->weight[i] = 0.1;
      }
   }
}

void clipPos(AbsCoord *pos) {
   pos->x = MIN(MAX(pos->x, -FIELD_X_CLIP), FIELD_X_CLIP);
   pos->y = MIN(MAX(pos->y, -FIELD_Y_CLIP), FIELD_Y_CLIP);
}

float distToPoint(Point a, Point b) {
   return (sqrt(SQUARE(a.x - b.x) + SQUARE(a.y - b.y)));
}

float calcGaussian_x_mu(float x_mu, float sigma_sq) {
   float gaussian = -1;
   if (sigma_sq != 0) {
      gaussian = (1.0 / sqrt(2.0 * M_PI * sigma_sq)) *
         exp(-SQUARE(x_mu) / (2.0 * sigma_sq));
   }
   return gaussian;
}

float calcGaussian(float x, float mu, float sigma_sq) {
   return calcGaussian_x_mu(x - mu, sigma_sq);
}

/* returns the point of intersection given 2 lines requires this line to
 * be valid! */
Point getIntersectPF(Line lines[2]) {
   Point point;

   /* integer overflow when t3 is > 1 million, use int64_t instead */
   int64_t x =
      ((int64_t) lines[0].t2 * lines[1].t3 -
         (int64_t) lines[1].t2 * lines[0].t3) /
      ((int64_t) lines[1].t2 * lines[0].t1 -
       (int64_t) lines[1].t1 * lines[0].t2);

   int64_t y = 0;
   if (lines[0].t2 != 0) {
      y = (-lines[0].t1 * x / lines[0].t2) +
        (-lines[0].t3 / lines[0].t2);
   } else if (lines[1].t2 != 0) {
      y = (-lines[1].t1 * x / lines[1].t2) +
        (-lines[1].t3 / lines[1].t2);
   } else {
      llog(ERROR) << "Vision gave two non intersecting lines!" << endl;
   }

   point.x = x;
   point.y = y;

   return point;
}

EdgeInfo getEdgeInfo(Line line, const Edge edge) {
   EdgeInfo edgeInfo = {0};

   float dist;
   float heading = 0;
   float theta;
   float xIntercept;
   float yIntercept;

   float xDist = 0;
   float yDist = 0;

   /* work out the x & y intercepts of the line in RR coord system. work out
    * the straight line distance to the field edge given */
   dist = 0;
   if (line.t1 != 0) {
      xIntercept = -line.t3 / line.t1;
      if (line.t2 != 0) {
         yIntercept = -line.t3 / line.t2;
         /* XXX: check all the asin/acos stuff is not nan */
         heading = atan2(yIntercept, ABS(xIntercept));
         dist = xIntercept * sin(ABS(heading));
      } else {
         dist = xIntercept;
         heading = M_PI/2;
      }
   } else {
      if (line.t2 != 0) {
         yIntercept = -line.t3 / line.t2;
         dist = yIntercept;
         heading = M_PI;
      } else {
         llog(ERROR) << "line given is " << line.t1 << "x + " << line.t2 <<
            "y + " << line.t3 << "=0" << endl;
         llog(ERROR) << "No condition is met for the line given." <<
            "Heading is set to 0!!!" << endl;
      }
   }

   dist = ABS(dist);
   /* work out where the robot is facing and sign of distance to be used */
   if (edge.start.x == edge.end.x) {
      if (edge.start.x > 0) {
         // top edge
         if (heading >= 0) {
            theta = -M_PI / 2 + heading;
         } else {
            theta = M_PI / 2 + heading;
         }
         xDist = dist;
      } else {
         // bottom edge
         if (heading >= 0) {
            theta = M_PI / 2 + heading;
         } else {
            theta = -M_PI / 2 + heading;
         }
         xDist = -dist;
      }
   } else {
      if (edge.start.y > 0) {
         // left edge
         if (heading >= 0) {
            theta = heading;
         } else {
            theta = M_PI + heading;
         }
         yDist = dist;
      } else {
         // right edge
         if (heading >= 0) {
            theta = -M_PI + heading;
         } else {
            theta = heading;
         }
         yDist = -dist;
      }
   }

   edgeInfo.xDist = xDist;
   edgeInfo.yDist = yDist;
   edgeInfo.robotHeading = theta;

   return edgeInfo;
}

PFilter::PFilter() {
   /* Initialise variables */
   numRobotPos = 0;

   /* initially we use the particle filter */
   firstObs = true;

   seed = time(NULL);

   robotPos.resize(MAX_ROBOT_POS);

   resetBoundingBox();

   llog(INFO) << "PFilter Initialised" << endl;
}

PFilter::~PFilter() {
   llog(INFO) << "PFilter Destroyed" << endl;
}

void PFilter::resetBoundingBox() {
   topLeft.x = INT_MIN;
   topLeft.y = INT_MIN;

   btmRight.x = INT_MAX;
   btmRight.y = INT_MAX;
}

bool PFilter::updateParticleByTwoPosts() {
   getPosFromPosts();

   return (robotsPF.size() == 1);
}

bool PFilter::updateParticleByPost(Particle *robot, int post) {
   bool updated = false;

   int sign = ourPost ? 1 : -1;

   float postX = -sign*GOAL_POST_ABS_X_DIST;
   float postY[2] = {
      -sign*GOAL_POST_ABS_Y_DIST,
      sign*GOAL_POST_ABS_Y_DIST
   };

   Point postPt = Point(postX, postY[post]);

   llog(VERBOSE) << "robot " << robot->toString() << endl;

   /* Calculate the distance between the robot and the post seen */
   float distToPost = distToPoint(robot->point(), postPt);
   llog(VERBOSE) << "distToPost = " << distToPost << " obsPost dist = "
      << obsPosts[0].distance << endl;

   /* update weights for x & y */
   robot->weight[WEIGHT_X] *= calcGaussian(obsPosts[0].distance, distToPost,
         obsPosts[0].var[0]);
   robot->weight[WEIGHT_Y] = robot->weight[WEIGHT_X];

   /* find robot heading based on the obs */
   float obsHeading = NORMALISE(atan2(postY[post] - robot->pos.y,
            postX - robot->pos.x) - obsPosts[0].heading);

   /* update weights for theta */
   float headingDiff = minHeadingDiff(obsHeading, robot->pos.theta);

   /** If heading is within the threashhold then keep & update the
    * particle -- threashold is -PI ~ PI */
   robot->weight[WEIGHT_THETA] *= calcGaussian_x_mu(DEG2RAD(headingDiff),
         obsPosts[0].var[1]);

   updated = true;

   return updated;
}

bool PFilter::localise() {
   llog(VERBOSE) << "localisePF ..." << endl;

   /* figure out which team's posts we are seeing */
   if (obsWhichPosts != pNONE) {
      if (team_red) {
         if (obsWhichPosts >= pYELLOW_LEFT && obsWhichPosts <= pYELLOW_EITHER) {
            ourPost = true;
         } else {
            ourPost = false;
         }
      } else {
         if (obsWhichPosts >= pBLUE_LEFT && obsWhichPosts <= pBLUE_EITHER) {
            ourPost = true;
         } else {
            ourPost = false;
         }
      }
   }

   /* sanitise the field edges first */
   fieldEdgeSanity(obsNumEdges, obsEdges, obsWhichPosts, obsPosts);

   /* now start the filter */
   if (firstObs || numRobotPos == 0) {
      robotsPF.clear();
      /* swtiched from a KF or first time PF is called, generate a list
       * of particles
       */
      generateParticles();
      firstObs = false;
   } else {
      resetBoundingBox();

      /* filter the particles based on current observations */
      filterByFieldEdges();
      filterByGoalPosts();

      if (robotsPF.size() > 1) {
         llog(VERBOSE) << "Top left: " << topLeft.toString() << endl;
         llog(VERBOSE) << "Btm Right: " << btmRight.toString() << endl;

         llog(VERBOSE) << "dist: " << topLeft.distTo(btmRight) << endl;
         if (topLeft.distTo(btmRight) < BOUNDING_DIST) {
            llog(VERBOSE) << "Bounding box is small enough to collapse" << endl;
            robotsPF.resize(1);
         } else {
            normaliseWeights();
         }
      }
   }

   /* copy the particles into the robotPos array */
   robotPos.clear();
   float weightLimit = 0;
   if (robotsPF.size() > 0) {
      weightLimit = WEIGHT_LIMIT(robotsPF.size(), robotsPF[0].minWeight());
   }
   llog(VERBOSE) << "weightLimit = " << weightLimit <<
      " numRobots = " << robotsPF.size() << endl;
   vector<Particle>::iterator iter = robotsPF.begin();
   if (robotsPF.size() > 1) {
      while (iter != robotsPF.end()) {
         if (KEEP_PARTICLE((*iter), weightLimit)) {
            robotPos.push_back((*iter).getPos());
            ++iter;
         } else {
            llog(VERBOSE) << "PF: deleted " << (*iter).toString() << endl;
            iter = robotsPF.erase(iter);
         }
      }
   } else if (robotsPF.size() == 1) {
      robotPos.push_back(robotsPF[0].getPos());
   }

   numRobotPos = robotPos.size();
   llog(VERBOSE) << "localisePF " << numRobotPos << " robots" << endl;

   /* switch out of the PF */
   if (numRobotPos == 1) {
      firstObs = true;
   }
   return true;
}

void PFilter::normaliseWeights() {
   float weights[NUM_WEIGHTS] = {0.0};
   vector<Particle>::iterator iter = robotsPF.begin();
   /* sum the weights */
   while (iter != robotsPF.end()) {
      for (int i = 0; i < NUM_WEIGHTS; i++) {
         weights[i] += (*iter).weight[i];
      }
      ++iter;
   }

   llog(VERBOSE) << "the sum of weights are " << weights[WEIGHT_X] << ", " <<
      weights[WEIGHT_Y] << ", " << weights[WEIGHT_THETA] << endl;

   /* normalise */
   iter = robotsPF.begin();
   while (iter != robotsPF.end()) {
      for (int i = 0; i < NUM_WEIGHTS; i++) {
         (*iter).weight[i] /= weights[i];
      }
      ++iter;
   }

   llog(VERBOSE) << "done renormalising" << endl;
}

void PFilter::generateParticles() {
   llog(VERBOSE) << "generating particles ..." << endl;

   // generateFromPosts();

   if ((obsWhichPosts == pBLUE_BOTH || obsWhichPosts == pYELLOW_BOTH)) {
      generateFromPosts();
      llog(VERBOSE) << robotsPF.size() << " from posts" << endl;
   } else if (obsNumEdges > 1) {
      generateFromFieldEdges();
      filterByGoalPosts();
   } else if (obsWhichPosts != pNONE) {
      generateFromPosts();
      llog(VERBOSE) << robotsPF.size() << " from posts" << endl;
      /* we have a position already from seeing two posts, skip the edge
       * part */
      filterByFieldEdges();
      llog(VERBOSE) << robotsPF.size() << " left after filtering by edges"
         << endl;
   } else if (obsNumEdges > 0) {
      generateFromFieldEdges();
      llog(VERBOSE) << robotsPF.size() << " from edges" << endl;
   }

   llog(VERBOSE) << "generated " << robotsPF.size() << " particles" << endl;
}

void PFilter::getPosFromPosts() {
   AbsCoord obs;
   if (posFromTwoPosts(obsPosts, ourPost,
            obs.x, obs.y, obs.theta,
            obs.var[X], obs.var[Y], obs.var[THETA])) {
      robotsPF.clear();
      robotsPF.push_back(Particle(obs));
   }
}

void PFilter::generateFromPosts() {
   AbsCoord obs;
   float angularStep;
   float enclosingAngle;
   int cartesianX, cartesianY;
   int i;

   if (obsWhichPosts == pNONE) return;

   if (obsWhichPosts == pBLUE_BOTH || obsWhichPosts == pYELLOW_BOTH) {
      getPosFromPosts();
   // one post case
   } else if (!isnan(obsPosts[0].distance) && !isnan(obsPosts[0].heading)) {
      int numPosts = 1;
      int refPost = LEFT;
      if (obsWhichPosts == pBLUE_EITHER || obsWhichPosts == pYELLOW_EITHER) {
         numPosts = 2;
      } else if (obsWhichPosts == pBLUE_RIGHT ||
                 obsWhichPosts == pYELLOW_RIGHT) {
            refPost = RIGHT;
      }
      RRCoord rrPost = obsPosts[0];
      for (int j = 0; j < numPosts; j++) {
         /* scan around the circle center at the post,
          * RADIANCE_STEP at a time,
          * starting from 0 rad on the cartesian plane
          */
         if (rrPost.distance != 0 && IS_VALID_DIST(rrPost.distance)) {
            for (i = 0; i < NUM_SEGMENTS; i++) {
               /* caculate the current step size */
               angularStep = RADIANCE_SETP * i;

               /* get the cartesian distance from post, assuming its (0, 0) */
               cartesianY = rrPost.distance * sin(angularStep);
               cartesianX = rrPost.distance * cos(angularStep);

               /* work out the enclosing angle the post makes to our coordinate
                * system
                */
               enclosingAngle =
                  NORMALISE(atan2f(cartesianX, cartesianY) - M_PI);

               /* convert from cartesian (x, y) to our coordinate system */
               obs.x = posts_abs[ourPost][refPost].x + cartesianY;
               obs.y = posts_abs[ourPost][refPost].y + cartesianX;

               obs.theta = NORMALISE(enclosingAngle - rrPost.heading);

               /* clip the robot pos to the field */
               obs.x = MIN(MAX(obs.x, -FIELD_X_CLIP), FIELD_X_CLIP);
               obs.y = MIN(MAX(obs.y, -FIELD_Y_CLIP), FIELD_Y_CLIP);

               robotsPF.push_back(Particle(obs));
            }
         }
         refPost++;
      }
   }
}

/* Unless specified otherwise, this function uses our coordinate system */
void PFilter::generateFromEdge(Edge edge, Line line) {
   AbsCoord obs;
   Point from;
   Point step;

   /* work out direction of step */
   if (edge.start.x == edge.end.x) {
      step.y = LINE_STEP;
   } else {
      step.x = LINE_STEP;
   }

   EdgeInfo edgeInfo = getEdgeInfo(line, edge);

   /* step thru the points on the line to find the best one or
    * add them all
    */
   from = edge.start;
   while (from.x >= edge.end.x && from.y >= edge.end.y) {
      float x = from.x - edgeInfo.xDist;
      float y = from.y - edgeInfo.yDist;

      obs.theta = edgeInfo.robotHeading;

      /* clip the robot pos to the field */
      obs.x = MIN(MAX(x, -FIELD_X_CLIP), FIELD_X_CLIP);
      obs.y = MIN(MAX(y, -FIELD_Y_CLIP), FIELD_Y_CLIP);

      robotsPF.push_back(Particle(obs));

      from.x -= step.x;
      from.y -= step.y;
   }
}

/* Localise off two intersecting field edges */
void PFilter::generateFromTwoEdges(Edge edges[], Line lines[],
      Point corner) {
   EdgeInfo edgeInfoA = getEdgeInfo(lines[0], edges[0]);
   EdgeInfo edgeInfoB = getEdgeInfo(lines[1], edges[1]);
   AbsCoord obs;

   obs.x = corner.x - edgeInfoA.xDist - edgeInfoB.xDist;
   obs.y = corner.y - edgeInfoA.yDist - edgeInfoB.yDist;
   obs.theta = edgeInfoA.robotHeading;

   /* clip the robot pos to the field */
   obs.x = MIN(MAX(obs.x, -FIELD_X_CLIP), FIELD_X_CLIP);
   obs.y = MIN(MAX(obs.y, -FIELD_Y_CLIP), FIELD_Y_CLIP);

   robotsPF.push_back(Particle(obs));
}


void PFilter::generateFromFieldEdges() {
   /* no edges seen or the edges are invalid */
   if (obsNumEdges <= 0 || !isValidEdge(obsEdges[0])
         || (obsNumEdges > 1 && !isValidEdge(obsEdges[1]))) {
      return;
   }

   /* take each field edge as a possible line seen */
   if (obsNumEdges == 1) {
      for (int i = 0; i < E_COUNT; i++) {
         generateFromEdge(edges_abs[i], obsEdges[0]);
      }
   } else if (obsNumEdges == 2) {
      Edge edges[MAX_OBS_EDGES];
      Point ptIntersect = getIntersectPF(obsEdges);

      llog(VERBOSE) << "point of intersection " << ptIntersect.x << " "
         << ptIntersect.y << endl;
      /* from the position of the intersection we know the orientation
       * of the edges we've seen */
      /* TODO: find which edge is the one going along 0 or 1 */
      if (ptIntersect.y >= 0) {
         edges[0] = edges_abs[E_TOP];
         edges[1] = edges_abs[E_LEFT];
         generateFromTwoEdges(edges, obsEdges, Point(TOP_LEFT_FIELD_X,
                  TOP_LEFT_FIELD_Y));

         edges[0] = edges_abs[E_LEFT];
         edges[1] = edges_abs[E_BOTTOM];
         generateFromTwoEdges(edges, obsEdges, Point(BOTTOM_LEFT_FIELD_X,
                  BOTTOM_LEFT_FIELD_Y));

         edges[0] = edges_abs[E_BOTTOM];
         edges[1] = edges_abs[E_RIGHT];
         generateFromTwoEdges(edges, obsEdges, Point(BOTTOM_RIGHT_FIELD_X,
                  BOTTOM_RIGHT_FIELD_Y));

         edges[0] = edges_abs[E_RIGHT];
         edges[1] = edges_abs[E_TOP];
         generateFromTwoEdges(edges, obsEdges, Point(TOP_RIGHT_FIELD_X,
                  TOP_RIGHT_FIELD_Y));
      } else {
         llog(VERBOSE) << "using clock-wise orientation" << endl;
         edges[0] = edges_abs[E_LEFT];
         edges[1] = edges_abs[E_TOP];
         generateFromTwoEdges(edges, obsEdges, Point(TOP_LEFT_FIELD_X,
                  TOP_LEFT_FIELD_Y));

         edges[0] = edges_abs[E_BOTTOM];
         edges[1] = edges_abs[E_LEFT];
         generateFromTwoEdges(edges, obsEdges, Point(BOTTOM_LEFT_FIELD_X,
                  BOTTOM_LEFT_FIELD_Y));

         edges[0] = edges_abs[E_RIGHT];
         edges[1] = edges_abs[E_BOTTOM];
         generateFromTwoEdges(edges, obsEdges, Point(BOTTOM_RIGHT_FIELD_X,
                  BOTTOM_RIGHT_FIELD_Y));

         edges[0] = edges_abs[E_TOP];
         edges[1] = edges_abs[E_RIGHT];
         generateFromTwoEdges(edges, obsEdges, Point(TOP_RIGHT_FIELD_X,
                  TOP_RIGHT_FIELD_Y));
      }
   }
}

void PFilter::filterByFieldEdges() {
   if (obsNumEdges == 0) return;

   AbsCoord obs;

   vector<Particle>::iterator iter = robotsPF.begin();
   while (iter != robotsPF.end()) {
      bool updated = false;
      for (int i = 0; i < obsNumEdges; ++i) {
         float obs_state[STATE_VEC_DIM] = {0.0f};
         float obs_var[STATE_VEC_DIM] = {0.0f};
         bool obs_mask[STATE_VEC_DIM] = {false};
         float d;

         float state_vec[STATE_VEC_DIM] = {
            (*iter).pos.x, (*iter).pos.y, (*iter).pos.theta
         };
         float distToLine;
         posFromFieldEdge(obsEdges[i], state_vec,
               obs_state, obs_var, obs_mask, distToLine, d);
         updated = (obs_mask[X] || obs_mask[Y]);
         if (!updated) {
            break;
         } else {
            /* update the Gaussian */
            (*iter).weight[WEIGHT_X] *=
               calcGaussian(d, distToLine, obs_var[X]);
            (*iter).weight[WEIGHT_Y] = (*iter).weight[WEIGHT_X];
            (*iter).weight[WEIGHT_THETA] *=
               calcGaussian_x_mu(DEG2RAD(minHeadingDiff(obs_state[THETA],
                           (*iter).pos.theta)), obs_var[THETA]);
         }
      }
      /* keep the particle */
      if (!updated) {
         llog(VERBOSE) << "edges: deleted robot pos " << (*iter).toString()
            << " ";
         iter = robotsPF.erase(iter);
         llog(VERBOSE) << robotsPF.size() << " left " << endl;
      } else {
         updateBoundingBox(*iter);

         ++iter;
      }
   }

   /* sort the particles in decending order of weights */
   sort(robotsPF.begin(), robotsPF.end(), compareParticles);
}

void PFilter::updateBoundingBox(Particle& particle) {
   if (particle.pos.x > topLeft.x && particle.pos.y > topLeft.y) {
      topLeft.x = particle.pos.x;
      topLeft.y = particle.pos.y;
   } else if (particle.pos.x < btmRight.x && particle.pos.y < btmRight.y) {
      btmRight.x = particle.pos.x;
      btmRight.y = particle.pos.y;
   }
}

void PFilter::filterByFieldLines() {
}

void PFilter::filterByGoalPosts() {
   /* no posts seen -- exit */
   if (obsWhichPosts == pNONE) return;

   Particle robot;
   bool updated = false;

   if (obsWhichPosts == pBLUE_BOTH || obsWhichPosts == pYELLOW_BOTH) {
      if (!updateParticleByTwoPosts()) {
         llog(ERROR) << "The posts from vision doesn't provide"
            " a valid position for the robot!" << endl;
      }
   } else {
      /* filter each particle by the post(s) seen */
      vector<Particle>::iterator iter = robotsPF.begin();
      while (iter != robotsPF.end()) {
         if (obsWhichPosts == pBLUE_LEFT || obsWhichPosts == pYELLOW_LEFT) {
            updated = updateParticleByPost(&(*iter), LEFT);
         } else if (obsWhichPosts == pBLUE_RIGHT ||
                    obsWhichPosts == pYELLOW_RIGHT) {
            updated = updateParticleByPost(&(*iter), RIGHT);
         } else if (obsWhichPosts == pBLUE_EITHER ||
                    obsWhichPosts == pYELLOW_EITHER) {
            /* picks the best weighting out of the two if saw either */
            Particle leftPos = *iter;
            Particle rightPos = *iter;

            bool updateLeft = false;
            bool updateRight = false;

            updateLeft = updateParticleByPost(&leftPos, LEFT);
            updateRight = updateParticleByPost(&rightPos, RIGHT);

            updated = (updateLeft || updateRight);

            if (updateLeft && updateRight) {
               bool leftBetter = (leftPos.getWeight() > rightPos.getWeight());
               (*iter) = leftBetter ? leftPos : rightPos;
            } else if (updateLeft) {
               (*iter) = leftPos;
            } else if (updateRight) {
               (*iter) = rightPos;
            }
         }
         /* if updated then keep it, otherwise remove this element from the
          * list
          */
         if (!updated) {
            llog(VERBOSE) << "posts: deleted robot pos " << (*iter).toString()
               << " ";
            iter = robotsPF.erase(iter);
            llog(VERBOSE) << robotsPF.size() << " left " << endl;
         } else {
            updateBoundingBox(*iter);

            ++iter;
         }
      }

      /* sort the particles in decending order of weights */
      sort(robotsPF.begin(), robotsPF.end(), compareParticles);
   }
}
