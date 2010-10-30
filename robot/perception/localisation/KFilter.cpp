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

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include "KFilter.hpp"
#include "utils/angles.hpp"
#include "LocalisationDefs.hpp"
#include "LocalisationUtils.hpp"

using namespace std;

KFilter::KFilter() {
   ranLastCycle = false;
}

void KFilter::getStateEstimate() {
   stateFromAbsCoord(robotPos[0], state_vec, var_vec);
}

void KFilter::stateFromAbsCoord(const AbsCoord &pos, float *state, float *var) {
   state[X] = pos.x;
   state[Y] = pos.y;
   state[THETA] = pos.theta;
   var[X] = pos.var[0];
   var[Y] = pos.var[1];
   var[THETA] = pos.var[2];
}

AbsCoord KFilter::absCoordFromStateEstimate(float state[],
                                            float var[],
                                            bool mask[]) {
   AbsCoord pos;
   if (mask[X]) {
      pos.x = state[X];
      pos.var[X] = var[X];
   } else {
      pos.x = state_vec[X];
      pos.var[X] = var_vec[X];
   }
   if (mask[Y]) {
      pos.y = state[Y];
      pos.var[Y] = var[Y];
   } else {
      pos.y = state_vec[Y];
      pos.var[Y] = var_vec[Y];
   }
   if (mask[THETA]) {
      pos.theta = state[THETA];
      pos.var[THETA] = var[THETA];
   } else {
      pos.theta = state_vec[THETA];
      pos.var[THETA] = var_vec[THETA];
   }
   return pos;
}

bool KFilter::localise() {
   robotPos.clear();
   if (!ranLastCycle) {
      llog(DEBUG1) << "fetching state from PF" << endl;
      getStateEstimate();
      kidnapFactor = 0.01;
   }
   processUpdate();
   if (acActive.body.actionType == ActionCommand::Body::REF_PICKUP) {
      var_vec[X] = SQUARE(FULL_FIELD_LENGTH);
      var_vec[Y] = SQUARE(FULL_FIELD_WIDTH);
      var_vec[THETA] = SQUARE(M_PI);
      kidnapFactor = 10.0;
   }
   if (acActive.body.actionType != ActionCommand::Body::REF_PICKUP &&
       acActive.body.actionType != ActionCommand::Body::DEAD &&
       acActive.body.actionType != ActionCommand::Body::GETUP_FRONT &&
       acActive.body.actionType != ActionCommand::Body::GETUP_BACK) {
      twoPostUpdate();
      fieldEdgeSanity(obsNumEdges, obsEdges, obsWhichPosts, obsPosts);
      postEdgeUpdate();
      fieldEdgeUpdate();
      fieldLineUpdate();
      onePostUpdate();
   }
   prev_odometry = odometry;
   bool mask[3] = {true, true, true};

   robotPos.insert(robotPos.begin(),
         absCoordFromStateEstimate(state_vec, var_vec, mask));
   numRobotPos = robotPos.size();
   ranLastCycle = true;
   return true;
}

void KFilter::processUpdate() {
   if (ranLastCycle) {
      Odometry deltaOdometry = odometry - prev_odometry;
      float deltaF = deltaOdometry.forward;
      float deltaS = deltaOdometry.left;
      float deltaT = deltaOdometry.turn;
      state_vec[X] = state_vec[X] +
         deltaF*cos(state_vec[THETA]) - deltaS*sin(state_vec[THETA]);
      var_vec[X] = SQUARE(sqrt(var_vec[X]) +
            (ABS(deltaF) + ABS(deltaS))/4.0 + 0.3);

      state_vec[Y] = state_vec[Y] +
         deltaF*sin(state_vec[THETA]) + deltaS*cos(state_vec[THETA]);
      var_vec[Y] = SQUARE(sqrt(var_vec[Y]) +
            (ABS(deltaF) + ABS(deltaS))/4.0 + 0.3);

      state_vec[THETA] = NORMALISE(state_vec[THETA] + deltaT);
      var_vec[THETA] = SQUARE(sqrt(var_vec[THETA]) + (ABS(deltaT)/4.0 + 0.001));
   }
}

void KFilter::obsUpdate(float *obs_state, float *obs_var, bool *obs_mask) {
   bool outlier = false;
   for (int i = 0; i < STATE_VEC_DIM; ++i) {
      if (obs_mask[i] && !isnan(obs_var[i]) && !isnan(obs_state[i])) {
         float observationError;
         if (i == THETA) {
            observationError = sqrt(SQUARE(NORMALISE(state_vec[i]
                     - obs_state[i])) / obs_var[i]);
         } else {
            observationError = sqrt(SQUARE(state_vec[i] - obs_state[i])
                  / obs_var[i]);
         }
         if (observationError > 2.0) {
            outlier = true;
         }
         kidnapFactor = kidnapFactor*0.9 + observationError*0.1;
      }
   }
   if (!outlier) {
      robotPos.push_back(absCoordFromStateEstimate(
               obs_state, obs_var, obs_mask));
      for (int i = 0; i < STATE_VEC_DIM; ++i) {
         if (obs_mask[i] && !isnan(obs_var[i]) && !isnan(obs_state[i])) {
            float k = 10000.0 / (10000.0 + obs_var[i]);
            if (i == THETA) {
               state_vec[i] = NORMALISE(state_vec[i] +
                     k*(NORMALISE(obs_state[i] - state_vec[i])));
            } else {
               state_vec[i] = state_vec[i] + k*(obs_state[i] - state_vec[i]);
            }
         }
      }
   }
}

void KFilter::geoUpdate(float *obs_state, float *obs_var, bool *obs_mask) {
   robotPos.push_back(absCoordFromStateEstimate(
            obs_state, obs_var, obs_mask));
   for (int i = 0; i < STATE_VEC_DIM; ++i) {
      if (obs_mask[i] && !isnan(obs_var[i]) && !isnan(obs_state[i])) {
         float k = var_vec[i] / (var_vec[i] + obs_var[i]);
         if (i == THETA) {
            state_vec[i] = NORMALISE(state_vec[i] +
                  k*(NORMALISE(obs_state[i] - state_vec[i])));
            var_vec[i] = (1 - k) * var_vec[i];
         } else {
            state_vec[i] = state_vec[i] + k*(obs_state[i] - state_vec[i]);
            var_vec[i] = (1 - k) * var_vec[i];
         }
      }
   }
}

void KFilter::fieldEdgeUpdate() {
   float obs_state[STATE_VEC_DIM] = {0.0f};
   float obs_var[STATE_VEC_DIM] = {0.0f};
   bool obs_mask[STATE_VEC_DIM] = {false};
   for (int i = 0; i < obsNumEdges; ++i) {
      float ignoreme1;  // ignore this variable
      float ignoreme2;  // ignore this variable
      posFromFieldEdge(obsEdges[i], state_vec, obs_state, obs_var, obs_mask,
            ignoreme1, ignoreme2);
      obsUpdate(obs_state, obs_var, obs_mask);
   }
}

void KFilter::fieldLineUpdate() {
   bool newData = false;
   bool mask[3] = {true, true, true};
   AbsCoord obs = fieldLineLocalisation->localiseFromFieldLines(
         absCoordFromStateEstimate(state_vec, var_vec, mask), newData);
   if (newData) {
      float obs_state[STATE_VEC_DIM] = {0.0f};
      float obs_var[STATE_VEC_DIM] = {0.0f};
      bool obs_mask[STATE_VEC_DIM] = {true, true, true};
      stateFromAbsCoord(obs, obs_state, obs_var);
      obs_var[X] = SQUARE(sqrt(obs_var[X]) + STD_DEV_XY_CONST);
      obs_var[Y] = SQUARE(sqrt(obs_var[Y]) + STD_DEV_XY_CONST);
      obs_var[THETA] = SQUARE(sqrt(obs_var[THETA]) + STD_DEV_T_CONST);
      obs_state[X] = MIN(MAX(obs_state[X], -FIELD_X_CLIP), FIELD_X_CLIP);
      obs_state[Y] = MIN(MAX(obs_state[Y], -FIELD_Y_CLIP), FIELD_Y_CLIP);
      obsUpdate(obs_state, obs_var, obs_mask);
   }
}

void KFilter::onePostUpdate() {
   if (obsWhichPosts != pNONE &&
         obsWhichPosts != pYELLOW_BOTH && obsWhichPosts != pBLUE_BOTH) {
      float obs_state[STATE_VEC_DIM] = {0.0f};
      float obs_var[STATE_VEC_DIM] = {0.0f};
      bool obs_mask[STATE_VEC_DIM] = {false};
      bool ours = ((obsWhichPosts <= pBLUE_EITHER) ^ team_red);
      int sign = (ours ? 1 : -1);
      float postX = -sign*GOAL_POST_ABS_X_DIST;
      float leftPostY = -sign*GOAL_POST_ABS_Y_DIST;
      float rightPostY = sign*GOAL_POST_ABS_Y_DIST;
      if (obsWhichPosts != pBLUE_BOTH && obsWhichPosts != pYELLOW_BOTH
            && !isnan(obsPosts[0].distance && !isnan(obsPosts[0].heading))) {
         bool left;
         if (obsWhichPosts == pBLUE_EITHER || obsWhichPosts == pYELLOW_EITHER) {
            // determine if it's the left or right post
            float k = 2000000.0f;  // post heading weight factor
            float psi, postDist;
            psi = atan2f(leftPostY - state_vec[Y], postX - state_vec[X]);
            postDist = sqrt(SQUARE(state_vec[X] - postX)
                          + SQUARE(state_vec[Y] - leftPostY));
            float n2left = sqrt(k*SQUARE(NORMALISE(psi - state_vec[THETA]
                                                       - obsPosts[0].heading))
                                + SQUARE(obsPosts[0].distance - postDist));

            psi = atan2f(rightPostY - state_vec[Y], postX - state_vec[X]);
            postDist = sqrt(SQUARE(state_vec[X] - postX)
                          + SQUARE(state_vec[Y] - rightPostY));
            float n2right = sqrt(k*SQUARE(NORMALISE(psi - state_vec[THETA]
                                                       - obsPosts[0].heading))
                                + SQUARE(obsPosts[0].distance - postDist));
            if (n2left < n2right) {
              left = true;
            } else {
              left = false;
            }
         } else {
            if (obsWhichPosts == pBLUE_LEFT || obsWhichPosts == pYELLOW_LEFT) {
               left = true;
            } else {
               left = false;
            }
         }
         // determine estimated robot position
         float postY = (left ? leftPostY : rightPostY);
         if (postX - state_vec[X] != 0.0f) {
            int add = ((state_vec[X] > postX) ? 1: -1);
            float m = (postY - state_vec[Y]) / (postX - state_vec[X]);
            obs_state[X] = postX + add*(sqrt(SQUARE(obsPosts[0].distance) /
                     (1 + SQUARE(m))));
            obs_state[Y] = postY + m*(obs_state[X] - postX);
            obs_state[THETA] = NORMALISE(atan2f(postY - state_vec[Y],
                     postX - state_vec[X])
                  - obsPosts[0].heading);
            obs_var[X] = SQUARE(sqrt(obsPosts[0].var[0]) +
                  STD_DEV_XY_CONST);
            obs_var[Y] = obs_var[X];
            obs_var[THETA] = SQUARE(sqrt(obsPosts[0].var[1])
                  + STD_DEV_T_CONST);
            obs_mask[X] = true;
            obs_mask[Y] = true;
            obs_mask[THETA] = true;
         }
      }
      obs_state[X] = MIN(MAX(obs_state[X], -FIELD_X_CLIP), FIELD_X_CLIP);
      obs_state[Y] = MIN(MAX(obs_state[Y], -FIELD_Y_CLIP), FIELD_Y_CLIP);
      obsUpdate(obs_state, obs_var, obs_mask);
   }
}

//
void KFilter::twoPostUpdate() {
   if (obsWhichPosts == pBLUE_BOTH || obsWhichPosts == pYELLOW_BOTH) {
      float obs_state[STATE_VEC_DIM] = {0.0f};
      float obs_var[STATE_VEC_DIM] = {0.0f};
      bool obs_mask[STATE_VEC_DIM] = {true, true, true};
      bool ours = ((obsWhichPosts <= pBLUE_EITHER) ^ team_red);
      if ((obsPosts[0].distance + obsPosts[1].distance)/2.0f < 2500.0f) {
         // new two post update using intersection of closest-post circle and
         // subtended circle
         int sign = (ours ? 1 : -1);
         float postX = -sign*GOAL_POST_ABS_X_DIST;
         float leftPostY = -sign*GOAL_POST_ABS_Y_DIST;
         float rightPostY = sign*GOAL_POST_ABS_Y_DIST;
         if (isnan(obsPosts[0].distance) || isnan(obsPosts[0].heading) ||
               isnan(obsPosts[1].distance) || isnan(obsPosts[1].heading)) {
            llog(ERROR) << "Vision returned a post with NaN data" << endl;
         } else {
            float subangle = ABS(NORMALISE(obsPosts[0].heading
                     - obsPosts[1].heading));
            float subRadius = GOAL_WIDTH/(2*sin(subangle));
            float xd     = GOAL_WIDTH/(2.0f*tan(subangle));
            float xc     = postX + sign*xd;  // xc is center of subtended circle
            int closestPost = obsPosts[0].distance > obsPosts[1].distance;
            float postRadius = obsPosts[closestPost].distance;
            float closestPostY = closestPost? rightPostY:leftPostY;
            float furtherPostY = closestPost? leftPostY:rightPostY;
            // refer Intersection of Two Circles by Paul Bourke 1997
            float d = hypot(xc-postX, closestPostY);
            float a = (SQUARE(postRadius)-SQUARE(subRadius)+SQUARE(d))/
                      (2.0f*d);
            float h = sqrt(SQUARE(postRadius)-SQUARE(a));
            if (isnanf(h)) {
               a = d + subRadius;
               h = 0.0f;
            }
            float x2 = postX + a/d*(xc-postX);
            float y2 = closestPostY + a/d*(0.0f-closestPostY);
            obs_state[X] = x2 + h*(0.0f-closestPostY)/d;
            obs_state[Y] = y2 - h*(xc-postX)/d;
            float dist = hypot(obs_state[X]-postX, obs_state[Y]-furtherPostY);
            if (ABS(obs_state[X])>FIELD_LENGTH/2.0f || dist < postRadius) {
               obs_state[X] = x2 - h/(0.0f-closestPostY)/d;
               obs_state[Y] = y2 + h*(xc-postX)/d;
            }
            obs_state[X] = MIN(MAX(obs_state[X], -FIELD_X_CLIP), FIELD_X_CLIP);
            obs_state[Y] = MIN(MAX(obs_state[Y], -FIELD_Y_CLIP), FIELD_Y_CLIP);
            obs_state[THETA] = NORMALISE(atan2(leftPostY - obs_state[Y],
                     postX - obs_state[X])
                  - obsPosts[0].heading);
            obs_var[X] = SQUARE(sqrt((obsPosts[0].var[0] +
                        obsPosts[1].var[0])/2.0) + STD_DEV_XY_CONST);
            obs_var[Y] = obs_var[X];
            obs_var[THETA] = SQUARE(sqrt(obsPosts[0].var[1]) + STD_DEV_T_CONST);
            geoUpdate(obs_state, obs_var, obs_mask);
         }
      } else {
         // old two-post update using position est. from distances and
         // correction by projecting line from goal center to subtended circ.
         if (posFromTwoPosts(obsPosts, ours,
                  obs_state[X], obs_state[Y], obs_state[THETA],
                  obs_var[X], obs_var[Y], obs_var[THETA])) {
            // use subtended angle to improve dist
            int sign = (ours ? 1 : -1);
            float postX = -sign*GOAL_POST_ABS_X_DIST;
            float subangle = ABS(NORMALISE(obsPosts[0].heading
                     - obsPosts[1].heading));
            float xcenter = postX + sign*(GOAL_WIDTH)/(2*tan(subangle));
            float radius = (GOAL_WIDTH)/(2*sin(subangle));
            float m = obs_state[Y]/(obs_state[X]-postX);
            float a = SQUARE(m) + 1;
            float b = -2*SQUARE(m)*postX - 2*xcenter;
            float c = SQUARE(m)*SQUARE(postX) +
               SQUARE(xcenter) - SQUARE(radius);
            obs_state[X] = (-b + sign*sqrt(SQUARE(b) - 4*a*c))/(2*a);
            obs_state[Y] = m*(obs_state[X] - postX);
            if ((GOAL_WIDTH/(2*tan(subangle)) + radius) > 4700) {
               obs_mask[Y] = false;
            }
            obs_state[X] = MIN(MAX(obs_state[X], -FIELD_X_CLIP), FIELD_X_CLIP);
            obs_state[Y] = MIN(MAX(obs_state[Y], -FIELD_Y_CLIP), FIELD_Y_CLIP);
            geoUpdate(obs_state, obs_var, obs_mask);
         }
      }
   }
}

void KFilter::postEdgeUpdate() {
   bool obs_mask[STATE_VEC_DIM] = {true, true, true};
   bool ours = ((obsWhichPosts <= pBLUE_EITHER) ^ team_red);
   int sign = (ours ? 1 : -1);
   if (obsWhichPosts != pBLUE_BOTH && obsWhichPosts != pYELLOW_BOTH
         && obsWhichPosts != pNONE
         && (obsPosts[0].distance > 1700 || !ours)) {
      vector<AbsCoord> hypotheses;
      float obs_state[STATE_VEC_DIM] = {0.0f};
      float obs_var[STATE_VEC_DIM] = { SQUARE(STD_DEV_XY_CONST),
         SQUARE(STD_DEV_XY_CONST),
         SQUARE(DEG2RAD(15.0)) };
      float maxAngleDifference = DEG2RAD(15.0);
      for (int j = 0; j < obsNumEdges; j++) {
         float postX = -sign*GOAL_POST_ABS_X_DIST;
         float leftPostY = -sign*GOAL_POST_ABS_Y_DIST;
         float rightPostY = sign*GOAL_POST_ABS_Y_DIST;
         float a = atan2f(((float)-obsEdges[j].t1)*((float)obsEdges[j].t3),
               ((float)obsEdges[j].t2)*((float)obsEdges[j].t3));
         float d = ABS(obsEdges[j].t3)/sqrt(
               SQUARE(obsEdges[j].t1)+SQUARE(obsEdges[j].t2));

         bool either = (obsWhichPosts == pBLUE_EITHER ||
               obsWhichPosts == pYELLOW_EITHER);
         bool left = (obsWhichPosts == pBLUE_LEFT ||
               obsWhichPosts == pYELLOW_LEFT);

         // looking at front field edge
         {
            obs_state[X] = (FULL_FIELD_LENGTH/2) - d;
            obs_state[THETA] = NORMALISE(M_PI_2 - a);
            float y[2];
            float t[2];
            float yoffset  = SQUARE(obsPosts[0].distance)
               - SQUARE(obs_state[X] - postX);

            if (yoffset >= 0) {
               if (left || either) {
                  y[0] = leftPostY + sqrt(yoffset);
                  t[0] = NORMALISE(atan2f(leftPostY - y[0],
                           postX - obs_state[X]) - obs_state[THETA]);
                  y[1] = leftPostY - sqrt(yoffset);
                  t[1] = NORMALISE(atan2f(leftPostY - y[1],
                           postX - obs_state[X]) - obs_state[THETA]);
                  for (int i = 0; i < 2; ++i) {
                     if (ABS(NORMALISE(t[i] - obsPosts[0].heading))
                           < maxAngleDifference) {
                        obs_state[Y] = y[i];
                        hypotheses.push_back(absCoordFromStateEstimate(
                                 obs_state, obs_var, obs_mask));
                     }
                  }
               }
               if (!left || either) {
                  y[0] = rightPostY - sqrt(yoffset);
                  t[0] = NORMALISE(atan2f(rightPostY - y[0],
                           postX - obs_state[X]) - obs_state[THETA]);
                  y[1] = rightPostY + sqrt(yoffset);
                  t[1] = NORMALISE(atan2f(rightPostY - y[1],
                           postX - obs_state[X]) - obs_state[THETA]);
                  for (int i = 0; i < 2; ++i) {
                     if (ABS(NORMALISE(t[i] - obsPosts[0].heading))
                           < maxAngleDifference) {
                        obs_state[Y] = y[i];
                        hypotheses.push_back(absCoordFromStateEstimate(
                                 obs_state, obs_var, obs_mask));
                     }
                  }
               }
            }
         }

         // looking at back field edge
         {
            obs_state[X] = d - (FULL_FIELD_LENGTH/2);
            obs_state[THETA] = NORMALISE(-M_PI_2 - a);
            float y[2];
            float t[2];
            float yoffset  = SQUARE(obsPosts[0].distance)
               - SQUARE(obs_state[X] - postX);
            if (yoffset >= 0) {
               if (left || either) {
                  y[0] = leftPostY + sqrt(yoffset);
                  t[0] = NORMALISE(atan2f(leftPostY - y[0],
                           postX - obs_state[X]) - obs_state[THETA]);
                  y[1] = leftPostY - sqrt(yoffset);
                  t[1] = NORMALISE(atan2f(leftPostY - y[1],
                           postX - obs_state[X]) - obs_state[THETA]);
                  for (int i = 0; i < 2; ++i) {
                     if (ABS(NORMALISE(t[i] - obsPosts[0].heading))
                           < maxAngleDifference) {
                        obs_state[Y] = y[i];
                        hypotheses.push_back(absCoordFromStateEstimate(
                                 obs_state, obs_var, obs_mask));
                     }
                  }
               }
               if (!left || either) {
                  y[0] = rightPostY - sqrt(yoffset);
                  t[0] = NORMALISE(atan2f(rightPostY - y[0],
                           postX - obs_state[X]) - obs_state[THETA]);
                  y[1] = rightPostY + sqrt(yoffset);
                  t[1] = NORMALISE(atan2f(rightPostY - y[1],
                           postX - obs_state[X]) - obs_state[THETA]);
                  for (int i = 0; i < 2; ++i) {
                     if (ABS(NORMALISE(t[i] - obsPosts[0].heading))
                           < maxAngleDifference) {
                        obs_state[Y] = y[i];
                        hypotheses.push_back(absCoordFromStateEstimate(
                                 obs_state, obs_var, obs_mask));
                     }
                  }
               }
            }
         }

         // looking at left field edge
         {
            obs_state[Y] = (FULL_FIELD_WIDTH/2) - d;
            obs_state[THETA] = NORMALISE(M_PI - a);
            float x;
            float t;
            // left || either ?
            if (obsWhichPosts != pBLUE_RIGHT &&
                  obsWhichPosts != pYELLOW_RIGHT) {
               float xoffset = SQUARE(obsPosts[0].distance)
                  - SQUARE(obs_state[Y] - leftPostY);
               if (xoffset >= 0) {
                  x = postX + sign*sqrt(xoffset);
                  t = NORMALISE(atan2f(leftPostY - obs_state[Y],
                           postX - x) - obs_state[THETA]);
                  if (ABS(NORMALISE(t - obsPosts[0].heading))
                        < maxAngleDifference) {
                     obs_state[X] = x;
                     hypotheses.push_back(absCoordFromStateEstimate(
                              obs_state, obs_var, obs_mask));
                  }
               }
            }
            // !left || either ?
            if (obsWhichPosts != pBLUE_LEFT && obsWhichPosts != pYELLOW_LEFT) {
               float xoffset = SQUARE(obsPosts[0].distance)
                  - SQUARE(obs_state[Y] - rightPostY);
               if (xoffset >= 0) {
                  x = postX + sign*sqrt(xoffset);
                  t = NORMALISE(atan2f(rightPostY - obs_state[Y],
                           postX - x) - obs_state[THETA]);
                  if (ABS(NORMALISE(t - obsPosts[0].heading))
                        < maxAngleDifference) {
                     obs_state[X] = x;
                     hypotheses.push_back(absCoordFromStateEstimate(
                              obs_state, obs_var, obs_mask));
                  }
               }
            }
         }

         // looking at right field edge
         {
            obs_state[Y] = d - (FULL_FIELD_WIDTH/2);
            obs_state[THETA] = NORMALISE(-a);
            float x;
            float t;
            // !left || either ?
            if (obsWhichPosts != pBLUE_RIGHT &&
                  obsWhichPosts != pYELLOW_RIGHT) {
               float xoffset = SQUARE(obsPosts[0].distance)
                  - SQUARE(obs_state[Y] - leftPostY);
               if (xoffset >= 0) {
                  x = postX + sign*sqrt(xoffset);
                  t = NORMALISE(atan2f(leftPostY - obs_state[Y],
                           postX - x) - obs_state[THETA]);
                  if (ABS(NORMALISE(t - obsPosts[0].heading))
                        < maxAngleDifference) {
                     obs_state[X] = x;
                     hypotheses.push_back(absCoordFromStateEstimate(
                              obs_state, obs_var, obs_mask));
                  }
               }
            }
            // left || either ?
            if (obsWhichPosts != pBLUE_LEFT && obsWhichPosts != pYELLOW_LEFT) {
               float xoffset = SQUARE(obsPosts[0].distance)
                  - SQUARE(obs_state[Y] - rightPostY);
               if (xoffset >= 0) {
                  x = postX + sign*sqrt(xoffset);
                  t = NORMALISE(atan2f(rightPostY - obs_state[Y],
                           postX - x) - obs_state[THETA]);
                  if (ABS(NORMALISE(t - obsPosts[0].heading))
                        < maxAngleDifference) {
                     obs_state[X] = x;
                     hypotheses.push_back(absCoordFromStateEstimate(
                              obs_state, obs_var, obs_mask));
                  }
               }
            }
         }
         // pick what to return here
         vector<AbsCoord> goodHypotheses;
         if (hypotheses.size() > 0) {
            vector<AbsCoord>::iterator i;
            for (i = hypotheses.begin(); i != hypotheses.end(); ++i) {
               if (ABS((*i).x) <= FIELD_LENGTH/2 + 700
                     && ABS((*i).y) <= FIELD_WIDTH/2 + 700) {
                  goodHypotheses.push_back(*i);
               }
            }
            if (goodHypotheses.size() == 1) {
               float obs_state[STATE_VEC_DIM] = {0.0f};
               float obs_var[STATE_VEC_DIM] = {0.0f};
               stateFromAbsCoord(goodHypotheses[0], obs_state, obs_var);
               obs_state[X] = MIN(MAX(
                        obs_state[X], -FIELD_X_CLIP), FIELD_X_CLIP);
               obs_state[Y] = MIN(MAX(
                        obs_state[Y], -FIELD_Y_CLIP), FIELD_Y_CLIP);
               geoUpdate(obs_state, obs_var, obs_mask);
            }
         }
      }
   }
}
