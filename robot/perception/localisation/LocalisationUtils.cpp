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

#include "LocalisationUtils.hpp"
#include "LocalisationDefs.hpp"
#include "utils/log.hpp"

using namespace std;

bool posFromTwoPosts(const RRCoord *obsPosts, const bool &ours,
      float &x, float &y, float &t,
      float &xvar, float &yvar, float &tvar) {
      int sign = (ours ? 1 : -1);
      float postX = -sign*GOAL_POST_ABS_X_DIST;
      float leftPostY = -sign*GOAL_POST_ABS_Y_DIST;
      float rightPostY = sign*GOAL_POST_ABS_Y_DIST;
      if (isnan(obsPosts[0].distance) || isnan(obsPosts[0].heading) ||
            isnan(obsPosts[1].distance) || isnan(obsPosts[1].heading)) {
         llog(ERROR) << "Vision returned a post with NaN data" << endl;
      } else {
         y = (SQUARE(obsPosts[0].distance)
               - SQUARE(obsPosts[1].distance)
               + SQUARE(rightPostY) - SQUARE(leftPostY))
            / (2 * (rightPostY - leftPostY));
         float xoffset = (SQUARE(obsPosts[0].distance)
               - SQUARE(y - leftPostY));
         if (xoffset >= 0) {
            x = postX + sign*sqrt(SQUARE(obsPosts[0].distance)
                  - SQUARE(y - leftPostY));
            t = NORMALISE(atan2(leftPostY - y,
                     postX - x)
                  - obsPosts[0].heading);
            xvar = SQUARE(sqrt((obsPosts[0].var[0] +
                        obsPosts[1].var[0])/2.0) + STD_DEV_XY_CONST);
            yvar = xvar;
            tvar = SQUARE(sqrt(obsPosts[0].var[1]) + STD_DEV_T_CONST);
            x = MIN(MAX(x, -FIELD_X_CLIP), FIELD_X_CLIP);
            y = MIN(MAX(y, -FIELD_Y_CLIP), FIELD_Y_CLIP);
            return true;
         }
      }
      return false;
}

void posFromFieldEdge(const Line &obsEdge,
      const float *state_vec, float *obs_state, float *obs_var, bool *obs_mask,
      float &expDistToEdge, float &d) {
      float a = atan2f(((float)-obsEdge.t1)*((float)obsEdge.t3),
            ((float)obsEdge.t2)*((float)obsEdge.t3));
      d = ABS(obsEdge.t3)/sqrt(
            SQUARE(obsEdge.t1)+SQUARE(obsEdge.t2));
      if (ABS(NORMALISE(state_vec[THETA] - M_PI_2 + a)) < PI_8) {
         // looking at front field edge
         obs_mask[X] = true;
         obs_state[X] = (FULL_FIELD_LENGTH/2) - d;
         obs_mask[THETA] = true;
         obs_state[THETA] = NORMALISE(M_PI_2 - a);
         expDistToEdge =  (FULL_FIELD_LENGTH/2) - state_vec[X];
      }
      if (ABS(NORMALISE(state_vec[THETA] - M_PI + a)) < PI_8) {
         // looking at left field edge
         obs_mask[Y] = true;
         obs_state[Y] = (FULL_FIELD_WIDTH/2) - d;
         obs_mask[THETA] = true;
         obs_state[THETA] = NORMALISE(M_PI - a);
         expDistToEdge =  (FULL_FIELD_LENGTH/2) - state_vec[Y];
      }
      if (ABS(NORMALISE(state_vec[THETA] + M_PI_2 + a)) < PI_8) {
         // looking at back field edge
         obs_mask[X] = true;
         obs_state[X] = d - (FULL_FIELD_LENGTH/2);
         obs_mask[THETA] = true;
         obs_state[THETA] = NORMALISE(-M_PI_2 - a);
         expDistToEdge =  (FULL_FIELD_LENGTH/2) + state_vec[X];
      }
      if (ABS(NORMALISE(state_vec[THETA] + a)) < PI_8) {
         // looking at right field edge
         obs_mask[Y] = true;
         obs_state[Y] = d - (FULL_FIELD_WIDTH/2);
         obs_mask[THETA] = true;
         obs_state[THETA] = NORMALISE(-a);
         expDistToEdge =  (FULL_FIELD_LENGTH/2) + state_vec[Y];
      }
      // approximation of camera height, good enough for variance calculations
      int camera_height = 500;
      float phi = atan(d/camera_height);
      float var = SQUARE(STD_DEV_XY_CONST +
            camera_height*(tan(phi+DEG2RAD(3))-tan(phi)));
      obs_var[X] = obs_var[Y] = var;
      obs_var[THETA] = SQUARE(STD_DEV_T_CONST);
      obs_state[X] = MIN(MAX(obs_state[X], -FIELD_X_CLIP), FIELD_X_CLIP);
      obs_state[Y] = MIN(MAX(obs_state[Y], -FIELD_Y_CLIP), FIELD_Y_CLIP);
}

void fieldEdgeSanity(int &obsNumEdges, Line *obsEdges,
      const WhichPosts &obsWhichPosts, const RRCoord *obsPosts) {
   bool dodgy[2] = {false, false};
   static const float min_edge_goal_dist = FIELD_LENGTH_OFFSET/2.0f;
   if (obsWhichPosts != pNONE) {
      for (int i = 0; i < obsNumEdges; ++i) {
         float x = obsPosts[0].distance*cos(obsPosts[0].heading);
         float y = obsPosts[0].distance*sin(obsPosts[0].heading);
         float dist = ABS(obsEdges[i].t1*x + obsEdges[i].t2*y + obsEdges[i].t3)
            / sqrt(SQUARE(obsEdges[i].t1) + SQUARE(obsEdges[i].t2));
         if (dist < min_edge_goal_dist) {
            dodgy[i] = true;
         }
         if (obsWhichPosts == pBLUE_BOTH || obsWhichPosts != pYELLOW_BOTH) {
            for (int i = 0; i < obsNumEdges; ++i) {
               float x = obsPosts[1].distance*cos(obsPosts[1].heading);
               float y = obsPosts[1].distance*sin(obsPosts[1].heading);
               float dist = ABS(obsEdges[i].t1*x +
                                obsEdges[i].t2*y + obsEdges[i].t3)
                  / sqrt(SQUARE(obsEdges[i].t1) + SQUARE(obsEdges[i].t2));
               if (dist < min_edge_goal_dist) {
                  dodgy[i] = true;
               }
            }
         }
      }
   }

   if (dodgy[1]) {
      obsNumEdges = 1;
   }
   if (dodgy[0] && obsNumEdges == 2) {
      obsEdges[0] = obsEdges[1];
      obsNumEdges = 1;
   } else if (dodgy[0]) {
      obsNumEdges = 0;
   }
   if (obsNumEdges == 2) {
      float a0 = atan2f(((float)-obsEdges[0].t1)*((float)obsEdges[0].t3),
            ((float)obsEdges[0].t2)*((float)obsEdges[0].t3));
      float a1 = atan2f(((float)-obsEdges[1].t1)*((float)obsEdges[1].t3),
            ((float)obsEdges[1].t2)*((float)obsEdges[1].t3));
      float theta = ABS(NORMALISE(a0 - a1));
      if (theta < DEG2RAD(80) || theta > DEG2RAD(100)) {
         // crazy angles between lines
         obsNumEdges = 0;
      }
   }
}
