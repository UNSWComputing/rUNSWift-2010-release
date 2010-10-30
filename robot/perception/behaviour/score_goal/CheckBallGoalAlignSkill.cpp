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

#include "perception/behaviour/score_goal/CheckBallGoalAlignSkill.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "utils/log.hpp"
#include "utils/basic_maths.hpp"

void CheckBallGoalAlignSkill::startNewAlignment() {
   stage = START_SCAN;
   hasSeenTwoGoals = false;
   isAligning = true;
   goalPositions.clear();
   isAligned = false;
   seenBlue = false;
   seenYellow = false;
}

bool CheckBallGoalAlignSkill::getIsAligned() {
   return hasJustCheckedAlignment() && isAligned;
}
bool CheckBallGoalAlignSkill::getIsLiningUp() {
   return isAligning;
}
bool CheckBallGoalAlignSkill::hasJustCheckedAlignment() {
   /* Hax to make it turn longer if it did not see many
      goals */
   if (goalPositions.size() < 2 && !isAligned) {
      return (!isAligning &&
               timer.elapsed_ms() - finishAlignTime < 4*POST_ALIGN_THRESHOLD);
   }
   return (!isAligning &&
         timer.elapsed_ms() - finishAlignTime < POST_ALIGN_THRESHOLD);
}
void CheckBallGoalAlignSkill::execute(SkillParams *params,
      ActionCommand::All *actions) {
   float heading = params->vRrBallLocation.heading;
   int turnDirection = heading/ABS(heading);

   const float *angles = params->sensors.joints.angles;
   float headYaw = angles[Joints::HeadYaw];
   float headPitch = angles[Joints::HeadPitch];
   if (isAligning) {
      buildGoalVector(params);
   }
   switch (stage) {
      case START_SCAN:
         actions->head = ActionCommand::Head(START_SCAN_YAW,
               SCAN_PITCH,
               false,
               SCAN_SPEED,
               0.2);
         if (ABS(headYaw - START_SCAN_YAW) < DEG2RAD(5) &&
               ABS(headPitch - SCAN_PITCH) < DEG2RAD(5)) {
            stage = SCAN;
         }
         break;
      case SCAN:
         actions->head = ActionCommand::Head(END_SCAN_YAW,
               SCAN_PITCH,
               false,
               SCAN_SPEED,
               0.2);
         if (ABS(headYaw - END_SCAN_YAW) < DEG2RAD(5) &&
               ABS(headPitch - SCAN_PITCH) < DEG2RAD(5)) {
            stage = START_SCAN;
            isAligning = false;
            determineIfAligned();
            finishAlignTime = timer.elapsed_ms();
         }
         break;
   }
}

void CheckBallGoalAlignSkill::buildGoalVector(SkillParams *params) {
   WhichPosts posts = params->posts.which;
   const RRCoord *post = params->posts.pos;
   int i = 0;
   llog(INFO) << "Detected posts: " << posts << std::endl;
   switch (posts) {
      case pBLUE_BOTH:
         goalPositions.push_back(post[i]);
         i++;
      case pBLUE_EITHER:
      case pBLUE_RIGHT:
      case pBLUE_LEFT:
         goalPositions.push_back(post[i]);
         // seenBlue = true;
         break;
      case pYELLOW_BOTH:
         // hasSeenTwoGoals = true;
         // goalPositions.push_back(post[i]);
         i++;
      case pYELLOW_EITHER:
      case pYELLOW_RIGHT:
      case pYELLOW_LEFT:
         // seenYellow = true;
         // goalPositions.push_back(post[i]);
         break;
   }
}
void CheckBallGoalAlignSkill::determineIfAligned() {
   llog(INFO) << "Goals found: " << std::endl;
   if (goalPositions.size() < 2) {
      isAligned = false;
      return;
   }
   removeOutlierGoals();
   for (int i = 0; i < goalPositions.size(); i++) {
      llog(INFO) << "   " << RAD2DEG(goalPositions[i].heading) << "deg, "
         << RAD2DEG(goalPositions[i].distance) << " mm" << std::endl;
   }
   float min, max;
   min = max = goalPositions[0].heading;
   for (int i = 1; i < goalPositions.size(); i++) {
      if (goalPositions[i].heading < min) {
         min = goalPositions[i].heading;
      }
      if (goalPositions[i].heading > max) {
         max = goalPositions[i].heading;
      }
   }
   float minSum, maxSum;
   int minCount, maxCount;
   minCount = maxCount = 0;
   minSum = maxSum = 0;
   for (int i = 0; i < goalPositions.size(); i++) {
      if (ABS(goalPositions[i].heading-min) < DEG2RAD(10)) {
         minSum += goalPositions[i].heading;
         minCount++;
      }
      if (ABS(goalPositions[i].heading-max) < DEG2RAD(10)) {
         maxSum += goalPositions[i].heading;
         maxCount++;
      }
   }

   llog(INFO) << "Max: " << RAD2DEG(max) << " Min: " << RAD2DEG(min)
              << std::endl;
   llog(INFO) << "Av Max: " << RAD2DEG(maxSum/maxCount) << " Av Min: "
              << RAD2DEG(minSum/minCount) << std::endl;
   min = minSum/minCount;
   max = maxSum/maxCount;
   if (max > 0 && min < 0 && ABS(RAD2DEG(max-min)) > 10) {
      if (!(seenBlue && seenYellow)) {
         isAligned = true;
         return;
      }
   }
   isAligned = false;
}

void CheckBallGoalAlignSkill::removeOutlierGoals() {
   if (goalPositions.size() < 4) return;
   /* Goals that are not outliers */
   std::vector<RRCoord> goodGoals;

   for (int i = 0; i < goalPositions.size(); i++) {
      float pos = goalPositions[i].heading;
      bool isFound = false;
      for (int j = 0; j < goalPositions.size(); j++) {
         if (i != j) {
            float otherPos = goalPositions[j].heading;
            if (RAD2DEG(ABS(pos-otherPos)) < 5) {
               goodGoals.push_back(goalPositions[i]);
               isFound = true;
               break;
            }
         }
      }
      if (!isFound) {
         llog(INFO) << "Goal with heading: " << RAD2DEG(pos)
                    << " is an outlier" << std::endl;
      }
   }
   goalPositions = goodGoals;
}
int CheckBallGoalAlignSkill::getTurnDirection() {
   if (goalPositions.size() < 2) return 1;
   float sum;
   for (int i = 0; i < goalPositions.size(); i++) {
      sum += goalPositions[i].heading;
   }
   float avg = sum/goalPositions.size();
   llog(INFO) << "Turn dir: " << (ABS(avg)/avg)
              << " 2 goals: " << hasSeenTwoGoals << std::endl;

   if (avg < 0) {
      return 1;
   } else {
      return -1;
   }
}

