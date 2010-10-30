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

#include "perception/behaviour/score_goal/ScoreGoalSkill.hpp"
#include "utils/log.hpp"

ScoreGoalSkill::ScoreGoalSkill() {
   headTrack = (Skill *) new HeadTrackSkill();
   gotoBall = (Skill *) new GotoBallSkill();
   findBall = (Skill *) new FindBallSkill();
   checkGoalAlign = new CheckBallGoalAlignSkill();
   alignFoot = new AlignFootSkill();
   lastBallSeenTime = timer.elapsed_ms();
   isInitial = true;
}

ScoreGoalSkill::~ScoreGoalSkill() {
   delete headTrack;
   delete gotoBall;
   delete findBall;
   delete checkGoalAlign;
   delete alignFoot;
}

void ScoreGoalSkill::execute(
      SkillParams *params, ActionCommand::All *actions) {
   float heading = params->vRrBallLocation.heading;
   float distance = params->vRrBallLocation.distance;
   actions->body.actionType = ActionCommand::Body::STAND;

   if (isInitial) {
      actions->body.actionType = ActionCommand::Body::INITIAL;
      isInitial = false;
      return;
   }

   /* The head is either scanning to see if the goals
    * are in front of us. Or the head is following the ball
    */
   if (checkGoalAlign->getIsLiningUp()) {
      checkGoalAlign->execute(params, actions);
      return;
   } else {
      headTrack->execute(params, actions);
   }

   /* We often loose the ball. This is remembering the last 
    * Position and time that we saw the ball
    */
   if (params->vNumBalls == 1) {
      actions->leds.leftEye = ActionCommand::LED::rgb(true, false, false);
      llog(INFO) << "Ball is at "  << heading << ", "
                 << distance << std::endl;
      lastBallSeenTime = timer.elapsed_ms();
      lastBallPosition = params->vRrBallLocation;
   } else {
      actions->leds.leftEye = ActionCommand::LED::rgb(false, true, false);
   }
   /* This is a special case after we check alignment with goals
    * If we are aligned then SHOOT. If not then walk around ball
    */
   if (checkGoalAlign->hasJustCheckedAlignment()) {
      llog(INFO) << "Check alignment "
                 << checkGoalAlign->getIsAligned();
      if (checkGoalAlign->getIsAligned() == true) {
         actions->head = ActionCommand::Head(0, 0, false, 0.3, 0.3);
         // actions->body = ActionCommand::Body(150, 0, 0);
         actions->body = ActionCommand::Body::KICK;
         return;
      } else {
         llog(INFO) << "Turning robot" << std::endl;
         actions->head = ActionCommand::Head(0, 0, false, 0.3, 0.3);
         actions->body = ActionCommand::Body(ActionCommand::Body::WALK,
                              0,
                              checkGoalAlign->getTurnDirection() *100,
                              checkGoalAlign->getTurnDirection()*DEG2RAD(-20));
         return;
      }
   }
   /* This is the main flow of the behaviour */
   if (timer.elapsed_ms() - lastBallSeenTime <= BALL_SEEN_TOLERENCE) {
      params->vRrBallLocation = lastBallPosition;
      if (alignFoot->isAligned()) {
          checkGoalAlign->startNewAlignment();
          checkGoalAlign->execute(params, actions);
      } else if (params->vRrBallLocation.distance < 280 &&
          ABS(params->vRrBallLocation.heading) < DEG2RAD(30)) {
          alignFoot->execute(params, actions);
          llog(INFO) << "Aligning" << std::endl;
      } else {
         gotoBall->execute(params, actions);
      }
   } else {
      findBall->execute(params, actions);
   }
}

