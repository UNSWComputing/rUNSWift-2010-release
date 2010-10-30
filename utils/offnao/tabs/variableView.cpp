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

#include "variableView.hpp"
#include <QTreeWidgetItem>
#include <QStringList>
#include <QString>
#include <string>
#include <utility>
#include "utils/ActionCommand.hpp"
#include "blackboard/Blackboard.hpp"

using namespace std;

const char* strPostEnum[] = {
   "pNONE",
   "pBLUE_LEFT",
   "pBLUE_RIGHT",
   "pBLUE_BOTH",
   "pBLUE_EITHER",
   "pYELLOW_LEFT",
   "pYELLOW_RIGHT",
   "pYELLOW_BOTH",
   "pYELLOW_EITHER"
};

VariableView::VariableView() {
   perceptionHeading = new QTreeWidgetItem(this, QStringList(QString("Perception")), 1);
   perceptionHeading->setExpanded(true);
   perceptionAverageFPS  = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Framerate: ")), 1);
   perceptionKinematicsTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Kinematics time: ")), 1);
   perceptionVisionTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Vision time: ")), 1);
   perceptionLocalisationTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Localisation time: ")), 1);
   perceptionBehaviourTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Behaviour time: ")), 1);
   perceptionTotalTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Total time: ")), 1);

   visionHeading = new QTreeWidgetItem(this, QStringList(QString("Vision")), 1);
   visionHeading->setExpanded(true);

   visionNumBalls = new QTreeWidgetItem(visionHeading,
         QStringList(QString("# Balls = ")), 1);
   visionBallPos = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Ball Pos = ")), 1);
   visionBallPosRobotRelative  = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Robot relative Ball Pos = ")), 1);
   visionWhichPosts = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Which posts = ")), 1);
   visionPost1 = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Post 1 = ")), 1);
   visionPost2 = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Post 2 = ")), 1);

   this->setHeaderLabel(QString("State variables"));

   localisationHeading = new QTreeWidgetItem(this,
         QStringList(QString("Localisation")), 1);
   localisationHeading->setExpanded(true);
   localisationRobotPos = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Robot pos = ")), 1);

   motionHeading = new QTreeWidgetItem(this, QStringList(QString("Motion")), 1);
   motionHeading->setExpanded(true);

   behaviourHeading = new QTreeWidgetItem(this,
                                          QStringList(QString("Behaviour")), 1);
   behaviourBodyRequest = new QTreeWidgetItem(behaviourHeading,
                     QStringList(QString("request.body.actionType = ")), 1);
   behaviourHeading->setExpanded(true);

   this->addTopLevelItem(visionHeading);
   this->addTopLevelItem(localisationHeading);
   this->addTopLevelItem(motionHeading);
   this->addTopLevelItem(behaviourHeading);
}

void VariableView::redraw(NaoData *naoData) {
   Frame frame = naoData->getCurrentFrame();
   Blackboard *blackboard = (frame.blackboard);
   if(!blackboard) return;

   // struct RobotPos wrapperPos[4];
   // readArray(localisation, robot, wrapperPos);
   const vector<AbsCoord> &pos = readFrom(localisation, robotPos);
   WhichPosts posts = readFrom(vision, posts);
   stringstream spos;
   int numRobots = readFrom(localisation, numRobotPos);
   bool usingPF = readFrom(localisation, usingPF);
   double kidnapFactor = readFrom(localisation, kidnapFactor);
   if (pos.size() > 0) {
      if (usingPF) {
         spos << "using PF" << endl;
      } else {
         spos << "not using PF" << endl;
      }
      spos << "numRobots " << numRobots << endl;
      spos << "kidnapFactor " << kidnapFactor << endl;
      spos << "Robot @ (" << pos[0].x << "," << pos[0].y << ") " <<
         pos[0].theta << " " << RAD2DEG(pos[0].theta) << ", " << endl;
      spos << "Variance: x " << pos[0].var[0] << ", y " <<
         pos[0].var[1] << " theta " << pos[0].var[2] << endl;
   }
   // spos << "Kidnap Factor: " << wrapperPos[0].kidnapFactor << endl;

   if (frame.isLocalizationDebugging) {
      spos << "Real @ (" << frame.actualPosition.x() << ", " <<
         frame.actualPosition.y() << ") " << frame.actualHeading << " "
         << RAD2DEG(frame.actualHeading) << endl;
   }
   localisationRobotPos->setText(0, spos.str().c_str());

   updateVision(naoData);
   updateBehaviour(naoData);
}

template <class T>
const char* VariableView::createSufPref(string pref, T t, string suff) {
   stringstream s;
   s << pref << t << suff;
   return s.str().c_str();
}

void VariableView::updateBehaviour(NaoData *naoData) {
   Blackboard *blackboard = (naoData->getCurrentFrame().blackboard);
   if (!blackboard) return;

   string actionName;
   switch (readFrom(behaviour, request).body.actionType) {
   case ActionCommand::Body::NONE: actionName = "NONE"; break;
   case ActionCommand::Body::STAND: actionName = "STAND"; break;
   case ActionCommand::Body::WALK: actionName = "WALK"; break;
   case ActionCommand::Body::INITIAL: actionName = "INITIAL"; break;
   case ActionCommand::Body::KICK: actionName = "KICK"; break;
   case ActionCommand::Body::GETUP_FRONT: actionName = "GETUP_FRONT"; break;
   case ActionCommand::Body::GETUP_BACK: actionName = "GETUP_BACK"; break;
   }
   behaviourBodyRequest->setText(0, createSufPref("request.body.actionType = ",
                                                  actionName, ""));
}

void VariableView::updateVision(NaoData *naoData) {
   Blackboard *blackboard = (naoData->getCurrentFrame().blackboard);
   if (!blackboard) return;


   uint32_t total = readFrom(perception, total);

   times.push_back(total);
   if (times.size() > 10) times.pop_front();
   int sum = 0;
   for (unsigned int i = 0; i < times.size(); i++) sum += times[i];
   int averagetime = sum/times.size();
   float fps = 1000000.0/total;


   perceptionAverageFPS->setText(0, createSufPref("Framerate: ", fps, " fps"));
   perceptionKinematicsTime->setText(0, createSufPref("Kinematics time: ", readFrom(perception, kinematics), ""));
   perceptionVisionTime->setText(0, createSufPref("Vision time: ", readFrom(perception, vision), ""));
   perceptionLocalisationTime->setText(0, createSufPref("Localisation time: ", readFrom(perception, localisation), ""));
   perceptionBehaviourTime->setText(0, createSufPref("Behaviour time: ", readFrom(perception, behaviour), ""));
   perceptionTotalTime->setText(0, createSufPref("Total time: ", total, ""));

   stringstream sBallPos;
   int numBalls = readFrom(vision, numBalls);
   pair<int16_t, int16_t> ballLocation = readFrom(vision, ballInCameraCoords);
   if (numBalls == 0) {
   } else {
      sBallPos << "Ball is at (" << ballLocation.first <<
         ", " << ballLocation.second << ")";
   }
   visionBallPos->setText(0, sBallPos.str().c_str());

   WhichPosts posts = readFrom(vision, posts);
   RRCoord post[MAX_POSTS];
   readArray(vision, post, post);

   stringstream sPosts;

   int numEdges = readFrom(vision, numEdges);
   Line edges[MAX_FIELD_EDGES];
   readArray(vision, edges, edges);

   sPosts << "numEdges " << numEdges << endl;
   for (int i = 0; i < numEdges; i++) {
      sPosts << "Edge 1: " << edges[i].t1 << "x + " << edges[i].t2 <<
         "y + " << edges[i].t3 << endl;
   }

   sPosts << "WhichPosts is " << strPostEnum[posts];
   visionWhichPosts->setText(0, sPosts.str().c_str());

   stringstream sPostPosRelative1;
   stringstream sPostPosRelative2;
   if (posts == pBLUE_BOTH || posts == pYELLOW_BOTH) {
      sPostPosRelative1 << "P1 @ " << post[0].distance << ", " <<
         RAD2DEG(post[0].heading) << ", " << post[0].heading << ", ("
         << post[0].var[0] << ", " << post[0].var[1] << ")";
      sPostPosRelative2 << "P2 @ " << post[1].distance << ", " <<
         RAD2DEG(post[1].heading) << ", " << post[1].heading << ", ("
         << post[1].var[0] << ", " << post[1].var[1] << ")";
      visionPost1->setText(0, sPostPosRelative1.str().c_str());
      visionPost2->setText(0, sPostPosRelative2.str().c_str());
   } else if (posts != pNONE) {
      sPostPosRelative1 << "P1 @ " << post[0].distance << ", " <<
         RAD2DEG(post[0].heading) << ", " << post[0].heading << ", ("
         << post[0].var[0] << ", " << post[0].var[1] << ")";
      sPostPosRelative2 << "";
      visionPost1->setText(0, sPostPosRelative1.str().c_str());
      visionPost2->setText(0, sPostPosRelative2.str().c_str());
   }

   numBalls = readFrom(vision, numBalls);
   RRCoord ball[MAX_BALLS];
   readArray(vision, ball, ball);

   stringstream sBallPosRelative;
   if(numBalls == 1) {
      sBallPosRelative << "Ball @ " << ball[0].distance << ", " <<
         RAD2DEG(ball[0].heading) << ", " << ball[0].heading;
   }
   visionBallPosRobotRelative->setText(0, sBallPosRelative.str().c_str());

   this->visionNumBalls->setText(0,createSufPref("# Balls: ", numBalls, ""));
}
