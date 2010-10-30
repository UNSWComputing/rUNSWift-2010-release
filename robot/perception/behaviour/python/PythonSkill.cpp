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

#include "perception/behaviour/python/PythonSkill.hpp"
#include <boost/regex.hpp>
#include <string>
#include <list>
#include "utils/log.hpp"
#include "utils/speech.hpp"
#include "utils/incapacitated.hpp"

using std::string;
using std::endl;
using std::cerr;
using std::list;

void initRobot(void); /* Forward declaration */
static const char *BehaviourModuleName = "Behaviour";
static PyObject *behaviourModule = NULL;
static PyObject *behaviourPyFunc = NULL;
static const char *AcModuleName = "ActionCommand";
static PyObject *acModule = NULL;
static SkillParams *pyParams;
static ActionCommand::All *pyActions;
static WhichCamera *pyCamera;
static bool *pyUsePF;
static string pythonSkill = string("");

/* helper for putting things in dictionaries */
static inline void PySafeDict(PyObject *dict, const char *key, PyObject *obj) {
   PyDict_SetItemString(dict, key, obj);
   Py_DECREF(obj);
}

static inline void PySafeAttr(PyObject *o, const char *attr, PyObject *val) {
   PyObject_SetAttrString(o, attr, val);
   Py_DECREF(val);
}

void PythonSkill::reloadPython() {
   llog(INFO) << "Starting Python Interpreter" << std::endl;
   /* Initialize the Python interpreter.  Required. */
   if (Py_IsInitialized()) {
      Py_Finalize();
   }
   Py_Initialize();
   llog(INFO) << "Python Interpreter Running" << std::endl;

   behaviourPyFunc = NULL;
   /* Add a static module (defined at the bottom of this file) */
   initRobot();

   /* Set path to behaviours, and import */
   string sysappend = "sys.path.append('"+path+"')";
   PyRun_SimpleString("import sys");
   PyRun_SimpleString(sysappend.c_str());

   llog(INFO) << "Importing ActionCommand" << endl;
   if ((acModule = PyImport_ImportModule(AcModuleName)) == NULL) {
      llog(ERROR) << "Error Importing ActionCommand" << endl;
      if (PyErr_Occurred()) PyErr_Print();
   }
   llog(INFO) << "Imported ActionCommand" << endl;

   llog(INFO) << "Importing Behaviour" << endl;
   if ((behaviourModule = PyImport_ImportModule(BehaviourModuleName)) == NULL) {
      llog(ERROR) << "Error Importing Behaviour" << endl;
      if (PyErr_Occurred()) PyErr_Print();
   }
   llog(INFO) << "Imported Behaviour" << endl;
}

PythonSkill::PythonSkill(string path, string pythonclass, SkillParams *params,
      WhichCamera *camera, bool *usePF)
   : path(path) {
   llog(INFO) << "Constructing PythonSkill with skill" << pythonclass << endl;
   pyParams = params;
   pyCamera = camera;
   pyUsePF = usePF;
   pythonSkill = pythonclass;
   reloadPython();
   /* Set up inotify to monitor changes to python code */
   inotify_fd = inotify_init();
   int wd = inotify_add_watch(inotify_fd, path.c_str(),
         IN_MODIFY|IN_ATTRIB|IN_MOVED_FROM|IN_MOVED_TO|IN_DELETE);
   if (wd < 0) {
      llog(ERROR) << "Failed to start watching directory: " << path << endl;
   }
   path += "/skills";

   wd = inotify_add_watch(inotify_fd, path.c_str(),
         IN_MODIFY|IN_ATTRIB|IN_MOVED_FROM|IN_MOVED_TO|IN_DELETE);
   if (wd < 0) {
      llog(ERROR) << "Failed to start watching directory: " << path << endl;
   }
   inotify_timeout.tv_sec = 0;
   inotify_timeout.tv_usec = 0;
};

PythonSkill::~PythonSkill() {
   Py_Finalize();
   llog(INFO) << "Python Interpreter Destroyed" << std::endl;
}

void PythonSkill::execute(SkillParams *params,
      ActionCommand::All *actions, WhichCamera *camera, bool *usePF) {
   pythonSkill = params->pythonSkill;
   /* Check if Behaviour module needs a reload */
   FD_ZERO(&inotify_fdss);
   FD_SET(inotify_fd, &inotify_fdss);
   int ret = select(inotify_fd+1, &inotify_fdss, NULL, NULL, &inotify_timeout);
   int i, len;
   if (ret < 0) {
      llog(ERROR) << "select on inotify fd failed" << endl;
   } else if (ret && FD_ISSET(inotify_fd, &inotify_fdss)) {
      /* inotify event(s) available! */
      i = 0;
      len = read(inotify_fd, inotify_buf, INBUF_LEN);
      if (len < 0) {
         llog(ERROR) << "read on inotify fd failed" << endl;
      } else if (len) {
         while (i < len) {
            struct inotify_event *event;
            event = (struct inotify_event *) &inotify_buf[i];
            llog(VERBOSE) << "inotify event available for " <<
               event->name << endl;
            if (event->len) {
               boost::regex matchRegex(".*\\.py$");
               if (regex_match(event->name, matchRegex)) {
                  llog(INFO) << "Detected change in " << event->name << endl;
                  reloadPython();
                  if (behaviourModule == NULL) {
                     llog(ERROR) << "Error Reloading Behaviour" << endl;
                     if (PyErr_Occurred()) PyErr_Print();
                  } else {
                     llog(INFO) << "Behaviour module reloaded" << endl;
                  }
                  break;
               }
            }
            i += sizeof(struct inotify_event) + event->len;
         }
      }
   }

   /* Run Python behaviours */
   pyParams = params;
   pyActions = actions;
   pyCamera = camera;
   pyUsePF = usePF;
   PyObject *args = Py_BuildValue("()");
   PyObject *result = NULL;
   if (behaviourPyFunc != NULL) {
      result = PyEval_CallObject(behaviourPyFunc, args);
      Py_DECREF(args);
      if (result != NULL) {
         Py_DECREF(result);
      }
   }
   if (PyErr_Occurred() && PyErr_ExceptionMatches(PyExc_KeyboardInterrupt)) {
      cerr << endl;
      cerr << "###########################" << endl;
      cerr << "##    SIGINT RECEIVED    ##" << endl;
      cerr << "##       IN PYSKILL      ##" << endl;
      cerr << "##  ATTEMPTING SHUTDOWN  ##" << endl;
      cerr << "###########################" << endl;
      attemptingShutdown = true;
      PyErr_Clear();
   }
   if (behaviourPyFunc == NULL || result == NULL) {
      if (PyErr_Occurred()) PyErr_Print();
      actions->body.actionType = ActionCommand::Body::STAND;
      actions->head.yaw = 0.0;
      actions->head.pitch = 0.0;
      actions->head.isRelative = false;
      actions->head.yawSpeed = 0.2;
      actions->head.pitchSpeed = 0.2;
      struct timeval tv;
      gettimeofday(&tv, NULL);
      if ((tv.tv_usec/250000) & 1) {
         ActionCommand::LED::rgb err(1, 0, 1);
         actions->leds = ActionCommand::LED(0, 0, err, err, err, err, err);
      } else {
         ActionCommand::LED::rgb off(0, 0, 0);
         actions->leds = ActionCommand::LED(0, 0, off, off, off, off, off);
      }
      static bool said = false;
      if (tv.tv_sec % 60 == 0 && !said) {
         SAY("Python error");
      }
      said = (tv.tv_sec % 2 == 0);
   }
}

/* A static Python 'Robot' module */

/* C++ -> Python conversion functions */

static PyObject * PyObject_FromArray(const float array[], int n) {
   PyObject *t = PyTuple_New(n);
   for (int i = 0; i < n; ++i) {
      PyTuple_SetItem(t, i, PyFloat_FromDouble(array[i]));
   }
   return t;
}
static PyObject * PyObject_FromRRCoord(const RRCoord &rrcoord, bool var) {
   PyObject *d = PyDict_New();
   PySafeDict(d, "heading", PyFloat_FromDouble(rrcoord.heading));
   PySafeDict(d, "distance", PyFloat_FromDouble(rrcoord.distance));
   if (var) {
      PySafeDict(d, "variance", PyObject_FromArray(rrcoord.var, 2));
   }
   if (PyErr_Occurred()) PyErr_Print();
   return d;
}

static PyObject * PyObject_FromAbsCoord(const AbsCoord &abscoord, bool var) {
   PyObject *d = PyDict_New();
   PySafeDict(d, "x", PyInt_FromLong(abscoord.x));
   PySafeDict(d, "y", PyInt_FromLong(abscoord.y));
   PySafeDict(d, "theta", PyFloat_FromDouble(abscoord.theta));
   if (var) {
      PySafeDict(d, "variance",
            PyObject_FromArray(abscoord.var, 3));
   }
   if (PyErr_Occurred()) PyErr_Print();
   return d;
}

static PyObject * PyObject_FromTeamInfo(const TeamInfo &info) {
   PyObject *d = PyDict_New();
   PySafeDict(d, "teamNumber", PyInt_FromLong(info.teamNumber));
   PySafeDict(d, "teamColour", PyInt_FromLong(info.teamColour));
   PySafeDict(d, "score", PyInt_FromLong(info.score));
   PyObject *players = PyTuple_New(MAX_NUM_PLAYERS);
   for (int i = 0 ; i < MAX_NUM_PLAYERS; ++i) {
      PyObject *player = PyDict_New();
      PySafeDict(player, "penalty",
            PyInt_FromLong(info.players[i].penalty));
      PySafeDict(player, "secsTillUnpenalised",
            PyInt_FromLong(info.players[i].secsTillUnpenalised));
      PyTuple_SetItem(players, i, player);
   }
   PySafeDict(d, "players", players);
   return d;
}

static PyObject * PyObject_FromGameControlData(
      const RoboCupGameControlData &data) {
   PyObject *d = PyDict_New();
   PySafeDict(d, "state", PyInt_FromLong(data.state));
   PySafeDict(d, "firstHalf", PyBool_FromLong(data.firstHalf));
   PySafeDict(d, "kickOffTeam", PyInt_FromLong(data.kickOffTeam));
   PySafeDict(d, "secondaryState", PyInt_FromLong(
            data.secondaryState));
   PySafeDict(d, "secsRemaining", PyInt_FromLong(data.secsRemaining));
   PyObject *teams = PyTuple_New(2);
   PyTuple_SetItem(teams, 0, PyObject_FromTeamInfo(data.teams[0]));
   PyTuple_SetItem(teams, 1, PyObject_FromTeamInfo(data.teams[1]));
   PySafeDict(d, "teams", teams);
   return d;
}

/* Python -> C++ conversion functions */
static int convertAcHead(PyObject *obj, void *acHead) {
   ActionCommand::Head *ac = (ActionCommand::Head *) acHead;
   PyObject *yaw = PyObject_GetAttrString(obj, "_yaw");
   if (yaw != NULL && yaw != Py_None && PyFloat_Check(yaw)) {
      ac->yaw = PyFloat_AsDouble(yaw);
   }
   PyObject *pitch = PyObject_GetAttrString(obj, "_pitch");
   if (pitch != NULL && pitch != Py_None && PyFloat_Check(pitch)) {
      ac->pitch = PyFloat_AsDouble(pitch);
   }
   PyObject *isRelative = PyObject_GetAttrString(obj, "_isRelative");
   if (isRelative != NULL && isRelative != Py_None &&
       PyInt_Check(isRelative)) {
      ac->isRelative = (PyInt_AsLong(isRelative) != 0);
   }
   PyObject *yawSpeed = PyObject_GetAttrString(obj, "_yawSpeed");
   if (yawSpeed != NULL && yawSpeed != Py_None && PyFloat_Check(yawSpeed)) {
      ac->yawSpeed = PyFloat_AsDouble(yawSpeed);
   }
   PyObject *pitchSpeed = PyObject_GetAttrString(obj, "_pitchSpeed");
   if (pitchSpeed != NULL && pitchSpeed != Py_None &&
       PyFloat_Check(pitchSpeed)) {
      ac->pitchSpeed = PyFloat_AsDouble(pitchSpeed);
   }
   if (PyErr_Occurred()) PyErr_Print();
   return 1;
}

static int convertAcBody(PyObject *obj, void *acBody) {
   ActionCommand::Body *ac = (ActionCommand::Body *) acBody;
   PyObject *actionType = PyObject_GetAttrString(obj, "_actionType");
   if (actionType != NULL && actionType != Py_None && PyInt_Check(actionType)) {
      ac->actionType =
         (ActionCommand::Body::ActionType)PyInt_AsLong(actionType);
   }
   PyObject *forward = PyObject_GetAttrString(obj, "_forward");
   if (forward != NULL && forward != Py_None &&
       PyInt_Check(forward)) {
      ac->forward = PyInt_AsLong(forward);
   }
   PyObject *left = PyObject_GetAttrString(obj, "_left");
   if (left != NULL && left != Py_None &&
       PyInt_Check(left)) {
      ac->left = PyInt_AsLong(left);
   }
   PyObject *turn = PyObject_GetAttrString(obj, "_turn");
   if (turn != NULL && turn != Py_None &&
       PyFloat_Check(turn)) {
      ac->turn = PyFloat_AsDouble(turn);
   }
   PyObject *power = PyObject_GetAttrString(obj, "_power");
   if (power != NULL && power != Py_None &&
       PyFloat_Check(power)) {
      ac->power = PyFloat_AsDouble(power);
   }
   if (PyErr_Occurred()) PyErr_Print();
   return 1;
}

static int convertAcLeds(PyObject *obj, void *acLeds) {
   bool r, g, b;
   ActionCommand::LED *ac = (ActionCommand::LED *) acLeds;
   PyObject *leftEar = PyObject_GetAttrString(obj, "_leftEar");
   if (leftEar != NULL && leftEar != Py_None && PyInt_Check(leftEar)) {
      ac->leftEar = PyInt_AsLong(leftEar);
   }
   PyObject *rightEar = PyObject_GetAttrString(obj, "_rightEar");
   if (rightEar != NULL && rightEar != Py_None && PyInt_Check(rightEar)) {
      ac->rightEar = PyInt_AsLong(rightEar);
   }
   PyObject *leftEye = PyObject_GetAttrString(obj, "_leftEye");
   if (leftEye != NULL && leftEye != Py_None && PyTuple_Check(leftEye)
      && PyTuple_Size(leftEye) == 3) {
      r = g = b = false;
      PyObject *rP = PyTuple_GetItem(leftEye, 0);
      if (PyInt_Check(rP)) {
         r = (PyInt_AsLong(rP) != 0);
      }
      PyObject *gP = PyTuple_GetItem(leftEye, 1);
      if (PyInt_Check(gP)) {
         g = (PyInt_AsLong(gP) != 0);
      }
      PyObject *bP = PyTuple_GetItem(leftEye, 2);
      if (PyInt_Check(bP)) {
         b = (PyInt_AsLong(bP) != 0);
      }
      ac->leftEye = ActionCommand::LED::rgb(r, g, b);
   }
   PyObject *rightEye = PyObject_GetAttrString(obj, "_rightEye");
   if (rightEye != NULL && rightEye != Py_None && PyTuple_Check(rightEye)
      && PyTuple_Size(rightEye) == 3) {
      r = g = b = false;
      PyObject *rP = PyTuple_GetItem(rightEye, 0);
      if (PyInt_Check(rP)) {
         r = (PyInt_AsLong(rP) != 0);
      }
      PyObject *gP = PyTuple_GetItem(rightEye, 1);
      if (PyInt_Check(gP)) {
         g = (PyInt_AsLong(gP) != 0);
      }
      PyObject *bP = PyTuple_GetItem(rightEye, 2);
      if (PyInt_Check(bP)) {
         b = (PyInt_AsLong(bP) != 0);
      }
      ac->rightEye = ActionCommand::LED::rgb(r, g, b);
      if (PyErr_Occurred()) PyErr_Print();
   }
   PyObject *chestButton = PyObject_GetAttrString(obj, "_chestButton");
   if (chestButton != NULL && chestButton != Py_None &&
        PyTuple_Check(chestButton) && PyTuple_Size(chestButton) == 3) {
      r = g = b = false;
      PyObject *rP = PyTuple_GetItem(chestButton, 0);
      if (PyInt_Check(rP)) {
         r = (PyInt_AsLong(rP) != 0);
      }
      PyObject *gP = PyTuple_GetItem(chestButton, 1);
      if (PyInt_Check(gP)) {
         g = (PyInt_AsLong(gP) != 0);
      }
      PyObject *bP = PyTuple_GetItem(chestButton, 2);
      if (PyInt_Check(bP)) {
         b = (PyInt_AsLong(bP) != 0);
      }
      ac->chestButton = ActionCommand::LED::rgb(r, g, b);
   }
   PyObject *leftFoot = PyObject_GetAttrString(obj, "_leftFoot");
   if (leftFoot != NULL && leftFoot != Py_None && PyTuple_Check(leftFoot)
      && PyTuple_Size(leftFoot) == 3) {
      r = g = b = false;
      PyObject *rP = PyTuple_GetItem(leftFoot, 0);
      if (PyInt_Check(rP)) {
         r = (PyInt_AsLong(rP) != 0);
      }
      PyObject *gP = PyTuple_GetItem(leftFoot, 1);
      if (PyInt_Check(gP)) {
         g = (PyInt_AsLong(gP) != 0);
      }
      PyObject *bP = PyTuple_GetItem(leftFoot, 2);
      if (PyInt_Check(bP)) {
         b = (PyInt_AsLong(bP) != 0);
      }
      ac->leftFoot = ActionCommand::LED::rgb(r, g, b);
   }
   PyObject *rightFoot = PyObject_GetAttrString(obj, "_rightFoot");
   if (rightFoot != NULL && rightFoot != Py_None && PyTuple_Check(rightFoot)
      && PyTuple_Size(rightFoot) == 3) {
      r = g = b = false;
      PyObject *rP = PyTuple_GetItem(rightFoot, 0);
      if (PyInt_Check(rP)) {
         r = (PyInt_AsLong(rP) != 0);
      }
      PyObject *gP = PyTuple_GetItem(rightFoot, 1);
      if (PyInt_Check(gP)) {
         g = (PyInt_AsLong(gP) != 0);
      }
      PyObject *bP = PyTuple_GetItem(rightFoot, 2);
      if (PyInt_Check(bP)) {
         b = (PyInt_AsLong(bP) != 0);
      }
      ac->rightFoot = ActionCommand::LED::rgb(r, g, b);
   }

   if (PyErr_Occurred()) PyErr_Print();
   return 1;
}


/* Register callback for Python 'Behaviour' module */
static PyObject *
Robot_setCallback(PyObject *dummy, PyObject *args) {
   PyObject *temp;

   if (PyArg_ParseTuple(args, "O:setCallback", &temp)) {
      if (!PyCallable_Check(temp)) {
         PyErr_SetString(PyExc_TypeError, "parameter must be callable");
         return NULL;
      }
      Py_XINCREF(temp);         /* Add a reference to new callback */
      Py_XDECREF(behaviourPyFunc);  /* Dispose of previous callback */
      behaviourPyFunc = temp;       /* Remember new callback */
      /* Boilerplate to return "None" */
      Py_INCREF(Py_None);
   }
   Py_RETURN_NONE;
}

/* Methods of Robot module that can be called from Python */
static PyObject * Robot_pythonSkill(PyObject *self, PyObject *args) {
   PyObject *skill = Py_BuildValue("s", pythonSkill.c_str());
   if (PyErr_Occurred()) PyErr_Print();
   return skill;
}

static PyObject * Robot_attemptShutdown(PyObject *self, PyObject *args) {
   attemptingShutdown = true;
   Py_RETURN_NONE;
}

static PyObject * Robot_vNumBalls(PyObject *self, PyObject *args) {
   return PyInt_FromLong(pyParams->vNumBalls);
}

static PyObject * Robot_vImBallLocation(PyObject *self, PyObject *args) {
   PyObject *t = PyTuple_New(2);
   PyTuple_SetItem(t, 0, PyInt_FromLong(pyParams->ballLocation.first));
   PyTuple_SetItem(t, 1, PyInt_FromLong(pyParams->ballLocation.second));
   if (PyErr_Occurred()) PyErr_Print();
   return t;
}

static PyObject * Robot_vRrBallLocation(PyObject *self, PyObject *args) {
   return PyObject_FromRRCoord(pyParams->vRrBallLocation, false);
}

static PyObject * Robot_lSonarLostCount(PyObject *self, PyObject *args) {
   return PyInt_FromSize_t(pyParams->sonarLostCount);
}

static PyObject * Robot_lBallLostCount(PyObject *self, PyObject *args) {
   return PyInt_FromSize_t(pyParams->ballLostCount);
}

static PyObject * Robot_lEgoBallPosAbs(PyObject *self, PyObject *args) {
   return PyObject_FromAbsCoord(pyParams->lEgoBallPosAbs, true);
}

static PyObject * Robot_lTeamBallPosAbs(PyObject *self, PyObject *args) {
   return PyObject_FromAbsCoord(pyParams->lTeamBallPosAbs, true);
}

static PyObject * Robot_lBallPosRr(PyObject *self, PyObject *args) {
   return PyObject_FromRRCoord(pyParams->lBallPosRr, false);
}

static PyObject * Robot_obstacles(PyObject *self, PyObject *args) {
   list<RobotObstacle>::const_iterator i;
   const list<RobotObstacle> &robots = pyParams->robotObstacles;
   PyObject *obstacles = PyTuple_New(robots.size());
   int j;
   for (j=0, i = robots.begin(); i != robots.end(); ++i, ++j) {
      PyObject *robot = PyDict_New();
      PySafeDict(robot, "pos", PyObject_FromRRCoord(i->pos, true));
      PySafeDict(robot, "lostCount", PyInt_FromLong(i->lostCount));
      PySafeDict(robot, "seenCount", PyInt_FromLong(i->seenCount));
      PySafeDict(robot, "type", PyInt_FromLong(i->type));
      PyTuple_SetItem(obstacles, j, robot);
   }
   return obstacles;
}

static PyObject * Robot_sensorValues(PyObject *self, PyObject *args) {
   PyObject *d = PyDict_New();
   PyObject *sensors = PyTuple_New(Sensors::NUMBER_OF_SENSORS);
   for (int i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i) {
      PyTuple_SetItem(sensors, i, PyFloat_FromDouble(
               pyParams->sensors.sensors[i]));
   }
   PyObject *joints = PyDict_New();
   PyObject *angles = PyTuple_New(Joints::NUMBER_OF_JOINTS);
   PyObject *stiffnesses = PyTuple_New(Joints::NUMBER_OF_JOINTS);
   PyObject *temperatures = PyTuple_New(Joints::NUMBER_OF_JOINTS);
   for (int i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
      PyTuple_SetItem(angles, i, PyFloat_FromDouble(
               pyParams->sensors.joints.angles[i]));
      PyTuple_SetItem(stiffnesses, i, PyFloat_FromDouble(
               pyParams->sensors.joints.stiffnesses[i]));
      PyTuple_SetItem(temperatures, i, PyFloat_FromDouble(
               pyParams->sensors.joints.temperatures[i]));
   }
   PyObject *sonar = PyTuple_New(Sonar::NUMBER_OF_READINGS);
   for (int i = 0; i < Sonar::NUMBER_OF_READINGS; ++i) {
      PyTuple_SetItem(sonar, i, PyFloat_FromDouble(
               pyParams->sensors.sonar[i]));
   }
   PySafeDict(joints, "angles", angles);
   PySafeDict(joints, "stiffnesses", stiffnesses);
   PySafeDict(joints, "temperatures", temperatures);
   PySafeDict(d, "sensors", sensors);
   PySafeDict(d, "joints", joints);
   PySafeDict(d, "sonar", sonar);
   if (PyErr_Occurred()) PyErr_Print();
   return d;
}

static PyObject * Robot_postInfo(PyObject *self, PyObject *args) {
   PyObject *d = PyDict_New();
   PyObject *pos = PyTuple_New(MAX_POSTS);
   for (int i = 0; i < MAX_POSTS; ++i) {
      PyTuple_SetItem(pos, i,
            PyObject_FromRRCoord(pyParams->posts.pos[i], true));
   }
   PySafeDict(d, "pos", pos);
   PyObject *onField = PyTuple_New(MAX_POSTS);
   for (int i = 0; i < MAX_POSTS; ++i) {
      if (pyParams->posts.onField[i]) {
         Py_INCREF(Py_True);
         PyTuple_SetItem(onField, i, Py_True);
      } else {
         Py_INCREF(Py_False);
         PyTuple_SetItem(onField, i, Py_False);
      }
   }
   PySafeDict(d, "onField", onField);
   PySafeDict(d, "which", PyInt_FromLong(pyParams->posts.which));
   return d;
}


static PyObject * Robot_vRobotLocations(PyObject *self, PyObject *args) {
   PyObject *t = PyTuple_New(pyParams->numRobots);
   int i;
   for (i = 0; i < pyParams->numRobots; i++) {
      PyObject *r = PyTuple_New(2);
      PyTuple_SetItem(r, 0,
            PyObject_FromRRCoord(pyParams->robotLocations[i], true));
      PyTuple_SetItem(r, 1, PyInt_FromLong(pyParams->robotTypes[i]));
      PyTuple_SetItem(t, i, r);
   }
   return t;
}

static PyObject * Robot_position(PyObject *self, PyObject *args) {
   return PyObject_FromAbsCoord(pyParams->robotPos, true);
}

static PyObject * Robot_gcData(PyObject *self, PyObject *args) {
   return PyObject_FromGameControlData(pyParams->gcData);
}

static PyObject * Robot_gcTeam(PyObject *self, PyObject *args) {
   return PyObject_FromTeamInfo(pyParams->gcTeam);
}

static PyObject * Robot_player(PyObject *self, PyObject *args) {
   return PyInt_FromLong(pyParams->player);
}

static PyObject * Robot_gameType(PyObject *self, PyObject *args) {
   return PyInt_FromLong(pyParams->gameType);
}

static PyObject * PyTuple_FromRGB(const ActionCommand::LED::rgb &rgb) {
   PyObject *t = PyTuple_New(3);
   PyTuple_SetItem(t, 0, PyBool_FromLong(rgb.red));
   PyTuple_SetItem(t, 1, PyBool_FromLong(rgb.green));
   PyTuple_SetItem(t, 2, PyBool_FromLong(rgb.blue));
   return t;
}

static PyObject * Robot_acActive(PyObject *self, PyObject *args) {
   // Borrow references to the classes
   PyObject *acClassDict = PyModule_GetDict(acModule);
   PyObject *headClass = PyDict_GetItemString(acClassDict, "Head");
   PyObject *bodyClass = PyDict_GetItemString(acClassDict, "Body");
   PyObject *ledClass = PyDict_GetItemString(acClassDict, "LED");
   // Create new instances (new reference)
   PyObject *headInst = PyInstance_New(headClass, NULL, NULL);
   PyObject *bodyInst = PyInstance_New(bodyClass, NULL, NULL);
   PyObject *ledInst = PyInstance_New(ledClass, NULL, NULL);
   // Set the required attributes
   ActionCommand::All &ac = pyParams->acActive;
   PySafeAttr(headInst, "yaw", PyFloat_FromDouble(ac.head.yaw));
   PySafeAttr(headInst, "pitch", PyFloat_FromDouble(ac.head.pitch));
   PySafeAttr(headInst, "isRelative", PyBool_FromLong(ac.head.isRelative));
   PySafeAttr(headInst, "yawSpeed", PyFloat_FromDouble(ac.head.yawSpeed));
   PySafeAttr(headInst, "pitchSpeed", PyFloat_FromDouble(ac.head.yawSpeed));

   PySafeAttr(bodyInst, "actionType", PyInt_FromLong(ac.body.actionType));
   PySafeAttr(bodyInst, "forward", PyInt_FromLong(ac.body.forward));
   PySafeAttr(bodyInst, "left", PyInt_FromLong(ac.body.left));
   PySafeAttr(bodyInst, "turn", PyFloat_FromDouble(ac.body.turn));
   PySafeAttr(bodyInst, "power", PyFloat_FromDouble(ac.body.power));

   PySafeAttr(ledInst, "leftEar", PyInt_FromLong(ac.leds.leftEar));
   PySafeAttr(ledInst, "rightEar", PyInt_FromLong(ac.leds.rightEar));
   PySafeAttr(ledInst, "leftEye", PyTuple_FromRGB(ac.leds.leftEye));
   PySafeAttr(ledInst, "rightEye", PyTuple_FromRGB(ac.leds.rightEye));
   PySafeAttr(ledInst, "chestButton", PyTuple_FromRGB(ac.leds.chestButton));
   PySafeAttr(ledInst, "leftFoot", PyTuple_FromRGB(ac.leds.leftFoot));
   PySafeAttr(ledInst, "rightFoot", PyTuple_FromRGB(ac.leds.rightFoot));
   // Return a tuple of headInst, bodyInst, ledInst
   PyObject *t = PyTuple_New(3);
   PyTuple_SetItem(t, 0, headInst);
   PyTuple_SetItem(t, 1, bodyInst);
   PyTuple_SetItem(t, 2, ledInst);
   return t;
}

static PyObject * Robot_teamData(PyObject *self, PyObject *args) {
   PyObject *d = PyDict_New();
   for (int player = 1; player <= 3; ++player) {
      if (!isIncapacitated(player)) {
         PyObject *p = PyDict_New();
         PySafeDict(p, "robotPos",
               PyObject_FromAbsCoord(pyParams->teamData[player-1].robotPos,
                  true));
         PySafeDict(p, "egoBallPosAbs",
               PyObject_FromAbsCoord(pyParams->teamData[player-1].egoBallPosAbs,
                  true));
         PySafeDict(p, "ballLostCount",
               PyInt_FromLong(pyParams->teamData[player-1].lostCount));
         PySafeDict(p, "ballPosRr",
               PyObject_FromRRCoord(pyParams->teamData[player-1].ballPosRr,
                  false));
         PyDict_SetItem(d, PyInt_FromLong(player), p);
         Py_DECREF(p);
      }
   }
   return d;
}

static PyObject * Robot_isIncapacitated(PyObject *self, PyObject *args) {
   int i;
   if (PyArg_ParseTuple(args, "i", &i) && i >= 1 && i <= 3) {
      if (isIncapacitated(i)) {
         Py_INCREF(Py_True);
         return Py_True;
      } else {
         Py_INCREF(Py_False);
         return Py_False;
      }
   } else {
      Py_RETURN_NONE;
   }
}

static PyObject * Robot_setAction(PyObject *self,
      PyObject *args) {
   if (!PyArg_ParseTuple(args, "|O&O&O&:setAction",
            convertAcHead, &(pyActions->head),
            convertAcBody, &(pyActions->body),
            convertAcLeds, &(pyActions->leds))) {
      llog(ERROR) << "Invalid setAction parameters" << endl;
      if (PyErr_Occurred()) PyErr_Print();
   }
   Py_RETURN_NONE;
}

static PyObject * Robot_say(PyObject *self, PyObject *args) {
   char *toSay;
   if (PyArg_ParseTuple(args, "s", &toSay)) {
      SAY(toSay);
   }
   Py_RETURN_NONE;
}

static PyObject * Robot_llog(PyObject *self, PyObject *args) {
   char *toLog;
   if (PyArg_ParseTuple(args, "s", &toLog)) {
      llog(INFO) << toLog << std::flush;
   }
   Py_RETURN_NONE;
}

static PyObject * Robot_usingPF(PyObject *self, PyObject *args) {
   return PyInt_FromLong(pyParams->usingPF);
}

static PyObject * Robot_kidnapFactor(PyObject *self, PyObject *args) {
   return PyFloat_FromDouble(pyParams->kidnapFactor);
}

static PyObject * Robot_whichCamera(PyObject *self, PyObject *args) {
   return PyInt_FromLong(pyParams->whichCamera);
}

static PyObject * Robot_setCamera(PyObject *self, PyObject *args) {
   int whichCamera;
   if (PyArg_ParseTuple(args, "i", &whichCamera)) {
      *pyCamera = (WhichCamera) whichCamera;
   }
   Py_RETURN_NONE;
}

static PyObject * Robot_usePF(PyObject *self, PyObject *args) {
   *pyUsePF = true;
   Py_RETURN_NONE;
}

/* Index of the methods available in the Robot module */
static PyMethodDef Robot_methods[] = {
   {"setCallback", (PyCFunction)Robot_setCallback,
      METH_VARARGS, "Set the Behaviour callback."},
   {"attemptShutdown", (PyCFunction)Robot_attemptShutdown,
      METH_VARARGS, "Request runswift executable to stop."},
   {"pythonSkill", (PyCFunction)Robot_pythonSkill,
      METH_NOARGS, "Return the desired top level skill."},
   {"vNumBalls", (PyCFunction)Robot_vNumBalls,
      METH_NOARGS, "Return the number of balls detected by Vision."},
   {"vImBallLocation", (PyCFunction)Robot_vImBallLocation,
      METH_NOARGS, "Return the position of the ball in the image."},
   {"vRrBallLocation", (PyCFunction)Robot_vRrBallLocation,
      METH_NOARGS, "Return the position of the ball relative to the robot."},
   {"vRobotLocations", (PyCFunction)Robot_vRobotLocations,
      METH_NOARGS, "Return the positions of robots in the image."},
   {"lBallLostCount", (PyCFunction)Robot_lBallLostCount,
      METH_NOARGS, "Return the number of frames since a ball was seen."},
   {"lSonarLostCount", (PyCFunction)Robot_lSonarLostCount,
      METH_NOARGS, "Return the number of frames since a nearby obstacle "
         "was detected."},
   {"lEgoBallPosAbs", (PyCFunction)Robot_lEgoBallPosAbs,
      METH_NOARGS, "Return the absolute position of the ball, "
         "according to our own observations"},
   {"lTeamBallPosAbs", (PyCFunction)Robot_lTeamBallPosAbs,
      METH_NOARGS, "Return the absolute position of the ball, "
         "according to the whole team's observations"},
   {"lBallPosRr", (PyCFunction)Robot_lBallPosRr,
      METH_NOARGS, "Return the filtered robot-relative position of the ball"},
   {"obstacles", (PyCFunction)Robot_obstacles,
      METH_NOARGS, "Filtered robot-relative positions of obstacles"},
   {"sensorValues", (PyCFunction)Robot_sensorValues,
      METH_NOARGS, "Return all the robot's sensor values."},
   {"postInfo", (PyCFunction)Robot_postInfo,
      METH_NOARGS, "Return which posts are in view"},
   {"position", (PyCFunction)Robot_position,
      METH_NOARGS, "Return where the robot is"},
   {"setAction", (PyCFunction)Robot_setAction,
      METH_VARARGS,
      "Set ActionCommand to be carried out by Motion."},
   {"gcData", (PyCFunction)Robot_gcData,
      METH_VARARGS,
      "Get the current GameController info"},
   {"gcTeam", (PyCFunction)Robot_gcTeam,
      METH_VARARGS,
      "Get the current GameController info for our team"},
   {"player", (PyCFunction)Robot_player,
      METH_VARARGS,
      "Get the current player number"},
   {"gameType", (PyCFunction)Robot_gameType,
      METH_VARARGS,
      "Get the type of game/challenge being played"},
   {"acActive", (PyCFunction)Robot_acActive,
      METH_VARARGS,
      "Get the active ActionCommand"},
   {"teamData", (PyCFunction)Robot_teamData,
      METH_VARARGS,
      "Get robot and ball positions from team members"},
   {"isIncapacitated", (PyCFunction)Robot_isIncapacitated,
      METH_VARARGS,
      "Check if a robot is out of action"},
   {"say", (PyCFunction)Robot_say,
      METH_VARARGS,
      "Uses flite to say a string out loud"},
   {"llog", (PyCFunction)Robot_llog,
      METH_VARARGS, "Write something to the logs."},
   {"usingPF", (PyCFunction)Robot_usingPF,
      METH_VARARGS, "Get whether we the PF is running"},
   {"kidnapFactor", (PyCFunction)Robot_kidnapFactor,
      METH_VARARGS, "Metric for how lost the roobt is"},
   {"setCamera", (PyCFunction)Robot_setCamera,
      METH_VARARGS, "Sets which camera to use from now on"},
   {"whichCamera", (PyCFunction)Robot_whichCamera,
      METH_VARARGS, "Get which camera the current frame was taken with"},
   {"usePF", (PyCFunction)Robot_usePF,
      METH_VARARGS, "Ask the particle filter to re-seed the Kalman filter"},
   {NULL, NULL}    /* sentinel */
};

/* Inserts the Robot module into the interpreter */
void initRobot(void) {
   llog(INFO) << "Adding Robot Module" << std::endl;
   PyImport_AddModule("Robot");
   Py_InitModule("Robot", Robot_methods);
   if (PyErr_Occurred()) PyErr_Print();
   llog(INFO) << "Robot Module Added" << std::endl;
}

