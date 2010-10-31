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

#include <boost/program_options.hpp>
#include <ext/slist>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>

#include "utils/options.hpp"

namespace po = boost::program_options;
using namespace std;
using namespace __gnu_cxx;

void populate_options(po::options_description &config_file_options) {
   po::options_description game_config("Game options");
   game_config.add_options()
      ("game.type,g", po::value<string>()->default_value("MATCH"),
       "Type of game/challenge (MATCH, DRIBBLE, OPEN, PASSING)");

   po::options_description player_config("Player options");
   player_config.add_options()
      ("player.number,n", po::value<int>()->default_value(2),
       "player number")
      ("player.team,T", po::value<int>()->default_value(19),
       "team number");

   po::options_description debug_config("Debugging options");
   debug_config.add_options()
      ("debug.log,l", po::value<string>()->default_value("SILENT"),
       "log level used by llog")
      ("debug.log.motion,m", po::value<bool>()->default_value(false),
       "allow llog from motion")
      ("debug.logpath",
      po::value<string>()->default_value("/var/volatile/runswift"),
       "where to store log files")
      ("debug.shutdowntime", po::value<int>()->default_value(0),
       "shutdown after arg seconds")
      ("debug.gamecontroller,G", po::value<bool>()->default_value(true),
       "enable GameController thread")
      ("debug.motion,M", po::value<bool>()->default_value(true),
       "enable Motion thread")
      ("debug.offnaotransmitter,O", po::value<bool>()->default_value(true),
       "enable OffNaoTransmitter thread")
      ("debug.perception,P", po::value<bool>()->default_value(true),
       "enable Perception thread")
      ("debug.vision,V", po::value<bool>()->default_value(true),
       "enable Vision module")
      ("debug.behaviour,B", po::value<bool>()->default_value(true),
       "enable Behaviour module")
      ("debug.naotransmitter,C", po::value<bool>()->default_value(true),
       "enable Nao Transmitter module")
      ("debug.naoreceiver,R", po::value<bool>()->default_value(true),
       "enable Nao Receiver module");

   po::options_description behaviour_config("Behaviour options");
   behaviour_config.add_options()
      ("behaviour.skill,s", po::value<string>()->default_value("python"),
       "c++ skill for behaviour to start in")
      ("behaviour.targetx,x", po::value<int>()->default_value(0),
       "x-coordinate robot must reach")
      ("behaviour.targety,y", po::value<int>()->default_value(0),
       "y-coordinate robot must reach")
      ("behaviour.path", po::value<string>()->default_value(
                                              "/home/nao/data/behaviours"),
       "path to python behaviours")
      ("behaviour.pythonclass,p",
       po::value<string>()->default_value("DribbleSkill"),
       "The desired top level Python skill class.")
      ("default.body", po::value<string>()->default_value("REF_PICKUP"),
       "default body action type if behaviour isn't running")
      ("default.forward", po::value<int>()->default_value(0),
       "default forward parameter for the walk (mm/step)")
      ("default.left", po::value<int>()->default_value(0),
       "default left parameter for the walk (mm/step)")
      ("default.turn", po::value<float>()->default_value(0.0f),
       "default turn parameter for the walk (deg/step)")
      ("default.power", po::value<float>()->default_value(1.0f),
       "default power parameter for the kick (0.0-1.0)");

   po::options_description motion_config("Motion options");
   motion_config.add_options()
      ("motion.effector,e", po::value<string>()->default_value("Agent"),
       "effector to be used by motion")
      ("motion.touch,t", po::value<string>()->default_value("Agent"),
       "touch to be used by motion")
      ("motion.walk,w", po::value<string>()->default_value("Fast"),
       "walk/stand generator to be used by motion")
      ("motion.path",
       po::value<string>()->default_value("/home/nao/data/pos/"),
       "the path of .pos files for ActionGenerator")
      ("walk.f", po::value<float>()->default_value(0.25),
       "frequency of coronal plane rocking (Hz)")
      ("walk.st", po::value<float>()->default_value(1.0),
       "stiffness of leg joints (0.0 - 1.0)")
      ("walk.r", po::value<float>()->default_value(20.0),
       "amplitude of coronal plane rocking (degrees)")
      ("walk.s", po::value<float>()->default_value(5.0),
       "spread of legs when standing (degrees)")
      ("walk.l", po::value<float>()->default_value(20.0),
       "amplitude of leg lift (degrees)")
      ("walk.fs", po::value<float>()->default_value(5.0),
       "amplitude of forward step (degrees)")
      ("walk.ls", po::value<float>()->default_value(0.0),
       "amplitude of left step (degrees)")
      ("walk.ts", po::value<float>()->default_value(0.0),
       "amplitude of turn step (degrees)")
      ("walk.b", po::value<float>()->default_value(20.0),
       "bend of legs when standing upright (degrees)")
      ("walk.m", po::value<float>()->default_value(10.0),
       "leg lift frequency multiplier (must be multiple of two)");

   po::options_description vision_config("Vision options");
   vision_config.add_options()
      ("vision.camera,c", po::value<string>()->default_value("Nao"),
       "camera to be used by vision")
      ("vision.camera_controls", po::value<string>()->default_value(""),
       "comma separated list of cid:value pairs of controls "
             "(cid offset from V4L2_CID_BASE)")
      ("vision.dumpframes,d", po::value<bool>()->default_value(false),
       "dump frames to disk")
      ("vision.dumprate,r", po::value<int>()->default_value(1000),
       "dump frames every arg milliseconds")
      ("vision.dumpfile,f", po::value<string>()->default_value("dump.yuv"),
       "file to store frames in");

   po::options_description kinematics_config("Kinematics options");
   kinematics_config.add_options()
      ("kinematics.bodyPitchOffset", po::value<float>()->default_value(0.0),
       "accounts for imperfections in all motors in the robots body.")
      ("kinematics.cameraoffsetXtop", po::value<float>()->default_value(0.0),
       "difference between real cameraX angle compared to aldebaran specs")
      ("kinematics.cameraoffsetYtop", po::value<float>()->default_value(0.0),
       "difference between real cameraY angle compared to aldebaran specs")
      ("kinematics.cameraoffsetXbottom", po::value<float>()->default_value(0.0),
       "difference between real cameraX angle compared to aldebaran specs")
      ("kinematics.cameraoffsetYbottom", po::value<float>()->default_value(0.0),
       "difference between real cameraY angle compared to aldebaran specs");

   po::options_description gamecontroller_config("GameController options");
   gamecontroller_config.add_options()
      ("gamecontroller.connect", po::value<bool>()->default_value(true),
       "whether the GameController should try to connect")
      ("gamecontroller.state", po::value<string>()->default_value("INITIAL"),
       "game state if gamecontroller not connected, can be: "
       "INITIAL, READY, SET, PLAYING, FINISHED")
      ("gamecontroller.secondarystate",
       po::value<string>()->default_value("NORMAL"),
       "secondary game state if gamecontroller not connected, can be: "
       "NORMAL, PENALTYSHOOT")
      ("gamecontroller.ourcolour", po::value<string>()->default_value("blue"),
       "our team colour if gamecontroller not connected")
      ("gamecontroller.opponentteam", po::value<int>()->default_value(1),
       "opponent team number if gamecontroller not connected")
      ("gamecontroller.ourscore", po::value<int>()->default_value(0),
       "our team's score if gamecontroller not connected")
      ("gamecontroller.opponentscore", po::value<int>()->default_value(0),
       "opponent team's score if gamecontroller not connected")
      ("gamecontroller.firsthalf", po::value<bool>()->default_value(true),
       "whether we're in the first half if gamecontroller not connected")
      ("gamecontroller.kickoffteam", po::value<string>()->default_value("red"),
       "which team kicks off if gamecontroller not connected")
      ("gamecontroller.secsremaining", po::value<int>()->default_value(600),
       "seconds left in the half if gamecontroller not connected");

   po::options_description transmitter_config("Transmitter options");
   transmitter_config.add_options()
      ("transmitter.address", po::value<string>()->default_value
       ("192.168.0.255"), "address to broadcast to")
      ("transmitter.port", po::value<int>()->default_value(13371),
       "port to broadcast on");

   po::options_description network_config("Networking options");
   network_config.add_options()
      ("network.wireless.iwconfig_flags", po::value<string>()->default_value
       ("essid RUNSWIFT"), "iwconfig arguments to connect wlan0")
      ("network.wireless.static", po::value<bool>()->default_value
       (true), "set the wireless to static")
      ("network.wireless.static.ifconfig_flags",
       po::value<string>()->default_value
       ("192.168.0.100 netmask 255.255.255.0 up"),
       "ifconfig arguments to connect wlan0")
      ("network.wired.static", po::value<bool>()->default_value
       (false), "set the wired to static")
      ("network.wired.static.ifconfig_flags", po::value<string>()->default_value
       (""), "ifconfig arguments to connect eth0");

   config_file_options.add(game_config).add(player_config)
      .add(gamecontroller_config).add(debug_config).add(behaviour_config)
      .add(motion_config).add(vision_config).add(kinematics_config)
      .add(transmitter_config).add(network_config);
}

po::options_description store_and_notify(int argc, char **argv,
                                         po::variables_map &vm,
                                         po::options_description* generic) {
   vector<string> vs(argv + 1, argv + argc);
   // for(int arg = 1; arg < argc; ++arg)
   //    vs.push_back(argv[arg]);
   return store_and_notify(vs, vm, generic);
}


po::options_description store_and_notify(vector<string> argv,
                                         po::variables_map &vm,
                                         po::options_description* generic) {
   po::options_description config_file_options;
   populate_options(config_file_options);

   po::options_description cmdline_options;
   if (generic)
      cmdline_options.add(*generic);
   cmdline_options.add(config_file_options);

   /** Config hierarchy:
    *  - first,  command line arguments
    *  - second, /home/nao/data/<hostname>.cfg
    *  - third,  /home/nao/data/runswift.cfg
    *  - fourth, `pwd`/runswift.cfg
    **/
   // arguments from this call
   store(po::command_line_parser(argv).options(cmdline_options).run(), vm);

   // arguments from previous calls
   static slist<vector<string> > argvs;
   for (slist<vector<string> >::const_iterator argv_ci = argvs.begin();
        argv_ci != argvs.end(); ++argv_ci)
      store(po::command_line_parser(*argv_ci).options(cmdline_options).run(),
            vm);

   ifstream ifs;
   string hostname;

   ifs.open("/etc/hostname");
   ifs >> hostname;
   ifs.close();

   ifs.open(string("/home/nao/data/"+hostname+".cfg").c_str());
   store(parse_config_file(ifs, config_file_options), vm);
   ifs.close();

   ifs.open("/home/nao/data/runswift.cfg");
   store(parse_config_file(ifs, config_file_options), vm);
   ifs.close();

   ifs.open("runswift.cfg");
   store(parse_config_file(ifs, config_file_options), vm);
   ifs.close();

   /** Doesn't do anything right now, but will 'notify' any variables
    * we try to set via program_options */
   po::notify(vm);

   // assuming all options were valid and no exception was thrown, save argv for
   // repeated use
   argvs.push_front(argv);

   return cmdline_options;
}

/** A little struct used to help output program options */
struct type_info_compare {
   bool operator()(const type_info* lhs,
                 const type_info* rhs) const {
                    return lhs->before(* rhs);
                 }
};

/** Wrapper for outputting a boost::program_options::varaible_value */
template <typename T>
      struct output_value {
   void operator()(const po::variable_value& v) const {
      cout << v.as< T >() << endl;
   }
};

void options_print(boost::program_options::variables_map &vm) {
   /** Populate this map with actions that will correctly
    * print each type of program option */
   map<const type_info*,
   boost::function<void(const po::variable_value&)>,
                        type_info_compare> action_map;
   action_map[&typeid(string)] = output_value<string>();
   action_map[&typeid(int)] = output_value<int>();
   action_map[&typeid(bool)] = output_value<bool>();
   action_map[&typeid(float)] = output_value<float>();

   po::variables_map::iterator it;
   cout << endl;
   for (it = vm.begin(); it != vm.end(); ++it) {
      // cout << setw(20) << left << it->first << ": ";
      const po::variable_value& v = it->second;
      if (!v.empty()) {
         cout << setw(20) << left << it->first << ": ";
         action_map[&v.value().type()](v);
      }
   }
}
