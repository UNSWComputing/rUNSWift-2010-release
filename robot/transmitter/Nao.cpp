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

#include <boost/asio.hpp>
#include <limits>
#include <string>
#include <vector>
#include "transmitter/Nao.hpp"
#include "utils/BroadcastData.hpp"
#include "utils/speech.hpp"
#include "blackboard/Blackboard.hpp"

using namespace boost::asio;
using namespace std;

const std::string NaoTransmitter::name("Nao Transmitter");

NaoTransmitter::NaoTransmitter()
   : service(),
   socket(service, ip::udp::v4()),
   broadcast_endpoint(ip::address::from_string(
      (blackboard->config)["transmitter.address"].as<string>()),
      (blackboard->config)["transmitter.port"].as<int>()) {
   socket_base::broadcast option(true);
   socket.set_option(option);
   boost::system::error_code ec;
   socket.connect(broadcast_endpoint, ec);
   if (ec) {
      llog(ERROR) << "could not connect: " << ec.message();
      sleep(5);
      SAY("not on the network", true);
      throw ec;
   }
   llog(INFO) << "Nao Transmitter constructed" << endl;
}

NaoTransmitter::~NaoTransmitter() {
   socket.close();
}

void NaoTransmitter::tick() {
   const vector<AbsCoord> &robotPos = readFrom(localisation, robotPos);
   AbsCoord pos;
   if (robotPos.size()) {
      pos = robotPos[0];
   }
   BroadcastData bd((blackboard->config)["player.number"].as<int>(),
                    (blackboard->config)["player.team"].as<int>(),
                    pos, readFrom(localisation, ballLostCount),
                    readFrom(localisation, egoBallPosAbs),
                    readFrom(localisation, ballPosRr),
                    readFrom(motion, active).body.actionType,
                    readFrom(motion, uptime));
   boost::system::error_code ec;
   socket.send(boost::asio::buffer(&bd, sizeof(BroadcastData)), 0, ec);

   if (ec) {
      llog(ERROR) << "could not send: " << ec.message();
      static time_t lastsaid = 0;
      time_t now = time(NULL);
      if (now >= lastsaid + 5) {
         SAY("nao transmitter " + ec.message());
         lastsaid = now;
      }
   }
}
