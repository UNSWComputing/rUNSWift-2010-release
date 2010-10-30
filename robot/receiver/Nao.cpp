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
#include <boost/bind.hpp>
#include <string>
#include <iostream>
#include "receiver/Nao.hpp"
#include "utils/BroadcastData.hpp"
#include "utils/log.hpp"
#include "blackboard/Blackboard.hpp"

using namespace boost::asio;
using namespace std;

const std::string NaoReceiver::name("Nao Receiver");

NaoReceiver::NaoReceiver(void(NaoReceiver::*handler)
   (const boost::system::error_code &error, std::size_t), int port)
   : service(), socket(service, ip::udp::endpoint(ip::udp::v4(),
                port)),
     recvBuffer() {
   llog(INFO) << "Nao Receiver constructed" << endl;
   startReceive(handler);
   t = new boost::thread(boost::bind(&boost::asio::io_service::run, &service));
   llog(INFO) << "Listening for data" << endl;
}

NaoReceiver::~NaoReceiver() {
   service.stop();
   t->join();
   delete t;
   if (socket.is_open()) {
      socket.close();
   }
}

void NaoReceiver::startReceive(void(NaoReceiver::*handler)
   (const boost::system::error_code& error, std::size_t)) {
   socket.async_receive_from(
      boost::asio::buffer((mutable_buffer::mutable_buffer((void *)&recvBuffer,
         sizeof(BroadcastData)))), remoteEndpoint,
      boost::bind(handler, this,
         boost::asio::placeholders::error,
         boost::asio::placeholders::bytes_transferred));
}

void NaoReceiver::naoHandler(const boost::system::error_code &error,
   std::size_t size) {
   BroadcastData &bd = recvBuffer;
   if (bd.playerNum >= 1 && bd.playerNum <= 3 &&
       bd.team == readFrom(receiver, team)) {
      writeTo(receiver, data[bd.playerNum-1], bd);
      writeTo(receiver, lastReceived[bd.playerNum-1], time(NULL));
   }
   startReceive(&NaoReceiver::naoHandler);
}

void NaoReceiver::stdoutHandler(const boost::system::error_code &error,
   std::size_t size) {
   BroadcastData &bd = recvBuffer;
   cout << "Received data from player " << bd.playerNum << endl;
   startReceive(&NaoReceiver::stdoutHandler);
}

void NaoReceiver::tick() {
}

