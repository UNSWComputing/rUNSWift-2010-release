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

#pragma once

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <string>
#include "utils/BroadcastData.hpp"
#include "blackboard/Blackboard.hpp"

class NaoReceiver {
  public:
   /**
    * The name of the module
    */
   static const std::string name;
   /**
    * Constructor.  Opens a socket for listening.
    */
   NaoReceiver(void(NaoReceiver::*handler)
      (const boost::system::error_code& error, std::size_t) =
     &NaoReceiver::naoHandler, int port =
     (blackboard->config)["transmitter.port"].as<int>());
   /**
    * Destructor. Closes the socket.
    */
   ~NaoReceiver();
   /**
    * One cycle of this thread
    */
   void tick();

   void naoHandler(const boost::system::error_code &error, std::size_t size);
   void stdoutHandler(const boost::system::error_code &error, std::size_t size);

  private:
   boost::asio::io_service service;
   boost::asio::ip::udp::socket socket;
   boost::asio::ip::udp::endpoint remoteEndpoint;
   BroadcastData recvBuffer;
   void startReceive(void(NaoReceiver::*handler)
      (const boost::system::error_code& error, std::size_t));
   boost::thread *t;
};

