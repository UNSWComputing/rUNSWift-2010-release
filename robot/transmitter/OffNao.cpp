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

#include <zlib.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <vector>
#include "perception/vision/rle.hpp"
#include "transmitter/OffNao.hpp"
#include "utils/log.hpp"
#include "utils/options.hpp"
#include "utils/AbsCoord.hpp"

using namespace std;
using boost::asio::ip::tcp;
using namespace boost;
using namespace boost::algorithm;
namespace po = boost::program_options;

const std::string OffNaoTransmitter::name("Off-Nao Transmitter");

OffNaoTransmitter::OffNaoTransmitter() {
   try {
      tcp::endpoint endpoint(tcp::v4(), 10125);
      acceptor_ = new tcp::acceptor(io_service_, endpoint);
      offnao_session_ptr new_session(new offnao_session(&io_service_, &room_));


      acceptor_->async_accept(new_session->socket(),
                              boost::bind(&OffNaoTransmitter::handle_accept,
                                          this, new_session,
                                          boost::asio::placeholders::error));
   } catch(const std::exception& e) {
      cerr << "Exception: " << e.what() << "\n";
   }
}

void OffNaoTransmitter::tick() {
   llog(VERBOSE) << name << ".. ticking away" << endl;
   io_service_.poll();
   room_.deliver();
   io_service_.poll();
}

OffNaoTransmitter::~OffNaoTransmitter() {
   delete acceptor_;
}

void OffNaoTransmitter::handle_accept(offnao_session_ptr session,
                                      const boost::system::error_code& error) {
   if (!error) {
      offnao_session_ptr new_session(new offnao_session(&io_service_, &room_));
      acceptor_->async_accept(new_session->socket(),
                             boost::bind(&OffNaoTransmitter::handle_accept,
                                         this, new_session,
                                         boost::asio::placeholders::error));
      session->start();
      boost::asio::socket_base::send_buffer_size option(2048);
      session->socket().set_option(option);
      boost::asio::socket_base::non_blocking_io command(false);
      session->socket().io_control(command);
      llog(DEBUG1) << name << "Created wireless link." << endl;
   } else {
      llog(ERROR) << name << error.message() << endl;
   }
}

OffNaoTransmitter::offnao_session::
offnao_session(boost::asio::io_service* io_service, offnao_room *room)
   :connection_(io_service), room_(*room), sendingMask(INITIAL_MASK) {
}

tcp::socket& OffNaoTransmitter::offnao_session::socket() {
   return connection_.socket();
}

void OffNaoTransmitter::offnao_session::start() {
   room_.join(shared_from_this());
   boost::asio::async_read(connection_.socket(),
                           // TODO(jayen): change this fixed size buffer to
                           // something more flexible
                       boost::asio::buffer(&receivedMask, sizeof(receivedMask)),
                           boost::bind(&offnao_session::handle_read,
                                       shared_from_this(),
                                       boost::asio::placeholders::error));
}

void OffNaoTransmitter::offnao_session::deliver() {
   // placeholder variable.  we should never have a write in progress, because
   // we do blocking deliveries with io_service.poll()
   bool write_in_progress = false;
   if (!write_in_progress) {
      OffNaoMask_t sendingMask = this->sendingMask;
      if ((sendingMask & SALIENCY_MASK) && !readFrom(vision, saliency))
         sendingMask &= (~SALIENCY_MASK);
      if ((sendingMask & RAW_IMAGE_MASK) && !readFrom(vision, currentFrame))
         sendingMask &= (~RAW_IMAGE_MASK);
      boost::asio::write(connection_.socket(),
                         boost::asio::buffer(&sendingMask,
                                             sizeof(sendingMask)));
      static uint8_t blackboard_cpy[sizeof(Blackboard)];
      static Colour vs[IMAGE_COLS/SALIENCY_DENSITY]
         [IMAGE_ROWS/SALIENCY_DENSITY];
      if (sendingMask & SALIENCY_MASK) {
         acquireLock(vision);
         memcpy(vs, readFrom(vision, saliency), sizeof(vs));
         memcpy(blackboard_cpy, blackboard, sizeof(Blackboard));
         releaseLock(vision);
      } else {
         memcpy(blackboard_cpy, blackboard, sizeof(Blackboard));
      }
      if (sendingMask & BLACKBOARD_MASK)
         boost::asio::write(connection_.socket(),
                            boost::asio::buffer(blackboard_cpy,
                                                sizeof(Blackboard)));
         connection_.sync_write(readFrom(localisation, robotObstacles));
      if (sendingMask & SALIENCY_MASK) {
         static uint8_t rleOutput[2 * IMAGE_ROWS/SALIENCY_DENSITY *
            IMAGE_COLS/SALIENCY_DENSITY];
         size_t size = rle::encode(rleOutput, (Colour *)vs);
         boost::asio::write(connection_.socket(),
               boost::asio::buffer(&size, sizeof(size)));
         boost::asio::write(connection_.socket(),
               boost::asio::buffer(rleOutput, size));
      }
      if (sendingMask & RAW_IMAGE_MASK) {
         // TODO(jayen): not YUV422 specific
         static Bytef zlibOutput[IMAGE_ROWS*IMAGE_COLS*2];
         static uint8_t vcf[IMAGE_ROWS*IMAGE_COLS*2];
         memcpy(vcf, readFrom(vision, currentFrame), sizeof(vcf));
         uLongf size = IMAGE_ROWS*IMAGE_COLS*2;
         if (compress2(zlibOutput, &size, vcf, IMAGE_ROWS*IMAGE_COLS*2,
                  Z_BEST_SPEED) !=
               Z_OK) {
            size = 0;
            boost::asio::write(connection_.socket(),
                  boost::asio::buffer(&size, sizeof(size)));
            boost::asio::write(connection_.socket(),
                  boost::asio::buffer
                  (readFrom(vision, currentFrame),
                   sizeof(uint8_t[IMAGE_ROWS*IMAGE_COLS*2])));
         } else {
            boost::asio::write(connection_.socket(),
                  boost::asio::buffer(&size, sizeof(size)));
            boost::asio::write(connection_.socket(),
                  boost::asio::buffer(zlibOutput, size));
         }
      }
      const vector<AbsCoord> &absCoord = readFrom(localisation, robotPos);
      if (sendingMask & PARTICLE_FILTER_MASK) {
         connection_.sync_write(absCoord);
      } else {
         if (readFrom(localisation, numRobotPos)) {
            connection_.sync_write(vector<AbsCoord>(absCoord.begin(),
                                                    absCoord.begin() + 1));
         } else {
            connection_.sync_write(vector<AbsCoord>());
         }
      }
   }
}

void OffNaoTransmitter::offnao_room::join(offnao_participant_ptr participant) {
   participants_.insert(participant);
}

void OffNaoTransmitter::offnao_room::
leave(offnao_participant_ptr participant) {
   participants_.erase(participant);
}

void OffNaoTransmitter::offnao_room::deliver() {
   for_each(participants_.begin(), participants_.end(),
                 boost::bind(&offnao_participant::deliver, _1));
}

void OffNaoTransmitter::offnao_session::
handle_write(boost::system::error_code const& error) {
   if (error)
      room_.leave(shared_from_this());
}

void OffNaoTransmitter::offnao_session::
handle_read(boost::system::error_code const& error) {
   if (!error) {
      llog(DEBUG1) << "Received Mask = " << receivedMask << endl;
      if (receivedMask & TO_NAO_MASKS) {
         if (receivedMask & COMMAND_MASK) {
            string command;
            connection_.sync_read(command);
            llog(INFO) << "Received command " << command << endl;

            vector<string> command_argv;
            split(command_argv, command, is_space());

            po::variables_map vm;
            try {
               po::options_description cmdline_options =
                  store_and_notify(command_argv, vm);
               blackboard->config = vm;
               options_print(vm);
               for (map<string, function<void()> >::const_iterator ci =
                      readFrom(thread, configCallbacks).begin();
                   ci != readFrom(thread, configCallbacks).end(); ++ci)
                  if (!ci->second.empty())
                     ci->second();
            } catch(program_options::error& e) {
               llog(WARNING) << "Error when parsing command line arguments: " <<
                                e.what() << endl;
            }
         }
      } else {
         sendingMask = receivedMask;
      }
      boost::asio::async_read(connection_.socket(),
                              boost::asio::buffer(&receivedMask,
                                                  sizeof(receivedMask)),
                              boost::bind(&offnao_session::handle_read,
                                          shared_from_this(),
                                          boost::asio::placeholders::error));
   } else {
      llog(DEBUG1) << "Received data with error!!!" << std::endl;
      room_.leave(shared_from_this());
   }
}
