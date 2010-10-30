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

#include <QDebug>
#include <QInputDialog>
#include <QMessageBox>
#include <QString>
#include <QStringList>
#include <QInputDialog>
#include <zlib.h>
#include <cmath>
#include <string>
#include <sstream>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/shared_ptr.hpp>

#include "readers/networkReader.hpp"
#include "perception/vision/rle.hpp"
#include "utils/Timer.hpp"
#include "progopts.hpp"
#include "blackboard/Blackboard.hpp"
#include "utils/AbsCoord.hpp"

using namespace std;
using namespace boost;

NetworkReader::NetworkReader(QString robotName, OffNaoMask_t mask) :
                            connection_(0),
                            isRecording(false),
                            robotName(robotName) {
   this->mask = mask;
   naoData.setPaused(false);
   isAlive = true;
}

NetworkReader::~NetworkReader() {
   if (isRecording) disconnect();
   isAlive = false;
}

/// Handle completion of a connect operation.
void NetworkReader::handle_connect(const boost::system::error_code& e,
      boost::asio::ip::tcp::resolver::iterator endpoint_iterator) {
   qDebug("Connected!");

   if (!e) {
      /* Successfully established connection. Start operation to read the list
       * of Blackboards. The connection::async_read() function will
       * automatically decode the data that is read from the underlying socket.
       */
      boost::asio::async_read(connection_->socket(),
            boost::asio::buffer(&received.mask,
               sizeof(received.mask)),
            boost::bind(&NetworkReader::handle_read, this,
               boost::asio::placeholders::error));

      // if (!(rand() % 10))
      write(mask);
      emit showMessage(QString("Connected! Now streaming"));
   } else if (endpoint_iterator != boost::asio::ip::tcp::resolver::iterator()) {
      // Try the next endpoint.
      connection_->socket().close();
      boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;
      connection_->socket().async_connect(endpoint,
            boost::bind(&NetworkReader::handle_connect, this,
               boost::asio::placeholders::error, ++endpoint_iterator));
   } else {
      /* An error occurred. Log it and return. Since we are not starting a new
       * operation the io_serviceasync_read will run out of work to do and the
       * wirelessClient will exit.
       */
      std::cerr << e.message() << std::endl;
      emit showMessage(QString::fromStdString(e.message()));
   }
}

/// Handle completion of a read operation.
void NetworkReader::handle_read(const boost::system::error_code& e) {
   if (!isAlive) {
      return;  // this class has already been destroyed.
   }
   if (!e) {
      static int counter = 0;
      static Timer t;
      emit showMessage(QString("average ms per packets: ") +
            QString::number(t.elapsed_ms()/++counter));

      try{
         if (received.mask) {
            Blackboard *blackboard = new Blackboard(config);
            if (received.mask & BLACKBOARD_MASK) {
            // this taken from http://www.boost.org/doc/libs/1_37_0/doc/html/boost_asio/example/echo/blocking_tcp_echo_client.cpp
               size_t reply_length = boost::asio::read(connection_->socket(),
                     boost::asio::buffer(blackboard, sizeof(Blackboard)));
               Q_ASSERT(reply_length == sizeof(Blackboard));

               // initialize blackboard.localisation.robotObstacles
               void *empty2 = &blackboard->localisation.robotObstacles;
               // override blackboard.localisation.robotObstacles that was
               // serialized bitwise from blackboard
               memcpy(&blackboard->localisation.robotObstacles, &empty2,
                      sizeof(empty2));
               memcpy((void**)&blackboard->localisation.robotObstacles + 1,
                      &empty2, sizeof(empty2));
               connection_->sync_read(blackboard->localisation.robotObstacles);
            }

         // initialize blackboard.vision.saliency
            Colour *vs = NULL;
            if (received.mask & SALIENCY_MASK) {
               static uint8_t rleOutput[2 * IMAGE_ROWS/SALIENCY_DENSITY *
                     IMAGE_COLS/SALIENCY_DENSITY];
               vs = (Colour*) malloc(sizeof(Colour[IMAGE_COLS/SALIENCY_DENSITY]
                     [IMAGE_ROWS/SALIENCY_DENSITY]));
               size_t size = 0;
               size_t reply_length = boost::asio::read(connection_->socket(),
                     boost::asio::buffer(&size, sizeof(size)));
               Q_ASSERT(reply_length == sizeof(size));
               reply_length = boost::asio::read(connection_->socket(),
                     boost::asio::buffer(rleOutput, size));
               Q_ASSERT(reply_length == size);
               rle::decode((Colour*)vs, rleOutput, size);
            }
            writeTo(vision, saliency, vs);

         // initialize blackboard.vision.currentFrame
            uint8_t *vcf = NULL;
            if (received.mask & RAW_IMAGE_MASK) {
            // TODO(jayen): not YUV422 specific
               static Bytef zlibOutput[IMAGE_ROWS*IMAGE_COLS*2];
               vcf =
                  (uint8_t*) malloc(sizeof(uint8_t[IMAGE_ROWS*IMAGE_COLS*2]));
               uLongf size = 0;
               size_t reply_length = boost::asio::read(connection_->socket(),
                     boost::asio::buffer
                           (&size, sizeof(size)));
               Q_ASSERT(reply_length == sizeof(size));
               if (size) {
                  reply_length = boost::asio::read(connection_->socket(),
                        boost::asio::buffer(zlibOutput,
                                            size));
                  Q_ASSERT(reply_length == size);
                  qDebug() << "Successfully decompressed " << size << " bytes.";
                  uLongf destLen = IMAGE_ROWS*IMAGE_COLS*2;
                  int zStatus = uncompress(vcf, &destLen, zlibOutput, size);
                  Q_ASSERT(zStatus == Z_OK);
                  Q_ASSERT(destLen = IMAGE_ROWS*IMAGE_COLS*2);
               } else {
               // size = 0 indicates no compression
                  size_t reply_length =
                        boost::asio::read
                        (connection_->socket(),
                         boost::asio::buffer
                               (vcf,
                                sizeof(uint8_t[IMAGE_ROWS*IMAGE_COLS*2])));
                  Q_ASSERT(reply_length ==
                        sizeof(uint8_t[IMAGE_ROWS*IMAGE_COLS*2]));
               }
            }
            writeTo(vision, currentFrame, (const uint8_t*) vcf);

            // initialize blackboard.localisation.robotPos
            static vector<AbsCoord> empty;
            // override blackboard.localisation.robotPos that was serialized
            // bitwise from blackboard
            memcpy(&blackboard->localisation.robotPos, &empty, sizeof(empty));
            connection_->sync_read(blackboard->localisation.robotPos);

            received.blackboard = blackboard;
            naoData.appendFrame(received);
            if(!naoData.getIsPaused()) {
               naoData.setCurrentFrame(naoData.getFramesTotal()-1);
            }
            // TODO(jayen): breaks over midnight
            static uint64_t lastnew = 0;
            struct timeval now;
            gettimeofday(&now, NULL);
            uint64_t now2 = now.tv_sec * 1000000ull + now.tv_usec;
            if (now2 >= lastnew + 250000) {
               emit newNaoData(&naoData);
               lastnew = now2;
            }
         }
         boost::asio::async_read(connection_->socket(),
                                 boost::asio::buffer(&received.mask,
                                       sizeof(received.mask)),
                                 boost::bind(&NetworkReader::handle_read, this,
                                             boost::asio::placeholders::error));
      } catch(boost::system::system_error &se) {
         qDebug() << "Error in receiving wireless data. " <<
               se.what() << endl;
         emit showMessage(se.what());
         // disconnect();
      }
   } else {
      qDebug("Error in receiving wireless data. Disconnected");
      std::cerr << e.message() << std::endl;
      emit showMessage("Error in data transmission. Disconnected...");
   }
}

void NetworkReader::run()  {
   Frame frame;
   int currentFrame = 0;
   emit showMessage(
        tr("Started session with nao. Hit record to begin stream..."));

   while (isAlive) {
//       if(!isRecording) {
//          // TODO(brockw): make this work with the camera tab
//          if (!naoData.getIsPaused() && naoData.getCurrentFrameIndex() <
//               naoData.getFramesTotal() - 1) {
//             naoData.nextFrame();
//             emit newNaoData(&naoData);
//          } else if (naoData.getFramesTotal() != 0) {
//             emit newNaoData(&naoData);
//          }
//       }
      currentFrame = naoData.getCurrentFrameIndex();
      if (currentFrame != naoData.getFramesTotal() - 1) {
         msleep(250);
      } else {
         msleep(200);  // this may be different soon. Hence the condition
      }
   }
   emit newNaoData(NULL);
}


   void NetworkReader::stopMediaTrigger() {
      if (isRecording)
         isRecording = disconnect();
      naoData.setPaused(true);
      emit showMessage(QString("Disconnected. Hit record to continue."));
   }

   void NetworkReader::recordMediaTrigger() {
      if (!isRecording) {
         isRecording = connect();
      }
      naoData.setPaused(false);
   }

   void NetworkReader::sendCommandLineString(QString item) {
      std::cout << item.toStdString() << std::endl;
      OffNaoMask_t sendingMask = COMMAND_MASK;
      boost::asio::write(connection_->socket(),
                         boost::asio::buffer(&sendingMask,
                                              sizeof(sendingMask)));
      connection_->sync_write(item.toStdString());
   }

   bool NetworkReader::disconnect() {
      std::cerr << "Try to disconnect!!" << std::endl;
      try {
         if (ioservice)
            ioservice->stop();

         if (cthread) {
            cthread->join();
            delete cthread;
            cthread = NULL;
         }

         if (connection_ && connection_->socket().is_open())
            connection_->socket().close();

         if (ioservice) {
            delete ioservice;
            ioservice = NULL;
         }
         if (connection_) {
            delete connection_;
            connection_ = NULL;
         }
         if (resolver) {
            delete resolver;
            resolver = NULL;
         }
         if (query) {
            delete query;
            query = NULL;
         }
      } catch(boost::system::system_error &se) {
         emit showMessage(QString("Could not disconnect to robot!"));
      }

      return false;
   }

   bool NetworkReader::connect() {
      std::cerr << "Try to connect!" << std::endl;
      try {
         ioservice = new boost::asio::io_service();
         connection_ = new Connection(ioservice);
         resolver = new boost::asio::ip::tcp::resolver(*ioservice);
         query = new boost::asio::ip::tcp::resolver::query(robotName.toStdString()
               , "10125");
         boost::asio::ip::tcp::resolver::iterator endpoint_iterator =
            resolver->resolve(*query);
         boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;

         connection_->socket().async_connect(endpoint,
               boost::bind(&NetworkReader::handle_connect, this,
                  boost::asio::placeholders::error, ++endpoint_iterator));
         cthread = new boost::thread(boost::bind(&boost::asio::io_service::run,
                  ioservice));
         std::cerr <<"Connected!" << std::endl;
      } catch(boost::system::system_error &se) {
         emit showMessage(QString("Could not connect to robot!"));
         return false;
      }
      return true;
   }

   void NetworkReader::write(const msg_t &msg) {
      ioservice->post(boost::bind(&NetworkReader::do_write, this, msg));
   }

   void NetworkReader::do_write(msg_t msg) {
      bool write_in_progress = !write_msgs_.empty();
      write_msgs_.push_back(msg);
      if (!write_in_progress) {
         boost::asio::async_write(connection_->socket(),
               boost::asio::buffer(&(write_msgs_.front()),
                  sizeof(write_msgs_.front())),
               boost::bind(&NetworkReader::handle_write, this,
                  boost::asio::placeholders::error));
      }
   }

   void NetworkReader::handle_write(const boost::system::error_code &error) {
      if (!error) {
         if (!write_msgs_.empty()) {
            boost::asio::async_write(connection_->socket(),
                  boost::asio::buffer(&(write_msgs_.front()),
                     sizeof(write_msgs_.front())),
                  boost::bind(&NetworkReader::handle_write, this,
                     boost::asio::placeholders::error));
            write_msgs_.pop_front();
         }
      } else {
         do_close();
      }
   }

   void NetworkReader::do_close() {
      connection_->socket().close();
   }
