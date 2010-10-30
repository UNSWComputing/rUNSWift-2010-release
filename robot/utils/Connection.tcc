#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <iostream>
#include "utils/log.hpp"

template <typename T, typename Handler>
void Connection::async_write(const T& t, Handler handler) {
   // Serialize the data first so we know how large it is.
   std::ostringstream archive_stream;
   boost::archive::binary_oarchive archive(archive_stream);
   archive << BOOST_SERIALIZATION_NVP(t);
   outbound_data_ = archive_stream.str();
   llog(DEBUG1) << outbound_data_.size();
   // Format the header.
   std::ostringstream header_stream;
   header_stream << std::setw(kHeaderLength)
      << std::hex << outbound_data_.size();
   if (!header_stream || header_stream.str().size() != kHeaderLength) {
      // Something went wrong, inform the caller.
      boost::system::error_code error(boost::asio::error::invalid_argument);
      socket_.io_service().post(boost::bind(handler, error));
      return;
}
   outbound_header_ = header_stream.str();

   // Write the serialized data to the socket. We use "gather-write" to send
   // both the header and the data in a single write operation.
   std::vector<boost::asio::const_buffer> buffers;
   buffers.push_back(boost::asio::buffer(outbound_header_));
   buffers.push_back(boost::asio::buffer(outbound_data_));
   llog(DEBUG1) << outbound_header_ << outbound_data_ << std::endl;
   boost::asio::async_write(socket_, buffers, handler);
}

template <typename T, typename Handler>
void Connection::async_read(T& t, Handler handler) {
   // Issue a read operation to read exactly the number of bytes in a header.
   void(Connection::*f) (
      const boost::system::error_code&,
      T&, boost::tuple<Handler>)
      = &Connection::handle_read_header<T, Handler>;
   boost::asio::async_read(socket_, boost::asio::buffer(inbound_header_),
      boost::bind(f,
         this, boost::asio::placeholders::error, boost::ref(t),
         boost::make_tuple(handler)));
}

template <typename T>
boost::system::error_code Connection::sync_write(const T& t) {
   // Serialize the data first so we know how large it is.
   std::ostringstream archive_stream;
   boost::archive::binary_oarchive archive(archive_stream);
   archive << BOOST_SERIALIZATION_NVP(t);
   outbound_data_ = archive_stream.str();
   llog(DEBUG1) << outbound_data_.size();
   // Format the header.
   std::ostringstream header_stream;
   header_stream << std::setw(kHeaderLength) << std::hex <<
         outbound_data_.size();
   if (!header_stream || header_stream.str().size() != kHeaderLength) {
      // Something went wrong, inform the caller.
      boost::system::error_code error(boost::asio::error::invalid_argument);
      return error;
   }
   outbound_header_ = header_stream.str();

   // Write the serialized data to the socket. We use "gather-write" to send
   // both the header and the data in a single write operation.
   std::vector<boost::asio::const_buffer> buffers;
   buffers.push_back(boost::asio::buffer(outbound_header_));
   buffers.push_back(boost::asio::buffer(outbound_data_));
   llog(DEBUG1) << outbound_header_ << outbound_data_ << std::endl;
   // TODO(jayen): check for errors
   boost::asio::write(socket_, buffers);
   return boost::system::errc::make_error_code(boost::system::errc::success);
}

template <typename T>
boost::system::error_code Connection::sync_read(T& t) {
   // Issue a read operation to read exactly the number of bytes in a header.
   // TODO(jayen): check for errors
   boost::asio::read(socket_, boost::asio::buffer(inbound_header_));
   // Determine the length of the serialized data.
   std::istringstream is(std::string(inbound_header_, kHeaderLength));
   std::size_t inbound_data_size = 0;
   if (!(is >> std::hex >> inbound_data_size)) {
      // Header doesn't seem to be valid. Inform the caller.
      boost::system::error_code error(boost::asio::error::invalid_argument);
      return error;
   }
   llog(DEBUG1) << inbound_data_size << std::endl;

   // Start an synchronous call to receive the data.
   inbound_data_.resize(inbound_data_size);
   // TODO(jayen): check for errors
   boost::asio::read(socket_, boost::asio::buffer(inbound_data_));
   // Extract the data structure from the data just received.
   try {
      std::string archive_data(&inbound_data_[0], inbound_data_.size());
      llog(DEBUG1) << std::string(inbound_header_, kHeaderLength) <<
            std::string(inbound_data_.begin(),
                        inbound_data_.end()) <<
            std::endl;
      std::istringstream archive_stream(archive_data);
      boost::archive::binary_iarchive archive(archive_stream);
      archive >> BOOST_SERIALIZATION_NVP(t);
   } catch(std::exception& e) {
         // Unable to decode data.
      boost::system::error_code error(boost::asio::error::invalid_argument);
      return error;
   }
   return boost::system::errc::make_error_code(boost::system::errc::success);
}

template <typename T, typename Handler>
void Connection::handle_read_header(const boost::system::error_code& e,
                                    T& t, boost::tuple<Handler> handler) {
   if (e) {
      boost::get<0>(handler)(e);
   } else {
      // Determine the length of the serialized data.
      std::istringstream is(std::string(inbound_header_, kHeaderLength));
      std::size_t inbound_data_size = 0;
      if (!(is >> std::hex >> inbound_data_size)) {
      // Header doesn't seem to be valid. Inform the caller.
         boost::system::error_code error(boost::asio::error::invalid_argument);
         boost::get<0>(handler)(error);
         return;
      }
      llog(DEBUG1) << inbound_data_size << std::endl;

      // Start an asynchronous call to receive the data.
      inbound_data_.resize(inbound_data_size);
      void(Connection::*f)(
         const boost::system::error_code&,
         T&, boost::tuple<Handler>)
      = &Connection::handle_read_data<T, Handler>;
      boost::asio::async_read(socket_, boost::asio::buffer(inbound_data_),
      boost::bind(f, this,
         boost::asio::placeholders::error, boost::ref(t), handler));
   }
}

template <typename T, typename Handler>
void Connection::handle_read_data(const boost::system::error_code& e,
                                 T& t, boost::tuple<Handler> handler) {
   if (e) {
      boost::get<0>(handler)(e);
   } else {
      // Extract the data structure from the data just received.
      try {
         std::string archive_data(&inbound_data_[0], inbound_data_.size());
         llog(DEBUG1) << std::string(inbound_header_, kHeaderLength) <<
                         std::string(inbound_data_.begin(),
                                     inbound_data_.end()) <<
                         std::endl;
         std::istringstream archive_stream(archive_data);
         boost::archive::binary_iarchive archive(archive_stream);
         archive >> BOOST_SERIALIZATION_NVP(t);
      } catch(std::exception& e) {
         // Unable to decode data.
         boost::system::error_code error(boost::asio::error::invalid_argument);
         boost::get<0>(handler)(error);
         return;
      }

      // Inform caller that data has been received ok.
      boost::get<0>(handler)(e);
   }
}
