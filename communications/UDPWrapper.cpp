#include "UDPWrapper.h"
#include <iostream>

UDPWrapper::UDPWrapper (unsigned short port, bool broadcast, std::string dest_ip):
  in_sock_(io_service_),
  out_sock_(io_service_),
  port_(port)
{
  out_sock_.open(udp::v4());
  in_sock_.open(udp::v4());
  try {
    // allow multiple binds
    boost::asio::socket_base::reuse_address option(true);
    in_sock_.set_option(option);

    in_sock_.bind(udp::endpoint(udp::v4(),port));
  } catch (std::exception &e) {
    std::cout << "UDPWrapper: Warning, not binding in socket" << std::endl;
  }

  try {
    destination_ = new udp::endpoint(boost::asio::ip::address_v4::from_string(dest_ip),port);
  } catch (std::exception &e) {
    std::cerr << "UDPWrapper::UDPWrapper Error setting destination ip" << std::endl;
  }

  if (broadcast) {
    boost::asio::socket_base::broadcast option(true);
    out_sock_.set_option(option);
  }

}

UDPWrapper::~UDPWrapper() {
  delete destination_;
  in_sock_.close();
  out_sock_.close();
}

void UDPWrapper::startListenThread(void* (*method)(void *), void *core) {
  pthread_create(&listen_thread_,NULL, method, core);
}

boost::asio::ip::address UDPWrapper::senderAddress() {
  return sender_.address();
}

bool UDPWrapper::sendToSender(const boost::asio::const_buffers_1 &buffers,std::size_t size) {
    boost::asio::ip::address previous = destination_->address();
    destination_->address(senderAddress());
    send(buffers,size);
    destination_->address(previous);

    // If you're reading this it's probably time to delete it:
    std::cout << "Sent UDP response to " << senderAddress() << "\n";
}

bool UDPWrapper::send(const boost::asio::const_buffers_1 &buffers,std::size_t size) {
  try {
    std::size_t res = out_sock_.send_to(buffers,*destination_);
    if (res != size) {
      //std::cerr << "UDPWrapper::send Error sending message, expected send size: " << size << " got " << res << std::endl;
      return false;
    }
  } catch (std::exception &e) {
    //std::cerr << "UDPWrapper::send Error sending message: " << e.what() << std::endl;
    return false;
  }
  return true;
}

bool UDPWrapper::recv(boost::asio::mutable_buffers_1 buffers, std::size_t size, bool acceptLowerSize) {
  try {
    std::size_t res = in_sock_.receive_from(buffers,sender_);
    if (acceptLowerSize) {
      if ((res > size) || (res <= 0)) {
        //std::cerr << "UDPWrapper::recv Error receiving message, expected receive size <= " << size << " got " << res << std::endl;
        return false;
      }
    } else {
      if (res != size) {
        //std::cerr << "UDPWrapper::recv Error receiving message, expected receive size: " << size << " got " << res << std::endl;
        return false;
      }
    }
  } catch (std::exception &e) {
    //std::cerr << "UDPWrapper::recv Error receiving message: " << e.what() << std::endl;
    return false;
  }
  return true;
}
