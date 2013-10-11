#ifndef UDPWRAPPER_AQS3G85P
#define UDPWRAPPER_AQS3G85P

#include <boost/asio.hpp>
#include <string>
#include <pthread.h>
#include <iostream>

using boost::asio::ip::udp;

class UDPWrapper {
public:
  UDPWrapper (unsigned short port, bool broadcast, std::string dest_ip);
  virtual ~UDPWrapper ();

  void startListenThread(void* (*method)(void *), void *core);

  template <class T>
  bool send(const T& msg) {
    return send(boost::asio::buffer(&msg,sizeof(msg)),sizeof(msg));
  }
  bool send(const char *buffer, std::size_t size) {
    return send(boost::asio::buffer(buffer,size),size);
  }
  bool sendToSender(const char *buffer, std::size_t size) {
    return sendToSender(boost::asio::buffer(buffer,size),size);
  }


  template <class T>
  bool recv(T &buffer) {
    return recv(boost::asio::buffer(&buffer,sizeof(buffer)), sizeof(buffer),false);
  }

  bool recv(char *buffer, std::size_t size) {
    return recv(boost::asio::buffer(buffer,size),size,true);
  }

  boost::asio::ip::address senderAddress();

private:
  unsigned short port_;
  bool recv(boost::asio::mutable_buffers_1 buffers,std::size_t size, bool acceptLowerSize);
  bool send(const boost::asio::const_buffers_1 &buffers,std::size_t size);
  bool sendToSender(const boost::asio::const_buffers_1 &buffers,std::size_t size);

  boost::asio::io_service io_service_;
  udp::socket in_sock_;
  udp::socket out_sock_;
  udp::endpoint *destination_;

  udp::endpoint sender_;

  pthread_t listen_thread_;
};

#endif /* end of include guard: UDPWRAPPER_AQS3G85P */
